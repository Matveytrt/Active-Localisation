"""Actual RGB, native active global costmap, plan and live ROS statistics."""
import argparse
from collections import deque
import json
import math
from pathlib import Path
import signal
import time
import cv2
import numpy as np
import yaml
import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry, Path as NavPath
from nav2_msgs.msg import Costmap
from nav2_msgs.srv import GetCostmap
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatusArray
from rtabmap_msgs.msg import LandmarkDetections
from tf2_ros import Buffer, TransformListener, TransformException


def yaw(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))


class Video(Node):
    def __init__(self,path,total_goals=2):
        super().__init__('isaac_demo_video')
        self.total_goals=total_goals
        self.path,self.rgb,self.pose,self.start=path,None,None,None
        self.frames,self.trace,self.plan=0,[],[]
        self.writer=None
        self.wall=time.monotonic();self.start_wall=None
        self.grid=None;self.grid_count=0;self.grid_received=0.;self.pending=None
        self.goals={};self.recoveries={};self.distance_remaining=None
        self.distance=0.;self.detections=0;self.ids=set()
        self.history=deque(maxlen=5000);self.error_sum=0.;self.error_n=0;self.last_error_stamp=-1
        self.error_current=None
        markers=yaml.safe_load(Path('/workspace/maps/isaac-20261109-global.landmarks.yaml').read_text())
        self.markers=[(m['position']['x'],m['position']['y'],m['id']) for m in markers['big_markers']]
        self.buffer=Buffer();self.listener=TransformListener(self.buffer,self)
        self.create_subscription(Image,'/torso_rgbd/image_raw',self.camera,qos_profile_sensor_data)
        self.create_subscription(Odometry,'/odom_gt',self.odometry,qos_profile_sensor_data)
        self.create_subscription(Costmap,'/global_costmap/costmap_raw',self.costmap,qos_profile_sensor_data)
        self.create_subscription(NavPath,'/plan',self.on_plan,10)
        self.create_subscription(LandmarkDetections,'/landmark_detections',self.landmarks,10)
        latched=QoSProfile(depth=10,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(GoalStatusArray,'/navigate_to_pose/_action/status',self.status,latched)
        self.create_subscription(NavigateToPose.Impl.FeedbackMessage,'/navigate_to_pose/_action/feedback',self.feedback,10)
        self.grid_client=self.create_client(GetCostmap,'/global_costmap/get_costmap')
        self.create_timer(1.,self.request_costmap)
        self.create_timer(1/15,self.draw)

    def now(self):return self.get_clock().now().nanoseconds*1e-9

    def camera(self,msg):
        assert msg.encoding in ('rgb8','bgr8'),msg.encoding
        image=np.frombuffer(msg.data,dtype=np.uint8).reshape(msg.height,msg.step)[:,:msg.width*3].reshape(msg.height,msg.width,3)
        self.rgb=cv2.cvtColor(image,cv2.COLOR_RGB2BGR) if msg.encoding=='rgb8' else image.copy()

    def odometry(self,msg):
        p,q=msg.pose.pose.position,msg.pose.pose.orientation
        current=(p.x,p.y,yaw(q))
        self.history.append((msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9,p.x,p.y))
        if self.start is None and (math.hypot(p.x+2,p.y+.5)>.015 or abs(current[2])>.03):
            self.start=self.now();self.start_wall=time.monotonic()
        if self.start is not None and self.pose is not None:
            self.distance+=math.hypot(p.x-self.pose[0],p.y-self.pose[1])
            self.trace.append((p.x,p.y))
        self.pose=current

    def costmap(self,msg):
        if msg.metadata.size_x and msg.metadata.size_y:
            self.grid=msg;self.grid_count+=1;self.grid_received=self.now()

    def request_costmap(self):
        if not self.grid_client.service_is_ready() or (self.pending is not None and not self.pending.done()):return
        self.pending=self.grid_client.call_async(GetCostmap.Request())
        self.pending.add_done_callback(lambda f:self.costmap(f.result().map) if f.result() is not None else None)

    def on_plan(self,msg):
        self.plan=[(p.pose.position.x,p.pose.position.y,yaw(p.pose.orientation)) for p in msg.poses]

    def landmarks(self,msg):
        if self.start is None:return
        self.detections+=len(msg.landmarks)
        self.ids.update(m.id for m in msg.landmarks)

    def status(self,msg):
        for goal in msg.status_list:self.goals[bytes(goal.goal_info.goal_id.uuid).hex()]=goal.status

    def feedback(self,msg):
        key=bytes(msg.goal_id.uuid).hex()
        self.recoveries[key]=max(self.recoveries.get(key,0),msg.feedback.number_of_recoveries)
        self.distance_remaining=msg.feedback.distance_remaining

    def position_error(self):
        if len(self.history)<3:return
        t,x,y=self.history[-3]
        if t==self.last_error_stamp:return
        try:tf=self.buffer.lookup_transform('map','base_link',Time(seconds=t))
        except TransformException:return
        self.error_current=math.hypot(tf.transform.translation.x-x,tf.transform.translation.y-y)
        self.last_error_stamp=t;self.error_sum+=self.error_current**2;self.error_n+=1

    def draw(self):
        if self.rgb is None or self.pose is None or self.start is None or self.grid is None:return
        if self.writer is None:
            self.writer=cv2.VideoWriter(str(self.path),cv2.VideoWriter_fourcc(*'avc1'),15,(1600,900))
            assert self.writer.isOpened(),'H264 encoder unavailable'
        self.position_error()
        image=np.full((900,1600,3),22,np.uint8)
        image[60:780,:960]=cv2.resize(self.rgb,(960,720))
        title='ACTIVE LOCALISATION | ISAAC SIM'+(' | LOOP' if self.total_goals==3 else '')
        cv2.putText(image,title,(20,39),cv2.FONT_HERSHEY_SIMPLEX,.95,(255,255,255),2,cv2.LINE_AA)
        cv2.putText(image,'LIVE ACTIVE GLOBAL COSTMAP',(986,38),cv2.FONT_HERSHEY_SIMPLEX,.69,(255,255,255),2,cv2.LINE_AA)
        meta=self.grid.metadata;mw,mh=meta.size_x,meta.size_y
        values=np.asarray(self.grid.data,dtype=np.uint8).reshape(mh,mw)[::-1]
        colored=cv2.applyColorMap(np.minimum(values,252),cv2.COLORMAP_TURBO)
        colored[values==0]=(238,238,238);colored[values==253]=(0,0,190)
        colored[values==254]=(0,0,255);colored[values==255]=(65,65,65)
        scale=min(568/mw,512/mh);w,h=round(mw*scale),round(mh*scale);ox,oy=990,85
        image[oy:oy+h,ox:ox+w]=cv2.resize(colored,(w,h),interpolation=cv2.INTER_NEAREST)
        origin=meta.origin.position;angle=yaw(meta.origin.orientation)
        def point(x,y):
            dx,dy=x-origin.x,y-origin.y
            gx=(math.cos(angle)*dx+math.sin(angle)*dy)/meta.resolution
            gy=(-math.sin(angle)*dx+math.cos(angle)*dy)/meta.resolution
            return int(ox+gx*scale),int(oy+(mh-1-gy)*scale)
        if len(self.plan)>1:
            cv2.polylines(image,[np.array([point(p[0],p[1]) for p in self.plan],np.int32)],False,(255,220,0),2)
            for x,y,a in self.plan[::3]:cv2.arrowedLine(image,point(x,y),point(x+.22*math.cos(a),y+.22*math.sin(a)),(255,220,0),1,tipLength=.3)
        for x,y,marker in self.markers:
            v=point(x,y);cv2.circle(image,v,5,(0,100,255),-1)
            cv2.putText(image,str(marker),(v[0]+6,v[1]),cv2.FONT_HERSHEY_SIMPLEX,.42,(0,70,255),1,cv2.LINE_AA)
        if len(self.trace)>1:cv2.polylines(image,[np.array([point(*p) for p in self.trace],np.int32)],False,(0,200,0),3)
        x,y,a=self.pose
        cv2.circle(image,point(x,y),7,(0,230,0),-1)
        cv2.arrowedLine(image,point(x,y),point(x+.55*math.cos(a),y+.55*math.sin(a)),(0,230,0),3,tipLength=.3)
        elapsed=self.now()-self.start;rtf=elapsed/max(.01,time.monotonic()-self.start_wall)
        goals=sum(s==4 for s in self.goals.values());recoveries=sum(self.recoveries.values())
        cv2.putText(image,'static + obstacles + loc_layer + inflation',(990,625),cv2.FONT_HERSHEY_SIMPLEX,.50,(220,220,220),1,cv2.LINE_AA)
        bar=cv2.applyColorMap(np.tile(np.linspace(1,252,520,dtype=np.uint8),(14,1)),cv2.COLORMAP_TURBO)
        image[644:658,1000:1520]=bar
        cv2.putText(image,'low cost                         high cost / obstacle',(990,682),cv2.FONT_HERSHEY_SIMPLEX,.48,(220,220,220),1,cv2.LINE_AA)
        cv2.putText(image,'Cyan: current Nav2 plan + yaw | Green: GT trace',(986,714),cv2.FONT_HERSHEY_SIMPLEX,.49,(255,230,130),1,cv2.LINE_AA)
        age=max(0,self.now()-self.grid_received)
        cv2.putText(image,f'Costmap updates {self.grid_count} | age {age:.1f}s | LIVE',(986,745),cv2.FONT_HERSHEY_SIMPLEX,.54,(100,240,120),1,cv2.LINE_AA)
        cv2.rectangle(image,(0,783),(1599,899),(34,34,34),-1)
        rmse=math.sqrt(self.error_sum/self.error_n)*1000 if self.error_n else None
        error='--' if rmse is None else f'{rmse:.1f} mm'
        top=[f'TIME  {elapsed:.1f} s',f'PATH  {self.distance:.2f} m',f'DETECTIONS  {self.detections}',f'GOALS  {goals}/{self.total_goals} SUCCEEDED']
        bottom=[f'RTF  {rtf:.2f}',f'LIVE XY RMSE  {error}',f'DET/m  {self.detections/max(.01,self.distance):.1f}',f'RECOVERIES  {recoveries}']
        for index in range(4):
            cv2.putText(image,top[index],(18+index*397,818),cv2.FONT_HERSHEY_SIMPLEX,.64,(255,255,255),1,cv2.LINE_AA)
            cv2.putText(image,bottom[index],(18+index*397,851),cv2.FONT_HERSHEY_SIMPLEX,.59,(170,235,235),1,cv2.LINE_AA)
        cv2.putText(image,'Actual ROS topics | ACTIVE / seed42 / noise x4 | Live diagnostics, separate from the benchmark series',(18,881),cv2.FONT_HERSHEY_SIMPLEX,.48,(190,190,190),1,cv2.LINE_AA)
        self.writer.write(image);self.frames+=1;self.last_frame=image

    def close(self):
        if self.writer:self.writer.release()
        if self.frames:cv2.imwrite(str(self.path.with_name(self.path.stem+'-last.jpg')),self.last_frame)
        self.path.with_suffix('.json').write_text(json.dumps({'frames':self.frames,'fps':15,'duration_s':self.frames/15,
            'costmap_updates':self.grid_count,'costmap_source':'/global_costmap/costmap_raw and /global_costmap/get_costmap',
            'detections':self.detections,'ids':sorted(self.ids),'goals_succeeded':sum(s==4 for s in self.goals.values()),'total_goals':self.total_goals,
            'recoveries_reported':sum(self.recoveries.values()),'gt_path_m':self.distance,
            'live_xy_rmse_m':math.sqrt(self.error_sum/self.error_n) if self.error_n else None,
            'live_error_samples':self.error_n,'note':'Live diagnostics are not the original benchmark statistics'},indent=2))


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--output',required=True);parser.add_argument('--total-goals',type=int,default=2);args=parser.parse_args()
    path=Path(args.output);path.parent.mkdir(parents=True,exist_ok=True);assert not path.exists(),'Refuse overwrite'
    stopping=False
    def stop(_sig,_frame):
        nonlocal stopping
        stopping=True
    rclpy.init(args=['video','--ros-args','-p','use_sim_time:=true'],signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGINT,stop);signal.signal(signal.SIGTERM,stop)
    node=Video(path,args.total_goals)
    try:
        while not stopping and time.monotonic()-node.wall<480:rclpy.spin_once(node,timeout_sec=.1)
    finally:
        node.close();node.destroy_node();rclpy.try_shutdown()


if __name__=='__main__':main()
