"""Actual torso RGB + map/GT overlay. Presentation recording, not evaluation."""
import argparse
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
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry


class Video(Node):
    def __init__(self, path):
        super().__init__('isaac_demo_video')
        self.path,self.rgb,self.pose,self.start=path,None,None,None
        self.frames,self.trace=0,[]
        self.writer=None
        self.wall=time.monotonic()
        prefix=Path('/workspace/maps/isaac-20261109-bounded')
        spec=yaml.safe_load(prefix.with_suffix('.yaml').read_text())
        self.origin,self.res=spec['origin'],spec['resolution']
        self.map=cv2.imread(str(prefix.with_suffix('.pgm')),cv2.IMREAD_GRAYSCALE)
        self.map=cv2.cvtColor(self.map,cv2.COLOR_GRAY2BGR)
        landmarks=yaml.safe_load(Path('/workspace/maps/isaac-20261109-global.landmarks.yaml').read_text())
        self.markers=[(m['position']['x'],m['position']['y'],m['id']) for m in landmarks['big_markers']]
        self.create_subscription(Image,'/torso_rgbd/image_raw',self.camera,qos_profile_sensor_data)
        self.create_subscription(Odometry,'/odom_gt',self.odometry,qos_profile_sensor_data)
        self.create_timer(1/15,self.draw)

    def camera(self,msg):
        assert msg.encoding in ('rgb8','bgr8'),msg.encoding
        image=np.frombuffer(msg.data,dtype=np.uint8).reshape(msg.height,msg.step)[:,:msg.width*3].reshape(msg.height,msg.width,3)
        self.rgb=cv2.cvtColor(image,cv2.COLOR_RGB2BGR) if msg.encoding=='rgb8' else image.copy()

    def odometry(self,msg):
        p,q=msg.pose.pose.position,msg.pose.pose.orientation
        yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        self.pose=(p.x,p.y,yaw)
        if self.start is None and (math.hypot(p.x+2,p.y+.5)>.015 or abs(yaw)>.03):
            self.start=self.get_clock().now().nanoseconds*1e-9

    def draw(self):
        if self.rgb is None or self.pose is None or self.start is None:return
        if self.writer is None:
            self.writer=cv2.VideoWriter(str(self.path),cv2.VideoWriter_fourcc(*'avc1'),15,(1280,720))
            assert self.writer.isOpened(),'H264 encoder unavailable'
        canvas=np.full((720,1280,3),24,np.uint8)
        canvas[:,:960]=cv2.resize(self.rgb,(960,720))
        cv2.rectangle(canvas,(0,0),(960,42),(20,20,20),-1)
        cv2.putText(canvas,'ACTIVE LOCALISATION | ISAAC SIM',(18,29),cv2.FONT_HERSHEY_SIMPLEX,.75,(255,255,255),2,cv2.LINE_AA)
        mh,mw=self.map.shape[:2];scale=min(288/mw,330/mh)
        w,h=round(mw*scale),round(mh*scale);ox,oy=976,100
        canvas[oy:oy+h,ox:ox+w]=cv2.resize(self.map,(w,h),interpolation=cv2.INTER_NEAREST)
        def point(x,y):
            return (int(ox+(x-self.origin[0])/self.res*scale),int(oy+(mh-1-(y-self.origin[1])/self.res)*scale))
        for x,y,marker in self.markers:
            v=point(x,y);cv2.circle(canvas,v,4,(0,100,255),-1)
            cv2.putText(canvas,str(marker),(v[0]+5,v[1]),cv2.FONT_HERSHEY_SIMPLEX,.35,(0,80,255),1,cv2.LINE_AA)
        self.trace.append(point(*self.pose[:2]))
        if len(self.trace)>1:cv2.polylines(canvas,[np.array(self.trace,np.int32)],False,(70,190,30),2)
        x,y,yaw=self.pose
        c=point(x,y);tip=point(x+.5*math.cos(yaw),y+.5*math.sin(yaw))
        cv2.circle(canvas,c,5,(60,220,30),-1);cv2.arrowedLine(canvas,c,tip,(30,220,60),2,tipLength=.3)
        elapsed=self.get_clock().now().nanoseconds*1e-9-self.start
        labels=['ACTIVE / seed 42 / noise x4','Torso RGB - live render','Map + GT path (overlay)',f'Simulation: {elapsed:5.1f} s',
                f'x {x:+.2f} m   y {y:+.2f} m',f'yaw {math.degrees(yaw):+.1f} deg','Green: robot / trajectory','Orange: mapped HD17 tags','New demo, not benchmark series']
        for index,label in enumerate(labels):
            yy=45 if index==0 else 455+(index-1)*27
            cv2.putText(canvas,label,(974,yy),cv2.FONT_HERSHEY_SIMPLEX,.40,(230,230,230),1,cv2.LINE_AA)
        self.writer.write(canvas);self.frames+=1

    def close(self):
        if self.writer:self.writer.release()
        self.path.with_suffix('.json').write_text(json.dumps({'frames':self.frames,'fps':15,'duration_s':self.frames/15,
            'content':'Actual torso RGB with map and GT evaluation overlay','starts':'first base movement','wall_seconds':time.monotonic()-self.wall},indent=2))


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--output',required=True);args=parser.parse_args()
    path=Path(args.output);path.parent.mkdir(parents=True,exist_ok=True);assert not path.exists(),'Refuse overwrite'
    stopping=False
    def stop(_sig,_frame):
        nonlocal stopping
        stopping=True
    rclpy.init(args=['video','--ros-args','-p','use_sim_time:=true'],signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGINT,stop);signal.signal(signal.SIGTERM,stop)
    node=Video(path)
    try:
        while not stopping and time.monotonic()-node.wall<480:rclpy.spin_once(node,timeout_sec=.1)
    finally:
        node.close();node.destroy_node();rclpy.try_shutdown()


if __name__=='__main__':main()
