"""Repeated real ROS message conversion + SIGINT. Synthetic fixture, not Nav2 data."""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import time
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped
import yaml


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--output',required=True)
    parser.add_argument('--cycles',type=int,default=20)
    args=parser.parse_args()
    output=Path(args.output);output.mkdir(parents=True,exist_ok=False)
    rclpy.init()
    node=Node('recorder_shutdown_fixture')
    odom_pub=node.create_publisher(Odometry,'/odom_gt',10)
    clock_pub=node.create_publisher(Clock,'/clock',10)
    tf_pub=node.create_publisher(TFMessage,'/tf',10)
    tick=0

    def publish():
        nonlocal tick
        tick+=1
        seconds=tick*.02
        clock=Clock();clock.clock.sec=int(seconds);clock.clock.nanosec=int((seconds%1)*1e9)
        clock_pub.publish(clock)
        msg=Odometry();msg.header.frame_id='world';msg.child_frame_id='base_link'
        msg.header.stamp=clock.clock;msg.pose.pose.position.x=-2.;msg.pose.pose.position.y=-.5
        msg.pose.pose.orientation.w=1.;odom_pub.publish(msg)
        transforms=[]
        for parent,child,x,y in [('map','odom',0.,0.),('odom','base_link',-2.,-.5)]:
            tf=TransformStamped();tf.header.frame_id=parent;tf.header.stamp=clock.clock;tf.child_frame_id=child
            tf.transform.translation.x=x;tf.transform.translation.y=y;tf.transform.rotation.w=1.
            transforms.append(tf)
        tf_pub.publish(TFMessage(transforms=transforms));rclpy.spin_once(node,timeout_sec=0)

    records=[]
    try:
        for index in range(args.cycles):
            name=f'cycle_{index:02d}'
            with (output/(name+'.log')).open('w') as log:
                process=subprocess.Popen(['python3','/repo/isaac/run_recorder.py','--ros-args',
                    '--params-file','/repo/navigation/src/al_eval/config/traj_recorder.yaml',
                    '-p','output_dir:='+str(output),'-p','run_name:='+name,'-p','use_sim_time:=true'],
                    stdout=log,stderr=subprocess.STDOUT)
                try:
                    deadline=time.monotonic()+20
                    while odom_pub.get_subscription_count()==0:
                        assert process.poll() is None,'Recorder exited before discovery'
                        assert time.monotonic()<deadline,'Recorder discovery timeout'
                        publish();time.sleep(.02)
                    for _ in range(35): publish();time.sleep(.02)
                    process.send_signal(signal.SIGINT)
                    for _ in range(8): publish();time.sleep(.005)
                    status=process.wait(timeout=10)
                    assert status==0,(name,status)
                    meta=yaml.safe_load((output/name/'meta.yaml').read_text())
                    assert meta['counters']['gt_written']>0
                    rows=[l for l in (output/name/'gt.tum').read_text().splitlines() if not l.startswith('#')]
                    assert len(rows)==meta['counters']['gt_written']
                    records.append({'cycle':index,'exit_code':status,'gt_samples':len(rows)})
                finally:
                    if process.poll() is None: process.kill();process.wait()
            print('RECORDER_CYCLE_PASSED',index,flush=True)
            deadline=time.monotonic()+5
            while odom_pub.get_subscription_count()>0 and time.monotonic()<deadline:
                publish();time.sleep(.02)
        (output/'result.json').write_text(json.dumps({'status':'passed','cycles':records},indent=2))
    finally:
        node.destroy_node();rclpy.try_shutdown()


if __name__=='__main__': main()
