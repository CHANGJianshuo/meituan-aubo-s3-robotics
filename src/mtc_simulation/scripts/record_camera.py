#!/usr/bin/env python3
"""Record the simulated overview camera; images are evidence, not perception input."""
import json
from pathlib import Path
import sys
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

class Recorder(Node):
    def __init__(self,run):
        super().__init__('v1_camera_recorder')
        self.run=run;self.writer=None;self.last=-1.;self.count=0
        self.sub=self.create_subscription(Image,'/overview/image',self.receive,qos_profile_sensor_data)
    def receive(self,msg):
        stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        if stamp-self.last<.2:return
        if msg.encoding not in ('rgb8','bgr8'):return
        image=np.frombuffer(msg.data,dtype=np.uint8).reshape(msg.height,msg.step)[:,:msg.width*3].reshape(msg.height,msg.width,3)
        if msg.encoding=='rgb8':image=cv2.cvtColor(image,cv2.COLOR_RGB2BGR)
        else:image=image.copy()
        if self.writer is None:self.writer=cv2.VideoWriter(str(self.run/'overview.mp4'),cv2.VideoWriter_fourcc(*'mp4v'),5,(msg.width,msg.height))
        cv2.putText(image,f'V1 Gazebo | simulation t={stamp:.2f} s',(20,30),cv2.FONT_HERSHEY_SIMPLEX,.7,(30,30,30),2)
        self.writer.write(image);cv2.imwrite(str(self.run/'overview_latest.png'),image)
        if self.count==0:cv2.imwrite(str(self.run/'overview_initial.png'),image)
        self.count+=1;self.last=stamp
    def close(self):
        if self.writer is not None:self.writer.release()
        (self.run/'camera.json').write_text(json.dumps({'frames':self.count,'last_sim_time':self.last,'topic':'/overview/image'},indent=2)+'\n')

if __name__=='__main__':
    run=Path(sys.argv[1]);rclpy.init(args=[]);node=Recorder(run)
    try:rclpy.spin(node)
    except (KeyboardInterrupt,rclpy.executors.ExternalShutdownException):pass
    finally:
        node.close();node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
