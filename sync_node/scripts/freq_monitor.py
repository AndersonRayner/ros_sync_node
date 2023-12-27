#!/usr/bin/env python3

# from https://answers.ros.org/question/48579/using-rostopic-hz-in-a-ros-node/
# orginal ros code is https://github.com/ros/ros_comm/blob/noetic-devel/tools/rostopic/src/rostopic/__init__.py

import roslib #; roslib.load_manifest('ssnode')

import rospy
import roslib 
import roslib.message
import rostopic 

from rostopic import ROSTopicHz 
from std_msgs.msg import Bool



class freq_monitor:
  def __init__(self):
    rospy.init_node("freq_monitor", anonymous=True)
    rospy.loginfo("Registered frequency tracking node")
    
    self.pub = rospy.Publisher('sensors_ok',Bool, queue_size=10) 
    self.rate = rospy.Rate(2) # 2Hz

    # subscribe to sensor data here
    self.mono_cam_rt = self.sensor_subscribe("/eo/mono/image")
    self.color_cam_rt = self.sensor_subscribe("/eo/color/image")
    self.imu_rt = self.sensor_subscribe("/imu/imu")
    self.thermal_rt = self.sensor_subscribe("/boson/thermal/image_raw")
    
    self.monitor([(self.mono_cam_rt, 30), (self.color_cam_rt, 30), (self.imu_rt, 180), (self.thermal_rt, 60)])


  def sensor_subscribe(self, topic, window_size=30, filter_expr=None):

    rt = ROSTopicHz(window_size, filter_expr=filter_expr)

    sub = rospy.Subscriber(topic, rospy.AnyMsg, rt.callback_hz)
    print("subscribed to [%s]"%topic)

    return rt
  
  def monitor(self, rt_arr):
    
    while not rospy.is_shutdown():

      sensor_ok = Bool()

      for rt in rt_arr:
        freq = rt[0].get_hz()
        if freq == None:
          sensor_ok.data = False
          break
        else:
          minimum_rt = rt[1]
          if (minimum_rt-2.5 < freq[0]):
            sensor_ok.data = True
          else:
            sensor_ok.data = False
            break 

      self.pub.publish(sensor_ok)
      self.rate.sleep()



if __name__ == '__main__':
    try:
      
      fm = freq_monitor()
    except rospy.ROSInterruptException: 
      pass