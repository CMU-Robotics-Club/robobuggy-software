#! /usr/bin/env python3
import rospy

# ROS Message Imports
from std_msgs.msg import Float32, Bool
from geometry_msgs.msg import PoseStamped

import threading
import numpy as np

class Controller:
  RATE = 10 # Hz
  MODE = ...
  WHEELBASE = 1.3

  def __init__(self):
    self.lock = threading.Lock()
    self.pose = None
    self.speed = None

    self.steering_angle = 0
    self.brake = 0

    self.steer_publisher = rospy.Publisher("buggy/inpyt/steering", Float32, queue_size=10)
    self.brake_publisher = rospy.Publisher("buggy/input/brake", Bool, queue_size=10)

    rospy.Subscriber("state/pose", PoseStamped, self.set_pose)
    rospy.Subscriber("state/speed", Float32, self.set_speed)


  def set_pose(self, msg):
    with self.lock:
      self.pose = msg.data
  
  def set_speed(self, msg):
    with self.lock:
      self.speed = msg.data
  
  def cmd_steering(self, angle):
    msg = Float32(angle)
    self.steer_publisher.publish(msg)
    with self.lock:
      self.steering_angle = angle
  
  def cmd_braking(self, is_brake):
    msg = Bool(is_brake)
    self.brake_publisher.publish(msg)
    with self.lock:
      self.brake = is_brake

  def step(self):
    # TODO: Fill in this function
    pass
    # Use 

  def run(self):
    rate = rospy.Rate(self.RATE)
    while not rospy.is_shutdown():
      with self.lock:
        self.step()
      rate.sleep()


if __name__ == '__main__':
  rospy.init_node("controller")
  controller = Controller()
  controller.run()