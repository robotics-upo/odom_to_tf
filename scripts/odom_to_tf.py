#!/usr/bin/env python
import roslib
roslib.load_manifest('odom_to_tf')
import rospy
import numpy as np

import tf
import tf2_ros
from nav_msgs.msg import Odometry
import geometry_msgs.msg
from sensor_msgs.msg import Imu
import tf_conversions.posemath as pm

class OdomToTF:
  def __init__(self):
    self.last_imu_orientation = None
    self.base_link = "/base_link"
    self.odometry_frame = rospy.get_param('~odom_frame_id', 'odom')
    self.base_link = rospy.get_param('~base_frame_id', 'base_link')
    self.override_stamp = rospy.get_param('~overridestamp', default=False)
    rospy.Subscriber("odom", Odometry, self.odometryCb)
    rospy.Subscriber("imu", Imu, self.imuCb)
    self.laser_frame = rospy.get_param('~laser_frame', 'os1_lidar')
    self.rotate = rospy.get_param('~rotate_90', True)
    listener = tf.TransformListener()
    self.br = tf2_ros.TransformBroadcaster()

    # Initial transforms
    self.last_published_transform = pm.fromMatrix(np.eye(4))
    self.last_odom_transform = pm.fromMatrix(np.eye(4))

    self.lidar_tf_pm = None

    frame_catched = False
    while not rospy.is_shutdown() and not frame_catched:
      try:
          self.lidar_tf = listener.lookupTransform(self.base_link, self.laser_frame, rospy.Time(0))
          frame_catched = True
          self.lidar_tf_pm = pm.fromTf(self.lidar_tf)
          #print "siiiiiiii"
      except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
    
    position = (0.0, 0.0, 0.0)
    orientation = (0.0, 0.0, -0.707, 0.707)
    self.rotation = pm.fromTf( (position, orientation))

  def odometryCb(self, msg):

    if self.lidar_tf_pm == None:
      print "Lidar tf not found"
      return
    #print "heyyyyyyyyyy"
    t = geometry_msgs.msg.TransformStamped()
    if self.override_stamp:
      t.header.stamp = rospy.Time.now()
    else:
      t.header.stamp = msg.header.stamp
    t.header.frame_id = self.odometry_frame
    t.child_frame_id = self.base_link

    position = (msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z)
    orientation = (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                   msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)

    # self.current_odom_transform = tf.TransformerROS.fromTranslationRotation(position, orientation)
    self.current_odom_transform = pm.fromTf( (position, orientation))
    self.base_link_tf = self.lidar_tf_pm * self.current_odom_transform * self.lidar_tf_pm.Inverse()
    if self.rotate:
      self.base_link_tf = self.rotation * self.base_link_tf; 

    a = pm.toMsg(self.base_link_tf)
    t.transform.rotation.x = a.orientation.x
    t.transform.rotation.y = a.orientation.y
    t.transform.rotation.z = a.orientation.z
    t.transform.rotation.w = a.orientation.w
    if self.rotate:
    	t.transform.translation.x = -a.position.y
   	t.transform.translation.y = a.position.x
    else:
    	t.transform.translation.x = a.position.x
    	t.transform.translation.y = a.position.y

    t.transform.translation.z = a.position.z
    self.br.sendTransform(t)

    #delta_t = self.last_odom_transform.Inverse() * self.base_link_tf
    #self.last_published_transform = self.last_published_transform * delta_t
    #a = pm.toMsg(self.last_published_transform)
    #if self.last_imu_orientation != None:
    #  a.orientation = self.last_imu_orientation
    #  self.last_odom_transform = pm.fromMsg(a)
    #t.transform.rotation.x = a.orientation.x
    #t.transform.rotation.y = a.orientation.y
    #t.transform.rotation.z = a.orientation.z
    #t.transform.rotation.w = a.orientation.w
    #t.transform.translation.x = self.last_published_transform[(0,3)]
    #t.transform.translation.y = self.last_published_transform[(1,3)]
    #t.transform.translation.z = self.last_published_transform[(2,3)]

    #self.last_odom_transform = self.base_link_tf
    #self.br.sendTransform(t)

  def imuCb(self, msg):
    self.last_imu_orientation = msg.orientation

if __name__ == '__main__':
    last_quaternion = None
    rospy.init_node('odom_to_tf_broadcaster')
    a = OdomToTF()
    rospy.spin()
