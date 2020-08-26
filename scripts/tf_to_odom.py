#!/usr/bin/env python
import roslib
roslib.load_manifest('odom_to_tf')
import rospy
import tf
from nav_msgs.msg import Odometry
import geometry_msgs.msg

class TFToOdom:
  def __init__(self):
    odometry_frame = rospy.get_param('~odom_frame_id', 'odom')
    base_frame = rospy.get_param('~base_frame_id', 'base_link')
    odom_pub = rospy.Publisher("odom", Odometry, queue_size=2)
    listener = tf.TransformListener()
    
    rate_hz = rospy.get_param('~rate', default=10.0)
    rate = rospy.Rate(rate_hz)

    frame_catched = False
    
    odom_msg = Odometry()
    odom_msg.header.frame_id = odometry_frame
    odom_msg.header.seq = 0
    
    last_pos = None
    last_rot = None
    
    rospy.loginfo('Tf to Odom intialized. Odom frame: %s, Base frame: %s', odometry_frame, base_frame )
    
    while not rospy.is_shutdown():
      try:
          pos, rot = listener.lookupTransform(odometry_frame, base_frame, rospy.Time(0))
          
        #   print pos
        #   print rot
          
          if last_pos is not None:
            odom_msg.header.stamp = rospy.Time.now()
            odom_msg.pose.pose.position.x = pos[0]
            odom_msg.pose.pose.position.y = pos[1]
            odom_msg.pose.pose.position.z = pos[2]
            odom_msg.pose.pose.orientation.x = rot[0]
            odom_msg.pose.pose.orientation.y = rot[1]
            odom_msg.pose.pose.orientation.z = rot[2]
            odom_msg.pose.pose.orientation.w = rot[3]
            
            # TODO: calculate the twist!!
            
            odom_pub.publish(odom_msg)
            odom_msg.header.seq += 1
            
          rate.sleep()
          last_pos = pos
          last_rot = rot
          
      except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
    

if __name__ == '__main__':
    rospy.init_node('tf_to_odom')
    a = TFToOdom()
    rospy.spin()
