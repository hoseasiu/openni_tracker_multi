#!/usr/bin/env python
import roslib
roslib.load_manifest('openni_tracker_multi')
import rospy
import tf

if __name__ == '__main__':
    rospy.init_node('fixed_tf_broadcaster')
    tracker1_tf = tf.TransformBroadcaster()
    tracker2_tf = tf.TransformBroadcaster()
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        # the first kinect is 0.5m ahead of the origin
        tracker1_tf.sendTransform((0.25, 0.0, 0.0),
                         (0.0, 0.0, 0.0, 1.0),
                         rospy.Time.now(),
                         "kinect1_depth_frame",
                         "origin")
        # the second kinect is 0.5m behind the origin and yawed 90 degrees
        tracker2_tf.sendTransform((-0.25, 0.0, 0.0),
                         tf.transformations.quaternion_from_euler(0, 0, 3.1415),
                         rospy.Time.now(),
                         "kinect2_depth_frame",
                         "origin")
        rate.sleep()
