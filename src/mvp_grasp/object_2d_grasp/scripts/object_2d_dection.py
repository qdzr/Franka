#!/usr/bin/python
from tkinter.messagebox import NO
import tf
import rospy
from geometry_msgs.msg import Pose
from std_msgs.msg import Float32MultiArray, Int16, String


def retcallback(msg):
    rate = rospy.Rate(1)
    listener = tf.TransformListener()
    global object_class
    object_class = int(msg.data[0])
    object_frame = ['/object_{}'.format(object_class)]
    # print(type(object_frame[0]))
    rospy.loginfo('object_frame: {0}'.format(object_frame[0]))
    try:
        listener.waitForTransform('panda_link0', object_frame[0], rospy.Time(), rospy.Duration(4.0))
        (trans, rot) = listener.lookupTransform('panda_link0', object_frame[0], rospy.Time(0))
        rospy.loginfo('position: {0}'.format([trans[0], trans[1], trans[2]]))
        rospy.loginfo('orientation: {0}'.format([rot[0], rot[1], rot[2], rot[3]]))
        object_position_pose(trans, rot)
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        rospy.INFO('lookupTransform error')
    

def object_position_pose(trans, rot):
    global pub
    p = Pose()
    rate = rospy.Rate(50)
    p.position.x = trans[0]
    p.position.y = trans[1]
    p.position.z = 0.0335

    p.orientation.x = rot[0]
    p.orientation.y = rot[1]
    p.orientation.z = rot[2]
    p.orientation.w = rot[3]
    pub.publish(p)
    rate.sleep()


if __name__ == "__main__":
    rospy.init_node('pose_publish')
    pub = rospy.Publisher('/objection_position_pose', Pose, queue_size=10)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        rospy.loginfo('Subscriber topic \'/object\'')
        msg = rospy.wait_for_message('/objects', Float32MultiArray, timeout=30)
        if msg.data:
            ret = rospy.Subscriber('/objects', Float32MultiArray, retcallback)
        else:
            rospy.loginfo('Subscriber topic \'/object\' failed')
            continue
        # ret = rospy.wait_for_message('/objects', Float32MultiArray, timeout=20)
        # print(type(ret))
        # print(ret.data)
        rate.sleep()
        rospy.spin()


