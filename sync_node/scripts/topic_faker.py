#!/usr/bin/env python3

import rospy
from std_msgs.msg import Empty

class TopicPublisher:
    def __init__(self):
        rospy.init_node('topic_publisher_node', anonymous=True)

        self.topic1_pub = rospy.Publisher('/sensor_5Hz' , Empty, queue_size=10)
        self.topic2_pub = rospy.Publisher('/sensor_30Hz', Empty, queue_size=10)
        self.topic3_pub = rospy.Publisher('/sensor_60Hz', Empty, queue_size=10)

        self.skip_this_pub = False

        # Timers for publishing on topics
        rospy.Timer(rospy.Duration(1.0 / 5.0), self.publish_topic1)  # 5 Hz
        rospy.Timer(rospy.Duration(1.0 / 30.0), self.publish_topic2)  # 30 Hz
        rospy.Timer(rospy.Duration(1.0 / 60.0), self.publish_topic3)  # 60 Hz

    def publish_topic1(self, event):
        # For the first 5 seconds of every 30 s, don't publish 
        if ((rospy.get_time() % 30) < 5.0) :
            # do nothing
            return

        self.topic1_pub.publish(Empty())


    def publish_topic2(self, event):
        self.topic2_pub.publish(Empty())

    def publish_topic3(self, event):

        if ((rospy.get_time() % 60.0) > 30.0) :

            # Reduced rate half the time
            if self.skip_this_pub :
                # do nothing
                self.skip_this_pub = False
                return
            
            self.skip_this_pub = True

        self.topic3_pub.publish(Empty())

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        topic_publisher = TopicPublisher()
        topic_publisher.run()
    except rospy.ROSInterruptException:
        pass
