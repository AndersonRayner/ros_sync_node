#!/usr/bin/env python3

# from https://answers.ros.org/question/48579/using-rostopic-hz-in-a-ros-node/
# orginal ros code is https://github.com/ros/ros_comm/blob/noetic-devel/tools/rostopic/src/rostopic/__init__.py

import rospy

from rostopic import ROSTopicHz 
from sync_msgs.msg import sensorHealth, sensorHealthArray, sensorHealthArrayNamed

class freq_monitor:
    def __init__(self):

        # ROS Node
        rospy.init_node("~", anonymous=True)
        rospy.loginfo("Sensor status tracking node")
        self.rate = rospy.Rate(1)

        # Severity mapping
        self.severity_mapping = {
            sensorHealth.UNKNOWN: 1,
            sensorHealth.HEALTHY: 2,
            sensorHealth.UNHEALTHY: 3,
            sensorHealth.MISSING: 4,
        }

        # Parameters
        output_topic = rospy.get_param("~output_topic",default="/sensor_status")  # How to not need the name of the node in here?

        # Publisher
        self.pub = rospy.Publisher(output_topic,sensorHealthArray, queue_size=1) 
        # self.pub = rospy.Publisher(output_topic,sensorHealthArrayNamed, queue_size=1) 

        # Set up topic trackers
        topics = rospy.get_param("~topics")

        topic_trackers = []
        for sensor_name, values in topics.items():
            name = values['name']
            topic = values['topic']
            min_freq = values['min']
            max_freq = values['max']
            window = values['window']

            rospy.loginfo("%s: Tracking %s",sensor_name,name)

            #                                         # (Topic, window), name,      min,      max
            topic_trackers.append((self.sensor_subscribe(topic, window), name, min_freq, max_freq))

        # Run the main program
        while not rospy.is_shutdown():
            
            # Run the monitor
            self.monitor(topic_trackers)
            # self.monitor_named(topic_trackers)

            # Loop sleep until next time
            self.rate.sleep()

    def sensor_subscribe(self, topic, window, filter_expr=None):

        rospy.loginfo("\t%s (-w %d)",topic,window)

        topic_tracker = ROSTopicHz(window_size=window, filter_expr=filter_expr)
        rospy.Subscriber(topic, rospy.AnyMsg, topic_tracker.callback_hz)

        # Return frequency tracker
        return topic_tracker

    def fill_sensor(self, tracker) :

        sensor = sensorHealth()

        # Name
        sensor.name = tracker[1]

        # Rate
        rate_data = tracker[0].get_hz()
        if rate_data is None :
            sensor.rate = 0.0
        else :
            sensor.rate = rate_data[0]

        # Health
        if (sensor.rate > tracker[2]) and (sensor.rate < tracker[3]) :
            sensor.status = sensorHealth.HEALTHY
        elif (sensor.rate == 0) :
            sensor.status = sensorHealth.MISSING
        else :
            sensor.status = sensorHealth.UNHEALTHY

        return sensor

    def monitor_named(self, tracker_array):
        
        # Create message

        sensor_array = sensorHealthArrayNamed()
        sensor_array.header.stamp = rospy.Time.now()
        sensor_array.header.frame_id = 'topic_tracker'

        # Fill sensor slots
        sensor_array.gvrbot = self.fill_sensor(tracker_array[0])
        sensor_array.vn100  = self.fill_sensor(tracker_array[1])
        sensor_array.color  = self.fill_sensor(tracker_array[2])
        sensor_array.infra  = self.fill_sensor(tracker_array[3])
        sensor_array.pose   = self.fill_sensor(tracker_array[4])
        sensor_array.map    = self.fill_sensor(tracker_array[5])

        # Publish the message array
        self.pub.publish(sensor_array)

    def monitor(self, tracker_array):
        
        # Create message

        sensor_array = sensorHealthArray()
        sensor_array.header.stamp = rospy.Time.now()
        sensor_array.header.frame_id = 'topic_tracker'
        sensor_array.system = sensorHealth.HEALTHY

        # Loop through all tracked topics
        for tracker in tracker_array:

            # Create the sensor
            sensor_msg = sensorHealth()
            sensor_msg.name = tracker[1]

            # Work out the sensor rate
            rate_data = tracker[0].get_hz()

            if rate_data is None :
                sensor_msg.rate = 0.0
            else :
                sensor_msg.rate = rate_data[0] 

            # Update health
            sensor_msg.status = sensorHealth.UNKNOWN

            if (sensor_msg.rate > tracker[2]) and (sensor_msg.rate < tracker[3]) :
                sensor_msg.status = sensorHealth.HEALTHY
            elif (sensor_msg.rate == 0) :
                sensor_msg.status = sensorHealth.MISSING
            else :
                sensor_msg.status = sensorHealth.UNHEALTHY

            # Append sensor to sensor array
            sensor_array.sensor.append(sensor_msg)

            # Update overall system health if worse than current value
            if self.severity_mapping[sensor_msg.status] > self.severity_mapping[sensor_array.system]:
                sensor_array.system = sensor_msg.status

        # Publish the message array
        # sensor_array.system = sensorHealth.MISSING

        self.pub.publish(sensor_array)

# Main program
if __name__ == '__main__':
    try:
        # Start the frequency monitor
        freq_monitor()

    except rospy.ROSInterruptException:
        # Node is being shut down
        pass
