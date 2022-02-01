#!/usr/bin/env python3

import rospy
import rosnode
import dynamic_reconfigure.client
from zivid_camera.srv import *
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud2

class Sample:
    def __init__(self):
        rospy.init_node("sample_capture_py", anonymous=True)

        rospy.loginfo("Starting sample_capture.py")

        ca_suggest_settings_service = "zivid_camera/capture_assistant/suggest_settings"

        rospy.wait_for_service(ca_suggest_settings_service, 30.0)

        self.capture_assistant_service = rospy.ServiceProxy(
            ca_suggest_settings_service, CaptureAssistantSuggestSettings
        )

        rospy.wait_for_service("zivid_camera/capture", 30.0)

        rospy.Subscriber("zivid_camera/points/xyzrgba", PointCloud2, self.on_points)

        self.capture_service = rospy.ServiceProxy("zivid_camera/capture", Capture)

    def capture_assistant_suggest_settings(self):
        max_capture_time = rospy.Duration.from_sec(1.20)
        rospy.loginfo(
            "Calling capture assistant service with max capture time = %.2f sec",
            max_capture_time.to_sec(),
        )
        self.capture_assistant_service(
            max_capture_time=max_capture_time,
            ambient_light_frequency=CaptureAssistantSuggestSettingsRequest.AMBIENT_LIGHT_FREQUENCY_NONE,
        )

    def capture(self):
        rospy.loginfo("Calling capture service")
        self.capture_service()

    def on_points(self, data):
        rospy.loginfo("PointCloud received")
        self.capture_assistant_suggest_settings()
        self.capture()


if __name__ == "__main__":
    s = Sample()
    s.capture_assistant_suggest_settings()
    s.capture()
    rospy.spin()