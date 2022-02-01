import abc
import rospy
import roslib
import numpy as np
import time
import cv2
import math
import open3d as o3d
from cv_bridge import CvBridge, CvBridgeError

import os
import message_filters
from std_msgs.msg import String
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from cares_msgs.msg import StereoCameraInfo
import sensor_msgs.point_cloud2 as pc2

import cares_lib_ros.utils as utils

import json

from std_msgs.msg import Empty
try:
  from zivid_camera.srv import *
except ImportError as error:
  print("No Zivid install found, if you want to run zivid please make sure it is installed")


def image_msg_to_cv2(data):
  try:
      bridge = CvBridge()
      # if data.encoding == "bayer_bggr8":
      #     image = bridge.imgmsg_to_cv2(data, "bgr8")
      #     return image
      # elif data.encoding == "rgba":
      #     image = bridge.imgmsg_to_cv2(data, )
      #     return image
      image = bridge.imgmsg_to_cv2(data, "bgr8")
  except CvBridgeError as e:
      print(e)
  return image

def depth_msg_to_cv2(data):
    try:
      bridge = CvBridge()
      image = bridge.imgmsg_to_cv2(data, data.encoding)
    except CvBridgeError as e:
      print(e)
    return image

class DataSampler(object):
  __metaclass__ = abc.ABCMeta
  def __init__(
        self,
        sensor_name,
        image_topic,
        depth_image_topic,
        xyzrgb_topic,
        sensor_link,
        camera_info_topic="",
        stereo_info_topic=""
      ):

    self.sensor_name = sensor_name

    self.image_topics = {}
    self.image_topics['image'] = self.format_topic(image_topic)

    self.depth_image_topic = self.format_topic(depth_image_topic)
    self.camera_info_topic = self.format_topic(camera_info_topic)
    self.stereo_info_topic = self.format_topic(stereo_info_topic)
    self.xyzrgb_topic      = self.format_topic(xyzrgb_topic)
    
    self.image       = None
    self.depth_image = None
    self.camera_info = None
    self.xyzrgb      = None

    self.sensor_link = sensor_link

    self.time_stamp = None
  
    self.data_ready = False

    self.bridge = CvBridge()

  def format_topic(self, topic):
    return self.sensor_name+"/"+topic if topic != "" else ""

  def cb_once(self, *args):
    stream_ind = args[-1]
    subs = args[-2]
    min_time = args[-3]
    
    num_subs = len(args)-3
    msgs = list(args[:num_subs])

    msg_time = msgs[0].header.stamp
    if msg_time < min_time:
      return

    self.time_stamp = msgs[0].header.stamp
    for i, data in enumerate(msgs):
      if stream_ind[i] == 'image':
        self.image_msg = data
        self.image = image_msg_to_cv2(data)

      elif stream_ind[i] == 'left':
        self.left_image_msg = data
        self.left_image = image_msg_to_cv2(data)

      elif stream_ind[i] == 'right':
        self.right_image_msg = data
        self.right_image = image_msg_to_cv2(data)
          
      elif stream_ind[i] == 'depth_image':
        self.depth_image_msg = data                
        self.depth_image = depth_msg_to_cv2(data)

      elif stream_ind[i] == 'camera_info':
        self.camera_info = data

      elif stream_ind[i] == 'stereo_info':
        self.stereo_info = data

      elif stream_ind[i] == 'points':
        self.xyzrgb_msg_header = data.header
        # get opend3d point cloud from point cloud message
        if type(self.xyzrgb) == o3d.geometry.PointCloud:
            self.xyzrgb.clear()
            del self.xyzrgb
        self.xyzrgb = utils.pointCloud2Open3D(data)
          
    # unregister subscribers
    [sub.unregister() for sub in subs]
    self.data_ready = True

  def sample_multiple_streams(
      self, 
      rgb_image=True,
      depth_image=True,
      points=True, 
      camera_info=True):
    
    subs = []
    stream_ind = []
    self.data_ready = False
    
    if rgb_image:
      for key, value in self.image_topics.items():
        if value != "":
          sub_image_once = message_filters.Subscriber(value, Image)
          subs.append(sub_image_once)
          stream_ind.append(key)
          print(value)
    
    if depth_image and self.depth_image_topic != "":
      sub_depth_image_once = message_filters.Subscriber(self.depth_image_topic, Image)
      subs.append(sub_depth_image_once)
      stream_ind.append('depth_image')
      print(self.depth_image_topic)
    
    if points and self.xyzrgb_topic != "":    
      sub_xyzrgb_once = message_filters.Subscriber(self.xyzrgb_topic, PointCloud2)
      subs.append(sub_xyzrgb_once)
      stream_ind.append('points')
      print(self.xyzrgb_topic)

    if camera_info:
      sub_info_once = None
      if self.stereo_info_topic != "":
        sub_info_once = message_filters.Subscriber(self.stereo_info_topic, StereoCameraInfo)
        subs.append(sub_info_once)  
        stream_ind.append('stereo_info')
        print(self.stereo_info_topic)

      if self.camera_info_topic != "":
        sub_info_once = message_filters.Subscriber(self.camera_info_topic, CameraInfo)
        subs.append(sub_info_once)  
        stream_ind.append('camera_info')
        print(self.camera_info_topic)

    min_time = rospy.Time.now()
    ts = message_filters.TimeSynchronizer([sub for sub in subs], 20)
    ts.registerCallback(self.cb_once, min_time, subs, stream_ind)
    
    rate = rospy.Rate(5)#Hz
    while self.data_ready == False:
        self.trigger_capture()
        rate.sleep()
    self.data_ready = False

  def save_2d_image(self, image, file_path):
    cv2.imwrite(file_path+'_image_color.png', image)

  def save_depth_image(self, depth_image, filepath):
    # cv2.imwrite(filepath+'_depth.exr', depth_image)
    # cv2.imwrite(filepath+'_depth.png', depth_image)
    cv2.imwrite(filepath+'_depth.tif', depth_image)

  def save_camera_info(self, camera_info, filepath):
    self.save_raw(str(camera_info),filepath+"_camera_info.yaml")

  def save_stereo_info(self, stereo_info, filepath):
    self.save_raw(str(stereo_info),filepath+"_stereo_info.yaml")

  def save_xyzrgb(self, xyzrgb, file_path):        
    o3d.io.write_point_cloud(file_path+'_points.ply', xyzrgb)

  def save_raw(self,file, filepath):
    with open(filepath,"w") as f:
      f.write(file)

  def save(self, filepath):
    if self.image is not None:
        self.save_2d_image(self.image, filepath)
    if self.depth_image is not None:
        self.save_depth_image(self.depth_image, filepath)
    if self.camera_info is not None:
        self.save_camera_info(self.camera_info, filepath)
    if self.xyzrgb is not None:
        self.save_xyzrgb(self.xyzrgb, filepath)

  def trigger_capture(self):
    pass

class DepthDataSampler(DataSampler):
  def __init__(self, sensor_name="depth"):
    image_topic = rospy.get_param('~'+sensor_name+'/image_topic', "")

    depth_image_topic = rospy.get_param('~'+sensor_name+'/depth_image_topic', "")
    camera_info_topic = rospy.get_param('~'+sensor_name+'/camera_info_topic', "")
    xyzrgb_topic      = rospy.get_param('~'+sensor_name+'/xyzrgb_topic', "")
    sensor_link       = rospy.get_param('~'+sensor_name+'/sensor_link', "")

    super(DepthDataSampler, self).__init__(
          sensor_name=sensor_name,
          image_topic=image_topic,
          depth_image_topic=depth_image_topic,
          xyzrgb_topic=xyzrgb_topic,
          sensor_link=sensor_link,
          camera_info_topic=camera_info_topic
        )

class ZividDepthDataSampler(DepthDataSampler):
  def __init__(self, sensor_name="zivid_camera"):
    super(ZividDepthDataSampler, self).__init__(
          sensor_name=sensor_name
        )

    ca_suggest_settings_service = "zivid_camera/capture_assistant/suggest_settings"

    rospy.wait_for_service(ca_suggest_settings_service, 30.0)

    self.capture_assistant_service = rospy.ServiceProxy(
        ca_suggest_settings_service, CaptureAssistantSuggestSettings
    )

    self.capture_assistant_suggest_settings()

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

  def trigger_capture(self):
    rospy.loginfo("Calling capture service")
    self.capture_service()

class KeaDepthDataSampler(DepthDataSampler):
  def __init__(self, sensor_name="kea"):
    super(KeaDepthDataSampler, self).__init__(
          sensor_name=sensor_name
        )

    self.capture_service = rospy.ServiceProxy("kea/capture", Empty)

  def trigger_capture(self):
    rospy.loginfo("Calling capture service")
    self.capture_service()

class StereoDataSampler(DataSampler):
  def __init__(self, sensor_name="stereo"):
    image_topic       = rospy.get_param('~'+sensor_name+'/image_topic', "")
    depth_image_topic = rospy.get_param('~'+sensor_name+'/depth_image_topic', "")
    xyzrgb_topic      = rospy.get_param('~'+sensor_name+'/xyzrgb_topic', "")

    camera_info_topic = rospy.get_param('~'+sensor_name+'/camera_info_topic', "")
    stereo_info_topic = rospy.get_param('~'+sensor_name+'/stereo_info_topic', "")
    
    sensor_link       = rospy.get_param('~'+sensor_name+'/sensor_link', "")
    
    super(StereoDataSampler, self).__init__(
          sensor_name=sensor_name,
          image_topic=image_topic,
          depth_image_topic=depth_image_topic,
          xyzrgb_topic=xyzrgb_topic,
          sensor_link=sensor_link,
          camera_info_topic=camera_info_topic,
          stereo_info_topic=stereo_info_topic
        )

    self.image_topics["left"]  = self.format_topic(rospy.get_param('~'+sensor_name+'/left_image_topic', ""))
    self.image_topics["right"] = self.format_topic(rospy.get_param('~'+sensor_name+'/right_image_topic', ""))

    self.left_image  = None
    self.right_image = None
    self.stereo_info = None
      
  def save(self, filepath):
    super(StereoDataSampler, self).save(filepath)

    if self.left_image is not None and self.right_image is not None:
        self.save_2d_image(self.left_image, filepath+"_left")
        self.save_2d_image(self.right_image, filepath+"_right")
    if self.stereo_info is not None:
        self.save_stereo_info(self.stereo_info, filepath)

class DataSamplerFactory(object):
  __metaclass__ = abc.ABCMeta
  def __init__(self):
    pass

  @staticmethod
  def create_datasampler(sensor):
    if "depth" in sensor:
      return DepthDataSampler(sensor_name=sensor)
    elif "stereo" in sensor:
      return StereoDataSampler(sensor_name=sensor)
    elif sensor == 'zivid_camera':
      return ZividDepthDataSampler(sensor_name=sensor)
    elif sensor == 'kea':
      return KeaDepthDataSampler(sensor_name=sensor)
    print("Undefined depth sensor type selected: "+str(sensor))
    return None
