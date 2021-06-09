from os import path
import os
import re

from natsort.natsort import natsorted
from . import utils
from .container import split_list


def find_files_matching(input_path, regex):
  frames = {}
  for filename in os.listdir(input_path):
    match = re.match(regex, filename)
    if match is not None:
      frames[match.group('frame')] = path.join(input_path, filename)

  return {k:frames[k] for k in natsorted(frames)}


def check_keys(dicts):
  fields = list(dicts.keys())
  keys_ref = set(dicts[fields[0]].keys())
  for k in fields[1:]:
    k1 = set(dicts[k].keys())
    assert k1 == keys_ref,\
      f"mismatch between {k} and {keys_ref}: {k1.symmetric_difference(keys_ref)}"
  return keys_ref

def find_scan_files(file_path, file_types):
  files = {k:find_files_matching(file_path, f"(?P<frame>\d+)_{pattern}")
        for k, pattern in file_types.items()}

  image_names = check_keys(files)
  files = {k:list(f.values()) for k, f in files.items()}
  files['image_names'] = image_names
  return files

def find_stereo_scan(file_path):
   return find_scan_files(file_path, 
    dict(left="left_rgb.png", right="right_rgb.png", transforms="transforms.yaml"))



def load_stereo_scan(scan):
  images = utils.load_images(scan["left"] + scan["right"])
  left, right = split_list(images, len(scan['left']))

  n = len(images)
  return dict(
    left = left, 
    right = right, 
    transforms   = utils.load_transforms(scan['transforms'])
  )