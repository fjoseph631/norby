import pyyolo
import cv2 as cv
import numpy as np
import pathlib
from pathlib import Path
import os
import itertools
import boxmot
from object_tracking import object_tracker
from object_tracking import object_detector
from ament_index_python.packages import get_package_share_directory
import pickle
from object_tracking.object_tracker import Object_Tracker
from tensorflow.keras import models, layers, optimizers, backend
from tensorflow.keras.models import load_model
import random
path = get_package_share_directory('object_tracking')
meta_path = str(path) + '/models/' + 'signs.data'
# general object detector paths
coco_config_path = str(path) + '/configs/' + 'coco-yolov4-tiny.cfg'
coco_meta_path = str(path) + '/models/' + 'coco.data'
coco_weights_path = str(path) + '/weights/' + 'coco-yolov4-tiny.weights'

# pyyolo detectors object
# sign_detector_object = pyyolo.YOLO( config_path, weights_path, meta_path, detection_threshold = .75, hier_threshold = 0.5, nms_threshold = 0.25 )
object_detector_object = pyyolo.YOLO(coco_config_path, coco_weights_path,
                                     coco_meta_path, detection_threshold=0.3, hier_threshold=0.5, nms_threshold=0.25)

# main
# tracker = boxmot.create_tracker(
#     tracking_method="bytetrack",
#     yolo_model="yolov9n.pt",
#     device="cuda"
# )

# tracker.track(source="input_video.mp4", save=True)
# tracker.track()

def track_objects(img, object_tracker):
   # classnames = []
   # with open( (str(path) + '/Datasets/signs.names')) as f:
   #   classnames = f.readlines()
   # create video capture, object tracker & both object detector objects
   default_detector = object_detector.Object_Detector(
      object_detector_object, "General Objects")
   # resize image to calibration size
   detected_car_boxes, detected_cars = default_detector.detect_objects(img)
   
   tracked_objects = object_tracker.track_objects(
      detected_car_boxes, detected_cars, img)

   for tracked_object in object_tracker.registered_objects:
      # only display fully registered tracks
      if not tracked_object.fully_registered:
         continue
      vector = tracked_object.KF.x.flatten()
      x1 = int(tracked_object.x_min)
      y1 = int(tracked_object.y_min)
      x2 = int(tracked_object.x_max)
      y2 = int(tracked_object.y_max)
      # cv.rectangle(img, (x1, y1), (x2, y2), (0, 0, 25), 2)
      cv.circle(img, (int(vector[0]), int(vector[1])), radius=1, color=(
         255, 95, 255), thickness=1)
      cv.putText(img, "Predicted Box", (int(vector[0]), int(
         vector[1])), 0, 0.5, (255, 255, 255), 2)
      ID = tracked_object.id
      cv.putText(img, str(tracked_object.id), (int(vector[0]), int(
         vector[1] + 20)), 0, 0.5, (255, 0, 255), 2)
      cv.putText(img, str(tracked_object.conf), (int(
         vector[0]), int(vector[1] + 40)), 0, 0.5, (255, 0, 0), 2)
   return img
