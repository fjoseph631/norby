import numpy as np
import cv2 as cv

# object detector class

""" methods
    detect object (frame)
"""

""" variables
    name
    detector object
"""


class Object_Detector:
   # init object
   def __init__(self, pyyolo_detector_object, name):
      self.detector_object = pyyolo_detector_object
      self.detected_objects = []
      self.name = name
    # detected objects
    # returns objects as numpy arrays and corresponding custom objects

   def detect_objects(self, frame):
      res = self.detector_object.detect(frame, rgb=False)
      self.detected_objects = []
      detected_objects = np.empty((len(res), 5), dtype=int)
      for i, det in enumerate(res):
         # add to detected object array
         xmin, ymin, xmax, ymax = det.to_xyxy()
         # cv.rectangle(frame, (xmin, ymin), (xmax, ymax), (0, 0, 0))
         org = ((xmin - 5), (ymax + 5))
         font = cv.FONT_HERSHEY_SIMPLEX
         fontScale = 1
         color = (255, 0, 0)
         thickness = 2
         # in place of deepsort's featrue vector, using a histogram of the image within the bounding box
         image = np.uint64(frame)
         ROI = image[ymin:ymax, xmin:xmax]
         # cv.putText(frame,det.name,org,font,fontScale,color,thickness, cv.LINE_AA)
         hist_full = cv.calcHist([ROI], [0], None, [256], [0, 256])
         d1 = Detected_Object(det, xmin, ymin, xmax,
                              ymax, hist_full, det.prob)
         self.detected_objects.append(d1)
         detected_objects[i] = np.array([xmin, ymin, xmax, ymax, det.prob])
      return detected_objects, self.detected_objects


# detected object class
"""
"""

"""
traits
    xmin
    ymin
    xmax
    ymax
    histogram
    confidence
"""


class Detected_Object:
   def __init__(self, name, x_min, y_min, x_max, y_max, hist, conf):
      self.x_min = x_min
      self.y_min = y_min
      self.x_max = x_max
      self.y_max = y_max
      self.name = name
      self.histogram = hist
      self.confidence = conf
