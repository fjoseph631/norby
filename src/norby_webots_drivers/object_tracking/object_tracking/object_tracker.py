import numpy as np
from filterpy.common import Q_discrete_white_noise
from filterpy.kalman import ExtendedKalmanFilter
from filterpy.kalman import KalmanFilter
from scipy.optimize.nonlin import Jacobian
import cv2 as cv


def linear_assignment(cost_matrix):
   try:
      import lap
      _, x, y = lap.lapjv(cost_matrix, extend_cost=True)
      return np.array([[y[i], i] for i in x if i >= 0])
   except ImportError:
      from scipy.optimize import linear_sum_assignment
      x, y = linear_sum_assignment(cost_matrix)
      return np.array(list(zip(x, y)))

# conversion methods between measurement types


def x_to_box(center_x, center_y, width, height):
   x_delta = (width - center_x) / 2
   y_delta = (height - center_y) / 2
   x1 = center_x - x_delta
   y1 = center_y - y_delta
   x2 = center_x + x_delta
   y2 = center_y + y_delta
   return np.array([x1, y1, x2, y2])


def box_to_x(box):
   center_x = np.median([box.x_min, box.x_max])
   center_y = np.median([box.y_min, box.y_max])
   return np.array([center_x, center_y])


def meas_to_x(box):
   center_x = np.median([box[0], box[2]])
   center_y = np.median([box[1], box[3]])
   return np.array([center_x, center_y])


def iou_batch(bb_test, bb_gt):
   """
   From SORT: Computes IOU between two bboxes in the form [x1,y1,x2,y2]
   """
   bb_gt = np.expand_dims(bb_gt, 0)
   bb_test = np.expand_dims(bb_test, 1)

   xx1 = np.maximum(bb_test[..., 0], bb_gt[..., 0])
   yy1 = np.maximum(bb_test[..., 1], bb_gt[..., 1])
   xx2 = np.minimum(bb_test[..., 2], bb_gt[..., 2])
   yy2 = np.minimum(bb_test[..., 3], bb_gt[..., 3])
   w = np.maximum(0., xx2 - xx1)
   h = np.maximum(0., yy2 - yy1)
   wh = w * h
   #np.nan_to_num(div, copy=False, nan=0.0, posinf=0.0, neginf=0.0)
   o = wh / ((bb_test[..., 2] - bb_test[..., 0]) * (bb_test[..., 3] - bb_test[..., 1])
            + (bb_gt[..., 2] - bb_gt[..., 0]) * (bb_gt[..., 3] - bb_gt[..., 1]) - wh)
   return(o)


class Tracked_Object():
   def __init__(self, dt, detected_object, id):
      self.dt = dt
      self.x_min = detected_object.x_min
      self.y_min = detected_object.y_min
      self.x_max = detected_object.x_max
      self.y_max = detected_object.y_max
      self.width = self.x_max - self.x_min
      self.height = self.y_max - self.y_min
      self.histogram = detected_object.histogram
      self.history = []
      self.centroid_x = np.median(
         [detected_object.x_min, detected_object.x_max])
      self.centroid_y = np.median(
         [detected_object.y_min, detected_object.y_max])
      self.last_hit_time = 0
      self.fully_registered = False
      self.deregister = False
      self.hit_streak = 1
      self.init_kf(self.dt)
      self.id = id
      self.conf = detected_object.confidence

   def init_kf(self, dt):
      self.KF = KalmanFilter(dim_x=4, dim_z=2, dim_u=2)
      self.dt = dt
      # init with position of centroid
      self.KF.x = np.array([[self.centroid_x],
                           [self.centroid_y],
                           [0.],
                           [0.]])
      self.KF.F = np.array([[1., 0., self.dt, 0.],
                           [0., 1., 0., self.dt],
                           [0., 0., 1., 0],
                           [0., 0., 0., 1.]])
      self.KF.R = np.eye(self.KF.dim_z)
      self.KF.P = np.array([[1000., 0., self.dt * 100000, 0.],
                           [0., 1000., 0., self.dt * 100000.0],
                           [0., 0., 100000., 0.],
                           [0., 0., 0., 100000.]])

      self.KF.H = np.array([[1., 0., 0., 0.],
                           [0., 1., 0., 0.]])
      self.KF.B = np.array([[1 * (self.dt ** 2)/2, 0.],
                           [0, 1 * (self.dt**2)/2],
                           [1 * self.dt, 0],
                           [0, 1 * self.dt]]),
      self.KF.Q = Q_discrete_white_noise(dim=4, dt=self.dt, var=self.dt)


# object tracker object
"""methods
"""
"""
variables
"""


class Object_Tracker():
   def __init__(self, dt):
      self.next_id = 1
      self.dt = dt
      self.max_unmeasured_time = self.dt * 10
      self.registered_objects = []
      self.iou_min_threshold = 0.15
      # min number of measuremnts to consider track as legitimate
      self.min_hit_streak = 3

   def predict_new_locations(self):
      # predict new locations for each tracked objects
      for track in self.registered_objects:
         track.KF.predict()

   def associate_detections_to_tracks(self, detected_objects, tracked_objects, min_threshold, img):
      empty = []
      if (len(tracked_objects) == 0):
         return np.empty((0, 2), dtype=int), (detected_objects), empty
      tracks = self.convert_tracks_to_array()
      dets = self.convert_dets_to_array(detected_objects)
      iou_matrix = iou_batch(dets, tracks)
      hist_matrix = np.zeros(
         (len(detected_objects), len(tracked_objects)), dtype=float)

      a = len(tracked_objects)
      b = len(detected_objects)
      if (a > 0 and b > 0):
         for i in range(b):
               for j in range(a):
                  hist_matrix[i][j] = cv.compareHist(
                     detected_objects[i].histogram, tracked_objects[j].histogram, cv.HISTCMP_CORREL)

      #np.nan_to_num(hist_matrix, copy=False, nan=0.0, posinf=1, neginf=None)
      #iou_matrix_n = np.nan_to_num(iou_matrix, copy=True, nan=0.0, posinf=0.0, neginf=0.0)
      matched_indices2 = np.empty(shape=(0, 2))

      combined_matrix = hist_matrix + iou_matrix
      if min(iou_matrix.shape) > 0:
         a = (hist_matrix > self.iou_min_threshold).astype(np.int32)
         if a.sum(1).max() == 1 and a.sum(0).max() == 1:
               matched_indices = np.stack(np.where(a), axis=1)
         else:
               #matched_indices = linear_assignment(-hist_matrix)
               #matched_indices2 = linear_assignment(-iou_matrix_n)
               matched_indices = linear_assignment(-combined_matrix)
      else:
         matched_indices = np.empty(shape=(0, 2))

      matches = []
      for m in matched_indices:
         if(iou_matrix[m[0], m[1]] < min_threshold):
               np.delete(matched_indices, m)
         else:
               matches.append(m.reshape(1, 2))

      # if unmatched, create new object track
      unmatched_detections = []
      for d, det in enumerate(detected_objects):
         if (d not in matched_indices[:, 0]):
               unmatched_detections.append(det)
      unmatched_tracks = []
      for t, trk in enumerate(tracked_objects):
         if (t not in matched_indices[:, 1]):
               unmatched_tracks.append(trk)

      if(len(matches) == 0):
         matches = np.empty((0, 2), dtype=int)
      else:
         matches = np.concatenate(matches, axis=0)
      return matches, (unmatched_detections), (unmatched_tracks)

   def convert_tracks_to_array(self):
      if(len(self.registered_objects) == 0):
         return np.empty((0, 5), dtype=int)

      tracks = np.zeros((len(self.registered_objects), 5))
      for t, trk in enumerate(tracks):
         self.registered_objects[t].KF.predict()
         trk[:] = [self.registered_objects[t].x_min, self.registered_objects[t].y_min, self.registered_objects[t].x_max,
                     self.registered_objects[t].y_max, 0]
      return tracks

   def convert_dets_to_array(self, det_list):
      if(len(det_list) == 0):
         return np.empty((0, 5), dtype=int)
      dets = np.zeros((len(det_list), 5), dtype=int)
      for t, det in enumerate(det_list):
         dets[t] = [det.x_min, det.y_min,
                     det.x_max, det.y_max, det.confidence]
      return dets

   def track_objects(self, detected_object_boxes, detected_object_list, img):
      matched_indices, unmatched_detections, unmatched_tracks = self.associate_detections_to_tracks(
         detected_object_list, self.registered_objects, self.iou_min_threshold, img)
      if(len(matched_indices) == 0):
         unmatched_detections = detected_object_list
         unmatched_tracks = self.registered_objects

      for match in matched_indices:
         index1 = match[0]
         index2 = match[1]
         box = np.array([detected_object_boxes[index1][0], detected_object_boxes[index1][1], detected_object_boxes[index1][2],
                           detected_object_boxes[index1][3]])
         # sign = self.registered_objects[index2]
         # x11 = sign.x_min
         # x21 = sign.x_max
         # y11 = sign.y_min
         # y21 = sign.y_max
         #cv.rectangle(img, (int(x11), int(y11)), (int(x21), int(y21)), (0, 0, 0), 2)
         self.registered_objects[index2].KF.update(meas_to_x(box))
         self.registered_objects[index2].last_hit_time = 0
         self.registered_objects[index2].hit_streak += 1
         #self.registered_objects[index2].histogram = detected_object_list[index1].histogram
         if self.registered_objects[index2].hit_streak >= self.min_hit_streak:
               self.registered_objects[index2].fully_registered = True
         x1 = box[0]
         x2 = box[2]
         y1 = box[1]
         y2 = box[3]
         # x11 = sign.x_min
         # x21 = sign.x_max
         # y11 = sign.y_min
         # y21 = sign.y_max

         #cv.rectangle(img, (int(x1), int(y1)), (int(x2), int(y2)), (120, 120, 120), 2)
         #cv.rectangle(img, (int(x11), int(y11)), (int(x21), int(y21)), (150, 150, 150), 2)
         # cv.putText(img, "Detected Object " + str(index1) + "Associated to Registered Object " + str(index2) ,
         # ((int(x1) + 25),(int(y1) - 25)), 1, 1, (255, 255, 255), 2)
         #self.registered_objects[index2].histogram = detected_object_list[index1].histogram
         self.update_track(
               self.registered_objects[index2], detected_object_list[index1])

      for trk in unmatched_tracks:
         new_track = None
         for t, trks in enumerate(self.registered_objects):
               if (trk.id == trks.id):
                  new_track = trks
                  break
         index1 = self.registered_objects.index(new_track)
         self.registered_objects[index1].last_hit_time += self.dt
         self.registered_objects[index1].hit_streak = 0
         if(self.registered_objects[index1].last_hit_time >= self.max_unmeasured_time):
               self.deregister(self.registered_objects[index1])
         if(self.registered_objects[index1].fully_registered == False):
               self.deregister(self.registered_objects[index1])

         x1 = new_track.x_min
         y1 = new_track.y_min
         x2 = new_track.x_max
         y2 = new_track.y_max
         #cv.rectangle(img, (int(x1), int(y1)), (int(x2), int(y2)), (255, 0, 255), 2)
         #cv.putText(img, "Unmatched Track " + str(new_track.id),  ((int(x1) + 25),(int(y1) -25)), 1, 1, (255, 255, 255), 2)

         self.update_track(self.registered_objects[index1], None)

      for obj in (unmatched_detections):
         index1 = unmatched_detections.index(obj)
         x1 = obj.x_min
         x2 = obj.x_max
         y1 = obj.y_min
         y2 = obj.y_max
         #cv.putText(img, "Unmatched Object " + str(index1),  ((int(x1) + 25),(int(y1) - 25)), 1, 1, (255, 255, 255), 2)
         #cv.rectangle(img, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 255), 2)
         self.register(obj)

      # removed all outdated tracks
      self.remove_tracks()
      return self.registered_objects

   # actually removes objects
   def remove_tracks(self):
      for track in self.registered_objects:
         if(track.deregister == True):
               self.registered_objects.pop(
                  self.registered_objects.index(track))
      #self.registered_objects = [track for track in self.registered_objects if track.deregister == False]

   def update_track(self, obj, det):
      obj.history.append(
         np.array([obj.x_min, obj.y_min, obj.x_max, obj.y_max]))
      vec = obj.KF.x.flatten()
      x_delta = (obj.width) / 2
      y_delta = (obj.height) / 2
      #print(obj.x_min, vec[0] - x_delta )
      obj.x_min = vec[0] - x_delta
      obj.x_max = vec[0] + x_delta
      obj.y_min = vec[1] - y_delta
      obj.y_max = vec[1] + y_delta
      obj.centroid_x = vec[0]
      obj.centroid_y = vec[1]
      obj.width = obj.x_max - obj.x_min
      obj.height = obj.y_max - obj.y_min
      if (det != None):
         obj.histogram = det.histogram

    # registers new object
   def register(self, new_object):
      new_track = Tracked_Object(self.dt, new_object, self.next_id)
      new_object.id = self.next_id
      self.registered_objects.append(new_track)
      self.next_id += 1
      # print(new_track)

   # logically removes object
   def deregister(self, old_object):
      index = self.registered_objects.index(old_object)
      self.registered_objects[index].deregister = True
