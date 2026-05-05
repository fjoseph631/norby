# ROS Client Library for Python
from numpy.linalg.linalg import det
import rclpy
import numpy as np
from numpy import asarray
from filterpy.kalman import KalmanFilter
from scipy.optimize.nonlin import Jacobian
import numpy as np
from filterpy.common import Q_discrete_white_noise

# Handles the creation of nodes


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


def linear_assignment(cost_matrix):
    try:
        import lap
        _, x, y = lap.lapjv(cost_matrix, extend_cost=True)
        return np.array([[y[i], i] for i in x if i >= 0])
    except ImportError:
        from scipy.optimize import linear_sum_assignment
        x, y = linear_sum_assignment(cost_matrix)
        return np.array(list(zip(x, y)))


# object tracker object
"""methods
"""
"""
variables
"""


class Radar_Object_Tracker():
    def __init__(self, dt):
        self.next_id = 1
        self.dt = dt
        self.max_unmeasured_time = self.dt * 20
        self.registered_objects = []
        self.iou_min_threshold = 0.10
        # min number of measuremnts to consider track as legitimate
        self.min_hit_streak = 3

    def predict_new_locations(self):
        # predict new locations for each tracked objects
        for track in self.registered_objects:
            track.KF.predict()

    def associate_detections_to_tracks(self, detected_objects, tracked_objects, min_threshold):
        empty = []
        if (len(tracked_objects) == 0):
            return np.empty((0, 5), dtype=int), (detected_objects), empty
        tracks = self.convert_tracks_to_array()
        dets = self.convert_dets_to_array(detected_objects)
        iou_matrix = iou_batch(dets, tracks)
        #np.nan_to_num(hist_matrix, copy=False, nan=0.0, posinf=1, neginf=None)
        #iou_matrix_n = np.nan_to_num(iou_matrix, copy=True, nan=0.0, posinf=0.0, neginf=0.0)

        if min(iou_matrix.shape) > 0:
            a = (iou_matrix > self.iou_min_threshold).astype(np.int32)
            if a.sum(1).max() == 1 and a.sum(0).max() == 1:
                matched_indices = np.stack(np.where(a), axis=1)
            else:
                matched_indices = linear_assignment(-iou_matrix)
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

        tracks = np.zeros((len(self.registered_objects), 4), dtype=float)
        for t, trk in enumerate(tracks):
            self.registered_objects[t].KF.predict()
            trk[:] = [self.registered_objects[t].centroid_x - self.registered_objects[t].width,
                      self.registered_objects[t].centroid_y -
                      self.registered_objects[t].height,
                      self.registered_objects[t].centroid_x +
                      self.registered_objects[t].width,
                      self.registered_objects[t].centroid_y +
                      self.registered_objects[t].height
                      ]
        return tracks

    def convert_dets_to_array(self, det_list):
        if(len(det_list) == 0):
            return np.empty((0, 4), dtype=int)
        dets = np.zeros((len(det_list), 4), dtype=float)
        for t, det in enumerate(det_list):
            dets[t] = [det[0], det[1], det[2], det[3]]
        return dets

    def track_objects(self, detected_object_list, tracked_object_list):
        matched_indices, unmatched_detections, unmatched_tracks = self.associate_detections_to_tracks(
            detected_object_list, self.registered_objects, self.iou_min_threshold)
        if(len(matched_indices) == 0):
            unmatched_detections = detected_object_list
            unmatched_tracks = self.registered_objects
        for match in matched_indices:
            index1 = match[0]
            index2 = match[1]
            box = np.array([detected_object_list[index1][0],
                           detected_object_list[index1][1]])

            self.registered_objects[index2].KF.update((box))
            self.registered_objects[index2].last_hit_time = 0
            self.registered_objects[index2].hit_streak += 1
            #self.registered_objects[index2].histogram = detected_object_list[index1].histogram
            if self.registered_objects[index2].hit_streak >= self.min_hit_streak:
                self.registered_objects[index2].fully_registered = True
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

            #cv.rectangle(img, (int(x1), int(y1)), (int(x2), int(y2)), (255, 0, 255), 2)
            #cv.putText(img, "Unmatched Track " + str(new_track.id),  ((int(x1) + 25),(int(y1) -25)), 1, 1, (255, 255, 255), 2)

            self.update_track(self.registered_objects[index1], None)

        for obj in (unmatched_detections):
            index1 = unmatched_detections.index(obj)
            self.register(obj)

        # removed all outdated tracks
        self.remove_tracks()
        
        return [ track for track in self.registered_objects if track.fully_registered == True]

    # actually removes objects
    def remove_tracks(self):
        for track in self.registered_objects:
            if(track.deregister == True):
                self.registered_objects.pop(
                    self.registered_objects.index(track))

    def update_track(self, obj, det):
        obj.history.append(np.array([obj.centroid_x, obj.centroid_y]))
        vec = obj.KF.x.flatten()
        obj.centroid_x = vec[0]
        obj.centroid_y = vec[1]

    # registers new object
    def register(self, new_object):
        new_track = Tracked_Object(self.dt, new_object, self.next_id)
        new_track.id = self.next_id
        self.registered_objects.append(new_track)
        self.next_id += 1
        # print(new_track)

    # logically removes object
    def deregister(self, old_object):
        index = self.registered_objects.index(old_object)
        self.registered_objects[index].deregister = True


class Tracked_Object():
    def __init__(self, dt, detected_object, id):
        self.dt = dt
        self.history = []
        self.centroid_x = np.median([detected_object[0], detected_object[2]])
        self.centroid_y = np.median([detected_object[1], detected_object[3]])
        self.width = detected_object[2] - detected_object[0]
        self.height = detected_object[3] - detected_object[1]
        self.vx = detected_object[4]
        self.vy = detected_object[5]
        self.last_hit_time = 0
        self.fully_registered = False
        self.deregister = False
        self.hit_streak = 1
        self.init_kf(self.dt)
        self.id = id

    def init_kf(self, dt):
        # get from xml
        self.KF = KalmanFilter(dim_x=4, dim_z=2, dim_u=2)
        self.dt = dt
        # init with position of centroid
        self.KF.x = np.array([[self.centroid_x],
                              [self.centroid_y],
                              [self.vx],
                              [self.vy]])
        self.KF.F = np.array([[1., 0., self.dt, 0.],
                              [0., 1., 0., self.dt],
                              [0., 0., 1., 0],
                              [0., 0., 0., 1.]])
        self.KF.R = np.eye(self. KF.dim_z)
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
