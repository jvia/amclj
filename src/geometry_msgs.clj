(ns geometry-msgs
  (:require [std-msgs]
            [rosclj.msg :refer :all]
            [incanter.core :refer [matrix identity-matrix]]))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Point
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defrecord Point [x y z])

(defmethod from-ros "geometry_msgs/Point" [pt]
  (bean-clean pt))

(defn point [& {:keys [x y z] :or {x 0 y 0 z 0}}]
  (->Point x y z))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Quaternion
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defrecord Quaternion [x y z w])

(defmethod from-ros "geometry_msgs/Quaternion" [quat]
  (bean-clean quat))

(defn quaternion [& {:keys [x y z w]
                     :or {x 0 y 0 z 0 w 1}}]
  (->Quaternion x y z w))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Pose
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defrecord Pose [position orientation])

(defmethod from-ros "geometry_msgs/Pose" [pose]
  (-> (bean-clean pose)
      (update-in [:position] from-ros)
      (update-in [:orientation] from-ros)))

(defn pose [& {:keys [position orientation]
               :or {position (point) orientation (quaternion)}}]
  (->Pose position orientation))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; PoseStamped
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defrecord PoseStamped [header pose])

(defmethod from-ros "geometry_msgs/PoseStamped" [pose]
  (-> (bean-clean pose)
      (update-in [:header] from-ros)
      (update-in [:pose] from-ros)))

(defn pose-stamped [& {:keys [header pose]
                       :or {header (std-msgs/header) pose (pose)}}]
  (->PoseStamped header pose))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Pose2D
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defrecord Pose2D [x y theta])

(defmethod from-ros "geometry_msgs/Pose2D" [pose]
  (bean-clean pose))

(defn pose2d [& {:keys [x y theta] :or {x 0 y 0 theta 0}}]
  (->Pose2D x y theta))


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; PoseWithCovariance
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defrecord PoseWithCovariance [pose covariance])

(defmethod from-ros "geometry_msgs/PoseWithCovariance" [pose]
  (-> (bean-clean pose)
      (update-in [:pose] from-ros)
      (update-in [:covariance] #(-> % vec (matrix 6)))))

(defn pose-with-covariance [& {:keys [pose covariance]
                               :or {pose (pose) covariance (identity-matrix 6)}}]
  (->PoseWithCovariance pose covariance))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; PoseWithCovarianceStamped
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defrecord PoseWithCovarianceStamped [header pose covariance])

(defmethod from-ros "geometry_msgs/PoseWithCovarianceStamped" [pose]
  (-> (bean-clean pose)
      (update-in [:header] from-ros)
      (update-in [:pose] from-ros)))

(defn pose-with-covariance-stamped [& {:keys [header pose-cov]
                                       :or {header (std-msgs/header)
                                            pose-cov (pose-with-covariance)}}]
  (->PoseWithCovariance header pose-cov))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Vector3
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defrecord Vector3 [x y z])

(defmethod from-ros "geometry_msgs/Vector3" [vec3]
  (bean-clean vec3))

(defn vector3 [& {:keys [x y z] :or {x 0 y 0 z 0 }}]
  (->Vector3 x y z))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Transform
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defrecord Transform [translation rotation])

(defmethod from-ros "geometry_msgs/Transform" [transform]
  (-> (bean-clean transform)
      (update-in [:translation] from-ros)
      (update-in [:rotation] from-ros)))

(defn transform [& {:keys [translation rotation]
                    :or {translation (vector3) rotation (quaternion)}}]
  (->Transform translation rotation))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; TransformStamped
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defrecord TransformStamped [header child-frame-id transform])

(defmethod from-ros "geometry_msgs/TransformStamped" [transform]
  (-> (bean-clean transform)
      (update-in [:header] from-ros)
      (update-in [:transform] from-ros)))

(defn transform-stamped [& {:keys [header child-frame-id transform]
                            :or {header (std-msgs/header)
                                 child-frame-id ""
                                 transform (transform)}}]
  (->TransformStamped header child-frame-id transform))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; PoseArray
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defrecord PoseArray [header poses])

(defmethod from-ros "geometry_msgs/PoseArray" [posearr]
  (-> (bean-clean posearr)
      (update-in [:header] from-ros)
      (update-in [:poses] #(map from-ros (vec %)))))

(defn pose-array [& {:keys [header poses] :or {header (std-msgs/header) poses []}}]
  (->PoseArray header poses))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Twist
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defrecord Twist [linear angular])

(defmethod from-ros "geometry_msgs/Twist" [twist]
  (-> (bean-clean twist)
      (update-in [:linear] from-ros)
      (update-in [:angular] from-ros)))

(defn twist [& {:keys [linear angular] :or {linear (vector3) angular (vector3)}}]
  (->Twist linear angular))


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; TwistWithCovariance
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defrecord TwistWithCovariance [twist covariance])

(defmethod from-ros "geometry_msgs/TwistWithCovariance" [twist]
  (-> (bean-clean twist)
      (update-in [:twist] from-ros)
      (update-in [:covariance] #(-> % vec (matrix 6)))))

(defn twist-with-covariance [& {:keys [twist covariance]
                               :or {twist (twist) covariance (identity-matrix 6)}}]
  (->TwistWithCovariance twist covariance))
