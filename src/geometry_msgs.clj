(ns geometry-msgs
  (:require [std-msgs]
            [rosclj.msg :refer :all]
            [incanter.core :refer [matrix identity-matrix to-vect]]))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Point
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defrecord Point [x y z])

(defmethod from-ros "geometry_msgs/Point" [pt]
  (bean-clean pt))

(with-msg-factory [factory "geometry_msgs/Point"
                   (str "float64 x\n"
                        "float64 y\n"
                        "float64 z\n")]
  (defmethod to-ros :geometry_msgs.Point [pt]
    (doto (.newFromType factory "geometry_msgs/Point")
      (.setX (:x pt))
      (.setY (:y pt))
      (.setZ (:z pt)))))

(defn point [& {:keys [x y z] :or {x 0 y 0 z 0}}]
  (->Point x y z))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Quaternion
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defrecord Quaternion [x y z w])

(defmethod from-ros "geometry_msgs/Quaternion" [quat]
  (bean-clean quat))

(with-msg-factory [factory "geometry_msgs/Quaternion"
                   (str "float64 x\n"
                        "float64 y\n"
                        "float64 z\n"
                        "float64 w\n")]
  (defmethod to-ros :geometry_msgs.Quaternion [quat]
    (doto (.newFromType factory "geometry_msgs/Quaternion")
      (.setX (:x quat))
      (.setY (:y quat))
      (.setZ (:z quat))
      (.setW (:w quat)))))

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


(with-msg-factory [f "geometry_msgs/Pose"
                   (str "Point position\n"
                        "Quaternion orientation\n")]
  (defmethod to-ros :geometry_msgs.Pose [pose]
    (doto (.newFromType f "geometry_msgs/Pose")
      (.setPosition (to-ros (:position pose)))
      (.setOrientation (to-ros (:orientation pose))))))

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

(with-msg-factory [f "geometry_msgs/PoseStamped" "Header header\nPose pose\n"]
  (defmethod to-ros :geometry_msgs.PoseStamped [pose]
    (doto (.newFromType f "geometry_msgs/PoseStamped")
      (.setHeader (to-ros (:header pose)))
      (.setPose (to-ros (:pose pose))))))

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

(with-msg-factory [f "geometry_msgs/PoseWithCovariance"
                   (str "Pose pose\n"
                        "float64[36] covariance")]
  (defmethod to-ros :geometry_msgs.PoseWithCovariance [pose]
    (doto (.newFromType f "geometry_msgs/PoseWithCovariance")
      (.setPose (to-ros (:pose pose)))
      (.setCovariance
       (double-array (flatten (to-vect (:covariance pose))))))))

(defn pose-with-covariance [& {:keys [pose covariance]
                               :or {pose (pose) covariance (identity-matrix 6)}}]
  (->PoseWithCovariance pose covariance))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; PoseWithCovarianceStamped
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defrecord PoseWithCovarianceStamped [header pose])

(defmethod from-ros "geometry_msgs/PoseWithCovarianceStamped" [pose]
  (-> (bean-clean pose)
      (update-in [:header] from-ros)
      (update-in [:pose] from-ros)))

(with-msg-factory [f "geometry_msgs/PoseWithCovarianceStamped"
                   (str "Header header\n"
                        "PoseWithCovariance pose")]
  (defmethod to-ros :geometry_msgs.PoseWithCovarianceStamped [pose]
    (doto (.newFromType f "geometry_msgs/PoseWithCovarianceStamped")
      (.setHeader (to-ros (:header pose)))
      (.setPose (to-ros (:pose pose))))))

(defn pose-with-covariance-stamped [& {:keys [header pose-cov]
                                       :or {header (std-msgs/header)
                                            pose-cov (pose-with-covariance)}}]
  (->PoseWithCovarianceStamped header pose-cov))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Vector3
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defrecord Vector3 [x y z])

(defmethod from-ros "geometry_msgs/Vector3" [vec3]
  (bean-clean vec3))

(with-msg-factory [f "geometry_msgs/Vector3"
                   "float64 x\nfloat64 y\nfloat64 z"]
  (defmethod to-ros :geometry_msgs.Vector3 [vec3]
    (doto (.newFromType f "geometry_msgs/Vector3")
      (.setX (:x vec3))
      (.setY (:y vec3))
      (.setZ (:z vec3)))))

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

(with-msg-factory [f "geometry_msgs/Transform"
                   "Vector3 translation\nQuaternion rotation"]
  (defmethod to-ros :geometry_msgs.Transform [transform]
    (doto (.newFromType f "geometry_msgs/Transform")
      (.setTranslation (to-ros (:translation transform)))
      (.setRotation (to-ros (:rotation transform))))))

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

(with-msg-factory [f "geometry_msgs/TransformStamped"
                   (str "Header header\n"
                        "string child_frame_id\n"
                        "Transform transform\n")]
  (defmethod to-ros :geometry_msgs.TransformStamped [transform]
    (doto (.newFromType f "geometry_msgs/TransformStamped")
      (.setHeader (to-ros (:header transform)))
      (.setChildFrameId (:child-frame-id transform))
      (.setTransform (to-ros (:transform transform))))))

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


(with-msg-factory [f "geometry_msgs/PoseArray"
                   (str "Header header\n"
                        "Pose[] poses")]
  (defmethod to-ros :geometry_msgs.PoseArray [posearr]
    (doto (.newFromType f "geometry_msgs/PoseArray")
      (.setHeader (to-ros (:header posearr)))
      (.setPoses (map to-ros (:poses posearr))))))

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

(with-msg-factory [f "geometry_msgs/Twist"
                   (str "Vector3  linear\n"
                        "Vector3  angular")]
  (defmethod to-ros :geometry_msgs.Twist [twist]
    (doto (.newFromType f "geometry_msgs/Twist")
      (.setLinear (-> twist :linear to-ros))
      (.setAngular (-> twist :angular to-ros)))))

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

(with-msg-factory [f "geometry_msgs/TwistWithCovariance"
                   (str "Twist twist\n"
                        "float64[36] covariance\n")]
  (defmethod to-ros :geometry_msgs.TwistWithCovariance [twist]
    (doto (.newFromType f "geometry_msgs/TwistWithCovariance")
      (.setTwist (-> twist :twist to-ros))
      (.setCovariance (-> twist :covariance to-vect flatten double-array)))))

(defn twist-with-covariance [& {:keys [twist covariance]
                                :or {twist (twist) covariance (identity-matrix 6)}}]
  (->TwistWithCovariance twist covariance))
