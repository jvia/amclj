(ns amclj.msg)

#_(defprotocol IConvert
    "Conversion beteen ROS types and Clojure types"
    (rosfrom [rosmsg])
    (rosto   [cljmsg]))

(defn- bean-clean
  "Convert message to a bean and remove useless information"
  [msg]
  (dissoc (bean msg) :class :instance))

(defn from-ros-dispatch
  "rosjava message dynamically implement the message interface using a
  MessageImpl backing class. We need to do some massaging to get out
  some data on which we can dispatch."
  [msg]
  (.getType (.getInstance msg)))

;; TODO: automatically generate this based on messages on system
(defmulti from-ros from-ros-dispatch)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Geometry messages
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defmethod from-ros "geometry_msgs/Point" [pt]
  (dissoc (bean pt) :class :instance))

(defmethod from-ros "geometry_msgs/Quaternion" [quat]
  (dissoc (bean quat) :class :instance))

(defmethod from-ros "geometry_msgs/Pose" [pose]
  {:position    (from-ros (.getPosition pose))
   :orientation (from-ros (.getOrientation pose))})

(defmethod from-ros "geometry_msgs/PoseWithCovariance" [pose]
  (let [pose (bean-clean pose)]
    (-> pose
        (update-in [:pose] from-ros)
        (update-in [:covariance] vec))))

(defmethod from-ros "geometry_msgs/PoseWithCovarianceStamped" [pose]
  (let [pose (bean-clean pose)]
    (-> pose
        (update-in [:header] from-ros)
        (update-in [:pose] from-ros))))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; nav_msgs
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defmethod from-ros "nav_msgs/OccupancyGrid" [map]
  (let [map (bean-clean map)]
    (-> map
        (update-in [:header] from-ros)
        (update-in [:info] from-ros)
        (update-in [:data] #(let [raw-data % #_(.getData %)
                                  size (.readableBytes raw-data)
                                  array (byte-array size)
                                  _ (.readBytes raw-data array)]
                              array
                              #_(vec array))))))

(defmethod from-ros "nav_msgs/MapMetaData" [metadata]
  (-> (bean-clean metadata)
      (update-in [:origin] from-ros)))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Sensor Messages
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defmethod from-ros "sensor_msgs/LaserScan" [scan]
  (let [msg (bean-clean scan)]
    (-> msg
        (update-in [:intensities] vec)
        (update-in [:ranges] vec))))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Standard Messages
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defmethod from-ros "std_msgs/Header" [hdr]
  (bean-clean hdr))


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; tf Messages
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defmethod from-ros "tf/tfMessage" [tf]
  (bean-clean tf))

