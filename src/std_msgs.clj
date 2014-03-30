(ns std-msgs
  (:refer-clojure :exclude [java.lang.String])
  (:require [rosclj.msg :refer :all]))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Bool
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defrecord Bool [data])

(defmethod from-ros "std_msgs/Bool" [bool]
  (bean-clean bool))

(with-msg-factory [factory "std_msgs/Bool" "bool data"]
  (defmethod to-ros :std_msgs.Bool [bool]
    (doto (.newFromType factory "std_msgs/Bool")
      (.setData (:data bool)))))

(defn bool [& {:keys [data] :or {data false}}]
  (->Bool data))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Duration
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defrecord Duration [secs nsecs])

(defmethod from-ros org.ros.message.Duration [duration]
  {:secs (.secs duration) :nsecs (.nsecs duration)})

(with-msg-factory [factory "std_msgs/Duration" "duration data"]
  (defmethod to-ros :std_msgs.Duration [duration]
    (doto (.newFromType factory "std_msgs/Duration")
      (.setData (org.ros.message.Duration. (:secs duration) (:nsecs duration))))))

(defn duration [& {:keys [secs nsecs] :or {secs 0 nsecs 0}}]
  (->Duration secs nsecs))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Time
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defrecord Time [secs nsecs])

(defmethod from-ros 'org.ros.message.Time [duration]
  {:secs (.secs duration) :nsecs (.nsecs duration)})

(with-msg-factory [factory "std_msgs/Time" "time data"]
  (defmethod to-ros :std_msgs.Time [time]
    (doto (.newFromType factory "std_msgs/Time")
      (.setData (org.ros.message.Time. (:secs time) (:nsecs time))))))

(defn time [& {:keys [secs nsecs] :or {secs 0 nsecs 0}}]
  (->Time secs nsecs))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Header
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defrecord Header [seq stamp frame-id])

(defmethod from-ros "std_msgs/Header" [header]
  (map->Header
   (-> (bean-clean header)
       (update-in [:stamp] from-ros))))

(with-msg-factory [factory
                   "std_msgs/Header"
                   "# Standard metadata for higher-level stamped data types.\n# This is generally used to communicate timestamped data \n# in a particular coordinate frame.\n# \n# sequence ID: consecutively increasing ID \nuint32 seq\n#Two-integer timestamp that is expressed as:\n# * stamp.secs: seconds (stamp_secs) since epoch\n# * stamp.nsecs: nanoseconds since stamp_secs\n# time-handling sugar is provided by the client library\ntime stamp\n#Frame this data is associated with\n# 0: no frame\n# 1: global frame\nstring frame_id\n"]
  (defmethod to-ros :std_msgs.Header [header]
    (doto (.newFromType factory "std_msgs/Header")
      (.setFrameId (:frame-id header))
      (.setSeq (:seq header))
      (.setStamp (org.ros.message.Time. (-> header :stamp :secs)
                                        (-> header :stamp :nsecs))))))

(defn header [& {:keys [seq stamp frame-id] :or {seq 0 stamp (time) frame-id ""}}]
  (->Header seq stamp frame-id))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; String
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defmethod from-ros "std_msgs/String" [str]
  (:data (bean-clean str)))


(with-msg-factory [factory "std_msgs/String" "string data"]
  (defmethod to-ros :java.lang.String [str]
    (doto (.newFromType factory "std_msgs/String")
      (.setData str))))



