(ns std-msgs
  (:refer-clojure :exclude [java.lang.String])
  (:require [rosclj.msg :refer :all]))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Bool
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defrecord Bool [data])

(defmethod from-ros "std_msgs/Bool" [bool]
  (bean-clean bool))

(defn bool [& {:keys [data] :or {data false}}]
  (->Bool data))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Duration
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defrecord Duration [secs nsecs])

(defmethod from-ros org.ros.message.Duration [duration]
  {:secs (.secs duration) :nsecs (.nsecs duration)})

(defn duration [& {:keys [secs nsecs] :or {secs 0 nsecs 0}}]
  (->Duration secs nsecs))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Time
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defrecord Time [secs nsecs])

(defmethod from-ros org.ros.message.Time [duration]
  {:secs (.secs duration) :nsecs (.nsecs duration)})

(defn time [& {:keys [secs nsecs] :or {secs 0 nsecs 0}}]
  (->Time secs nsecs))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Header
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defrecord Header [seq stamp frame-id])

(defmethod from-ros "std_msgs/Header" [header]
  (-> (bean-clean header)
      (update-in [:stamp] from-ros)))

(defn header [& {:keys [seq stamp frame-id] :or {seq 0 stamp (time) frame-id ""}}]
  (->Header seq stamp frame-id))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; String
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defmethod from-ros "std_msgs/String" [str]
  (:data (bean-clean str)))

(defmethod to-ros java.lang.String [str]
  ;; convert to ros serialization of string
  )
