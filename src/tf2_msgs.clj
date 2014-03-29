(ns tf2-msgs (:require [rosclj.msg :refer :all]))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; TFMessage
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defrecord TFMessage [transforms])

(defmethod from-ros "tf2_msgs/TFMessage" [tf]
  (-> (bean-clean tf)
      (update-in [:transforms] #(pmap from-ros %))))

(defn tf [& {:keys [transforms] :or {transforms '()}}]
  (->TFMessage transforms))


