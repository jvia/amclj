(ns tf2-msgs (:require [rosclj.msg :refer :all]))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; TFMessage
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defrecord TFMessage [transforms])

(defmethod from-ros "tf2_msgs/TFMessage" [tf]
  (-> (bean-clean tf)
      (update-in [:transforms] #(pmap from-ros %))))

(with-msg-factory [factory
                   "tf2_msgs/TFMessage"
                   "geometry_msgs/TransformStamped[] transforms\n"]
  (defmethod to-ros :tf2_msgs.TFMessage [tf2]
    (let [transforms (map to-ros (:transforms tf2))]
      (doto (.newFromType factory "tf2_msgs/TFMessage")
        (.setTransformations (make-array ))))))

(defn tf [& {:keys [transforms] :or {transforms '()}}]
  (->TFMessage transforms))

