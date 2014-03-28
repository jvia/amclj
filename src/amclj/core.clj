(ns amclj.core
  (:require [amclj.ros :refer :all]
            [amclj.msg :as msg]
            [clojure.core.async :refer [go chan >! <! >!! <!! sliding-buffer]]
            [clojure.core.async :as async]
            [clojure.reflect :as reflect]
            [gavagai.core :as g]))


(defn sleepy-identity
  "Artificial delay to allow rosjava to initialize"
  [x]
  (Thread/sleep 2000) x)

(def ^:dynamic *map* (atom nil))
(def scan (chan (sliding-buffer 1)))
(def pose (chan (sliding-buffer 1)))
(def tf (chan (sliding-buffer 1)))



(defn occupancy->map [og] (msg/from-ros og))
#_(defn occupancy->map [og]
  {:header (header->map (.getHeader og))
   :metadata (map-meta->map (.getInfo og))
   :data (let [raw-data (.getData og)
               size (.readableBytes raw-data)
               array (byte-array size)
               _ (.readBytes raw-data array)]
           array
           #_(vec array))})

(defn avg [coll]
  (/ (reduce + coll) (count coll)))

(defn make-amcl-node []
  (-> (rosnode "amcl")
      start sleepy-identity
      ;; Subscriptions
      (subscribe "/scan" sensor_msgs.LaserScan #(go (>! scan (msg/from-ros %))))
      (subscribe  "/tf" tf.tfMessage #(go (>! tf (msg/from-ros %))))
      (subscribe "/initialpose" geometry_msgs.PoseWithCovarianceStamped #(go (>! pose (msg/from-ros %))))
      (subscribe "/map" nav_msgs.OccupancyGrid #(reset! *map* (msg/from-ros %)))
      ;; Publications
      (add-publisher "/amcl_pose" geometry_msgs.PoseWithCovarianceStamped)
      (add-publisher "/paticlecloud" geometry_msgs.PoseArray)
      (add-publisher "/tf" tf.tfMessage)))

#_(def amcl (make-amcl-node))

(defn wait-for-map []
  (when (nil? @*map*)
    (println "Waiting for map...")
    (Thread/sleep 1000)
    (recur)))

(defn publish-loop [times]
  (wait-for-map)
  (go (loop [times times]
      (when-not (zero? times)
        (let [scan' (<! scan)]
          (println "Received data...publishing response" times)
          (publish amcl "/amcl_pose" (new-message amcl "/amcl_pose"))
          (publish amcl "/paticlecloud" (new-message amcl "/paticlecloud"))
          (recur (dec times)))))))



