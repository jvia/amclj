(ns amclj.core
  (:require [amclj.ros :refer :all]
            [amclj.pf :refer [uniform-initialization calculate-control mcl pose-estimate]]
            [rosclj.msg :refer [from-ros to-ros]]
            [clojure.core.async :refer [go chan >! <! >!! <!! sliding-buffer]]
            [clojure.core.async :as async]
            [clojure.reflect :as reflect]
            [gavagai.core :as g]))


(defn sleepy-identity
  "Artificial delay to allow rosjava to initialize"
  [x]
  (Thread/sleep 2000) x)

(def ^:dynamic *map* (atom nil))
(def ^:dynamic *pose* (atom nil))
(def scan (chan (sliding-buffer 1)))
(def pose (chan (sliding-buffer 1)))
(def tf (chan (sliding-buffer 1)))
(def odom (chan (sliding-buffer 1)))

(defn avg [coll]
  (/ (reduce + coll) (count coll)))


(defn make-amcl-node []
  (-> (rosnode "amcl")
      start sleepy-identity
      ;; Subscriptions
      (subscribe "/scan" sensor_msgs.LaserScan #(go (>! scan (from-ros %))))
      (subscribe  "/tf" tf2_msgs.TFMessage #(go (>! tf (from-ros %))))
      (subscribe  "/odom" nav_msgs.Odometry #(go (>! odom (from-ros %))))
      (subscribe "/initialpose" geometry_msgs.PoseWithCovarianceStamped #(reset! *pose* (from-ros %)))
      (subscribe "/map" nav_msgs.OccupancyGrid #(reset! *map* (from-ros %)))
      ;; Publications
      (add-publisher "/amcl_pose" geometry_msgs.PoseWithCovarianceStamped)
      (add-publisher "/particlecloud" geometry_msgs.PoseArray)
      (add-publisher "/tf" tf2_msgs.TFMessage)))

(defn update-tf [pose tf time])

(defn wait-for-map []
  (when (nil? @*map*)
    (println "Waiting for map...")
    (Thread/sleep 1000)
    (recur)))

(defn -main []
  (wait-for-map)
  (let [
        ;;amcl (make-amcl-node)
        num-particles 100
        ]
    (go
      (loop [particles (uniform-initialization @*map* num-particles)
             laser-scan (<! scan)
             last-tf nil
             curr-tf (<! tf)]
        (when (running? amcl)
          (let [control (calculate-control last-tf curr-tf)
                particles' (mcl laser-scan control @*map*)
                pose (pose-estimate particles)
                current-time (-> laser-scan :header :stamp)
                tf' (update-tf pose tf current-time)]
            (publish amcl "/particlecloud" (to-ros (new-message amcl "/particlecloud") particles'))
            (publish amcl "/amcl_pose" (to-ros (new-message amcl "/amcl_pose")  pose))
            (publish amcl "/tf" (to-ros (new-message amcl "/tf")  tf'))
            (recur particles' (<! scan) curr-tf (<! tf))))))))

;; reset the node

(comment

  (when-not (nil? amcl)
    (stop amcl)
    (Thread/sleep 1000)
    (def amcl nil))
  (def amcl (make-amcl-node))
  
  (def amcl (make-amcl-node))



  (defn publish-loop [times]
    (wait-for-map)
    (go (loop [times times]
          (when-not (zero? times)
            (let [scan' (<! scan)]
              (println "Received data...publishing response" times)
              (publish amcl "/amcl_pose" (new-message amcl "/amcl_pose"))
              (publish amcl "/paticlecloud" (new-message amcl "/paticlecloud"))
              (recur (dec times)))))))



  (def simple-node (-> (rosnode "basic")
                       start sleepy-identity
                       (subscribe "/chatter" std_msgs.String  #(println (from-ros %)))
                       (add-publisher "/chatter" std_msgs.String)
                       (subscribe "/pose" geometry_msgs.Pose #(println (from-ros %)))
                       (add-publisher "/pose" geometry_msgs.Pose)))


  )


