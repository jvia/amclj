(ns amclj.core
  (:require [sensor-msgs]
            [tf2-msgs]
            [std-msgs]
            [amclj.ros :refer :all]
            [amclj.pf :refer [uniform-initialization calculate-control mcl pose-estimate]]
            [clojure.core.async :refer [go chan >! <! >!! <!! sliding-buffer]]
            [clojure.core.async :as async]
            [clojure.reflect :as reflect]
            [incanter.stats :as stats]
            [taoensso.timbre :as timbre]))

(timbre/refer-timbre)

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
  (let [node (start (rosnode "amcl"))]
    (wait-until-connected node)
    (try
      (-> node
          ;; Subscriptions
          (subscribe "/scan" sensor_msgs.LaserScan #(go (>! scan %)))
          (subscribe  "/tf" tf2_msgs.TFMessage #(go (>! tf %)))
          (subscribe  "/odom" nav_msgs.Odometry #(go (>! odom %)))
          (subscribe "/initialpose" geometry_msgs.PoseWithCovarianceStamped #(do (debug "Received pose") (reset! *pose* %)))
          (subscribe "/map" nav_msgs.OccupancyGrid #(do (debug "Received map") (reset! *map* %)))
          ;; Publications
          (add-publisher "/pose2" geometry_msgs.PoseStamped)
          (add-publisher "/particlecloud" geometry_msgs.PoseArray)
          (add-publisher "/tf" tf2_msgs.TFMessage))
      (catch Exception e
        (stop node)
        (error e "Error initializing node")
        node))))

(defn update-tf [pose tf time])

(defn wait-for-map []
  (when (nil? @*map*)
    (println "Waiting for map...")
    (Thread/sleep 1000)
    (recur)))

#_(def amcl (make-amcl-node))

;; reset the node

(comment

  (when-not (nil? amcl)
    (stop amcl)
    (Thread/sleep 1000)
    (def amcl nil))
  
  
  (def amcl (make-amcl-node))


  #_(defn -main []
      (wait-for-map)
      (let [amcl (make-amcl-node)
            num-particles 100]
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
                (publish amcl "/particlecloud" particles')
                (publish amcl "/amcl_pose" pose)
                (publish amcl "/tf" tf')
                (recur particles' (<! scan) curr-tf (<! tf))))))))
  

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
                       (subscribe "/chatter" std_msgs.String  #(println %))
                       (add-publisher "/chatter" std_msgs.String)
                       (subscribe "/pose" geometry_msgs.Pose #(println %))
                       (add-publisher "/pose" geometry_msgs.Pose)))


  )



(defn publish-silly-cloud [node]
  (when (running? node)
    (when-let [pose @*pose*]
      (let [fake-cloud (geometry-msgs/pose-array
                        :header (std-msgs/header :frameId "base_link")
                        :poses (repeatedly 1000 #_(-> pose :pose :pose)
                                           #(amclj.pf/gaussian-noise (-> pose :pose :pose) [0 0.5] [0 0.01])))
            fake-pose (geometry-msgs/pose-stamped
                       :header (std-msgs/header :frameId "base_link")
                       :pose (pose-estimate (:poses fake-cloud)))]
        (publish node "/particlecloud" fake-cloud)
        (publish node "/pose2" fake-pose)))))
