(ns amclj.core
  (:require [sensor-msgs]
            [tf2-msgs]
            [std-msgs]
            [amclj.ros :refer :all]
            [amclj.pf :refer :all]
            [clojure.core.async :refer [go chan >! <! >!! <!! sliding-buffer alts!! timeout]]
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
(def laser-ch (chan (sliding-buffer 1)))
(def tf-ch (chan (sliding-buffer 1)))
(def odom (chan (sliding-buffer 1)))

(defn make-amcl-node []
  (let [node (start (rosnode "amcl"))]
    (wait-until-connected node)
    (try
      (-> node
          ;; Subscriptions
          (subscribe "/scan" sensor_msgs.LaserScan #(go (>! laser-ch %)))
          (subscribe  "/tf" tf2_msgs.TFMessage #(go (>! tf-ch %)))
          #_(subscribe  "/odom" nav_msgs.Odometry #(go (>! odom %)))
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


(defn wait-for-map []
  (when (nil? @*map*)
    (println "Waiting for map...")
    (Thread/sleep 1000)
    (recur)))

(defn -main []
  (let [;;amcl (make-amcl-node)
          mcl-ch (monte-carlo-localization tf-ch laser-ch *pose* *map*)]
      (loop []
        (let [[particles pose tf] (<!! mcl-ch)]
          (debug (:header particles))
          (debug pose)
          (debug tf)
          (recur)
          #_(-> amcl
                (publish "/particlecloud" particles)
                (publish "/pose2" pose)
                (publish "/tf" tf))))))

;; (defn odom-printer []
;;   (when-let [pose @*pose*]
;;     (loop [particles (geometry-msgs/pose-array
;;                       :header (std-msgs/header :frameId "map")
;;                       :poses (repeatedly 100 #(gaussian-noise (-> pose :pose :pose) [0 0.1] [0 0.01])))
;;            curr-odom (:transform (get-transform tf "odom" "base_footprint"))
;;            prev-odom nil]
;;       (let [control (calculate-control curr-odom prev-odom)
;;             particles' (odom-update control particles)
;;             pose (geometry-msgs/pose-stamped :header (std-msgs/header :frameId "map")
;;                                              :pose (pose-estimate (:poses particles')))
;;             tf' (update-tf (:pose pose) tf nil)]
;;         (debug (map vals (vals control)))
;;         (debug (map vals (vals (:pose pose))))
;;         (Thread/sleep 1000)
;;         (recur particles'
;;                (:transform (get-transform tf "odom" "base_footprint"))
;;                curr-odom)))))


;; reset the node

;; (comment

;;   (when-not (nil? amcl)
;;     (stop amcl)
;;     (Thread/sleep 1000)
;;     (def amcl nil))


;;   (defn publish-loop [times]
;;     (wait-for-map)
;;     (go (loop [times times]
;;           (when-not (zero? times)
;;             (let [scan' (<! scan)]
;;               (println "Received data...publishing response" times)
;;               (publish amcl "/amcl_pose" (new-message amcl "/amcl_pose"))
;;               (publish amcl "/paticlecloud" (new-message amcl "/paticlecloud"))
;;               (recur (dec times)))))))



;;   (def simple-node (-> (rosnode "basic")
;;                        start sleepy-identity
;;                        (subscribe "/chatter" std_msgs.String  #(println %))
;;                        (add-publisher "/chatter" std_msgs.String)
;;                        (subscribe "/pose" geometry_msgs.Pose #(println %))
;;                        (add-publisher "/pose" geometry_msgs.Pose)))


;;   )

;; (defn publish-gaussian-cloud [node]
;;   (when (running? node)
;;     (loop [particles nil, prev-odom nil]
;;       (if-let [pose @*pose*]
;;         ;; we have pose, so initialize new cloud there
;;         (let [particles (geometry-msgs/pose-array
;;                          :header (std-msgs/header :frameId "map")
;;                          :poses (repeatedly 100 #(gaussian-noise (-> pose :pose :pose) [0 0.1] [0 0.01])))]
;;           (reset! *pose* nil)
;;           (recur particles prev-odom))
;;         ;; updating based on odom
;;         (if (not (nil? particles))
;;           (let [odometry (:transform (get-transform tf "odom" "base_footprint"))
;;                 _ (debug "Odomdetry calculated")
;;                 control (calculate-control odometry prev-odom)
;;                 _ (debug "Control calculated")
;;                 particles' (odom-update control particles)
;;                 _ (debug "Prediction applied")
;;                 est-pose (geometry-msgs/pose-stamped :header (std-msgs/header :frameId "map")
;;                                                      :pose (pose-estimate (:poses particles')))
;;                 _ (debug "Pose estimated")
;;                 tf' (update-tf (:pose est-pose) tf nil)
;;                 _ (debug "New TF created")]
;;             (publish node "/tf" tf')
;;             (publish node "/particlecloud" particles')
;;             (publish node "/pose2" est-pose)
;;             (recur particles' odometry))
;;           (recur nil nil))))))


;; ;; Potentiall an error (not sure if it looks odd because there is no transform with the map frame
;; (defn publish-uniform-cloud [node] 
;;   (when (running? node)
;;     (let [uniform-cloud (geometry-msgs/pose-array
;;                          :header (std-msgs/header :frameId "map")
;;                          :poses (uniform-initialization @*map* 5000))
;;           pose (geometry-msgs/pose-stamped
;;                 :header (std-msgs/header :frameId "map")
;;                 :pose (pose-estimate (:poses uniform-cloud)))
;;           new-tf (update-tf (:pose pose) tf nil)]
;;       (publish node "/particlecloud" uniform-cloud)
;;       (publish node "/pose2" pose)
;;       (publish node "/tf" new-tf))))


;; This code repeatedly publishes transforms.
;;
;; (repeatedly 500 #(do (publish amcl "/tf" (amclj.pf/update-tf (geometry-msgs/pose
;;                                                               :position (geometry-msgs/vector3 :x 16.5 :y 16.18)
;;                                                               :orientation (geometry-msgs/quaternion :z 0.97 :w 0.23))
;;                                                              tf nil))
;;                      (Thread/sleep 100)))

