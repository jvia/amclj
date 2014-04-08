(ns amclj.core
  (:require
   [std-msgs]
   [sensor-msgs]
   [tf2-msgs]
   [amclj.ros :refer :all]
   [amclj.pf :refer :all]
   [amclj.quaternions :as quat]
   [clojure.core.async :refer [go chan >! <! >!! <!! sliding-buffer alts!! timeout]]
   [clojure.core.async :as async]
   [clojure.reflect :as reflect]
   [incanter.stats :as stats]
   [taoensso.timbre :as timbre]
   [amclj.map :as map]))


(timbre/refer-timbre)

(defn sleepy-identity
  "Artificial delay to allow rosjava to initialize"
  [x]
  (Thread/sleep 2000) x)

(def ^:dynamic *map* (if (resolve '*map*) *map* (atom nil)))
(def ^:dynamic *pose* (atom nil))
(def laser-ch (chan (sliding-buffer 1)))
(def tf-ch (chan (sliding-buffer 1)))
(def odom-ch (chan (sliding-buffer 1)))

(defn make-amcl-node []
  (let [node (start (rosnode "amcl"))]
    (wait-until-connected node)
    (try
      (-> node
          ;; Subscriptions
          (subscribe "/scan" sensor_msgs.LaserScan #(go (>! laser-ch %)))
          (subscribe  "/tf" tf2_msgs.TFMessage #(go (>! tf-ch %)))
          (subscribe  "/odom" nav_msgs.Odometry #(go (>! odom-ch %)))
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

#_(reset! *pose*
        (geometry-msgs/pose-with-covariance-stamped
         :header (std-msgs/header :frameId "map")
         :pose (geometry-msgs/pose-with-covariance
                :pose (geometry-msgs/pose
                       :position (geometry-msgs/point :x 15 :y 15)))))
(defn -main [amcl]
  (info "Starting node")
  (let [;;amcl (make-amcl-node)
        mcl-ch (monte-carlo-localization tf-ch laser-ch odom-ch *pose* *map*)]
    (debug "Results channel created.")
    (loop []
      (let [[particles pose tf] (<!! mcl-ch)]
        #_(debug (str "Recieved data: [" (type particles) ", " (type pose) ", " (type tf) "]"))
        (publish amcl "/particlecloud" particles)
        (publish amcl "/tf" tf)
        (publish amcl "/pose2" pose)
        (debug "Published data")
        (recur)))))

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
(defn publish-uniform-cloud [node]
  (try
    (when (running? node)
      (let [uniform-cloud (geometry-msgs/pose-array
                           :header (std-msgs/header :frameId "map")
                           :poses (map/uniform-initialization @*map* 200))
            _ (debug "Cloud created")
            new-tf (update-tf (geometry-msgs/pose) tf-ch)]
        (publish node "/particlecloud" uniform-cloud)
        (publish node "/tf" new-tf)))
    (catch Exception e (error e))))


;; This code repeatedly publishes transforms.
;;
;; (repeatedly 500 #(do (publish amcl "/tf" (amclj.pf/update-tf (geometry-msgs/pose
;;                                                               :position (geometry-msgs/vector3 :x 16.5 :y 16.18)
;;                                                               :orientation (geometry-msgs/quaternion :z 0.97 :w 0.23))
;;                                                              tf nil))
;;                      (Thread/sleep 100)))


#_(defn test-apply-motion-model [amcl-node]
    (let [init (geometry-msgs/pose-array
                :header (std-msgs/header :frameId "map")
                :poses [(geometry-msgs/pose
                         :position (geometry-msgs/point :x 16 :y 16))])]
      (with-log-level :trace
        (try
          (loop [particles init]
            (let [control (geometry-msgs/transform
                           :translation (geometry-msgs/vector3 :x 0)
                           :rotation (quat/from-heading (- (/ Math/PI 8))))
                  updated-particles (apply-motion-model control particles)
                  pose-stamped (assoc (pose-estimate (:poses updated-particles))
                                 :header (std-msgs/header :frameId "map"))
                  tf   (update-tf (:pose pose-stamped) tf-ch)]
              (debug (map vals (vals (:pose pose-stamped))))
              (publish amcl "/particlecloud" updated-particles)
              (publish amcl "/pose2" pose-stamped)
              (publish amcl "/tf" tf)
              (Thread/sleep 1000)
              (recur updated-particles)))
          (catch Exception e (error e))))))


(defn publish-all-possible-poses [amcl]
  (with-log-level :trace
    (try
      (let [particles (geometry-msgs/pose-array
                       :header (std-msgs/header :frameId "map")
                       :poses (map/saturate-map @*map*))
            _ (debug "Created " (count (:poses particles)) " particles")
            tf   (update-tf (geometry-msgs/pose) tf-ch)]
        (debug "Particles: " (count (:poses particles)))
        (publish amcl "/particlecloud" particles)
        (publish amcl "/tf" tf))
      (catch Exception e (error e)))))
