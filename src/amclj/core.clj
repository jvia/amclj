(ns amclj.core
  (:require [amclj.ros :refer :all]))

(defn sleepy-identity
  "Artificial delay to allow rosjava to initialize"
  [x]
  (Thread/sleep 2000) x)

(defn simple-node []
  (let [output (atom nil)]
    (add-watch output :printer (fn [k r os ns] (println k r os (.getData ns))))
    {:node (-> (rosnode "basic")
               start sleepy-identity
               (add-publisher "/chatter" std_msgs.String)
               (subscribe "/chatter" std_msgs.String #(reset! output (bean %))))
     :vars {:output output}}))

(def ^:dynamic *map* (atom nil))
(defn make-amcl-node []
  (let [map (atom nil)
        tf  (atom nil)
        initialpose (atom nil)
        scan (atom nil)]
    {:map map :tf tf :initialpose initialpose :scan scan
     :node
     (-> (rosnode "amcl")
         start sleepy-identity
         ;; Subscriptions
         (subscribe "/scan"  sensor_msgs.LaserScan #(reset! scan (bean %)))
         (subscribe  "/tf" tf.tfMessage #(reset! tf (bean %)))
         (subscribe "/initialpose" geometry_msgs.PoseWithCovarianceStamped #(reset! initialpose (bean %)))
         (subscribe "/map" nav_msgs.OccupancyGrid #(reset! map (bean %)))
         ;; Publications
         (add-publisher "/amcl_pose" geometry_msgs.PoseWithCovarianceStamped)
         (add-publisher "/paticlecloud" geometry_msgs.PoseArray)
         (add-publisher "/tf" tf.tfMessage))}))


