(ns amclj.core
  (:require [amclj.ros :refer :all]))

(defn sleepy-identity
  "Artificial delay to allow rosjava to initialize"
  [x]
  (Thread/sleep 2000) x)

(def output (atom nil))
(add-watch output :key (fn [k r os ns] (println k r os (.getData ns))))

(def basic-node
  (-> (rosnode "basic")
      start sleepy-identity
      (add-publisher "/chatter" std_msgs.String/_TYPE)
      (subscribe "/chatter" std_msgs.String/_TYPE #(reset! output %))))

(def counter (atom 0))
(publish basic-node "/chatter"
         (doto (new-message basic-node "/chatter")
           (.setData (str "Hello world" (swap! counter inc)))))
(def ^:dynamic *map* (atom nil))


;; (def node
;;   (-> (rosnode "amcl")
;;       start sleepy-identity
;;       ;; Subscriptions
;;       #_(subscribe "/scan"  sensor_msgs.LaserScan/_TYPE)
;;       #_(subscribe  "/tf" tf.tfMessage/_TYPE)
;;       (subscribe "/initialpose" geometry_msgs.PoseWithCovarianceStamped/_TYPE)
;;       (subscribe "/map" nav_msgs.OccupancyGrid/_TYPE
;;                  (proxy [org.ros.message.MessageListener] []
;;                    (onNewMessage [occupancy-grid] (reset! *map* occupancy-grid))))
;;       (subscribe "/map" nav_msgs.OccupancyGrid/_TYPE #(reset! *map* %))
;;       ;; Publications
;;       (add-publisher )))

#_(.addShutdownHook (Runtime/getRuntime)
                  (Thread. (fn [] (println "Shutting down..."))))





