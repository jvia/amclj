(ns amclj.pf
  (:require [geometry-msgs :refer :all]
            [nav-msgs :refer :all]
            [tf2-msgs]
            [incanter.core :refer :all]
            [incanter.stats :as stats]
            [clojure.core.async :refer [<!! <! >! >!! go go-loop chan]]
            [taoensso.timbre :as log]
            [amclj.quaternions :as quat]
            [amclj.map :refer [uniform-initialization gaussian-initialization]]
            [amclj.motion :refer  [sample-motion-model-odometry]]
            [amclj.sensor :refer [likelihood-field-range-finder-model]]))

(def num-particles 10)

(defn initialize-pf
  "Initialize a particle filter based on a map and optionally on a
  pose.

  If no pose is provided, the particles are uniformly distributed
  among the map. If a pose is provided, then the particles are
  distributed in a Gaussian about the pose."
  [map num-particles & {:keys [pose] :or {pose nil}}]
  (geometry-msgs/pose-array
   :header (std-msgs/header :frameId "map")
   :poses (if (nil? pose)
            (uniform-initialization map num-particles)
            (gaussian-initialization map num-particles pose))))

(defn add-pose
  "Add a number poses together"
  [a & rest]
  (let [vals (if (nil? rest) [a] (concat [a] rest))]
    (geometry-msgs/pose
     :position
     (geometry-msgs/point
      :x (reduce #(+ %1 (get-in %2 [:position :x])) 0 vals)
      :y (reduce #(+ %1 (get-in %2 [:position :y])) 0 vals)
      :z (reduce #(+ %1 (get-in %2 [:position :z])) 0 vals))
     :orientation
     (geometry-msgs/quaternion
      :x (reduce #(+ %1 (get-in %2 [:orientation :x])) 0 vals)
      :y (reduce #(+ %1 (get-in %2 [:orientation :y])) 0 vals)
      :z (reduce #(+ %1 (get-in %2 [:orientation :z])) 0 vals)
      :w (reduce #(+ %1 (get-in %2 [:orientation :w])) 0 vals)))))

(defn minus-pose
  "Add a number poses together"
  [a b & rest]
  (let [vals (if (nil? rest) [a b] (concat [a b] rest))]
    (geometry-msgs/pose
     :position
     (geometry-msgs/point
      :x (reduce #(- %1 (get-in %2 [:position :x])) 0 vals)
      :y (reduce #(- %1 (get-in %2 [:position :y])) 0 vals)
      :z (reduce #(- %1 (get-in %2 [:position :z])) 0 vals))
     :orientation
     (geometry-msgs/quaternion
      :x (reduce #(- %1 (get-in %2 [:orientation :x])) 0 vals)
      :y (reduce #(- %1 (get-in %2 [:orientation :y])) 0 vals)
      :z (reduce #(- %1 (get-in %2 [:orientation :z])) 0 vals)
      :w (reduce #(- %1 (get-in %2 [:orientation :w])) 0 vals)))))

(defn pose-estimate
  "Estimate the pose from the particle cloud."
  [particles]
  (let [raw-estimate (apply add-pose particles)
        raw-heading (reduce + (map (comp quat/to-heading :orientation) particles))
        normalizer (count particles)]
    (geometry-msgs/pose-stamped
     :pose
     (geometry-msgs/pose
      :position (geometry-msgs/point
                 :x (/ (get-in raw-estimate [:position :x]) normalizer)
                 :y (/ (get-in raw-estimate [:position :y]) normalizer)
                 :y (/ (get-in raw-estimate [:position :y]) normalizer))
      :orientation (geometry-msgs/quaternion 
                    :x (/ (get-in raw-estimate [:orientation :x]) normalizer)
                    :y (/ (get-in raw-estimate [:orientation :y]) normalizer)
                    :z (/ (get-in raw-estimate [:orientation :z]) normalizer)
                    :w (/ (get-in raw-estimate [:orientation :w]) normalizer))))))


(defn calculate-control
  "Given a channel of TF data, find an odom to base_footprint
  transform as the control."
  [new-transform old-transform]
  (assert (not (nil? (:translation new-transform)))
          (str "new-transform does not have a translation field: " (type new-transform)))
  (assert (not (nil? (:translation old-transform)))
          (str "old-transform does not have a translation field: " (type old-transform)))
  (println (type new-transform) (type old-transform))
  (if (nil? old-transform)
    new-transform
    (geometry-msgs/transform
     :translation (geometry-msgs/vector3
                   :x (- (-> new-transform :translation :x) (-> old-transform :translation :x))
                   :y (- (-> new-transform :translation :y) (-> old-transform :translation :y))))))


(defn get-transform
  "Given a channel to TFMessage data, find a transform between the
  parent and child frames."
  [tf-ch parent child]
  ;; TODO: this is a hack and assume the first transform is the one we want
  (loop [tf (<!! tf-ch)]
    (if (and (= parent (-> tf :transforms first :header :frameId))
             (= child  (-> tf :transforms first :childFrameId)))
      (-> tf :transforms first :transform)
      (recur (<!! tf-ch)))))

(defn update-tf
  "- pose-estimate: a geometry_msgs.Pose
   - tf: current tf"
  [pose tf-ch]
  (let [odom-tf (get-transform tf-ch "odom" "base_footprint")
        transform (geometry-msgs/transform-stamped
                   :header (std-msgs/header :frameId "/map")
                   :child-frame-id "/odom"
                   :transform
                   (geometry-msgs/transform
                    :translation (geometry-msgs/vector3
                                  :x (- (-> pose :position :x) (-> odom-tf :translation :x))
                                  :y (- (-> pose :position :y) (-> odom-tf :translation :y))
                                  :z (- (-> pose :position :z) (-> odom-tf :translation :z)))
                    :rotation  (geometry-msgs/quaternion 
                                :w (- (-> pose :orientation :w) (-> odom-tf :rotation :w))  
                                :x (- (-> pose :orientation :x) (-> odom-tf :rotation :x))  
                                :y (- (-> pose :orientation :y) (-> odom-tf :rotation :y))  
                                :z (- (-> pose :orientation :z) (-> odom-tf :rotation :z)))))]
    (tf2-msgs/tf :transforms [transform])))



;; TODO
(defn apply-motion-model
  "Updates the particle cloud based on the control (and implicitly the
  motion model)"
  [control particles]
  (assoc particles
    :poses (for [particle (:poses particles)]
             (sample-motion-model-odometry particle control))))

;; TODO
(defn apply-sensor-model
  "Returns a set of weighted paticles based on the scan the measurement model."
  [scan map particles]
  (assoc particles
    :poses (for [particle (:poses particles)]
             (likelihood-field-range-finder-model
              scan particle map))))

(defn tournament-selection [particles]
  particles)

(defn init-and-reset [map-atom pose-atom]
  (let [particles (initialize-pf @map-atom num-particles :pose @pose-atom)]
    (reset! pose-atom nil)
    particles))

(defn log-odom [curr-odom prev-odom]
  (let [ax (-> curr-odom :pose :pose :position :x)
        ay (-> curr-odom :pose :pose :position :y)
        ao (-> curr-odom :pose :pose :orientation quat/to-heading)
        bx (-> prev-odom :pose :pose :position :x)
        by (-> prev-odom :pose :pose :position :y)
        bo (-> prev-odom :pose :pose :orientation quat/to-heading)]
    (log/debug "curr: " [ax ay ao] "; prev: " [bx by bo])))

(defn mcl* [tf-ch laser-ch odom-ch pose-atom map-atom result-ch]
  (try
    (let [part-init (initialize-pf @map-atom num-particles :pose @pose-atom)]
      ;; clear it out when we're done
      (reset! pose-atom nil)
      (loop [particles part-init
             prev-odom (nav-msgs/odometry)]
        (let [particles'(if (nil? @pose-atom) particles (init-and-reset map-atom pose-atom))
              curr-odom (<!! odom-ch)
              particles'' (->> particles'
                               (apply-motion-model [curr-odom prev-odom])
                               (apply-sensor-model (<!! laser-ch) @map-atom)
                               tournament-selection)
              pose-stamped (assoc (pose-estimate (:poses particles'')) :header (std-msgs/header :frameId "map"))
              tf   (update-tf (:pose pose-stamped) tf-ch)]
          (log/debug "MCL data calculated")
          (when (>!! result-ch [particles'' pose-stamped tf])
            (recur particles'' curr-odom)))))
    (catch Exception e
      (log/error e))))

(defn monte-carlo-localization
  "Perform monte carlo localization.

Expects:
  - tf-ch: a channel of tf2_msgs.TFMessage data
  - laser-ch: a channel of sensor_msgs.LaserScan data
  - odom-ch: a channel of nav_msgs.Odometry data
  - pose-atom: an atom potentially containing an initial geometry_msgs.PoseWithCovarianceStamped
  - map-atom: an atom containing the map in the form nav_msgs.OccupancyGrid

Returns:
  - a channel from which [geometry_msgs.PoseArray geometry_msgs.PoseStamped tf2_msgs.TFMessage] 
    data can be pulled and then published to ROS.
"
  [tf-ch laser-ch odom-ch pose-atom map-atom]
  (let [result-ch (chan)]
    (go (mcl* tf-ch laser-ch odom-ch pose-atom map-atom result-ch))
    result-ch))

;;(defn amcl [particles control measurement map])

;;(defn kld-mcl [particles control measurement map epsilon delta])


