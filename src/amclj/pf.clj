(ns amclj.pf
  (:require [geometry-msgs :refer :all]
            [nav-msgs :refer :all]
            [amclj.model :as model]
            [incanter.core :refer :all]
            [incanter.stats :as stats]
            [clojure.core.async :refer [<!! <! >! >!! go go-loop chan]]
            [taoensso.timbre :as log]))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Quaternions
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defn mult-quaternion
  "Multiply two quaternions together."
  [qa qb]
  (let [qaw (:w qa) qax (:x qa) qay (:y qa) qaz (:z qa)
        qbw (:w qb) qbx (:x qb) qby (:y qb) qbz (:z qb)]
    (geometry-msgs/quaternion
     :x (+ (* qax qbw) (* qaw qbx) (* qay qbz) (- (* qaz qby)))
     :y (+ (* qaw qby) (- (* qax qbz)) (* qay qbw) (* qaz qbx)) 
     :z (+ (* qaw qbz) (* qax qby) (- (* qay qbx)) (* qaz qbw))
     :w (- (* qaw qbw) (* qax qbx) (* qay qby) (* qaz qbz)))))


(defn rotate-quaternion
  "Roatate a quaternion about some heading."
  [quat heading]
  (let [pitch 0 bank 0
        c1 (Math/cos (/ heading 2))
        s1 (Math/sin (/ heading 2))
        c2 (Math/cos (/ pitch 2))
        s2 (Math/sin (/ pitch 2))
        c3 (Math/cos (/ bank 2))
        s3 (Math/sin (/ bank 2))
        c1c2 (* c1 c2)
        s1s2 (* s1 s2)]
    (mult-quaternion
     (geometry-msgs/quaternion
      :w (- (* c1c2 c3) (* s1s2 s3))
      :x (+ (* c1c2 s3) (* s1s2 c3))
      :y (- (* c1 s2 c3) (* s1 c2 s3))
      :z (+ (* s1 c2 c3) (* c1 s2 s3)))
     quat)))


(defn create-quaternion
  "Cretate a unit quaternion."
  []
  (let [heading 0 pitch 0 bank 0
        c1 (Math/cos (/ heading 2))
        s1 (Math/sin (/ heading 2))
        c2 (Math/cos (/ pitch 2))
        s2 (Math/sin (/ pitch 2))
        c3 (Math/cos (/ bank 2))
        s3 (Math/sin (/ bank 2))
        c1c2 (* c1 c2)
        s1s2 (* s1 s2)]
    (geometry-msgs/quaternion
     :w (- (* c1c2 c3) (* s1s2 s3))
     :x (+ (* c1c2 s3) (* s1s2 c3))
     :y (+ (* s1 c2 c3) (* c1 s2 s3))
     :z (- (* c1 s2 c3) (* s1 c2 s3)))))

(defn heading->quat
  "Given a heading (yaw) in radians, convert it to a quaternion."
  [heading]
  (let [initial (geometry-msgs/quaternion)]
    (rotate-quaternion initial heading)))

(defn quat->heading
  "Given a quaternion, return the heading (yaw) in radians."
  [quat]
  (let [qx (-> quat :x)
        qy (-> quat :y)
        qz (-> quat :z)
        qw (-> quat :w)
        test (+ (* qx qy) (* qz qw))]
    (cond
     ;; singularity as north pole
     (> test 0.4999) (* 2  (Math/atan2 qx qw))
     ;; singularity at south pole
     (< test -0.499) (* -2 (Math/atan2 qx qw))
     :else
     #_(Math/atan2   (- (* 2 qy qw) (* 2 qx qz)) (- 1 (* 2 qy qy) (* 2 qz qz)))
     (Math/atan2 (- (* 2 qz qw) (* 2 qx qy)) (- 1 (* 2 qz qz) (* 2 qy qy))))))


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; OccupancyGrid
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

#_(defn map-get [map x y]
    (let [width (-> map :info :width)
          height (-> map :info :height)
          idx (+ x (* y width))]
      (assert (and (>= x 0) (>= y 0) (< x width) (< y height)))
      (sel (-> map :data) x y)))


;; // Convert from map index to world coords
;; #define MAP_WXGX(map, i) (map->origin_x + (((i) - map->size_x / 2) * map->scale))
;; #define MAP_WYGY(map, j) (map->origin_y + (((j) - map->size_y / 2) * map->scale))

;; // Convert from world coords to map coords
;; #define MAP_GXWX(map, x) (floor((x - map->origin_x) / map->scale + 0.5) + map->size_x / 2)
;; #define MAP_GYWY(map, y) (floor((y - map->origin_y) / map->scale + 0.5) + map->size_y / 2)

;; // Test to see if the given map coords lie within the absolute map bounds.
;; #define MAP_VALID(map, i, j) ((i >= 0) && (i < map->size_x) && (j >= 0) && (j < map->size_y))

;; // Compute the cell index for the given map coords.
;; #define MAP_INDEX(map, i, j) ((i) + (j) * map->size_x)
(defn inhabitable?
  "Can a robot occupy the given location in the map?"
  [map pose]
  (let [x (int (-> pose :position :x))
        y (int (-> pose :position :y))]
    (zero? (sel (-> map :data) x y))))

(defn map->world
  "Convert from map index into world coordinates."
  [map pose]
  (-> pose
      (update-in [:position :x] #(* % (-> map :info :resolution)))
      (update-in [:position :y] #(* % (-> map :info :resolution))))
  ;; (let [origin-x (-> map :info :origin :position :x)
  ;;       origin-y (-> map :info :origin :position :y)
  ;;       height  (-> map :info :height)
  ;;       width (-> map :info :width)
  ;;       resolution (-> map :info :resolution)
  ;;       mx (-> pose :position :x)
  ;;       my (-> pose :position :y)
  ;;       x (+ origin-x (* (- mx (/ width 2)) resolution))
  ;;       y (+ origin-y (* (- my (/ width 2)) resolution))]
  ;;   (-> pose
  ;;       (assoc-in [:position :x] x)
  ;;       (assoc-in [:position :y] y)))
  )

(defn world->map
  "Convert from world coordinates to map coorindates"
  [map pose]
  (-> pose
      (update-in [:position :x] #(Math/round (/ % (-> map :info :resolution))))
      (update-in [:position :y] #(Math/round (/ % (-> map :info :resolution)))))
  ;; #_(let [origin-x (-> map :info :origin :position :x)
  ;;       origin-y (-> map :info :origin :position :y)
  ;;       height  (-> map :info :height)
  ;;       width (-> map :info :width)
  ;;       resolution (-> map :info :resolution)
  ;;       x (-> pose :position :x)
  ;;       y (-> pose :position :y)
  ;;       mx (+ (Math/floor (+ (/ (- x origin-x) resolution) 0.5))
  ;;             (/ width 2))
  ;;       my (+ (Math/floor (+ (/ (- y origin-y) resolution) 0.5))
  ;;             (/ height 2))]
  ;;   (-> pose
  ;;       (assoc-in [:position :x] mx)
  ;;       (assoc-in [:position :y] my)))
  )

(defn random-pose
  "Create a pose uniformly sampled from the grid defined by a map."
  ([width height] (random-pose 0 width 0 height))
  ([min-x max-x min-y max-y]
     (let [[x] (stats/sample-uniform 1 :min min-x :max max-x)
           [y] (stats/sample-uniform 1 :min min-y :max max-y)]
       (geometry-msgs/pose 
        :position (geometry-msgs/point :x x :y y)
        :orientation (rotate-quaternion (geometry-msgs/quaternion :x 0 :y 0 :z 0 :w 1)
                                        (stats/sample-normal 1))))))


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Particle Filter
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defn uniform-initialization
  "Uniform create particles across the map"
  [map num-particles]
  (log/trace  (type map) (str num-particles ": " (type num-particles)))
  (let [min-x 0, max-x (-> map :info :width)
        min-y 0, max-y (-> map :info :height)]
    (loop [particles []]
      (if (= num-particles (count particles))
        (do (log/trace "Returning" (count particles) "particles")
            particles)
        (let [proposal (random-pose min-x max-x min-y max-y)]
          (if (inhabitable? map proposal)
            (recur (conj particles (map->world map proposal)))
            (recur particles)))))))

(defn gaussian-initialization [map num-particles pose]
  ;; Assumes a geometry_msgs.PoseWithCovarianceStamped
  (repeatedly num-particles
              #(gaussian-noise (-> pose :pose :pose) [0 0.1] [0 0.01])))


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
            (do (log/debug "Uniform initialization")
                (uniform-initialization map num-particles))
            (do (log/debug "Gaussian initialization")
                (gaussian-initialization map num-particles pose)))))

(defn add-pose
  "Add a number poses together"
  [a b & rest]
  (let [vals (if (nil? rest) [a b] (concat [a b] rest))]
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

(defn pose-estimate
  "Estimate the pose from the particle cloud."
  [particles]
  (let [raw-estimate (apply add-pose particles)
        raw-heading (reduce + (map (comp quat->heading :orientation) particles))
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


(defn beam-range-finder-model [])
(defn likelihood-field-range-finder-model [])

(defn sample-motion-model [control particle]
  (-> particle
      (update-in [:position :x] (partial + (* (-> control :translation :x) (stats/sample-normal 1 :mean 0 :sd 0.2))))
      (update-in [:position :y] (partial + (* (-> control :translation :y) (stats/sample-normal 1 :mean 0 :sd 0.2))))))

(defn odom-update [control particles]
  (assoc particles :poses
         (for [particle (:poses particles)]
           (sample-motion-model control particle))))

(defn measurement-model [measurement particle map])


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
      (-> tf :transforms first)
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
                                  :x (- (-> pose :position :x) (-> odom-tf :transform :translation :x))
                                  :y (- (-> pose :position :y) (-> odom-tf :transform :translation :y))
                                  :z (- (-> pose :position :z) (-> odom-tf :transform :translation :z)))
                    :rotation  (geometry-msgs/quaternion 
                                :w (- (-> pose :orientation :w) (-> odom-tf :transform :rotation :w))  
                                :x (- (-> pose :orientation :x) (-> odom-tf :transform :rotation :x))  
                                :y (- (-> pose :orientation :y) (-> odom-tf :transform :rotation :y))  
                                :z (- (-> pose :orientation :z) (-> odom-tf :transform :rotation :z)))))]
    (tf2-msgs/tf :transforms [transform])))

(defn gaussian-noise
  "Apply noise to a geometry_msgs.Pose object. Noises should be
  two-element vectors of [mean sd]."
  [pose trans-noise rotation-noise]
  (assert (and (vector? trans-noise) (vector? rotation-noise)) "Noises must be vectors of [mean sd]")
  (let [x  (-> pose :position :x)
        y  (-> pose :position :y)
        z  (-> pose :position :z)
        rx (-> pose :orientation :x)
        ry (-> pose :orientation :y)
        rz (-> pose :orientation :z)
        rw (-> pose :orientation :w)
        [trans-mean trans-sd] trans-noise
        [rot-mean rot-sd] rotation-noise]
    (geometry-msgs/pose
     :position (geometry-msgs/point :x (+ x (stats/sample-normal 1 :mean trans-mean :sd trans-sd))
                                    :y (+ y (stats/sample-normal 1 :mean trans-mean :sd trans-sd))
                                    :z 0)
     :orientation (rotate-quaternion (-> pose :orientation) (stats/sample-normal 1 :mean rot-mean :sd rot-mean)))))

;; TODO
(defn apply-motion-model
  "Updates the particle cloud based on the control (and implicitly the
  motion model)"
  [control particles]
  (throw (UnsupportedOperationException.)))

;; TODO
(defn apply-sensor-model
  "Returns a set of weighted paticles based on the scan the measurement model."
  [scan particles map]
  (throw (UnsupportedOperationException.)))

;; TODO
(defn weighted-sample
  "Sample count many particles to propagate to the next generation."
  [count weighted-particles]
  (throw (UnsupportedOperationException.)))


(defn mcl* [tf-ch laser-ch pose-atom map-atom result-ch]
  (try
    (let [particles (initialize-pf @map-atom 100 :pose @pose-atom)]
      ;; clear it out when we're done
      (reset! pose-atom nil)
      (log/debug (str "Particle initialized: " (count particles) " particles"))
      (loop [particles particles
             prev-transform (geometry-msgs/transform)]
        (let [particles' (if (nil? @pose-atom) particles
                             (let [p (initialize-pf @map-atom 100 :pose @pose-atom)]
                               (reset! pose-atom nil) p))
              _ (log/trace "|particles'| = " (count particles'))
              curr-transform (get-transform tf-ch "odom" "base_footprint")
              _ (log/debug "Received transform")
              ;;control (calculate-control  cur-transform prev-transform)
              ;;particles' (apply-motion-model control particles)
              ;;weight-particles (apply-sensor-model (<!! laser-ch) particles' @map)
              ;;particles'' (weighted-sampled weight-particles)
              particles''' particles'
              pose-stamped (assoc (pose-estimate (:poses particles'''))
                             :header (std-msgs/header :frameId "map"))
              _ (log/debug "Pose estimated")
              tf   (update-tf (:pose pose-stamped) tf-ch)
              _ (log/debug "TF calculated")]
          (when (>!! result-ch [particles''' pose-stamped tf])
            (log/debug "Result pushed to channel")
            (recur particles''' curr-transform)))))
    (catch Exception e
      (log/error e))))

(defn monte-carlo-localization
  "Perform monte carlo localization.

Expects:
  - tf-ch: a channel of tf2_msgs.TFMessage data
  - laser-ch: a channel of sensor_msgs.LaserScan data
  - pose-atom: an atom potentially containing an initial geometry_msgs.PoseWithCovarianceStamped
  - map-atom: an atom containing the map in the form nav_msgs.OccupancyGrid

Returns:
  - a channel from which [geometry_msgs.PoseArray geometry_msgs.PoseStamped tf2_msgs.TFMessage] 
    data can be pulled and then published to ROS.
"
  [tf-ch laser-ch pose-atom map-atom]
  (let [result-ch (chan)]
    (go (mcl* tf-ch laser-ch pose-atom map-atom result-ch))
    result-ch))

;;(defn amcl [particles control measurement map])

;;(defn kld-mcl [particles control measurement map epsilon delta])


