(ns ^{:doc "Code for working with OccupancyGrid maps"}
  amclj.map
  (:require [geometry-msgs :refer [pose quaternion point]]
            [amclj.quaternions :as quat]
            [incanter.core :refer [sel]]
            [incanter.stats :as stats]
            [taoensso.timbre :as log]))

(defn- dimensions [map]
  [(-> map :info :width)
   (-> map :info :height)])

(defn get-map [map]
  (:data map))

(defn get-meta [map]
  (:info map))

(defn get-cell
  "Get the value at the location (x,y) in the map."
  [map x y]
  (let [[width height] (dimensions map)]
    (if (or (> x width) (< x 0) (> y height) (< y 0)) -1
        (incanter.core/sel (:data map) y x))))

(defn occupied? [map x y]
  (> (get-cell map x y) 0))

(defn map->world
  "Convert from map index into world coordinates."
  ([map pose]
     (-> pose
         (update-in [:position :x] #(* % (-> map :info :resolution)))
         (update-in [:position :y] #(* % (-> map :info :resolution))))))

(defn world->map
  "Convert from world coordinates to map coorindates"
  [map pose]
  (if (vector? pose)
    (let [[x y] pose]
      [(Math/round (/ x (-> map :info :resolution)))
       (Math/round (/ y (-> map :info :resolution)))])
    ;; assume pose
    (-> pose
        (update-in [:position :x] #(Math/round (/ % (-> map :info :resolution))))
        (update-in [:position :y] #(Math/round (/ % (-> map :info :resolution)))))))

(defn inhabitable?
  "Can a robot occupy the given location in the map?"
  ([map x y]
     (zero? (get-cell map x y)))
  ([map pose]
     (inhabitable? map
                   (int (-> pose :position :x))
                   (int (-> pose :position :y)))))

(def uninhabitable? (comp not inhabitable?))

(defn random-pose
  "Create a pose uniformly sampled from the grid defined by a map."
  ([width height] (random-pose 0 width 0 height))
  ([min-x max-x min-y max-y]
     (let [[x] (stats/sample-uniform 1 :min min-x :max max-x)
           [y] (stats/sample-uniform 1 :min min-y :max max-y)]
       (geometry-msgs/pose 
        :position (geometry-msgs/point :x x :y y)
        :orientation (quat/rotate (geometry-msgs/quaternion :x 0 :y 0 :z 0 :w 1)
                                  (stats/sample-normal 1))))))


(defn uniform-initialization
  "Uniform create particles across the map"
  [map num-particles]
  ;;(log/trace  (type map) (str num-particles ": " (type num-particles)))
  (let [min-x 0, max-x (-> map :info :width)
        min-y 0, max-y (-> map :info :height)]
    (loop [particles []]
      (if (= num-particles (count particles))
        (do ;;(log/trace "Returning" (count particles) "particles")
          particles)
        (let [proposal (random-pose min-x max-x min-y max-y)]
          (if (inhabitable? map proposal)
            (recur (conj particles (map->world map proposal)))
            (recur particles)))))))


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
     :position (geometry-msgs/point
                :x (+ x (stats/sample-normal 1 :mean trans-mean :sd trans-sd))
                :y (+ y (stats/sample-normal 1 :mean trans-mean :sd trans-sd))
                :z 0)
     :orientation (quat/rotate (-> pose :orientation) (stats/sample-normal 1 :mean rot-mean :sd rot-mean)))))

(defn gaussian-initialization [map num-particles pose]
  ;; Assumes a geometry_msgs.PoseWithCovarianceStamped
  (repeatedly num-particles
              #(gaussian-noise (-> pose :pose :pose) [0 0.1] [0 0.01])))


(defn saturate-map [map]
  "Saturate a map with particles (do not use this lightly)."
  (let [width (:width (get-meta map))
        height (:height (get-meta map))]
    (for [x (range width)
          y (range height)
          :when (and (even? x) (even? y)
                     (inhabitable? map x y))]
      (map->world map (pose :position (point :x x :y y))))))
