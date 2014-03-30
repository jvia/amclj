(ns amclj.pf
  (:require [geometry-msgs]
[incanter.core :refer :all]
            [incanter.stats :as stats]))

(defn inhabitable?
  "Can a robot occupy the given location in the map?"
  [map pose]
  (let [x (int (-> pose :position :x))
        y (int (-> pose :position :y))]
    (zero? (sel (-> map :data) x y))))


(defn mult-quaternion [qa qb]
  (let [qaw (:w qa) qax (:x qa) qay (:y qa) qaz (:z qa)
        qbw (:w qb) qbx (:x qb) qby (:y qb) qbz (:z qb)]
    (geometry-msgs/quaternion
     :x (+ (* qax qbw) (* qaw qbx) (* qay qbz) (- (* qaz qby)))
     :y (+ (* qaw qby) (- (* qax qbz)) (* qay qbw) (* qaz qbx)) 
     :z (+ (* qaw qbz) (* qax qby) (- (* qay qbx)) (* qaz qbw))
     :w (- (* qaw qbw) (* qax qbx) (* qay qby) (* qaz qbz)))))


(defn rotate-quaternion [quat heading]
  (let [c1 (Math/cos (/ heading 2))
        s1 (Math/sin (/ heading 2))
        c2 1.0, s2 0.0, c3 1.0 s3 1.0
        c1c2 (* c1 c2)
        s1s2 (* s1 s2)]
    (mult-quaternion
     (geometry-msgs/quaternion :w (- (* c1c2 c3) (* s1s2 s3))
                               :x (+ (* c1c2 s3) (* s1s2 c3))
                               :y (- (* c1 s2 c3) (* s1 c2 s3))
                               :z (+ (* s1 c2 c3) (* c1 s2 s3)))
     quat)))


(defn random-pose
  "Create a pose uniformly sampled from the grid defined by a grid."
  ([width height] (random-pose 0 width 0 height))
  ([min-x max-x min-y max-y]
     (let [[x] (stats/sample-uniform 1 :min min-x :max max-x)
           [y] (stats/sample-uniform 1 :min min-y :max max-y)]
       (geometry-msgs/pose 
        :position (geometry-msgs/point :x x :y y)
        :orientation (rotate-quaternion (geometry-msgs/quaternion :x 0 :y 0 :z 0 :w 1)
                                        (stats/sample-normal 1))))))




(def num-particles 100)


(defn uniform-initialization [map num-paticles]
  (let [height (-> map :info :height)
        width (-> map :info :width)]
    (loop [particles []]
      (if (= num-paticles (count particles))
        (geometry-msgs/pose-array :poses particles)
        (let [proposal (random-pose width height)]
          (if (inhabitable? map proposal)
            (recur (conj particles proposal))
            (recur particles)))))))

(defn gaussian-initialization [map pose])

(defn initialize-pf [map & {:keys [pose]}]
  (if (nil? pose)
    (uniform-initialization map)
    (gaussian-initialization map pose)))

(defn add-pose [a b & rest]
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

(defn pose-estimate [particles]
  (let [raw-estimate (apply add-pose particles)
        normalizer (count particles)]
    (geometry-msgs/pose
     :position
     (geometry-msgs/point :x (/ (get-in raw-estimate [:position :x]) normalizer)
                          :y (/ (get-in raw-estimate [:position :y]) normalizer)
                          :y (/ (get-in raw-estimate [:position :y]) normalizer))
     :orientation
     (geometry-msgs/quaternion :x (/ (get-in raw-estimate [:orientation :x]) normalizer)
                               :y (/ (get-in raw-estimate [:orientation :y]) normalizer)
                               :z (/ (get-in raw-estimate [:orientation :z]) normalizer)
                               :w (/ (get-in raw-estimate [:orientation :w]) normalizer)))))


(defn beam-range-finder-model [])
(defn likelihood-field-range-finder-model [])
(defn sample-motion-model [control particle]
  )

(defn measurement-model [measurement particle map])

(defn mcl [particles control measurement map]
  (let [prediction (map #(sample-motion-model control %) particles)
        update (map #(measurement-model measurement % map) particles)]
    (for [[particle weight] update]
      (if (>= weight (Math/random))
        particle))))

(defn amcl [particles control measurement map])

(defn kld-mcl [particles control measurement map epsilon delta])

(defn calculate-control [prev-tf curr-tf])
