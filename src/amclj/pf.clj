(ns amclj.pf
  (:require [geometry-msgs :refer :all]
            [nav-msgs :refer :all]
            [incanter.core :refer :all]
            [incanter.stats :as stats]))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; OccupancyGrid
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defn inhabitable?
  "Can a robot occupy the given location in the map?"
  [map pose]
  (let [x (int (-> pose :position :x))
        y (int (-> pose :position :y))]
    (zero? (sel (-> map :data) x y))))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Quaternions
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defn mult-quaternion [qa qb]
  (let [qaw (:w qa) qax (:x qa) qay (:y qa) qaz (:z qa)
        qbw (:w qb) qbx (:x qb) qby (:y qb) qbz (:z qb)]
    (geometry-msgs/quaternion
     :x (+ (* qax qbw) (* qaw qbx) (* qay qbz) (- (* qaz qby)))
     :y (+ (* qaw qby) (- (* qax qbz)) (* qay qbw) (* qaz qbx)) 
     :z (+ (* qaw qbz) (* qax qby) (- (* qay qbx)) (* qaz qbw))
     :w (- (* qaw qbw) (* qax qbx) (* qay qby) (* qaz qbz)))))


(defn rotate-quaternion [quat heading]
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


(defn create-quaternion []
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

(defn heading->quat [heading]
  (let [initial (geometry-msgs/quaternion)]
    (rotate-quaternion initial heading)))


(defn quat->heading
  "Given a quaternion, return the heading (yaw)."
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

;;atan2(2*qy*qw-2*qx*qz , 1 - 2*qy2 - 2*qz2)
;; double sqx = q1.x*q1.x;
;; double sqy = q1.y*q1.y;
;; double sqz = q1.z*q1.z;
;; heading = atan2((2*q1.y*q1.w)-(2*q1.x*q1.z) , 1 - (2*sqy) - (2*sqz));
;;     attitude = asin(2*test);
;;     bank = atan2(2*q1.x*q1.w-2*q1.y*q1.z , 1 - 2*sqx - 2*sqz)



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
        raw-heading (reduce + (map (comp quat->heading :orientation) particles))
        normalizer (count particles)]
    (geometry-msgs/pose
     :position
     (geometry-msgs/point :x (/ (get-in raw-estimate [:position :x]) normalizer)
                          :y (/ (get-in raw-estimate [:position :y]) normalizer)
                          :y (/ (get-in raw-estimate [:position :y]) normalizer))
     :orientation
     #_(heading->quat (/ raw-heading normalizer))
     (geometry-msgs/quaternion 
      :x (/ (get-in raw-estimate [:orientation :x]) normalizer)
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

(defn calculate-control [prev-odom curr-odom]
  )


(defn update-tf [pose-cv-stamped tf time])





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
