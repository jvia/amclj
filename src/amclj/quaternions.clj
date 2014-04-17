(ns ^{:doc "Utilities for working with quaternions"}
  amclj.quaternions
  (:require [geometry-msgs :refer [quaternion]]
            [taoensso.timbre :as log]))


(defn mult
  "Multiply two quaternions together."
  [qa qb]
  (let [qaw (:w qa) qax (:x qa) qay (:y qa) qaz (:z qa)
        qbw (:w qb) qbx (:x qb) qby (:y qb) qbz (:z qb)]
    (quaternion
     :x (+ (* qax qbw) (* qaw qbx) (* qay qbz) (- (* qaz qby)))
     :y (+ (* qaw qby) (- (* qax qbz)) (* qay qbw) (* qaz qbx)) 
     :z (+ (* qaw qbz) (* qax qby) (- (* qay qbx)) (* qaz qbw))
     :w (- (* qaw qbw) (* qax qbx) (* qay qby) (* qaz qbz)))))


(defn rotate
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
    (mult (quaternion
           :w (- (* c1c2 c3) (* s1s2 s3))
           :x (+ (* c1c2 s3) (* s1s2 c3))
           :y (- (* c1 s2 c3) (* s1 c2 s3))
           :z (+ (* s1 c2 c3) (* c1 s2 s3)))
          quat)))


(defn unit
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
    (quaternion
     :w (- (* c1c2 c3) (* s1s2 s3))
     :x (+ (* c1c2 s3) (* s1s2 c3))
     :y (+ (* s1 c2 c3) (* c1 s2 s3))
     :z (- (* c1 s2 c3) (* s1 c2 s3)))))

(defn from-heading
  "Given a heading (yaw) in radians, convert it to a quaternion."
  [heading]
  
  (let [initial (quaternion)]
    (rotate initial heading)))

(defn from-rpy [roll pitch yaw]
  (let [half-yaw  (* 0.5 yaw)
        half-pitch (* 0.5 pitch)
        half-roll (* 0.5 roll)
        cosYaw (Math/cos half-yaw)
        sinYaw (Math/sin half-yaw)
        cosPitch (Math/cos half-pitch)
        sinPitch (Math/sin half-pitch)
        cosRoll (Math/cos half-roll)
        sinRoll (Math/sin half-roll)]
    (quaternion
     :x (- (* sinRoll cosPitch cosYaw) (* cosRoll sinPitch sinYaw))
     :y (+ (* cosRoll sinPitch cosYaw) (* sinRoll cosPitch sinYaw))
     :z (- (* cosRoll cosPitch sinYaw) (* sinRoll sinPitch cosYaw))
     :w (+ (* cosRoll cosPitch cosYaw) (* sinRoll sinPitch sinYaw)))))



(defn to-heading
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
     (Math/atan2 (-   (* 2 qz qw) (* 2 qx qy))
                 (- 1 (* 2 qz qz) (* 2 qy qy))))))
