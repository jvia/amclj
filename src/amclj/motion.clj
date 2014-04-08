(ns ^{:doc "Motion sampling code"
      :author "Jeremiah Via"}
  amclj.motion
  (:require [amclj.quaternions :as quat]
            [geometry-msgs :as geo :refer [pose]]
            [nav-msgs :as nav :refer [odometry]]
            [incanter.stats :as stats]
            [incanter.core :refer [pow]]
            [taoensso.timbre :as log]))

(def odom-type :diff)
(def alpha1 0.02)
(def alpha2 0.02)
(def alpha3 0.02)
(def alpha4 0.02)
(def alpha5 0.02)
(def frame-id "odom")
(def base-frame-id "base_footprint")
(def global-frame-id "map")

(defn- sample [var]
  (stats/sample-normal 1 :mean 0 :sd (Math/sqrt var)))

(defn sample-motion-model-odometry*
  [pose [x' y' theta'] [x y theta]]
  (let [;; pull out translation & heading
        p-theta (quat/to-heading (-> pose :orientation))
        ;; recover relative motion parameters from odom
        rot1  (- (Math/atan2 (- y' y) (- x' x)) theta)
        trans (Math/hypot (- x x') (- y y'))
        rot2  (- theta' theta rot1)
        ;; recover relative motion parameters for previous pose
        rot1*  (- rot1  (sample (+ (* alpha1 (pow rot1 2)) (* alpha2 (pow rot2 2)))))
        trans* (- trans (sample (+ (* alpha3 (pow trans 2)) (* alpha4 (pow rot1 2)) (* alpha4 (pow rot2 2)))))
        rot2*  (- rot2  (sample (+ (* alpha1 (pow rot2 2)) (* alpha2 (pow trans 2)))))]
    (-> pose
        (update-in [:position :x]
                   #(+ % (* trans* (Math/cos (+ p-theta rot1*)))))
        (update-in [:position :y]
                   #(+ % (* trans* (Math/sin (+ p-theta rot1*)))))
        (update-in [:orientation]
                   #(quat/from-heading
                     (+ (quat/to-heading %) rot1* rot2*))))))

#_(sample-motion-model-odometry prev-pose control)

;; public facing algo, does some data massaging to make the algorithm easier.
(defn sample-motion-model-odometry
  ([pose control-vec]
     (sample-motion-model-odometry pose (first control-vec) (second control-vec)))
  ([pose curr-odom prev-odom]
     (sample-motion-model-odometry*
        pose
        [(-> curr-odom :twist :twist :linear :x)
         (-> curr-odom :twist :twist :linear :y)
         (-> curr-odom :twist :twist :angular :z)]
        [(-> prev-odom :twist :twist :linear :x)
         (-> prev-odom :twist :twist :linear :y)
         (-> prev-odom :twist :twist :angular :z)])))
