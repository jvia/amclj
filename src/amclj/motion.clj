(ns ^{:doc "Motion sampling code"
      :author "Jeremiah Via"}
  amclj.motion
  (:require [amclj.quaternions :as quat]
            [geometry-msgs :as geo :refer [pose]]
            [nav-msgs :as nav :refer [odometry]]
            [incanter.stats :as stats]
            [incanter.core :refer [pow]]))

(def odom-type :diff)
(def alpha1 0.2)
(def alpha2 0.2)
(def alpha3 0.2)
(def alpha4 0.2)
(def alpha5 0.2)
(def frame-id "odom")
(def base-frame-id "base_footprint")
(def global-frame-id "map")

(defn- sample [var]
  (stats/sample-normal 1 :mean 0 :sd (Math/sqrt var)))

;;; For develop
(def prev-pose (geometry-msgs/pose))
(def prev-odom
  {:header
   {:seq 22547, :stamp {:secs 2254, :nsecs 800000000}, :frameId "odom"},
   :child-frame-id nil,
   :pose
   {:pose
    {:position {:x 0.24126429311053682, :y -0.02177154841439802, :z 0.0},
     :orientation
     {:x 0.0, :y 0.0, :z -0.12467473338522772, :w 0.992197667229329}},
    :covariance
    '((0.0 0.0 0.0 0.0 0.0 0.0)
      (0.0 0.0 0.0 0.0 0.0 0.0)
      (0.0 0.0 0.0 0.0 0.0 0.0)
      (0.0 0.0 0.0 0.0 0.0 0.0)
      (0.0 0.0 0.0 0.0 0.0 0.0)
      (0.0 0.0 0.0 0.0 0.0 0.0))},
   :twist
   {:twist
    {:linear {:x 0.06, :y 0.0, :z 0.0},
     :angular {:x 0.0, :y 0.0, :z 0.0}},
    :covariance
    '((0.0 0.0 0.0 0.0 0.0 0.0)
      (0.0 0.0 0.0 0.0 0.0 0.0)
      (0.0 0.0 0.0 0.0 0.0 0.0)
      (0.0 0.0 0.0 0.0 0.0 0.0)
      (0.0 0.0 0.0 0.0 0.0 0.0)
      (0.0 0.0 0.0 0.0 0.0 0.0))},
   :childFrameId ""})

(def curr-odom
  {:header
   {:seq 22580, :stamp {:secs 2258, :nsecs 100000000}, :frameId "odom"},
   :child-frame-id nil,
   :pose
   {:pose
    {:position {:x 0.25289124217106457, :y -0.0247403959254523, :z 0.0},
     :orientation
     {:x 0.0, :y 0.0, :z -0.12467473338522772, :w 0.992197667229329}},
    :covariance
    '((0.0 0.0 0.0 0.0 0.0 0.0)
      (0.0 0.0 0.0 0.0 0.0 0.0)
      (0.0 0.0 0.0 0.0 0.0 0.0)
      (0.0 0.0 0.0 0.0 0.0 0.0)
      (0.0 0.0 0.0 0.0 0.0 0.0)
      (0.0 0.0 0.0 0.0 0.0 0.0))},
   :twist
   {:twist
    {:linear {:x 0.0, :y 0.0, :z 0.0},
     :angular {:x 0.0, :y 0.0, :z 0.0}},
    :covariance
    '((0.0 0.0 0.0 0.0 0.0 0.0)
      (0.0 0.0 0.0 0.0 0.0 0.0)
      (0.0 0.0 0.0 0.0 0.0 0.0)
      (0.0 0.0 0.0 0.0 0.0 0.0)
      (0.0 0.0 0.0 0.0 0.0 0.0)
      (0.0 0.0 0.0 0.0 0.0 0.0))},
   :childFrameId ""})

(def control
  [curr-odom prev-odom])

(def p (-> curr-odom :pose :pose))
(let [{position :position orientation :orientation} p]
  [position orientation])
;;; End develop



(defn sample-motion-model-odometry*
  [pose
   {position' :position orientation' :orientation}
   {position  :position orientation  :orientation}]
  (let [;; pull out translation & heading
        {x' :x y' :y z' :z} position'
        theta' (quat/to-heading orientation')
        {x :x y :y z :z} position
        theta (quat/to-heading orientation)
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
     (let [curr-pose (-> curr-odom :pose :pose)
           prev-pose (-> prev-odom :pose :pose)]
       (sample-motion-model-odometry* pose curr-pose prev-pose))))
