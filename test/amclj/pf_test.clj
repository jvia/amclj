(ns amclj.pf-test
  (:require [midje.sweet :refer :all]
            [amclj.pf :refer :all]
            [geometry-msgs :as geo]))

(facts
 "about quaternion multiplication"
 (fact
  (mult-quaternion (geo/quaternion) (geo/quaternion)) => (geo/quaternion)))

(facts "about quaternion rotation")

(facts "about conversions between quaternion & euler headings")



;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Pose Arithmetic
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(facts "about poses"
       (fact "adding poses does 'the right thing'"
             (add-pose (geo/pose :position (geo/point :x 0 :y 0 :z 0)
                                 :orientation (geo/quaternion :x 0 :y 0 :z 0 :w 1))
                       (geo/pose :position (geo/point :x 1 :y 1 :z 1)
                                 :orientation (geo/quaternion :x 0 :y 0 :z 0 :w 1)))
             => (geo/pose :position (geo/point :x 1 :y 1 :z 1)
                                 :orientation (geo/quaternion :x 0 :y 0 :z 0 :w 2)))
       )
