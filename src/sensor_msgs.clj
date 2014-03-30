(ns sensor-msgs
  (:require [rosclj.msg :refer :all]
            [incanter.core :refer [matrix identity-matrix]]))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; LaserScan
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defrecord LaserScan [header
                      angle-min angle-max angle-increment
                      time-increment scan-time
                      range-min range-max
                      ranges intensities])

(defmethod from-ros "sensor_msgs/LaserScan" [scan]
  (let [msg (bean-clean scan)]
    (-> msg
        (update-in [:header] from-ros)
        (update-in [:intensities] vec)
        (update-in [:ranges] vec))))

(defn laser-scan [& {:keys [header angle-min angle-max angle-increment
                            time-increment scan-time
                            range-min range-max
                            ranges intensities]
                     :or {header (std-msgs/header)
                          angle-min 0 angle-max 0 angle-increment 0
                          ranges-min 0 ranges-max 0
                          ranges [] intensities []}}]
  (->LaserScan header angle-min angle-max angle-increment time-increment scan-time range-min range-max ranges intensities))

