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

(with-msg-factory [f "sensor_msgs/LaserScan"
                   (str "Header header\n"
                        "float32 angle_min\n"
                        "float32 angle_max\n"
                        "float32 angle_increment\n"
                        "float32 time_increment\n"
                        "float32 scan_time\n"
                        "float32 range_min\n"
                        "float32 range_max\n"
                        "float32[] ranges\n"
                        "float32[] intensities\n")]
  (defmethod to-ros :sensor_msgs.LaserScan [laser]
    (doto (.newFromType f "sensor_msgs/LaserScan")
      (.setHeader (-> laser :header to-ros))
      (.setAngleMin (-> laser :angle-min))
      (.setAngleMax (-> laser :angle-max))
      (.setAngleIncrement (-> laser :angle-increment))
      (.setTimeIncrement (-> laser :time-increment))
      (.setScanTime (-> laser :scan-time))
      (.setRangeMin (-> laser :range-min))
      (.setRangeMax (-> laser :range-max))
      (.setRanges (-> laser :ranges float-array))
      (.setIntensities (-> laser :intensities float-array)))))

(defn laser-scan [& {:keys [header angle-min angle-max angle-increment
                            time-increment scan-time
                            range-min range-max
                            ranges intensities]
                     :or {header (std-msgs/header)
                          angle-min 0 angle-max 0 angle-increment 0
                          time-increment 0 scan-time 0
                          range-min 0 range-max 0
                          ranges [] intensities []}}]
  (->LaserScan header angle-min angle-max angle-increment time-increment scan-time range-min range-max ranges intensities))

