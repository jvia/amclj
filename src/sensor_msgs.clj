(ns sensor-msgs
  (:require [rosclj.msg :refer :all]
            [incanter.core :refer [matrix identity-matrix]]))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;g
;; LaserScan
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defrecord LaserScan [header
                      angleMin angleMax angleIncrement
                      timeIncrement scanTime
                      rangeMin rangeMax
                      ranges intensities])

(defmethod from-ros "sensor_msgs/LaserScan" [scan]
  (map->LaserScan
   (let [msg (bean-clean scan)]
     (-> msg
         (update-in [:header] from-ros)
         (update-in [:intensities] vec)
         (update-in [:ranges] vec)))))

(with-msg-factory [f "sensor_msgs/LaserScan"
                   (str "Header header\n"
                        "float32 angle_min\n" "float32 angle_max\n" "float32 angle_increment\n"
                        "float32 time_increment\n" "float32 scan_time\n"
                        "float32 range_min\n" "float32 range_max\n"
                        "float32[] ranges\n"
                        "float32[] intensities\n")]
  (defmethod to-ros :sensor_msgs.LaserScan [laser]
    (doto (.newFromType f "sensor_msgs/LaserScan")
      (.setHeader (-> laser :header to-ros))
      (.setAngleMin (-> laser :angleMin))
      (.setAngleMax (-> laser :angleMax))
      (.setAngleIncrement (-> laser :angleIncrement))
      (.setTimeIncrement (-> laser :timeIncrement))
      (.setScanTime (-> laser :scanTime))
      (.setRangeMin (-> laser :rangeMin))
      (.setRangeMax (-> laser :rangeMax))
      (.setRanges (-> laser :ranges float-array))
      (.setIntensities (-> laser :intensities float-array)))))

(defn laser-scan [& {:keys [header angleMin angleMax angleIncrement
                            timeIncrement scanTime
                            rangeMin rangeMax
                            ranges intensities]
                     :or {header (std-msgs/header)
                          angleMin 0 angleMax 0 angleIncrement 0
                          timeIncrement 0 scanTime 0
                          rangeMin 0 rangeMax 0
                          ranges [] intensities []}}]
  (->LaserScan header angleMin angleMax angleIncrement timeIncrement scanTime rangeMin rangeMax ranges intensities))

