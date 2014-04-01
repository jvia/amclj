(ns nav-msgs
  (:require [rosclj.msg :refer :all]
            [incanter.core :refer [matrix identity-matrix]]))


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; MapMetaData
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defrecord MapMetaData [map-load-time resolution width height pose])

(defmethod from-ros "nav_msgs/MapMetaData" [map-meta]
  (map->MapMetaData
   (-> (bean-clean map-meta)
       (update-in [:mapLoadTime] from-ros)
       (update-in [:origin] from-ros))))

(defn map-meta-data [& {:keys [map-load-time resolution width height pose]
                        :or {time (std-msgs/time) pose (geometry-msgs/pose)
                             resolution 0 widht 0 height 0}}]
  (->MapMetaData map-load-time resolution width height pose))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; OccupancyGrid
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defrecord OccupancyGrid [header info data])

(defmethod from-ros "nav_msgs/OccupancyGrid" [map]
  (let [rows (.getHeight (.getInfo map))
        cols (.getWidth (.getInfo map))]
    (map->OccupancyGrid
     (-> (bean-clean map)
         (update-in [:header] from-ros)
         (update-in [:info] from-ros)
         (update-in [:data] #(let [raw-data % #_(.getData %)
                                   size (.readableBytes raw-data)
                                   array (byte-array size)
                                   _ (.readBytes raw-data array)]
                               (matrix (vec array) cols)))))))

(defn occupancy-grid [& {:keys [header info data]
                         :or {header (std-msgs/header)
                              info (map-meta-data)
                              data nil}}]
  (->OccupancyGrid header info data))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Odometry
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defrecord Odometry [header child-frame-id pose twist])

(defmethod from-ros "nav_msgs/Odometry" [odom]
  (map->Odometry
   (-> (bean-clean odom)
       (update-in [:header] from-ros)
       (update-in [:pose] from-ros)
       (update-in [:twist] from-ros))))

(defn odometry [& {:keys [header child-frame-id pose twist]
                   :or {header (std-msgs/header) child-frame-id ""
                        pose (geometry-msgs/pose-with-covariance)
                        twist (geometry-msgs/twist-with-covariance)}}]
  (->Odometry header child-frame-id pose twist))


