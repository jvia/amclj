(ns amclj.pf
  (:require [incanter.stats :as stats]))

(defn initialize-pf [map & {:keys [pose]}])

(defn motion-update [particles motion]
  particles)

(defn sensor-update [particles sensor]
  particles)

(defn mcl [particles motion sensor]
  (-> particles
      (motion-update motion)
      (sensor-update sensor)))

