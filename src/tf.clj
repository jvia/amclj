(ns tf
  (:require [rosclj.msg :refer :all]
            [incanter.core :refer [matrix identity-matrix]]))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; tfMessage
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defrecord tfMessage [transforms])

(defmethod from-ros "tf/tfMessage" [tf]
  (-> (bean-clean tf)
      (update-in [:transforms] #(pmap from-ros %))))

(defn tf [& {:keys [transforms] :or {transforms '()}}]
  (->tfMessage transforms))


