;; (ns user
;;   (:require
;;    [clojure.tools.namespace.repl :refer (refresh)]
;;    [amclj.core :refer :all]
;;    [amclj.ros :refer :as ros]))

;; (def node nil)

;; (defn init []
;;   (alter-var-root
;;    #'node
;;    (constantly
;;     (rosnode "amcl"))))

;; (defn start []
;;   (alter-var-root #'node start))

;; (defn stop []
;;   (alter-var-root #'node stop))

;; (defn go []
;;   (init)
;;   (start))

;; (defn reset []
;;   (stop)
;;   (refresh :after 'user/go))
