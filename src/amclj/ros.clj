(ns amclj.ros
  (:require [clojure.reflect :as reflect]
            [clojure.pprint :as pp]
            [clojure.string :as str])
  (:import org.ros.concurrent.CancellableLoop
           org.ros.namespace.GraphName
           [org.ros.node AbstractNodeMain ConnectedNode NodeMain NodeConfiguration DefaultNodeMainExecutor]
           [org.ros.node.topic Publisher Subscriber]))

(defn string-msg
  ([] (string-msg nil))
  ([msg]
     (let [data (atom msg)]
       (proxy [std_msgs.String] []
         (getData [] @data)
         (setData [newdata] (reset! data newdata))))))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Node creation
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defprotocol INode
  (start [_] "Start the node")
  (stop [_] "Stop the node")
  (running? [_] "Check if the node is currently running"))

(defrecord RosNode
    [name connection configuration
     executor nodemain publishers subscribers])

(defn- connection
  "Get the connection of a node"
  [node]
  @(:connection node))

(defn rosnode [name]
  (let [conn (atom nil)
        conf (NodeConfiguration/newPrivate)
        exec (DefaultNodeMainExecutor/newDefault)
        main (proxy [AbstractNodeMain] []
               (getDefaultNodeName [] (GraphName/of name))
               (onStart [connectedNode]
                 (reset! conn connectedNode)))
        rosnode (->RosNode name conn conf exec main {} {})]
    rosnode))

(extend-type RosNode
  INode
  (start [node]
    (let [executor (:executor node)
          nodemain (:nodemain node)
          configuration (:configuration node)]
      (. executor execute  nodemain configuration)
      node))
  (stop [node]
    (let [executor (:executor node)
          nodemain (:nodemain node)]
      (. executor shutdownNodeMain nodemain))
    node)
  (running? [node] (-> node :connection deref nil? not)))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Node communication
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defprotocol IPublish
  (publish [node topic msg] "Publish a message")
  (add-publisher [node topic type] "Add a new publisher.")
  (new-message [node topic] "Creates a new message to publish on the given topic"))


(defn- add-publisher* [node topic typestr]
  (when-not (running? node)
    (throw (IllegalStateException. (str "Node " (:name node) " is not running"))))
  (let [connection (connection node)
        publisher (.newPublisher connection topic typestr)]
    (update-in node [:publishers] assoc topic publisher)))

;; (defn- restr
;;   "Perform a string version of rest"
;;   [string]
;;   (apply str (rest string)))

;; (defn- javaize [name]
;;   (let [m (-> name str restr)]
;;     (str ".set" (str/capitalize m))))

;; (defmacro rosmsg [publisher data]
;;     `(doto (.newMessage ~publisher)
;;        ~@(for [[m v] (zipmap (map (comp symbol javaize key) data)
;;                              (map val data))]
;;            (list m v))))

(defn new-message* [node topic]
  (if-let [publisher (get-in node [:publishers topic])]
    (.newMessage publisher)
    (throw (IllegalArgumentException. (str "No publisher for " topic)))))

(defn- publish* [node topic msg]
  (if-let [publisher (get-in node [:publishers topic])]
    (.publish publisher msg)
    (throw (IllegalArgumentException. (str "No publisher for " topic)))))


(extend-type RosNode
  IPublish
  (publish [node topic msg] (publish* node topic msg))
  (add-publisher [node topic type] (add-publisher* node topic type))
  (new-message [node topic] (new-message* node topic)))


(defprotocol ISubscribe
  (subscribe [_ topic type callback-fn]))
