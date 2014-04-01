(ns amclj.ros
  (:require [clojure.string :as str]
            [rosclj.msg :refer [to-ros from-ros]])
  (:import org.ros.concurrent.CancellableLoop
           org.ros.namespace.GraphName
           [org.ros.node AbstractNodeMain ConnectedNode NodeMain NodeConfiguration DefaultNodeMainExecutor]
           [org.ros.node.topic Publisher Subscriber]))
#_(methods from-ros)
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

(defn rostype
  "Return the ROS message type from a message class."
  [rosmsg]
  (-> rosmsg str (str/replace "." "/") (str/replace "interface " "")))

(defn- connection
  "Get the connection of a node"
  [node]
  @(:connection node))

(defn rosnode
  "Create a ROS node of the given name. It is created but not yet
  connected and has no publishers or subscribers."
  [name]
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
      (assoc node :running true)))
  (stop [node]
    (let [executor (:executor node)
          nodemain (:nodemain node)]
      (. executor shutdownNodeMain nodemain))
    (reset! (:connection node) nil))
  (running? [node] (-> node :connection deref nil? not)))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Node communication
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defprotocol IPublish
  (publish [node topic msg] "Publish a message")
  (add-publisher [node topic type] "Add a new publisher.")
  (new-message [node topic] "Creates a new message to publish on the given topic"))

(defn- restr
  "Perform a string version of rest"
  [string]
  (apply str (rest string)))

(defn- javaize [name]
  (let [m (-> name str restr)]
    (str ".set" (str/capitalize m))))

(defmacro rosmsg [publisher data]
  `(#(doto (.newMessage %)
       ~@(for [[m v] (zipmap (map (comp symbol javaize key) data)
                             (map val data))]
           (list m v))) ~publisher))

(defn- add-publisher* [node topic typestr]
  (when-not (running? node)
    (throw (IllegalStateException. (str "Node " (:name node) " is not running"))))
  (let [connection (connection node)
        publisher (.newPublisher connection topic typestr)]
    (update-in node [:publishers] assoc topic publisher)))

(defn- new-message* [node topic]
  (if-let [publisher (get-in node [:publishers topic])]
    (.newMessage publisher)
    (throw (IllegalArgumentException. (str "No publisher for " topic)))))

(defn- publish* [node topic msg]
  (if-let [publisher (get-in node [:publishers topic])]
    (.publish publisher (to-ros msg))
    (throw (IllegalArgumentException. (str "No publisher for " topic)))))

(extend-type RosNode
  IPublish
  (publish [node topic msg] (publish* node topic msg))
  (add-publisher [node topic type] (add-publisher* node topic (rostype type)))
  (new-message [node topic] (new-message* node topic)))


(defprotocol ISubscribe
  (subscribe [node topic type callback-fn]
    "Subcribe to the given topic of type with a callback-fn. The
     callback-fn must be of one parameter."))

(defn- msg-listener [f]
  (proxy [org.ros.message.MessageListener] []
    (onNewMessage [msg]
      (f (from-ros msg)))))

(defn- subscribe* [node topic type callback-fn]
  (let [subscriber (or (get-in node [:subscribers topic])
                       (.newSubscriber (connection node) topic type))
        msg-fn (msg-listener callback-fn)
        ;; msg-fn (proxy subscriber [org.ros.message.MessageListener] []
        ;;               (onNewMessage [msg]
        ;;                 (callback-fn msg)))
        ]
    (.addMessageListener subscriber msg-fn)
    (update-in node [:subscribers] assoc topic subscriber)))

(extend-type RosNode
  ISubscribe
  (subscribe [node topic type callback-fn]
    (subscribe* node topic (rostype type) callback-fn)))


(defn ready? [node]
  (not (nil? @(:connection node))))

(defn wait-until-connected [node]
  (loop []
    (when-not (ready? node)
      (Thread/sleep 100)
      (recur))))
