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

(defprotocol IPublish
  (publish [_ topic type msg]))

(defprotocol ISubscribe
  (subscribe [_ topic type callback-fn ]))

(defprotocol INode
  (start [_] "Start the node")
  (stop [_] "Stop the node")
  (running? [_] "Check if the node is currently running"))

(defrecord RosNode
    [name connection configuration
     executor nodemain publishers subscribers]
  INode
  (start [node]
        (let [executor (:executor node)
              nodemain (:nodemain node)
              configuration (:configuration node)]
      (. executor execute  nodemain configuration)))
  (stop [node]
    (let [executor (:executor node)
          nodemain (:nodemain node)]
      (. executor shutdownNodeMain nodemain)))
  (running? [node] (-> node :connection deref nil? not)))

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

