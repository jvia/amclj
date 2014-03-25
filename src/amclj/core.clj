(ns amclj.core
  (:import org.ros.concurrent.CancellableLoop
           org.ros.namespace.GraphName
           [org.ros.node AbstractNodeMain ConnectedNode NodeMain NodeConfiguration DefaultNodeMainExecutor]
           org.ros.node.topic.Publisher))





(defn foo
  "I don't do a whole lot."
  [x]
  (println x "Hello, World!"))
