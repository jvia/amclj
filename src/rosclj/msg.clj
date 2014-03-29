;;Namespace to hold general message utilities.
;;
;;WARNING: This is an experiment for how auto-message generation will
;;work. It will eventually be remove
(ns rosclj.msg)

(defn from-ros-dispatch
  "rosjava message dynamically implement the message interface using a
  MessageImpl backing class. We need to do some massaging to get out
  some data on which we can dispatch."
  [msg]
  (let [class (class msg)]
    (cond
     (isa? org.ros.message.Time class)     'org.ros.message.Time
     (isa? org.ros.message.Duration class) 'org.ros.message.Duration
     :else(.getType (.getInstance msg)))))

(defmulti from-ros from-ros-dispatch)

(defmethod from-ros :default [params]
  (throw (IllegalArgumentException.
          (str "No dispatch for " params))))

(defn to-ros-dispatch [publisher msg] (type msg))

(defmulti to-ros to-ros-dispatch)

(defn bean-clean
  "Convert message to a bean and remove useless information"
  [msg]
  (dissoc (bean msg) :class :instance))

