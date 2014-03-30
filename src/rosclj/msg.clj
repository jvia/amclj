;;Namespace to hold general message utilities.
;;
;;WARNING: This is an experiment for how auto-message generation will
;;work. It will eventually be remove
(ns rosclj.msg
  (:import org.ros.internal.message.definition.MessageDefinitionReflectionProvider
           org.ros.message.MessageDefinitionProvider
           org.ros.internal.message.DefaultMessageFactory))

(defn bean-clean
  "Convert message to a bean and remove useless information"
  [msg]
  (dissoc (bean msg) :class :instance))

(defn make-definition-provider
  "Create a message definition provider."
  [msg-type msg-definition]
  (proxy [MessageDefinitionProvider] []
    (get [arg] (if (= arg msg-type) msg-definition
                   (throw (IllegalArgumentException. (str "No definition for" msg-type)))))))

(defmacro with-msg-factory [[name type definition] & body]
  `(let [provider# (proxy [MessageDefinitionProvider] []
                     (get [arg#]
                       (if (= arg# ~type) ~definition
                           (throw (IllegalArgumentException.
                                   (str "No definition for " ~type))))))
         ~name (DefaultMessageFactory. provider#)]
     ~@body))
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; from-ros
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
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
          (str "No dispatch for " (type params)))))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; to-ros
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defn to-ros-dispatch [msg]
  (keyword (clojure.string/replace (str (type msg))  "class " "")))

(defmulti to-ros to-ros-dispatch)

(defmethod to-ros :default [msg]
  (throw (IllegalArgumentException.
          (str "No dispatch for " (type msg)))))




