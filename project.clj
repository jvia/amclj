(defproject amclj "0.1.0-SNAPSHOT"
  :description "FIXME: write description"
  :url "http://example.com/FIXME"
  :license {:name "Eclipse Public License"
            :url "http://www.eclipse.org/legal/epl-v10.html"}
  :dependencies [[org.clojure/clojure "1.5.1"]
                 [org.clojure/core.async "0.1.278.0-76b25b-alpha"]
                 [incanter "1.5.4"]
                 ;;[org.ros.rosjava_core/rosjava "0.1.6"]
                 [ros.rosjava_core/rosjava_bootstrap "0.0.0-SNAPSHOT"]
                 [ros.rosjava_core/rosjava "0.0.0-SNAPSHOT"]
                 [ros.rosjava_core/rosjava_messages "0.0.0-SNAPSHOT"]
                 [midje "1.6.2"]
                 [com.taoensso/timbre "3.1.6"]]
  :profiles {:dev {:dependencies [[org.clojure/tools.namespace "0.2.4"]]
                   :source-paths ["dev"]}}
  :repositories [["rosjava" "https://github.com/rosjava/rosjava_mvn_repo/raw/master"]])
