(ns ^{:doc "Sensor sampling code"}
  amclj.sensor
  (:require [amclj.quaternions :as quat]
            [amclj.map :refer [map->world world->map occupied? uninhabitable?]]
            [geometry-msgs :as geo :refer [pose]]
            [nav-msgs :as nav :refer [odometry]]
            [incanter.stats :as stats]
            [incanter.core :refer [pow]]
            [taoensso.timbre :as log]))

(def min-range -1.0)
(def max-range -1.0)
(def max-beams 30)
(def z-hit 0.95)
(def z-short 0.1)
(def z-max 0.05)
(def z-rand 0.05)
(def sigma-hit 0.2)
(def lambda-short 0.1)
(def likelihood-max-dist 2.0)
(def model-type :likelihood-field)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Begin develop

(def scan
  {:header {:seq 35883, :stamp {:secs 3588, :nsecs 400000000},:frameId "base_laser_link"},
   :angleMin -1.5707964,
   :angleMax 1.5707964,
   :angleIncrement 0.006295777,
   :timeIncrement 0.0,
   :scanTime 0.0,
   :rangeMin 0.0,
   :rangeMax 5.6,
   :ranges [3.8 3.8000753 3.7502973 3.750669 3.7511895 3.701834 3.7026415
            3.7035959 3.7046978 3.655867 3.6071465 3.6086502 3.6102984
            3.5619233 3.5638344 3.565889 3.5680873 3.5704303 3.522595 3.5251908
            3.4775312 3.4803739 3.4833593 3.4864883 3.4897614 3.4425533 3.4460647
            3.4497204 3.4027336 3.4066215 3.4106536 3.414831 3.4191542 3.3725252
            3.377074 3.3817694 3.3353 3.3402157 3.2938128 3.298944 3.3042226
            3.3096492 3.3152254 3.3209524 3.2748494 3.280787 3.286877 3.2408483
            3.247145 3.2011178 3.2076166 3.2142694 3.221078 3.2280436 3.235168
            3.1892977 3.1966224 3.204108 3.2117562 3.1659095 3.1199634 3.127839
            3.1358795 3.1440861 3.1524613 3.1065066 3.115074 3.1238132 3.132726
            3.1418145 3.1510813 3.1050808 3.114542 3.124185 3.0780485 3.0878844
            3.0979066 3.1081176 3.1185203 3.129117 3.0828218 3.093616 3.1046097
            3.1158056 3.0692956 3.0806906 3.0922935 3.0455396 3.0573416 2.6561747
            2.666761 2.6775386 2.688511 2.6996808 2.6508062 2.6621246 2.6736465
            2.624344 2.6360123 2.6478903 2.6599817 2.6722903 2.6223829 2.6348417
            2.647525 2.660437 2.6099253 2.6229894 2.6362898 2.6498313 2.6636188
            2.6123486 2.6262944 2.640495 2.654956 2.6696827 2.6846807 2.6324568
            2.647626 2.6630769 2.6788163 2.6948502 2.7111852 2.6578841 2.6744075
            2.691245 2.666107 2.649738 2.7029798 2.686796 2.6709101 2.6553159 2.7076988
            2.6922832 2.6771474 2.728843 2.713885 2.6991966 2.6847723 2.6706064
            2.7214916 2.7074943 2.6937466 2.6802435 2.7304797 2.7171416 2.7040398
            2.6911697 2.740819 2.7281113 2.715628 2.703365 2.7524848 2.7403822 2.7891262
            2.7771873 2.7654595 2.7539392 2.7426224 2.8502674 2.8388736 2.8276823
            2.81669 2.8058934 2.8535252 2.8428943 2.8324544 2.8797982 2.8695257
            2.8594391 2.9065266 2.896609 3.3963215 3.4414961 3.4302762 3.475318
            3.464334 3.4535558 3.4985123 3.1004176 3.0912251 3.0822086 3.0733662
            3.0646954 3.110769 3.1022863 3.0939722 3.139962 3.1318386 3.1238809
            3.1160865 3.108454 3.2079115 3.2003448 3.19294 3.1856952 3.1786091
            3.1716797 3.2176542 3.2109237 3.204348 3.2503507 3.2439775 3.2377574
            3.335937 3.3818564 3.3758082 3.3699148 3.467688 3.461929 3.4563258
            3.5023828 3.4970086 3.4917889 3.5892737 3.5842156 3.5793135 3.5745664
            3.569973 3.5655327 3.6121192 3.6587386 3.6546378 3.7013948 3.748199 3.7444558 3.740868 3.7879412 3.7846181 3.7814507 3.8791966 3.8762624 3.9740958 3.9714086 4.6722283 4.669443 4.6668463 4.714592 4.7123456 4.7603974 4.858688 4.9070244 4.9054675 4.9041057 5.002999 5.0020075 5.051226 5.150638 5.1502295 5.1150255 5.1150255 5.1652303 5.16564 5.166254 5.2170935 5.318188 5.3694954 5.3709865 5.372691 5.474789 5.6 5.6 5.6 5.6 5.6 5.6 5.6 5.6 5.6 5.6 5.6 5.6 5.6 5.6 5.6 5.6 5.6 5.6 5.6 5.6 5.6 5.6 5.6 5.6 5.6 5.6 5.6 5.6 5.6 5.6 5.6 5.6 5.6 5.6 5.6 5.6 5.6 5.6 5.6 5.6 5.6 5.6 5.6 5.6 5.6 5.6 5.6 5.6 5.6 5.6 5.6 5.6 5.6 5.6 5.6 5.6 5.6 5.6 5.6 5.6 5.6 5.6 5.6 5.6 5.6 5.6 5.6 5.6 5.6 5.6 5.6 5.6 5.6 5.6 5.6 5.6 5.6 5.6 5.6 5.6 5.6 4.1139336 4.1309085 4.088071 3.9846601 3.759257 3.532048 3.2418156 0.61440116 0.61718816 0.6200253 0.62291336 0.6258535 0.62884647 0.5055147 0.5079962 0.51052254 0.5130946 0.5157132 0.51837933 0.52109396 0.523858 0.52667254 0.46334624 0.46590003 0.4685008 0.47114956 0.4738473 0.47659513 0.47939414 0.48224545 0.4851502 0.48810965 0.491125 0.49575564 0.49265403 0.48961022 0.48662302 0.48369107 0.48081318 0.47798818 0.4752149 0.47249225 0.46981913 0.46719447 0.4646173 0.46208665 0.45960152 0.457161 0.4547642 0.45241022 0.45009825 0.44782746 0.44559702 0.44340616 0.44125417 0.4391403 0.4370638 0.4971703 0.49488032 0.49263075 0.4904209 0.48825005 0.4861175 0.48402265 0.48196477 0.47994325 0.4779575 5.02286 5.0145664 5.0658 5.094521 5.088718 4.9667573 4.9043365 4.8261585 4.7671647 4.766637 4.709383 4.6996093 4.702586 4.649475 4.7103906 4.711187 4.66225 4.670335 4.680273 4.635986 4.703721 4.716652 4.730421 4.836489 5.480374 5.465042 5.341011 5.3265724 5.3124213 4.8660197 4.7996154 4.78753 4.775694 4.6570463 4.6459565 4.5818253 4.5713267 4.5610566 4.5510125 4.4355826 4.426207 4.4170456 4.3556194 4.294611 4.2862835 4.2259846 4.218155 4.158539 4.1511903 4.0922313 4.085348 4.0270214 4.020588 4.014334 3.9568708 3.9510477 3.8941586 3.888752 3.883514 3.8784435 3.8735394 3.9197066 4.067608 4.0629654 4.0584946 3.952839 3.948811 3.9449475 3.9412477 3.9377105 3.8334553 3.8303242 3.82735 3.8245316 3.7715812 3.7188518 3.7165604 3.714419 3.7124271 3.7105844 3.6086502 3.6071465 3.6057868 3.604571 3.6034987 3.5525343 3.5517597 3.551126 3.5506332 3.5002775 3.4500685 3.45],
   :intensities []})

(def particle
  (geometry-msgs/pose
   :position
   (geometry-msgs/point :x 15.9 :y 19.5)
   :orientation
   (geometry-msgs/quaternion :x 0 :y 0 :z 0.21 :w 0.97)))

;;(def og (deref amclj.core/*map*))

(def base-to-laser-transform
  (geometry-msgs/transform
   :translation (geometry-msgs/vector3 :x 0.135, :y 0.0, :z 0.452)
   :rotation (geometry-msgs/quaternion :x 0.0, :y 0.0, :z 0.0, :w 1.0)))


;; End develop
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defn steep?
  "Is the Y-distance greater than the X-distance?"
  [[x0 y0] [ x1 y1]]
  (> (Math/abs (- y1 y0))
     (Math/abs (- x1 x0))))

(defn- bearing
  "Compute the bearing of the kth laser beam in the scan."
  [k scan]
  (let [{angle-min :angleMin angle-inc :angleIncrement} scan]
    (+ angle-min (* k angle-inc))))

(defn- project-pose
  "Project a pose along a bearing up to a max range"
  [[x y] bearing max-range]
  [(+ x (* max-range (Math/cos bearing)))
   (+ y (* max-range (Math/sin bearing)))])

(defn- swap-xy [[x y]]
  [y x])

(defn- calc-step [[x1 y1] [x2 y2]]
  [(if (< x1 x2) 1 -1)
   (if (< y1 y2) 1 -1)])

(defn- extract-line [map [x0 y0] [max-x max-y] [x-step y-step] steep?]
  (let [delta-x (Math/abs (- x0 max-x))
        delta-y (Math/abs (- y0 max-y))
        delta-err delta-y]
    (loop [x x0, y y0, error 0]
      (cond
       (if steep? (uninhabitable? map y x)
           (uninhabitable? map x y)) (* (Math/hypot (- x x0) (- y y0))
                                        (-> map :info :resolution))
           (> x max-x) max-range
           :else
           (let [x' (+ x x-step)
                 error (+ error delta-err)
                 [y' error] (if (>= (* 2 error) delta-x)
                              [(+ y y-step) (- error delta-x)]
                              [y error])]
             (recur x' y' error))))))

(defn- map-range [[ox oy theta] obs-bearing map range-max]
  (let [bearing (+ theta obs-bearing)
        [x0 y0] (world->map map [ox oy])
        [x1 y1] (world->map map (project-pose [ox oy] obs-bearing max-range))
        steep (steep? [x1 y1] [x0 y0])
        ;; swap coordinates if steep
        [x0 y0] (if steep [y0 x0] [x0 y0])
        [x1 y1] (if steep  [y1 x1] [x0 y0])
        [xstep ystep] (calc-step [x0 y0] [x1 y1])]
    (extract-line map [x0 y0] [x1 y1] [xstep ystep] steep)))

(defn likelihood-field-range-finder-model* [scan [x y theta] grid]
  (let [pose-zk pose ;; TODO: modify based on transform between base_footprint & scan frames
        num-scans (count (:ranges scan))
        step (int (/ (dec num-scans) (dec max-beams)))]
    (for [i (range 0 num-scans step)]
      (let [obs-range (get-in scan [:ranges i])
            obs-bearing (bearing i scan)
            map-range (map-range [x y theta] obs-bearing grid (:rangeMax scan))
            z (- obs-range map-range)]
        ;; Calculate probability
        (let [pz1 (+ (* z-hit (Math/exp
                               (/ (- (* z z))
                                  (* 2 z-hit z-hit)))))
              pz2 (if (< z 0)
                    (* z-short lambda-short
                       (Math/exp (* (- lambda-short)
                                    obs-range))) 0)
              pz3 (if (= obs-range (:rangeMax scan))
                    (* 1.0 z-max) 0)
              pz4 (if (< obs-range (:rangeMax scan))
                    (* z-rand (/ 1.0 (:rangeMax scan))) 0)]
          (pow (+ pz1 pz2 pz3 pz4) 3))))))


(defn likelihood-field-range-finder-model
  "Calculates the likelihood of a pose given the scan and map,
  associng the result into :weight."
  [scan pose map]
  (let [x (get-in pose [:position :x])
        y (get-in pose [:position :y])
        theta (quat/to-heading (:orientation pose))]
    (assoc pose
      :weight (reduce + (likelihood-field-range-finder-model* scan [x y theta] map)))))
































