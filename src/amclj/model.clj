(ns amclj.model)
(def pf-model
  "Parameters to the MCL algorithm.
  
  Particle Filter Options:
    :min_particles (default 100) Minimum allowed number of particles.
  
  

  "
  {:min_particles 100
   :max_particle 5000
   :kld_err 0.01
   :kld_z 0.99
   :update_min_d 0.2
   :update_min_a (/ Math/PI 6.0)
   :resample_interval 2
   :transform_tolerance 0.1
   :recovery_alpha_slow 0.0
   :recovery_alpha_fast 0.0
   :initial_pose_x 0.0
   :initial_pose_y 0.0
   :initial_pose_a 0.0
   :initial_cov_xx (* 0.5 0.5)
   :initial_cov_yy (* 0.5 0.5)
   :initial_cov_aa (* (/ Math/PI 12) (/ Math/PI 12))
   :gui_publish_rate -1.0
   :save_pose_rate 0.5 
   :use_map_topic false
   :first_map_only false})

(def odom-model
  "Odometry Options:
    :odom_model_type (default :diff) Which model to use, either :diff
                     or :omni.
  
    :odom_alpha1 (default 0.2) Specifies the expected noise in
                 odometry's rotation estimate from the rotational
                 component of the robot's motion.
  
    :odom_alpha2 (default 0.2) Specifies the expected noise in
                 odometry's rotation estimate from translational
                 component of the robot's motion.
  
    :odom_alpha3 (default 0.2) Specifies the expected noise in
                 odometry's translation estimate from the translational
                 component of the robot's motion.
  
    :odom_alpha4 (default 0.2) Specifies the expected noise in
                 odometry's translation estimate from the rotational
                 component of the robot's motion.
  
    :odom_alpha5 (default 0.2) Translation-related noise parameter (only
                 used if model is :omni).
  
    :odom_frame_id (default \"odom\") Which frame to use for odometry.
  
    :base_frame_id (default \"base_link\") Which frame to use for the
                   robot base
  
    :global_frame_id (default \"map\") The name of the
                     coordinate frame published by the localization
                     system"
  {:odom_model_type :diff
   :odom_alpha1 0.2
   :odom_alpha2 0.2
   :odom_alpha3 0.2
   :odom_alpha4 0.2
   :odom_alpha5 0.2
   :odom_frame_id "odom"
   :base_frame_id "base_link"
   :global_frame_id "map"})

(def laser-model
  {:laser_min_range -1.0
   :laser_max_range -1.0
   :laser_max_beams 30
   :laser_z_hit 0.95
   :laser_z_short 0.1
   :laser_z_max 0.05
   :laser_z_rand 0.05
   :laser_sigma_hit 0.2
   :laser_lambda_short 0.1
   :laser_likelihood_max_dist 2.0
   :laser_model_type :likelihood_field})

