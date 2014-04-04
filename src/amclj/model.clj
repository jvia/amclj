(ns amclj.model)
(def pf
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



(def laser
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

