## RFS-SLAM

Testing codes for RFS-SLAM implementation with general motion model and stereo feature-base measurement

### Workflow 2D simulations
run sim_gen_data_2D.m after making desired changes to robot's trajectories. Save "truth" variable in /dataset directory

run PHD-SLAMx simulation by running sim_PHD_SLAMx_2D.m with the desired .mat file in /dataset. Save "simulation" variable in /PHD-SLAMx/sim_data_output directory.

To evaluate accuracy, run pose_estimation_eval_2D.m with the desired .mat file in /PHD-SLAMx/sim_data_output directory. 
