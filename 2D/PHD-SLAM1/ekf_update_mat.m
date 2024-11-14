function [H,U]= ekf_update_mat(quat,mu)
 
 H_3d = quat2rot(compact(quat),"frame");
 H = H_3d(1:2,1:2); %Only need the 2D rotation component of the rot matrix

 U= eye(2);