function [tj, Rj] = poseAdd3D(p_i , delta_ij)   
% Luca Carlone 
% pose_add = (p_j , p_i)  the second is the delta pose to be added
% returns the planar roto-translation that compose pj and pi

Rj = p_i.R * delta_ij.R;
tj = p_i.t + p_i.R * delta_ij.t;