function [tij,Rij] = poseSubNoisy3D(pose_i , pose_j, sigmaT, sigmaR)
% Luca Carlone
% pose_sub = (p_j , p_i) = inv(p_i) * p_j * p_noise
% returns the planar roto-translation that tranform pj in pi
% plus noise (sigmaT is the std of the noise on the cartesian components,
% sigmaR is the std of the noise added to the angle before wrapping)

% GT measurements
Ri = pose_i.R;
Rj = pose_j.R;
Rij = Ri' * Rj;
ti = pose_i.t;
tj = pose_j.t;
tij = Ri' * (tj - ti);

% Nominal noise
eRotVec  = gaussian_noise(sigmaR^2 * eye(3));
eTranVec = gaussian_noise(sigmaT^2 * eye(3));

% Add noise to measurements 
if(norm(eRotVec)>1e-7)
  eRot = axis_angle_to_mat([eRotVec'/norm(eRotVec) norm(eRotVec)]);
  Rij = Rij * eRot; % perturb the rotation measurement
end
tij = tij + eTranVec; 

end
