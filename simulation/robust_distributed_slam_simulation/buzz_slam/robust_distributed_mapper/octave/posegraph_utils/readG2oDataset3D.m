function [measurements, edges_id, readPoses] = readG2oDataset3D(input_file_g2o, orderEdgesFlag)
  %
  % Reads an input file in g2o format:
  % VERTEX_SE3:QUAT id(0-based) x y z qx qy qz qw (position & quaternion)
  % EDGE_SE3:QUAT id1 id2 dx dy dz dqx dqy dqz dqw 
  % I11 I12 I13 I14 I15 I16
  %     I22 I23 I24 I25 I26 
  %         I33 I34 I35 I36
  %             I44 I45 I46
  %                 I55 I56
  %                     I66
  % File is stored in the following matlab structures:
  % - poses (stores the VERTEX): 
  %   -- poses(i).R is the 3x3 rotation of the i-th node (indices are +1, 1-based)
  %   -- poses(i).t is the 3D position of the i-th node
  % - edges_id: is an mx2 matrix such that if the k-th row is [i j], it describes a measurement between i and j
  % - measurements (stores the EDGES):
  %   -- measurements.between(k).R is the k-th relative rotation measurement
  %   -- measurements.between(k).t is the k-th relative translation measurement
  %   -- measurements.between(k).Info is the 6x6 covariance of k-th measurement
  %
  % NOTE: input file has zero-based indices, but everything is converted to 1-based in matlab
  % 
  % Luca Carlone
  % Georgia Institute of Technology
  % Aug 6, 2014
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  
  if nargin < 2
    orderEdgesFlag = true;
  end
  
  fid_g2o = fopen(input_file_g2o,'r');
  
  unorderedFlag = 0;
  unorderedPoseIds = 0;
  
  nodeId = 0;
  edgeId = 0;
  
  epsQ = 1e-3; % numerical zero for quaternion norm check
  
  tline = fgets(fid_g2o);
  idNodes = [];
  
  while ischar(tline)
    
    foundNode = ( isempty(strfind(tline, 'VERTEX_SE3:QUAT '))==0 );
    %% VERTEX line
    if foundNode==1
      nodeId = nodeId+1;
      [name, id, x, y, z, qx, qy, qz, qw] = strread(tline, '%s %d %f %f %f %f %f %f %f');
      if (id+1 ~= nodeId)
        unorderedPoseIds = 1;
      end
      idNodes(end+1) = id;
      poses(nodeId).t = [x y z]';
      q = [qw, qx qy, qz]';
      if(  abs(norm(q)-1) > epsQ) 
        norm(q)
        error('Quaternion has not unit norm'); 
      else
        q = q/norm(q); % we normalize anyway
      end
      poses(nodeId).R = quat2rot(q);
    end
    
    foundEdge = ( isempty(strfind(tline, 'EDGE_SE3:QUAT '))==0 );
    %% EDGE line
    if foundEdge==1
      edgeId = edgeId+1;
      [name, id1, id2, dx, dy, dz, dqx, dqy, dqz, dqw, ...
        I11, I12, I13, I14, I15, I16, ...
             I22, I23, I24, I25, I26, ...
                  I33, I34, I35, I36, ...
                       I44, I45, I46, ...
                            I55, I56, ...
                            I66] = ... 
      strread(tline, '%s %f %f %f %f %f %f %f %f %f    %f %f %f %f %f %f    %f %f %f %f %f   %f %f %f %f   %f %f %f    %f %f    %f');
      if edgeId < nodeId && (id1 ~= edgeId-1 || id2 ~= edgeId) % only on spanning tree
        unorderedFlag = 1;
      end
      edges_id(edgeId,:) = [id1+1, id2+1];
      measurements.between(edgeId).t = [dx dy dz]';
      dq = [dqw, dqx, dqy, dqz]';
      if(  abs(norm(dq)-1) > epsQ)
        error('dQuaternion has not unit norm');
      else
        dq = dq/norm(dq); % we normalize anyway
      end
      measurements.between(edgeId).R =  quat2rot(dq);
      measurements.between(edgeId).Info = ...
           [I11, I12, I13, I14, I15, I16, 
            I12, I22, I23, I24, I25, I26, 
            I13, I23, I33, I34, I35, I36, 
            I14, I24, I34, I44, I45, I46, 
            I15, I25, I35, I45, I55, I56, 
            I16, I26, I36, I46, I56, I66]; 
      if norm(measurements.between(edgeId).Info - measurements.between(edgeId).Info',inf)>1e-5
        error('Nonsymmetric information matrix');
      end
    end
    tline = fgets(fid_g2o);
  end
  fclose(fid_g2o);
  
  if unorderedPoseIds == 1
    disp('readG2oDataset3D: incorrect node ids in g2o file')
    [indices, ordering] = sort(idNodes);
    indices = indices - indices(1);  indices = indices(:);
    if(max(indices - [0:length(indices)-1]') > 0)
      error('readG2oDataset3D: incorrect node ids, even after ordering')
    end
    readPoses  = poses(ordering);
  else
    readPoses  = [];
  end
  
  %if unorderedFlag==1 && orderEdgesFlag
  %  minInd = min ( edges_id(:,1), min(edges_id(:,2)) );
  %  if minInd>1
  %    warning(sprintf('minimum node id in g2o file is %d - replacing with 1 \n',minInd))
  %    edges_id(:,1:2) = edges_id(:,1:2) - (minInd+1) * ones(size(edges_id(:,1:2)));
  %  end
  %  disp('readG2oDataset3D: incorrect edge ids in g2o file, we are going to sort them')
  %  desiredOrder = orderingEdges(edges_id);
  %  edges_id = edges_id(desiredOrder,:);
 %   measurements.between = measurements.between(desiredOrder);
  %end