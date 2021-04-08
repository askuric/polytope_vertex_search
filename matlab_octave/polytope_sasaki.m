function [f_vert, inverse_count] = polytope_sasaki(J_n, t_min, t_max)
% version 24.09.2020
% Vertex search algorithm of convex polyhedron representing upperlimb manipulation ability
% Sasaki et al.

s = size(J_n);
% static variables - can be defined outside the function - not for every loop
persistent m n t_max_pert 
if isempty(t_max_pert)  || n ~=s(2)
  [m, n] = size(J_n);
  t_max_pert = [1 -1 1 -1 1 -1 1 -1;1 1 -1 -1 1 1 -1 -1;1 1 1 1 -1 -1 -1 -1];
end

% do the svd
[u s v] = svd(J_n);
V1 = v(:,1:m);
J_n_invT = pinv(J_n');

t_vertex = [];
inverse_count= 0;
% find all the combinations of submatrix with m zero slack variables
for i = 1:n-1
  for j = i+1:n
    for l = j+1:n
    
      % construct V11 and V12 submatrix  
      rest_ind = 1:n;
      rest_ind([i, j, l]) = [];  
      V11 = V1([i, j, l],:);
      V12 = V1(rest_ind,:);
    
      % if not singular matrix - solve system
      if( rank(V11) == m)
        % limit vector
        T1 = diag(t_max([i,j,l]))*t_max_pert;
        % calculate the other part of the limit vector
        T2 = V12*inv(V11)*T1;
        
        % if T2 possible 
        to_use = any(t_max(:, rest_ind)' - T2 < 0, 1) + any(t_min(:,rest_ind)' - T2 > 0,1) == 0;
        % rebuiild the torque vectors
        t_i = zeros(n,2^m);
        t_i([i,j,l],:) = T1;
        t_i(rest_ind,:) = T2;
        t_vertex = [t_vertex t_i(:,to_use)];
        inverse_count =  inverse_count + 1;
      end
    end
  end
end
% calculate the forces based on the vertex torques
f_vert = J_n_invT*( t_vertex );
        
