function [f_vert, inverse_count] = polytope_sasaki(J_n, t_min, t_max)
% version 24.09.2020
% Vertex search algorithm of convex polyhedron representing upperlimb manipulation ability
% Sasaki et al.

s = size(J_n);
% static variables - can be defined outside the function - not for every loop
persistent m n alphas 
if isempty(alphas)  || n ~=s(2)
  [m, n] = size(J_n);
  % permutation of alphas to achieve all the posible tau_o comvinations
  % each alpha is either 0 or 1
  alphas=[];
  for i=1:m
    alphas = [alphas; bitget(1:2^m,i)];
  end
end

% do the svd
[u s v] = svd(J_n);
V1 = v(:,1:m);
J_n_invT = pinv(J_n');

t_vertex = [];
inverse_count= 0;


v_origin_combinations = nchoosek(1:n,m);
% number of posible combinations
N = length(v_origin_combinations(:,1));  

% find all the combinations of submatrix with m zero slack variables
for i = 1:N
  v_comb = v_origin_combinations(i,:);
  % construct V11 and V12 submatrix  
  rest_ind = 1:n;
  rest_ind(v_comb) = [];  
  V11 = V1(v_comb,:);
  V12 = V1(rest_ind,:);

  % if not singular matrix - solve system
  if( rank(V11) == m)
    % limit vector
    T1 = diag(t_max(v_comb))*~alphas + diag(t_min(v_comb))*alphas;
    % calculate the other part of the limit vector
    T2 = V12*inv(V11)*T1;

    % if T2 possible 
    to_use = any(t_max(rest_ind)' - T2 < 0) + any(t_min(rest_ind)' - T2 > 0) == 0;
    % rebuiild the torque vectors
    t_i = zeros(n,2^m);
    t_i(v_comb,:) = T1;
    t_i(rest_ind,:) = T2;
    t_vertex = [t_vertex t_i(:,to_use)];
    inverse_count = inverse_count+1;
  end
end
% calculate the forces based on the vertex torques
f_vert = J_n_invT*( t_vertex );
        
