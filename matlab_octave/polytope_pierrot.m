function [f_vert, inverse_count] = polytope_pierrot(J_n, t_min, t_max)
% version 24.09.2020.
% Evaluation of Force Capabilities for Redundant manipulatiors
% P.Chiacchio et al.

s = size(J_n);
% static variables - can be defined outside the function - not for every loop
persistent m n
if isempty(n) || n ~=s(2)
  [m, n] = size(J_n);
end

[u s v] = svd(J_n);
r = rank(J_n);
V1 = v(:,1:m);

J_n_invT = pinv(J_n');

A = [V1, eye(n) zeros(n); -V1, zeros(n) eye(n)];
f_vertex_xy = [];
t_vertex = [];
alpha_vertex = [];
inverse_count = 0;
% find all the combinations of submatrix with m zero slack variables
for i = 1:2*n-2
  for j = i+1:2*n-1
    for l = j+1:2*n
      % exclude case when both i-th and n+i-th slack variable are zero
      if ( j == (n + i) || l == (n + i) || l == (n + j) ) 
        continue;
      end
      
      % remove the i-th and j-th slack variable
      A_ = A;
      A_(:,[m+i, m+j, m+l]) = [];
      
      % if not singular matrix - solve system
      if( rank(A_) == 2*n)
        x = inv(A_)*[t_max, t_max]';
        slack_variables = x(m+1:end);
        alpha = x(1:m)';
        % if all slack variables positive vertex found
        if( ~any(slack_variables < 0) )
          alpha_vertex = [alpha_vertex alpha'];
          t_vertex = [t_vertex sum(alpha.*V1, 2)];
        end
        inverse_count++;
      end
    end
  end
end
% calculate the forces based on the vertex torques
f_vert = J_n_invT*( t_vertex );
end