function [f_vert, inverse_count] = polytope_auctus(J_n,t_min,t_max)
% version 13.11.2020.
% On-line force capability evaluation based on eﬀicientpolytope vertex search
% Antun Skuric, Vincent Padois, David Daney

s = size(J_n);

% static variables - can be defined outside the function - should not be executed in every loop
persistent m n N To_min T_vec epsilon_zero v_origin_combinations alphas
if isempty(n) || isempty(m) || n ~=s(2) || m ~=s(1) % if the system changes rerun the intialisation
  % size of the jacobian matrix
  [m, n] = size(J_n); 
  % matrix containing base vectors tau_1,... tau_n
  T_vec = diag(t_max-t_min); 
  % matrix containing set of origin positions tau_o
  % for any combination of m base vectors
  % initially this matrix has 2^m origins set to tau_min
  To_min = repmat(t_min',1,2^m);
  % all combinations of m out of n base vectors 
  v_origin_combinations = nchoosek(1:n,m);
  % number of posible combinations
  N = length(v_origin_combinations(:,1));  
  % permutation of alphas to achieve all the posible tau_o comvinations
  % each alpha is either 0 or 1
  alphas=[];
  for i=1:m
    alphas = [alphas; bitget(1:2^m,i)];
  end
  % zero tolerance - for avoiding numerical problems 
  epsilon_zero = 1e-10; 
end

% svd of jacobian
[u s v] = svd(J_n);
V1 = v(:, 1:m);
V2 = v(:, m+1:end);

  
% torque and force vertices
t_vert = [];
f_vert = [];

% matrix inverse counter
inverse_count = 0;
for i = 1:N 
    % find all n-m 
    v_origin = v_origin_combinations(i,:);
    v_base = 1:n;
    v_base(v_origin) = []; % remove the vectors on the faces
    
    % construct the matrix containing all origins of parallel n-m faces
    % there is 2^m faces possible 
    To = To_min;    
    To(v_origin,:) = diag(t_max(v_origin))*~alphas + diag(t_min(v_origin))*alphas;
    % project them to null space
    To_v2 = V2'*To;
  
    % build T matrix
    % base vectors to be used vectors to be used
    T_base = T_vec(:, v_base);
    % T matrix definition - projection of base vectors to null space
    T = V2'*-T_base;
    % Check the condition for matrix inversion
    not_solvable = any(To_v2 > sum(max(T,0)')') + any(To_v2 < sum(min(T,0)')') > 0;
    if all(not_solvable)  % if none of the 2^m systems is solvable dont invert matrix
      continue;
    end 
    % use the computed check to discadrd not solvable systems
    To(:,not_solvable) = [];
    To_v2(:,not_solvable) = [];
    
    % Solving the linear matrix system
    % compute the matrix inverse
    T_inv = pinv(T);
    % sind all the comibations of aplhas for all the origins
    Alpha = T_inv*To_v2;
    
    % check if inverse correct - all errors should be 0
    t_err = any(abs(To_v2 - T*Alpha) > epsilon_zero);
    % remove the solutions that are not in polytope 0<Alpha<1
    to_remove = ( any(Alpha < -epsilon_zero) + any(Alpha-1 > epsilon_zero) + t_err ) > 0;
    Alpha(:, to_remove)=[];
    To(:, to_remove)=[];
    
    % add vertex torque
    t_vert = [t_vert To+T_base*Alpha]; 
    inverse_count++;
  end
  % calculate the forces based on the vertex torques
f_vert = pinv(J_n')*( t_vert );
end