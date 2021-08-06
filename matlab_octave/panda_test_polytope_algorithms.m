% load the full model of the robot
% https://frankaemika.github.io/docs/control_parameters.html
t_max = [87 	87 	87 	87 	12 	12 	12];
t_min = -[87 	87 	87 	87 	12 	12 	12];

% random robot configurations
t_auctus = [];
t_sasaki = [];
t_pierrot = [];
count_auctus = []; 
count_sasaki = [];
count_pierrot = [];
for kk=1:300
  % define random configuraition
  q_n = rand(1,7)*2*pi - pi;
  % coresponding jacobain
  J_n = J_panda(q_n);

  % run auctus algorithm
  tic
  [f_vert_auctus, count] = polytope_auctus(J_n,t_min,t_max);
  t_auctus = [t_auctus  toc];
  count_auctus = [count_auctus  count];
  % run sasaki algorithm  
  tic
  [f_vert_sasaki, count] = polytope_sasaki(J_n,t_min,t_max);
  t_sasaki = [t_sasaki  toc];  
  count_sasaki = [count_sasaki  count];
  % run sasaki algorithm  
  tic
  [f_vert_pierrot, count] = polytope_pierrot(J_n,t_min,t_max);
  t_pierrot = [t_pierrot  toc];  
  count_pierrot = [count_pierrot  count];
end
disp('auctus')
mean(t_auctus(10:end))
max(t_auctus(10:end))
std(t_auctus(10:end))
mean(count_auctus)
max(count_auctus)
std(count_auctus)
disp('sasaki')
mean(t_sasaki(10:end))
max(t_sasaki(10:end))
std(t_sasaki(10:end))
mean(count_sasaki)
max(count_sasaki)
std(count_sasaki)
disp('pierrot')
mean(t_pierrot(10:end))
max(t_pierrot(10:end))
std(t_pierrot(10:end))
mean(count_pierrot)
max(count_pierrot)
std(count_pierrot)

figure(10)
hold on
plot(t_auctus)
plot(t_sasaki)
plot(t_pierrot)
legend('auctus','sasaki','pierrot')
figure(20)
hold on
plot(count_auctus)
plot(count_sasaki)
plot(count_pierrot)
legend('auctus','sasaki','pierrot')
