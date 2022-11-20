% This file use GA optimization to tune the parameters of PID controller
close all; clear all;

% 
n_var = 3;
lb = [0 0 0];
ub = [1000 1000 500];

% GA options
ga_opt = gaoptimset('Display','off','Generations',20,'PopulationSize',50,'PlotFcns',@gaplotbestf);

% objective function
obj_fun = @(x) itae_cost(x);

% GA command
[x, best] = ga(obj_fun, n_var, [],[],[],[],lb, ub, [], ga_opt);

%% Run the system again with optimized values
Kp = x(1);
Ki = x(2);
Kd = x(3);

% Simulink model options
opt = simset('SrcWorkspace','current','solver','ode45');
sim('tf_vary',[0 10], opt);

% save the result
save('GA_res','x','best')

