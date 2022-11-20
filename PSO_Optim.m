close all; clear all;

% 
n_vars = 3;
lb = [0 0 0];
ub = [1000 1000 500];
fun=@(x) itae_cost(x);


%PSO Options
options=optimoptions('particleswarm','SwarmSize',30,HybridFcn='fmincon',PlotFcn='pswplotbestf',MaxIterations=50,MaxTime=Inf)

[x, fval]=particleswarm(fun,n_vars,lb,ub,options);

%%Run system with Optimized gains
Kp = x(1);
Ki = x(2);
Kd = x(3);

% Simulink model options
opt = simset('SrcWorkspace','current','solver','ode45');
sim('tf_vary',[0 10], opt);

% save the result
save('PSO_res','x','fval')




