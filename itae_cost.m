% This cost function uses Simulink model to compute the ITAE
function [J] = itae_cost(x)

Kp = x(1);
Ki = x(2);
Kd = x(3);

% Simulink model options
opt = simset('SrcWorkspace','current','solver','ode45');

try
    sim('tf_vary',[0 10], opt);
    J = ITAE(end);
    
catch
    J = 1e15;
end
