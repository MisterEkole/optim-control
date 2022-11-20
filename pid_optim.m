function [J] = pid_optim(x)
%DC Motor TF
s = tf('s');

plant = 104.9/(s^2 + 103.5*s + 2617);

%PID Controller: 
Kp = x(1)
Ki = x(2)
Kd = x(3)
cont = Kp + Ki/s + Kd*s;


%Closed Loop System Response
%step(feedback(plant*cont,1));

dt = 0.01;
time = 0:dt:1;
S = stepinfo(feedback(plant*cont,1));
  
%IAE    
%     [y t]=step(feedback(plant*cont,1),time);
%     for i=1:length(t)
%     error(i)=1-y(i);
%     end
%     J=sum(abs(error));

%ITAE
%     [y t]=step(feedback(plant*cont,1),time);
%     for i=1:length(t)
%     error(i)=abs(1-y(i))*t(i);
%     end
%     J=sum(error);
    
%ISE
%     [y t]=step(feedback(plant*cont,1),time);
%     for i=1:length(t)
%     error(i)=1-y(i);
%     end
%     error=error*error'; 
%     J=sum(error);
  
%ITSE
%     [y t]=step(system,time);
%     for i=1:length(t)
%     error(i)=((1-y(i))^2)*t(i);
%     end
%     J=sum(error);
    
%Modified ITAE 1
%     ITAE = ITAE + a*Overshoot + b*SSError + c*SettlingTime + d*RiseTime
   
    [y t]=step(feedback(plant*cont,1),time);
    for i=1:length(t)
    error(i)=abs(1-y(i))*t(i);
    
    end
 
   
  St = S.SettlingTime; 
Rt = S.RiseTime;
Ov = S.Overshoot; 
SSE = abs(1-y);
JE =sum(error);
% JRt = (sum(Rt))';
% JSt = (sum(St))';
% JOv = (sum(Ov))';
JSSE = (sum(SSE(end)))';
    a= 1; b=2; c=3; d=4;
  J = JE + c*St + a*Ov + b*JSSE + c*St + d*Rt;

%Modified ITAE 2
    %Settling Time, Rise Time Peak Time & Overshoot 
%     S = stepinfo(feedback(plant*cont,1));
%     [y t]=step(feedback(plant*cont,1),time);
%     for i=1:length(t)
%     error(i)=abs(1-y(i))*t(i);
%     SSE(i) = abs(1-y(i))*t(i);
%     end
%     JE =sum(error);
%     JRt = S.RiseTime;
%     JSt = S.SettlingTime;
%     JOv = S.Overshoot;
%     JSSE = sum(SSE);
%     a= 1.5; b=15; c=7; d=1;
%     J = JE + a*JOv + b*JSSE + c*JSt + d*JRt