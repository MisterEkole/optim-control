close all; clear all;
%DC Motor TF
s = tf('s');

plant = 104.9/(s^2 + 103.5*s + 2617);

%PID Controller: 
Kp = 305.4814
Ki = 7.7194e+03
Kd = 3.2058
cont = Kp + Ki/s + Kd*s;

dt = 0.01;
time = 0:dt:1;

%Modified ITAE 1
    %ITAE = ITAE + a*Overshoot + b*SSError + c*SettlingTime + d*RiseTime
   JSt = 0
    [y t]=step(feedback(plant*cont,1),time);
    for i=1:length(t)
    error(i)=abs(1-y(i))*t(i);
    S(i) = stepinfo(y(i),t(i));
    St(i) = S(i).SettlingTime;
    Rt(i) = S(i).RiseTime  *t(i);
    Ov(i) = S(i).Overshoot  *t(i);
    SSE(i) = abs(1-y(i))  *t(i);
    JSt = JSt + St(i)
    end
    JE =sum(error)
    JRt = sum(Rt(end))
    JSt = sum(St)
    JOv = sum(Ov(end))
    JSSE = sum(SSE(end))
    a= 1; b=2; c=3; d=4;
    J = JE + a*JOv + b*JSSE + c*JSt + d*Rt;