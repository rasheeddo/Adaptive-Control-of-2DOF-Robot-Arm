% Adaptive control "Desired Compensation Adaption Law (DCAL)" 
%                  "Inertial Related Controller (IRC)"

clear 

m1 = 0.233;   %kg
m2 = 0.2;
l1 = 0.192;
l2 = 0.185;
L1 = 0.27;
L2 = 0.23;
g = 9.81;

I1 = m1*l1^2;
I2 = m2*l2^2;

%% sampling 0.02s
kp1 = 1.5;
kp2 = 20.0;

kv1 = 0.2;
kv2 = 0.2;

ka = 0.1;

gam1 = 0.05;
gam2 = 0.3;

tf = 3;
ts = 0;
tick = 0.02;
t = 0;
%{
%% sampling 0.002s
kp = 100;
kv = 3;   % cannot be higher than 3
ka = 20;

gam1 = 20;
gam2 = 20;

tf = 3;
ts = 0;
tick = 0.002;
t = 0;
%}
%%
theta1 = 55*pi/180;
theta2 = -55*pi/180;

G1 = m1*g*l1*cos(theta1) + m2*g*L1*cos(theta1) + m2*g*l2*cos(theta1+theta2);
G2 = m2*g*l2*cos(theta1+theta2);


%% Sine Trajectory

A = pi/4;   % angle 
omega = 0.5;  % rad/s

the1 = theta1;
the2 = theta2;

tau = 5.0;
I = (240/413)*(tau/2.1);
I_com = (I*1000)/2.69;




