% Two link robot arm dynamics Computed Torque Control
clear 

m1 = 0.233;   %kg
m2 = 0.2;
l1 = 0.192;
l2 = 0.185;
L1 = 0.27;
L2 = 0.23;
g = 9.81;

I1 = (1/3)*m1*l1^2;
I2 = (1/3)*m2*l2^2;

Kp = 300;
Ki = 100;
Kv = 2*sqrt(Kp);

theta1 = 90*pi/180;
theta2 = -90*pi/180;

G1 = m1*g*l1*cos(theta1) + m2*g*L1*cos(theta1) + m2*g*l2*cos(theta1+theta2);
G2 = m2*g*l2*cos(theta1+theta2);

%% Desired Trajectory

rad2deg = 180/pi;
deg2rad = pi/180;

the_s = 0;
the_f = pi/2;

tf = 2;
ts = 0;
tick = 0.02;
t = 0;
count = 1;

for i = ts:tick:tf
    
    % Generate 5th order normalized function f
    f(count) = 6*(t/tf)^5 - 15*(t/tf)^4 + 10*(t/tf)^3;
    
    the(count) = the_s + (the_f - the_s)*f(count);
    
    if count == 1
        vel(count) = 0;
        acc(count) = 0;
    else
        vel(count) = (the(count) - the(count-1))/tick;
        acc(count) = (vel(count) - vel(count-1))/tick;
    end
   
    t = t + tick;
    count = count + 1;
    
end

%% Sine Trajectory

A = pi/4;   % angle 
omega = 1;  % rad/s

the1 = pi/6;
the2 = pi/8;