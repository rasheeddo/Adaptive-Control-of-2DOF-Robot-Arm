% Create 5th order polynomial for cartesian coordinate trajectory
clear 

% DH-parameters
a1 = 270; %mm  first link length
a2 = 400; %mm  second link length

%{
q1 = 30*pi/180;
q2 = 30*pi/180;
px = cos(q1+q2)*a2 + a1*cos(q1);
py = sin(q1+q2)*a2 + a1*sin(q1);
%}
% Input values
PX_start = 350;     % mm
PY_start = 550;     % mm

PX_goal = -350;     % mm
PY_goal = 550;      % mm

finish_time = 4;                                % total time [sec] using from start to goal points 
  

% Calculate others from input values
distance = sqrt( (PX_goal-PX_start)^2 + (PY_goal-PY_start)^2 ); % mm   % A distance between two points (vector norm)
points = round(distance);                                       % number of points on the line
ave_speed = distance/finish_time;               % average speed from start to goal [mm/s]  
gap_time = finish_time/points;                                  % a tiny time [sec] used to travel in a gap
gap = distance/points;                                          % let use minimum gap at 1mm

f = zeros(1,points);

% Calculate normalized 5th order function according to finish_time and gap_time
inc = gap_time;
for i = 1:points
    %f(i) = 6*(inc/finish_time)^5 - 15*(inc/finish_time)^4 + 10*(inc/finish_time)^3;
    f(i) = -2*(inc/finish_time)^3 + 3*(inc/finish_time)^2;
    inc = inc+gap_time;
end
time_array = gap_time:gap_time:finish_time;   % use for plotting graph with f

px = PX_start + (PX_goal - PX_start)*f;
py = PY_start + (PY_goal - PY_start)*f;

% Create zeros arrays
Q1 = zeros(1,points);
Q2 = zeros(1,points);

% Calculate Inverse Kinematics for every points on the path
for k=1:points
    L = sqrt(px(k)^2 + py(k)^2);

    pi_Q2 = acos( (px(k)^2 + py(k)^2 - a1^2 - a2^2)/(-2*a1*a2)); % Solution2
    Q2(k) = pi - pi_Q2;            % Solution1
    gam1 = asin( (a2*sin(Q2(k)))/L );
    alp = atan2(py(k),px(k));
    Q1(k) = alp - gam1;   % Solution1

end

% Make a plot of two links simulation according to the Q1 and Q2
for i=1:points
% Transformation matrices
T01 = [cos(Q1(i)), -sin(Q1(i)), 0, a1*cos(Q1(i));
       sin(Q1(i)), cos(Q1(i)), 0, a1*sin(Q1(i));
       0,   0,   1,   0;
       0,   0,   0,   1];           % Frame 1 respecto to 0
T12 = [cos(Q2(i)), -sin(Q2(i)), 0, a2*cos(Q2(i));
       sin(Q2(i)), cos(Q2(i)), 0, a2*sin(Q2(i));
       0,   0,   1,   0;
       0,   0,   0,   1];           % Frame 2 respecto to 1
   
T02 = T01*T12;                      % Frame 2 respecto to 0
% For plotting
P01 = [T01(1,4), T01(2,4), T01(3,4)];   % Position of frame 1 respecto 0
P02 = [T02(1,4), T02(2,4), T02(3,4)];   % Position of frame 2 respecto 0
% Origin coordinates
X0 = 0;
Y0 = 0;
Z0 = 0;
% For plotting links
link1X = [X0, P01(1)];
link1Y = [Y0, P01(2)];
link2X = [P01(1), P02(1)];
link2Y = [P01(2), P02(2)];
joint1 = [X0,Y0];
joint2 = [P01(1), P01(2)];
tip = [P02(1), P02(2)];

% Simulation of two links movement
figure(1)
plot(link1X, link1Y, 'r')
hold on
plot(link2X, link2Y, 'b')
plot(joint1(1),joint1(2), 'ro')
plot(joint2(1),joint2(2), 'bo')
plot(tip(1),tip(2), 'k*')
grid on
xlabel('x')
ylabel('y')
xlim([-600 600])
ylim([-600 600])
axis('square')
hold off
pause(0.001)

end

%figure(1)

% Convert Q1 and Q2 to angle in degrees to understand the joint behaviors
Theta1 = Q1*180/pi;
Theta2 = Q2*180/pi;

% Plot of required joint angles to generate the desired path
figure(2)
subplot(2,1,1)
plot(time_array,Theta1)
ylabel('\theta_1')
grid on
subplot(2,1,2)
plot(time_array,Theta2)
ylabel('\theta_2')
grid on

