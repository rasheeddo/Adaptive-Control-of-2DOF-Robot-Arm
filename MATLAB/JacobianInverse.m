clear

a1 = 211; %mm
a2 = 171; %mm

% Draw straigh line, let specify only one direction velocity
Xdot = -30; %mm/s
Ydot = 0; %mm/s

distance = 300; %mm
step_inc = 600; % divide straight line to "step_inc" segment
gap_dis = distance/step_inc;  % a gap distance between each segment

t = abs(gap_dis/Xdot);  % spending time [sec] to travel from point to point

theta1 = 30; %deg
theta2 = 30;
% Convert ang
q1 = theta1*pi/180;
q2 = theta2*pi/180;

T01 = [cos(q1), -sin(q1), 0, a1*cos(q1);
       sin(q1), cos(q1), 0, a1*sin(q1);
       0,   0,   1,   0;
       0,   0,   0,   1];
T12 = [cos(q2), -sin(q2), 0, a2*cos(q2);
       sin(q2), cos(q2), 0, a2*sin(q2);
       0,   0,   1,   0;
       0,   0,   0,   1];
T02 = T01*T12;
P01 = [T01(1,4), T01(2,4), T01(3,4)];
P02 = [T02(1,4), T02(2,4), T02(3,4)];
X0 = 0;
Y0 = 0;
Z0 = 0;

link1X = [X0, P01(1)];
link1Y = [Y0, P01(2)];
link2X = [P01(1), P02(1)];
link2Y = [P01(2), P02(2)];
joint1 = [X0,Y0];
joint2 = [P01(1), P01(2)];
tip = [P02(1), P02(2)];

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
xlim([-400 400])
ylim([-400 400])
axis('square')

Q1 = zeros(1,step_inc);
Q2 = zeros(1,step_inc);
Q1dot = zeros(1,step_inc);
Q2dot = zeros(1,step_inc);

for i=1:step_inc
   
J11 = -a1*sin(q1) - a2*sin(q1+q2);
J12 = -a2*sin(q1+q2);
J21 = a1*cos(q1) + a2*cos(q1+q2);
J22 = a2*cos(q1+q2);

multiplier = 1/(J11*J22 - J12*J21);

invJ11 = multiplier*J22;
invJ12 = -multiplier*J12;
invJ21 = -multiplier*J21;
invJ22 = multiplier*J11;

q1dot = invJ11*Xdot + invJ12*Ydot;   %rad/s
q2dot = invJ21*Xdot + invJ22*Ydot;   

q1 = q1 + q1dot*t;
q2 = q2 + q2dot*t;

Q1(i) = q1;
Q2(i) = q2;
Q1dot(i) = q1dot;
Q2dot(i) = q2dot;

end

THE1 = Q1*180/pi;
THE2 = Q2*180/pi;

THE1dot = Q1dot*180/pi;
THE2dot = Q2dot*180/pi;

for i=1:step_inc
T01 = [cos(Q1(i)), -sin(Q1(i)), 0, a1*cos(Q1(i));
       sin(Q1(i)), cos(Q1(i)), 0, a1*sin(Q1(i));
       0,   0,   1,   0;
       0,   0,   0,   1];
T12 = [cos(Q2(i)), -sin(Q2(i)), 0, a2*cos(Q2(i));
       sin(Q2(i)), cos(Q2(i)), 0, a2*sin(Q2(i));
       0,   0,   1,   0;
       0,   0,   0,   1];
T02 = T01*T12;
P01 = [T01(1,4), T01(2,4), T01(3,4)];
P02 = [T02(1,4), T02(2,4), T02(3,4)];
X0 = 0;
Y0 = 0;
Z0 = 0;

link1X = [X0, P01(1)];
link1Y = [Y0, P01(2)];
link2X = [P01(1), P02(1)];
link2Y = [P01(2), P02(2)];
joint1 = [X0,Y0];
joint2 = [P01(1), P01(2)];
tip = [P02(1), P02(2)];

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
xlim([-400 400])
ylim([-400 400])
axis('square')
pause(0.01)

end

figure(1)
hold off

figure(2)
subplot(2,1,1)
plot(THE1)
ylabel('\theta_1')
grid on
subplot(2,1,2)
plot(THE2)
ylabel('\theta_2')
grid on

figure(3)
subplot(2,1,1)
plot(THE1dot)
ylabel('\theta^{.}_1')
grid on
subplot(2,1,2)
plot(THE2dot)
ylabel('\theta^{.}_2')
grid on

%theta1dot = q1dot*180/pi;   %deg/s
%theta2dot = q2dot*180/pi;


