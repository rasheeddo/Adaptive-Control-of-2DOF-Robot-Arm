clear 


a1 = 211; %mm
a2 = 171; %mm

%{
q1 = 30*pi/180;
q2 = 30*pi/180;
px = cos(q1+q2)*a2 + a1*cos(q1);
py = sin(q1+q2)*a2 + a1*sin(q1);
%}
% Input values

X_length = 150;
Y_length = 20;

PX_A = 250;     % mm
PY_A = 150;     % mm

PX_B = 100;     % mm
PY_B = 150;      % mm

PX_C = 100;     % mm
PY_C = 170;     % mm

PX_D = 250;     % mm
PY_D = 170;      % mm

PX_E = 250;     % mm
PY_E = 190;     % mm

PX_F = 100;     % mm
PY_F = 190;      % mm

PX_G = 100;     % mm
PY_G = 210;     % mm

PX_H = 250;     % mm
PY_H = 210;      % mm


des_vel = 40;   % mm/s scalar

% A to B
distanceAB = sqrt( (PX_B-PX_A)^2 + (PY_B-PY_A)^2 ); % mm
vectorAB = [PX_B-PX_A, PY_B-PY_A]; % a vector of path
unit_vecAB = vectorAB/distanceAB;  % Specify the direction
Tip_velAB = unit_vecAB*des_vel;     % tip's velocity vector 
finish_timeAB = distanceAB/des_vel; % total time [sec] using from start to goal points
pointsAB = round(distanceAB);  % continuous points on the line
gapAB = distanceAB/pointsAB;  % let use minimum gap at 1mm
gap_timeAB = finish_timeAB/pointsAB;  % a tiny time [sec] used to travel in a gap


% B to C
distanceBC = sqrt( (PX_C-PX_B)^2 + (PY_C-PY_B)^2 ); % mm
vectorBC = [PX_C-PX_B, PY_C-PY_B]; % a vector of path
unit_vecBC = vectorBC/distanceBC;  % Specify the direction
Tip_velBC = unit_vecBC*des_vel;     % tip's velocity vector 
finish_timeBC = distanceBC/des_vel; % total time [sec] using from start to goal points
pointsBC = round(distanceBC);  % continuous points on the line
gapBC = distanceBC/pointsBC;  % let use minimum gap at 1mm
gap_timeBC = finish_timeBC/pointsBC;  % a tiny time [sec] used to travel in a gap


% C to D
distanceCD = sqrt( (PX_D-PX_C)^2 + (PY_D-PY_C)^2 ); % mm
vectorCD = [PX_D-PX_C, PY_D-PY_C]; % a vector of path
unit_vecCD = vectorCD/distanceCD;  % Specify the direction
Tip_velCD = unit_vecCD*des_vel;     % tip's velocity vector 
finish_timeCD = distanceCD/des_vel; % total time [sec] using from start to goal points
pointsCD = round(distanceCD);  % continuous points on the line
gapCD = distanceCD/pointsCD;  % let use minimum gap at 1mm
gap_timeCD = finish_timeCD/pointsCD;  % a tiny time [sec] used to travel in a gap

% D to E
distanceDE = sqrt( (PX_E-PX_D)^2 + (PY_E-PY_D)^2 ); % mm
vectorDE = [PX_E-PX_D, PY_E-PY_D]; % a vector of path
unit_vecDE = vectorDE/distanceDE;  % Specify the direction
Tip_velDE = unit_vecDE*des_vel;     % tip's velocity vector 
finish_timeDE = distanceDE/des_vel; % total time [sec] using from start to goal points
pointsDE = round(distanceDE);  % continuous points on the line
gapDE = distanceDE/pointsDE;  % let use minimum gap at 1mm
gap_timeDE = finish_timeDE/pointsDE;  % a tiny time [sec] used to travel in a gap

% E to F
distanceEF = sqrt( (PX_F-PX_E)^2 + (PY_F-PY_E)^2 ); % mm
vectorEF = [PX_F-PX_E, PY_F-PY_E]; % a vector of path
unit_vecEF = vectorEF/distanceEF;  % Specify the direction
Tip_velEF = unit_vecEF*des_vel;     % tip's velocity vector 
finish_timeEF = distanceEF/des_vel; % total time [sec] using from start to goal points
pointsEF = round(distanceEF);  % continuous points on the line
gapEF = distanceEF/pointsEF;  % let use minimum gap at 1mm
gap_timeEF = finish_timeEF/pointsEF;  % a tiny time [sec] used to travel in a gap

% F to G
distanceFG = sqrt( (PX_G-PX_F)^2 + (PY_G-PY_F)^2 ); % mm
vectorFG = [PX_G-PX_F, PY_G-PY_F]; % a vector of path
unit_vecFG = vectorFG/distanceFG;  % Specify the direction
Tip_velFG = unit_vecFG*des_vel;     % tip's velocity vector 
finish_timeFG = distanceFG/des_vel; % total time [sec] using from start to goal points
pointsFG = round(distanceFG);  % continuous points on the line
gapFG = distanceFG/pointsFG;  % let use minimum gap at 1mm
gap_timeFG = finish_timeFG/pointsFG;  % a tiny time [sec] used to travel in a gap

% G to H
distanceGH = sqrt( (PX_H-PX_G)^2 + (PY_H-PY_G)^2 ); % mm
vectorGH = [PX_H-PX_G, PY_H-PY_G]; % a vector of path
unit_vecGH = vectorGH/distanceGH;  % Specify the direction
Tip_velGH = unit_vecGH*des_vel;     % tip's velocity vector 
finish_timeGH = distanceGH/des_vel; % total time [sec] using from start to goal points
pointsGH = round(distanceGH);  % continuous points on the line
gapGH = distanceGH/pointsGH;  % let use minimum gap at 1mm
gap_timeGH = finish_timeGH/pointsGH;  % a tiny time [sec] used to travel in a gap

total_points = pointsAB + pointsBC + pointsCD + pointsDE + pointsEF +  pointsFG +  pointsGH;

pxAB = zeros(1,pointsAB);
pyAB = zeros(1,pointsAB);
pxBC = zeros(1,pointsBC);
pyBC = zeros(1,pointsBC);
pxCD = zeros(1,pointsCD);
pyCD = zeros(1,pointsCD);
pxDE = zeros(1,pointsDE);
pyDE = zeros(1,pointsDE);
pxEF = zeros(1,pointsEF);
pyEF = zeros(1,pointsEF);
pxFG = zeros(1,pointsFG);
pyFG = zeros(1,pointsFG);
pxGH = zeros(1,pointsGH);
pyGH = zeros(1,pointsGH);

for k=1:pointsAB
    if k == 1
        pxAB(k) = PX_A + unit_vecAB(1)*gapAB;
        pyAB(k) = PY_A + unit_vecAB(2)*gapAB;
        
        pxCD(k) = PX_C + unit_vecCD(1)*gapCD;
        pyCD(k) = PY_C + unit_vecCD(2)*gapCD;
        
        pxEF(k) = PX_E + unit_vecEF(1)*gapEF;
        pyEF(k) = PY_E + unit_vecEF(2)*gapEF;
        
        pxGH(k) = PX_G + unit_vecGH(1)*gapGH;
        pyGH(k) = PY_G + unit_vecGH(2)*gapGH;
    else
        pxAB(k) = pxAB(k-1) + unit_vecAB(1)*gapAB;
        pyAB(k) = pyAB(k-1) + unit_vecAB(2)*gapAB;
        
        pxCD(k) = pxCD(k-1) + unit_vecCD(1)*gapCD;
        pyCD(k) = pyCD(k-1) + unit_vecCD(2)*gapCD;
        
        pxEF(k) = pxEF(k-1) + unit_vecEF(1)*gapEF;
        pyEF(k) = pyEF(k-1) + unit_vecEF(2)*gapEF;
            
        pxGH(k) = pxGH(k-1) + unit_vecGH(1)*gapGH;
        pyGH(k) = pyGH(k-1) + unit_vecGH(2)*gapGH;
    end
end

for k=1:pointsBC
    if k == 1
        pxBC(k) = PX_B + unit_vecBC(1)*gapBC;
        pyBC(k) = PY_B + unit_vecBC(2)*gapBC;
        
        pxDE(k) = PX_D + unit_vecDE(1)*gapDE;
        pyDE(k) = PY_D + unit_vecDE(2)*gapDE;
        
        pxFG(k) = PX_F + unit_vecFG(1)*gapFG;
        pyFG(k) = PY_F + unit_vecFG(2)*gapFG;
    else
        pxBC(k) = pxBC(k-1) + unit_vecBC(1)*gapBC;
        pyBC(k) = pyBC(k-1) + unit_vecBC(2)*gapBC;
        
        pxDE(k) = pxDE(k-1) + unit_vecDE(1)*gapDE;
        pyDE(k) = pyDE(k-1) + unit_vecDE(2)*gapDE;
        
        pxFG(k) = pxFG(k-1) + unit_vecFG(1)*gapFG;
        pyFG(k) = pyFG(k-1) + unit_vecFG(2)*gapFG;
    end
end

px = [pxAB, pxBC, pxCD, pxDE, pxEF, pxFG, pxGH];
py = [pyAB, pyBC, pyCD, pyDE, pyEF, pyFG, pyGH];

Q1 = zeros(1,length(px));
Q2 = zeros(1,length(px));

for k =1:total_points
    %{
    Q = INV(px(j),py(j));
    Q1(j) = Q(1);
    Q2(j) = Q(2);
    %}
    
        L = sqrt(px(k)^2 + py(k)^2);

    pi_Q2 = acos( (px(k)^2 + py(k)^2 - a1^2 - a2^2)/(-2*a1*a2)); % Solution2
    Q2(k) = pi - pi_Q2;            % Solution1
    

    gam1 = asin( (a2*sin(Q2(k)))/L );
    gam2 = acos( (a2^2 - L^2 - a1^2)/(-2*L*a1) );
    

    alp = atan2(py(k),px(k));
   
    Q1(k) = alp - gam1;   % Solution1
    
end

for i=1:length(px)
    % Solution1 plot
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
hold off
pause(0.001)

end

%figure(1)


Theta1 = Q1*180/pi;
Theta2 = Q2*180/pi;

figure(2)
subplot(2,1,1)
plot(Theta1)
ylabel('\theta_1')
grid on
subplot(2,1,2)
plot(Theta2)
ylabel('\theta_2')
grid on