% Two Wheel Self-Balancing Robot: LQR Control Design

% Constant parameters of the two wheeled self-balanced robot
 
% Sampling Time
Ts = 0.01;
% Mass of a sigle wheel (unit:kg)
M_Wheel = 0.022;
% Mass of the robot body without wheel (unit:kg)
M_Body = 1.038;
% Inertial of the body around its barycenter (unit:kg*m^2)
I_BodyCenter = 0.004;
% Inertial of powertrain system around the wheel axis (unit:kg*m^2)
I_Powertrain = 2.11*10^(-4);
% Radius of a wheel (unit:m)
R_Wheel = 0.04;
% Length between center of wheel and barycenter (unit:m)
L = 0.1;
% Acceleration of gravity (unit:m/(s^2))
g = 9.81;
% Gear ratio (unit:1)
r_Gear = 20.4;
% Output stall torque of a single motor (unit: N*m)
T_Motor = 0.0276;
% Output free run speed of the motor (unit: n/s)
W_Motor = 40;
% Drive voltage of the motor (unit: V)
V_Motor = 12.0;


% Finalize the parameter in dynamic model
I_Total = 2 * (I_Powertrain+(M_Wheel*R_Wheel^2)/2);

% statespace model including motor dynamics
a1 = I_Total + (M_Body + M_Wheel)*R_Wheel^2;
a2 = M_Body * R_Wheel * L;
a3 = I_BodyCenter + M_Body*L^2;
a4 = M_Body * g * L;

% motor equation used: t = e*u - f*w
b1 = 2 * T_Motor; % stall torque of two motors
b2  = b1 / W_Motor;   % constant provides zero torque @ free run

%simplifications
c1 = 1 - (a2^2)/(a1*a3);
c2 = 1 + (a2/a3);
c3 = 1 + (a2/a1);

%theta
%thetadot
%phi
%phidot

% create state space represetation
A = [0 1 0 0;
     a4/(a3*c1) -(b2*c3)/(a3*c1) 0 (b2*c3)/(a3*c1);
     0 0 0 1;
     -(a2*a4)/(a1*a3*c1) (b2*c2)/(a1*c1) 0 -(b2*c2)/(a1*c1)];
B = [0 -(b1*c3)/(a3*c1) 0 (b1*c2)/(a1*c1) ]';
C = [1 0 0 0;
     0 0 1 0];
D = [0 0]';

states = {'theta' 'theta_dot' 'phi' 'phi_dot'};
inputs = {'u'};
outputs = {'theta'; 'phi'};
sys_ss = ss(A,B,C,D,'statename',states,'inputname',inputs,'outputname',outputs);
sys_tf = tf(sys_ss);
% poles, and examine the controllability
poles = eig(A);
controllability = rank(ctrb(sys_ss));

%              -171.1 s
%  ---------------------------------
%  s^3 + 13.17 s^2 - 94.44 s - 487.2

%
%LQR Design
%Tune Gains Parameters Here and check its response
Q = [50 00 0 0
     0 1 0 0
     0 0 1 0
     0 0 0 1];
R = 100;
K = lqr(A,B,Q,R)

Ac = [(A-B*K)];
Bc = [B];
Cc = [C];
Dc = [D];

% find Nbar for precompensator
Cn = [0 0 1 0];
s = size(A,1);
Z = [zeros([1,s]) 1];
N = inv([A,B;Cn,0])*Z';
Nx = N(1:s);
Nu = N(1+s);
Nbar=Nu + K*Nx

sys_cl = ss(Ac,Bc*Nbar,Cc,Dc,'statename',states,'inputname',inputs,'outputname',outputs)

t = 0:0.01:4;
dist = 0.1; % move balancebot 10cm
angle = (dist/R_Wheel)*ones(size(t));
angle(1) = 0;
[y,t,x]=lsim(sys_cl,angle,t);
figure;
[AX,H1,H2] = plotyy(t,y(:,1),t,R_Wheel*y(:,2),'plot');
set(get(AX(1),'Ylabel'),'String','body angle (radians)')
set(get(AX(2),'Ylabel'),'String','body position (meters)')
title('Step Response with LQR Control')

% find discrete time system
sys_d = c2d(sys_ss,Ts,'zoh')
A_d = sys_d.a;
B_d = sys_d.b;
C_d = sys_d.c;
D_d = sys_d.d;
[K_d] = dlqr(A_d,B_d,Q,R)

% discrete time closed loop
A_dc = [(A_d-B_d*K_d)];
B_dc = [B_d];
C_dc = [C_d];
D_dc = [D_d];

Nbar_d = K_d(3) % adjust Nbar for steadystate performance

states = {'x' 'x_dot' 'phi' 'phi_dot'};
inputs = {'phi'};
outputs = {'x'; 'phi'};
sys_dcl = ss(A_dc,B_dc*Nbar_d,C_dc,D_dc,Ts,'statename',states,'inputname',inputs,'outputname',outputs);

t = 0:0.01:4;
% Set a destination: move balancebot 10cm
dist = 0.3; 
angle = (dist/R_Wheel)*ones(size(t));
[y,t,x]=lsim(sys_dcl,angle,t);
figure
[AX,H1,H2] = plotyy(t,y(:,1),t,R_Wheel*y(:,2),'plot');
set(get(AX(1),'Ylabel'),'String','body angle (radians)')
set(get(AX(2),'Ylabel'),'String','body position (meters)')
title('Discrete Step Response with LQR Control')

%% video
animation(t, y(:,1), y(:,2), R_Wheel, true)

%% different DT

deltat = [0.01, 0.03, 0.05, 0.1 1];
m = length(deltat);
body = [];

for i = 1:m
    sys_d1 = c2d(sys_ss,deltat(i),'zoh')
    A_d1 = sys_d.a;
    B_d1 = sys_d.b;
    C_d1 = sys_d.c;
    D_d1 = sys_d.d;
    [K_d1] = dlqr(A_d1,B_d1,Q,R)

    A_dc1 = [(A_d1-B_d1*K_d1)];
    B_dc1 = [B_d1];
    C_dc1 = [C_d1];
    D_dc1 = [D_d1];

    Nbar_d1 = K_d1(3) % adjust Nbar for steadystate performance

    states = {'x' 'x_dot' 'phi' 'phi_dot'};
    inputs = {'phi'};
    outputs = {'x'; 'phi'};
    sys_dc2 = ss(A_dc1,B_dc1*Nbar_d1,C_dc1,D_dc1,deltat(i),'statename',states,'inputname',inputs,'outputname',outputs);

    t = 0:deltat(i):10;
    dist = 0.1; % move balancebot 10cm
    angle1 = (dist/R_Wheel)*ones(size(t));
    [y1,t,x1]=lsim(sys_dc2,angle1,t);
    figure
    [AX,H1,H2] = plotyy(t,y1(:,1),t,R_Wheel*y1(:,2),'plot');
    set(get(AX(1),'Ylabel'),'String','body angle (radians)')
    set(get(AX(2),'Ylabel'),'String','body position (meters)')
    title('Discrete Step Response with LQR Control and Precompensator')
end
