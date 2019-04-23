% This file is used to simulate the two-wheeled self-balanced robot
% The control strategy PID controller
% The robot is designed to forward under given trajectory without falling

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

I_Total = 2 * (I_Powertrain + 0.5*M_Wheel*R_Wheel^2);
% Inertial Terms
a1 = I_Total + (M_Body+M_Wheel)*R_Wheel^2;
a2 = M_Body * R_Wheel * L;
a3 = I_BodyCenter + M_Body * L^2;
a4 = M_Body * g * L;
% Driven Terms
a5 = 2 * T_Motor;
a6 = a5/W_Motor;

% Inner loop transfer function
Num_G1 = [(-a5*(a1+a2)), 0];
Den_G1 = [(a1*a3-a2^2), (a6*(a1+a3+2*a2)), (-a1*a4), (-a4*a6)];
Numerator_G1 = (1/Den_G1(1))*Num_G1;
Denominator_G1 = (1/Den_G1(1))*Den_G1;
G1 = tf(Numerator_G1, Denominator_G1)
% Check the poles and zeros of G1
  % disp('The poles of the inner loop transfer function G1 are:');
  % roots(Denominator_G1);
  % disp('The zeros of the inner loop transfer function G1 are:');
  % roots(Numerator_G1);

% Outer loop transfer function
Num_G2 = [-(a2+a3), 0, a4];
Den_G2 = [(a1+a2), 0, 0];
Numerator_G2 = (1/Den_G2(1))*Num_G2;
Denominator_G2 = (1/Den_G2(1))*Den_G2;
G2 = tf(Numerator_G2, Denominator_G2);
% Check the poles and zeros of G2
  % disp('The poles of the outer loop transfer function G2 are:');
  % roots(Denominator_G2);
  % disp('The zeros of the outer loop transfer function G2 are:');
  % roots(Numerator_G2);

% Add PID controller
% Ideal rise time for the inner loop
t_rise1 = 0.05;
% Pick crossover frequency
w_crossover2 = 1.8/t_rise1;
% The following parameter are chosen after trials
K_p1 = -24.2558;
K_i1 = -85.1381;
K_Pid_Inner = -1.7275;
Gain1 = 1.0;
Pid_Inner = pid(K_p1, K_i1, K_Pid_Inner);
OpenLoopTf1 = minreal(Pid_Inner*G1);
CloseLoopTf1 = feedback(OpenLoopTf1, 1)
% Check the steady state of inner loop
figure;
opt = stepDataOptions('StepAmplitude',1);
step(CloseLoopTf1,opt);

% Outer loop controller
% Ideal rise time for the inner loop
t_rise1 = 0.35;
% Pick crossover frequency
w_crossover2 = 1.8/t_rise1;
% The following parameter are chosen after trials
K_p2 = 0.02;
K_i2 = 0.01;
K_Pid_Outer = 0.01;

Pid_Outer = pid(K_p2, K_i2, K_Pid_Outer, 1/w_crossover2);
OpenLoopTf2 = minreal(Pid_Outer*G2*CloseLoopTf1);
CloseLoopTf2 = feedback(OpenLoopTf2, 1);
figure;
phi = 0.3;
opt = stepDataOptions('StepAmplitude', phi);
step(CloseLoopTf2, opt);

% Now find the discritized transfer function
Digital_Inner = c2d(Pid_Inner, Ts, 'tustin');
DigitalTf_Inner = tf(Digital_Inner);
Digital_Outer = c2d(Pid_Outer, Ts, 'tustin');
DigitalTf_Outer = tf(Digital_Outer);


deltat = [0.001 0.005 0.01 0.02, 0.03]
inn1 = c2d(G1, deltat(1))
id1 = c2d(Pid_Inner,deltat(1), 'tustin');
outer1 = c2d(G2, deltat(1))
od1 = c2d(Pid_Outer, deltat(1), 'tustin');

inn2 = c2d(G1, deltat(2))
id2 = c2d(Pid_Inner,deltat(2), 'tustin');
outer2 = c2d(G2, deltat(2))
od2 = c2d(Pid_Outer, deltat(2), 'tustin');

inn3 = c2d(G1, deltat(3))
id3 = c2d(Pid_Inner,deltat(3), 'tustin');
outer3 = c2d(G2, deltat(3))
od3 = c2d(Pid_Outer, deltat(3), 'tustin');

inn4 = c2d(G1, deltat(4))
id4 = c2d(Pid_Inner,deltat(4), 'tustin');
outer4 = c2d(G2, deltat(4))
od4 = c2d(Pid_Outer, deltat(4), 'tustin');

inn5 = c2d(G1, deltat(5))
id5 = c2d(Pid_Inner,deltat(5), 'tustin');
outer5 = c2d(G2, deltat(5))
od5 = c2d(Pid_Outer, deltat(5), 'tustin');  
  