
% =================
%    PID Values
% =================

KD0 = 0.0009;
KD1 = 1;

Nvalue = 100;

PID0=[0.731*KD0     0*KD1       1*KD0     Nvalue];
PID1=[205*KD1       0.01*KD1    1*KD1     Nvalue];

% =================
% Rotation Matrices
% =================

%R0_1 = [[cos(Theta1), -sin(Theta1), 0], [0, 0, -1],[sin(Theta1), cos(Theta1), 0]];
%R1_2 = [[cos(Theta2), -sin(Theta2), 0], [sin(Theta2), cos(Theta2), 0], [0, 0, 1]];

% ====================
% Displacement Vectors
% ====================

%D0_1 = [0, 0, Height];
%D1_2 = [Length2*cos(Theta2), Length2*sin(Theta2), 0];

% ====================
% Mech Dynamics
% ====================

Mech0n = [1];
Mech0d = [1];

Mech1n = [1];
Mech1d = [1];

% ====================
% Elec Dynamics
% ====================

Elec1n = [1];
Elec1d = [1];

% ====================
% Gains
% ====================

Sens0 = [1];
Sens1 = [1];
FB1 = [1];

% ====================
% Xd, Yd
% ====================

%zeta = 1;
%w0 = (4.65*zeta-1.3)/0.300; % overdamped
%w1 = (2.16*zeta+0.6)/0.300;
%Kfinal = 500;
%zeta = 0.362129654774;
%Kfinal = 305;
%wn = 44.9379419843;
%wn = 28;
%w0 = (2.16*zeta+0.6)/(0.104);

%Kd = 1;
%Kp = 20.28;
%Ki = 784;

%PID = tf([Kd, Kp/Kd, Ki/Kd], [1, 0]);
%PID = tf([1 32 256], [1, 0]);
%PID = I * PID;

%rlocus(PID)
Kfinal = 580;
Mp = (600-580)/580;
Zeta = sqrt((log(Mp))^2/((log(Mp))^2+pi^2));
wn = (2*pi/0.4)/sqrt(1-Zeta^2);

%Zeta = 0.73;
wn = 35;

H = tf(Kfinal*wn^2, [1, 2*Zeta*wn, wn^2]);
I = tf(Kfinal*wn^2, [1, 2*Zeta*wn, wn^2, 0]);

%PID stuff
Ku = 0.000918;
Kd = 1*Ku;
Ki = 0;
Kp = 24/Kd;
%rlocus(I)
PID = tf([Kd, Kp/Kd, Ki/Kd], [1, 0]);
TF_full = tf(1, I * PID + 1);


Mech0ntest = [Kfinal*wn^2];
Mech0dtest = [1 2*Zeta*wn wn^2];

clf;
hold on
stepplot(H)
plot(Time,Velocity)
hold off

%grid on;
%grid minor;