% This script contains all physical constants and specifications for the
% Laser Light Show

%%%%%%%%%%%%%%%%%%%%%%
% Physical Constants %
%%%%%%%%%%%%%%%%%%%%%%
D = 10; 
G = 9.81;               % m/s^2
FtPerM = 3.28;          % ft/m
RadPerDeg = pi/180;     % rad/deg
DegPerRad = 180/pi;     % deg/rad

Big = 1e6;              % General purpose large number
Sml = 1e-6;             % General purpose small number

%%%%%%%%%%%%%%%%%%
% Specifications %
%%%%%%%%%%%%%%%%%%


%  Joint Controller
%     Amplifier
% ==================
Amp0n = [1];
Amp0d = [1];
Amp1n = [1];
Amp1d = [1];
%    Back EMF
% ==================
BackEMF0 = 1;
BackEMF1 = 1;

%  Rotary Joints
% ===============

%  Joint 1 - Between yaw motor and pitch motor
% =============================================
Height = 5;            % cm


%  Joint 2 - Between pitch motor and end effector
% ================================================
Length2 = 5;            % cm
