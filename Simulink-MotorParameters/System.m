% This script sets the model parameters for the SLS 3-D Printer

% Example: Specifying a Dynamics Block
% n = [1 2 3];
% d = [4 5 6];
% Transfer Function = (s^2 + 2s + 3) / (4s^2 + 5s + 6)

% ========================
% PHYSICAL UNIT CONVERSION
% ========================
% Example: if you decide to work in (Kg), all masses must be represented
%          in (Kg) but the spec sheet may provide masses in (g)



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Over-write the default values from DEFAULT.m %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% ==========================
% Choose Motors
% ==========================

% Motor Unit Conversions
% ----------------------


% ==========================
% Motor Parameters
% ==========================

% Maximum Current
% ---------------


% =============================
% Q0 : Rotation about y-axis
% =============================

% Amplifier Dynamics: models current driver converting pwm to voltage
%assume linear?
Amp0n   = [12];               % Numerator
Amp0d   = [255];               % Denominator
AmpSat0 = 12;

% Electrical Motor Dynamics
Elec0n  = [1];               % Numerator
Elec0d  = [.00105 2.25];               % Denominator

% Torque Const & Back EMF
TConst0  = 0.001574542;
BackEMF0 = 0.09331;

% Mechanical Motor Dynamics
Mech0n  = [1];               % Numerator
Mech0d  = [0.0000002059 0.0000009585];               % Denominator
JntSat0 =  Big;

% Sensor Dynamics
Sens0    =  0;
SensSat0 =  Big;

% Static Friction
StFric0 = 0;

% =============================
% Q1 : Rotation about x-axis
% =============================

% Amplifier Dynamics: models current driver converting pwm to voltage
%assume linear?
Amp1n   = [12];               % Numerator
Amp1d   = [255];               % Denominator
AmpSat1 = 12;

% Electrical Motor Dynamics
R = 2.25;
L = .00105;
Elec1n  = [1];               % Numerator
Elec1d  = [.00105 2.25];               % Denominator

% Torque Const & Back EMF
TConst1  = 0.0193;
BackEMF1 = 0.001545;

% Mechanical Motor Dynamics
J = .0000025152;
B = .00001854; 
Mech1n  = [1];               % Numerator
Mech1d  = [0.0000025152 0.00001854];               % Denominator
JntSat1 =  Big;

% Sensor Dynamics
Sens1    =  0;
SensSat1 =  Big;

% Static Friction
StFric1 = 0;
% ==================
% TRANSFER FUNCTIONS
% ==================
myTF = tf(TConst1, [ (L*J), (L*B + J*R), (B*R + TConst1*BackEMF1)] ); 
stepplot(myTF); 
display(myTF); 
% You may prefer to put this section in a separate .m file
