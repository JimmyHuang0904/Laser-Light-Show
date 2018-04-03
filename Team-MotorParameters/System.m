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
% Q0 : Bottom Motor
% =============================

% Amplifier Dynamics: models current driver converting pwm to voltage
%assume linear?
Amp0n   = [12];               % Numerator
Amp0d   = [255];               % Denominator
AmpSat0 = 12;

% Electrical Motor Dynamics
Elec0n  = [1];               % Numerator
Elec0d  = [.002383 8];               % Denominator
L0 = .002382;
R0 = 8;

% Torque Const & Back EMF
TConst0  = 0.074557;
BackEMF0 = 0.074557;

% Mechanical Motor Dynamics
Mech0n  = [1];               % Numerator
Mech0d  = [.022033 .001361005];               % Denominator
JntSat0 =  Big;
J0 = .022033;
B0 = .001361005; 
%B0 = .013; 


% Sensor Dynamics
Sens0    =  0;
SensSat0 =  Big;

% Static Friction
StFric0 = 0;

% =============================
% Q1 : Top Motor
% =============================

% Amplifier Dynamics: models current driver converting pwm to voltage
%assume linear?
Amp1n   = [12];               % Numerator
Amp1d   = [255];               % Denominator
AmpSat1 = 12;

% Electrical Motor Dynamics
R = 7;
L = .00609;
Elec1n  = [1];               % Numerator
Elec1d  = [0.000609 7];               % Denominator

% Torque Const & Back EMF
TConst1  = 0.093002;
BackEMF1 = 0.093002;

% Mechanical Motor Dynamics
J = .005034;
B = .001262; 
Mech1n  = [1];               % Numerator
Mech1d  = [.005034 .001262];               % Denominator
JntSat1 =  Big;

% Sensor Dynamics
Sens1    =  0;
SensSat1 =  Big;

% Static Friction
StFric1 = 0;
% ==================
% TRANSFER FUNCTIONS
% ==================
myTF0 = tf(TConst0, [ (L0*J0), (L0*B0 + J0*R0), (B0*R0 + TConst0*BackEMF0)] ); 
stepplot(myTF0); 
display(myTF0);

%myTF = tf(TConst1, [ (L*J), (L*B + J*R), (B*R + TConst1*BackEMF1)] ); 
%stepplot(myTF); 
%display(myTF); 
% You may prefer to put this section in a separate .m file
