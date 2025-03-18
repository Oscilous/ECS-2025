% This program sets the model parameters for the Quanser Multi-DoF
% Torsion experiment.

clear all

%% Parameters of Quanser Rotary Servo Module (SRV02)

%%% CONSTANT SRV02 PARAMETERS

% Armature Resistance (Ohm)
Rm = 2.6;
% Motor Torque Constant (N.m/A)
kt = 0.00767;
% Motor Back-EMF Constant (V.s/rd)
km = 0.00767;
% Internal Gear Ratio (of the Planetary Gearbox)
Kgi = 14;
% Gearbox Efficiency
eta_g = 0.90; % = 0.90
% Motor ElectroMechanical Efficiency
eta_m = 0.69;

%%% MOTOR INERTIA

% Rotor Inertia (kg.m^2)
Jm_rotor = 3.9e-7;
% Tachometer Armature Inertia, if any (kg.m^2)
Jtach = 7e-8;
% Motor Equivalent Inertia (kg.m^2)
Jm = Jm_rotor + Jtach;

%%% GEAR CONFIGURATION

% External Gears Inertias (kg.m^2)
% J24: 24-tooth Gear Inertia (on the Motor Shaft)
m24 = 0.005; % mass (kg)
r24 = 0.5 / 2 * 0.0254; % radius (m)
J24 = m24 * r24^2 / 2;
% J72: 72-tooth Gear Inertia (on the Potentiometer Shaft)
m72 = 0.030; % mass (kg)
r72 = 1.5 / 2 * 0.0254; % radius (m)
J72 = m72 * r72^2 / 2;
% J120: 120-tooth Gear Inertia (on the Load Shaft)
m120 = 0.083; % mass (kg)
r120 = 2.5 / 2 * 0.0254; % radius (m)
J120 = m120 * r120^2 / 2;

% Moment of inertia and viscous damping based external gear config
% on High Gear Configuration: (1x) 24-tooth gear, (2x) 72-tooth gear, (1x) 120-tooth gear
Kge = 5;
Kg = Kgi * Kge;
% Equivalent moment of inertia including load (kg.m^2)
Jg = J24 + 2 * J72 + J120;
% Equivalent Viscous Damping Coefficient as seen at the Load (N.m.s/rd)
B1 = 15e-3;

% Moment of inertia of load attached to load shaft of SRV02
Jl_ext = 0.0001; % LOAD_TYPE - TORSION_2DOF

% Load moment of inertia: gears and external (kg.m^2)
J_load = Jg + Jl_ext;
% Equivalent moment of inertia including load (kg.m^2)    
J1 = Kg^2 * Jm * eta_g + J_load;

%%% FLEXIBLE COUPLING STIFFNESS (for 1 DOF Torsion experiment
Ks = 1.0;

%% Parameters of Quanser 1-DoF Torsion module

% Load support bar length (m)
Lb = 0.044;     % = 0.102

% Load support bar mass (kg)
Mb = 0.21;      % = 0.021

% Load support bar inertia (kg.m^2)
Jb = Mb * Lb^2 / 12;  % = 1.8207e-005

% Disc weight mass (kg)
Mw = 0.0022; % = 0.123

% Disc weight diameter (m)
Dw = 0.038; % = 1.5"

% Disc weight inertia about its cog (kg.m^2)
Jw_cog = Mw*(Dw/2)^2/2; % = 2.2202e-005

% inertial disc position on support arm
% distance from the pivot axis to disc cog (m)
% position symmetry of the 2 discs is assumed
dpw = 44.3e-3; % DISC_POSITION: 'A' with maximum spacing

% Disc weight inertia about pivot (kg.m^2)
Jw_piv = Jw_cog + Mw * dpw^2; % = 2.6359e-004

% Total load inertia (kg.m^2)
J2 = Jb + 2 * Jw_piv; % = 5.4538e-004

% Flexible Coupling Equivalent Viscous Damping Coefficient
% as seen at the Torsion module load shaft (N.m.s/rad)
B2 = 0.0015;

%% This section sets the A,B,C and D matrices for the 1-DoF Torsion model.
% (Check the matrices based on your own calculations)

% State-space matrices relative to applied torque
A = [0 0 1 0;
    0 0 0 1;
    -Ks/J1 Ks/J1 -B1/J1 0;
     Ks/J2 -Ks/J2 0 -B2/J2];
B = [0; 0; 1/J1; 0];
C = eye(2,4);
D = zeros(2,1);

% Add actuator dynamics
B = Kg * eta_g * kt * eta_m * B / Rm;
A(3,3) = A(3,3) - Kg*km*B(3);
A(4,3) = A(4,3) - Kg*km*B(4);
    

