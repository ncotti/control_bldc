%% Speed and torque control of a DC motor
clc; clear; close all;

%% Parameters
t = 0:0.001:5;

Kb = 2*120/190;         % Speed constant  [V/(rad/s)] (in Plecs Kb = Laf*Vf/Rf)
Kt = Kb;                % Torque constant [(N.m)/A]
La = 0.0043;            % Armature inductance [H]
Ra = 0.57;              % Armature Resistance [Ohm]
Jm = 0.0881;            % Motor inertia [N.m.s^2]
Bm = 0.5;               % Motor friction [N.m.s]

% PI representation for the torque, as:
% G(s) = Kc * (s + Zt)/s
Kc = 0.001;
Zt = 200;

u = @(t) heaviside(t);  % Input function

Mp = 5;                 % Desired Overshoot [%]
ts = 3;                 % 2% setting time [s]

%% Transfer function
G = tf( ...
    Kc*Kt*[1, Zt], ...
    [La*Jm, La*Bm + Jm*Ra + Kc*Jm, Bm*Ra + Kt*Kb + Kc*Bm + Kc*Jm*Zt, Kc*Bm*Zt]);
H = 1;

% Use these poles to calculate the PID transfer function
control_get_plc(Mp, ts)

% Gain of the PID: Kc = 20; Z = 1.45
GPID = 20*tf([1, 1.45],[1 0]);

% Plot root locus and closed loop temporal response
control_feedback(G*GPID, H, t, u);

% Kp = 20; Ki = 29
control_tf2pid(GPID)

