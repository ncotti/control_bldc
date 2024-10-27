%% Torque control of a DC Motor
clc; clear; close all;

%% Parameters
t = 0:0.01:100;           % Time vector

Kb = 2*120/190;         % Speed constant  [V/(rad/s)] (in Plecs Kb = Laf*Vf/Rf)
Kt = Kb;                % Torque constant [(N.m)/A]
La = 0.0043;            % Armature inductance [H]
Ra = 0.57;              % Armature Resistance [Ohm]
Jm = 0.0881;            % Motor inertia [N.m.s^2]
Bm = 0.5;               % Motor friction [N.m.s]

u = @(t) heaviside(t)/Kt;  % Input function

Mp = 0.1;               % Desired Overshoot [times]
ts = 2;                 % 2% setting time [s]

%% Transfer function
G = tf([Jm, Bm], [La*Jm, La*Bm + Jm*Ra, Bm*Ra + Kt*Kb]);    % Gain of the system
H = 1;                                                      % Feedback gain

% Use this poles to calculate the PID transfer function
closed_loop_poles = control_get_plc(Mp, ts)

% Gain of the PID
GPID = 1*tf([1 40], [1 0]);

% Plot root locus and closed loop temporal response
control_feedback(G*GPID, H, t, u);

% Kp = 0.01; Ki = 0.2
control_tf2pid(GPID)

