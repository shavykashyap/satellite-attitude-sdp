clc;
clear;
close all;

%% -------------------- Initial Conditions --------------------

initialPosition       = [3649700.0; 3308200.0; -4676600.0];   % in meters
initialVelocity       = [6000; 4500; 3000];                   % in meters per second
initialAttitude       = [1, 0, 0, 0];                         % initial quaternion
% initialAttitude     = [0.9063, 0.4226, 0, 0];               % alternative initial quaternion 

altitude              = 800000;                               % satellite altitude in meters (LEO)
initialAngularVelocity= [0.001; 0.002; 0];                    % initial angular velocity

finalAttitude         = [cosd(90/2); 0; 0; sind(90/2)];
% finalAttitude         = [-1/sqrt(2); 0; 0; -1/sqrt(2)];        % final quaternion

% Alternatives:
% finalAttitude       = [0.5; -0.5; 0.5; 0.5];
% finalAttitude       = [0.769; 0.369; 0.369; 0.369];
% finalAttitude       = [0.0258; 0.0258; 0.0258; 0.9990];

finalAngularVelocity  = [0; 0; 0];                            % final angular velocity

%% -------------------- Satellite Specifications --------------------

Cross_section_area    = 0.01;      % Cross-sectional area in m^2 (1U CubeSat)
mass                  = 1.33;      % Mass in kilograms (1U CubeSat)
sideLength            = 0.1;       % Side length in meters (10 cm)

%% -------------------- Inertia Tensor --------------------

J = calculateInertiaTensor(mass, sideLength);   % spacecraft inertia matrix

%% -------------------- Controller Gains --------------------

k = 0.32;
d = 0.8;

%% -------------------- Constrained Attitude Problem --------------------

theta_min   = 50 * (pi / 180);                            % minimum angular separation (radians)
delta_t     = 0.1;                                        % sampling time (seconds)

w           = [0; 0; 1];                                  % Sun vector (inertial coordinates)
v_b         = [sin(theta_min); 0; cos(theta_min)];                                  % Boresight vector (body frame, z-axis)

% Compute boresight vector in inertial frame
% v_t = rotateVector(v_b, w, initialAttitude);     % (initial attitude used)

% Reference boresight alternatives (commented)
% v_b_inertial = [0; 0.7660; 0.6428];                     % we get this when we use initialAttitude = [0.9063, 0.4226, 0, 0]; 
% v_b_inertial = [0.750; 0.433; 0.500];                   % from reference books


%% 3.  RUN THE MODEL  -----------------------------------------------------
tic
sim('SDP_only_Dynamics_test.slx');
toc
