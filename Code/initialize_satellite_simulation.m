clc;
clear;
close all;

%% -------------------- Initial Conditions --------------------

initialPosition        = [3649700.0; 3308200.0; -4676600.0];   % in meters
initialVelocity        = [6000; 4500; 3000];                   % in meters per second
altitude               = 800000;                               % satellite altitude in meters (LEO)
initialAngularVelocity = [0; 0; 0];                          % initial angular velocity (rad/s)
finalAngularVelocity   = [0; 0; 0];                      % final angular velocity (rad/s)

% 1
initialAttitude        =   [ 0.5; 0.5; 0.5; 0.5];   % wxyz
finalAttitude          =   [ 0.0258; 0.0258; 0.0258; 0.9990];   % wxyz

% finalAttitude          = [1;0;0;0];
% 2
% initialAttitude       =  [0.9063; 0.4226; 0; 0];                
% finalAttitude         =  [0.9239; 0.3827; 0; 0];

% 3
% initialAttitude       = [0.7071;0.7071; 0; 0];                  
% finalAttitude         = [0.769; 0.369; 0.369; 0.369];

% Normalization
initialAttitude         = initialAttitude/norm(initialAttitude);
finalAttitude           = finalAttitude/norm(finalAttitude);


%% -------------------- Satellite Specifications --------------------

Cross_section_area    = 0.01;      % Cross-sectional area in m^2 (1U CubeSat)
mass                  = 1.33;      % Mass in kilograms (1U CubeSat)
sideLength            = 0.1;       % Side length in meters (10 cm)

%% -------------------- Inertia Tensor --------------------

J = calculateInertiaTensor(mass, sideLength);   % spacecraft inertia matrix

% J = eye(3);
% v = [100, 250, 350];
% J = diag(v);

%% -------------------- Constrained Attitude Problem --------------------

% System Parameters

theta_min   = 25 * (pi / 180);                            % minimum angular separation (radians)
cos_th      = cos(theta_min);
delta_t     = 0.5;                                          % sampling time (seconds)

w           = [0; 0; 1];                                  % Sun vector (inertial coordinates)
v_b         = [0.750; 0.433; 0.500];                      % Boresight vector (body frame, z-axis)

%% ================= Preallocation ===================


% Initial state
q_curr   = initialAttitude;              % Unit quaternion  (reordering for algorithm)
q_curr   = [q_curr(2:4);q_curr(1)];      % reorder for algorithm ( scalar last) 

omega_curr = initialAngularVelocity;     % Angular velocity

% Desired final state
q_N = finalAttitude ;               %  target quaternion (reordering for algorithm)
omega_N = finalAngularVelocity;     %  target state

% Simulation parameters
T_total  = 30;                  % Total time [sec]
dt       = delta_t;              % Time step [sec]
N        = T_total / dt;         % Number of steps
% N = 3;

% T_total = N* dt;

% Pre-allocate
q_traj      = zeros(4, N);      % matlab representation for simulink
q_update    = zeros(4, N);      % reordered for update inside the loop
omega_traj  = zeros(3, N);
u_traj      = zeros(3, N);
alp_traj    = zeros(1, N);
att_const_traj = zeros(1,N);
time        = (0:N-1)*dt;

% Initialize with initial condition

q_init = [initialAttitude(2:4);initialAttitude(1)];
omega_init = initialAngularVelocity;

q_traj(:,1)     =  initialAttitude;
omega_traj(:,1) =  omega_init;

q_update(:,1) =  q_init;

% Final Attitude  
q_N_reordered = [q_N(2:4);q_N(1)];       % reorder for algorithm


% 
A = v_b * w' + w * v_b' - (v_b' * w + cos_th ) * eye(3);
b = cross(w,v_b);
d = v_b' * w - cos_th;
A_tilde = [A,  b;
           b', d];
% att_const_traj(1) = q_init' * A_tilde * q_init

%% ================= Offline Loop ====================

fprintf('Generating offline trajectory using SDP...\n');

tic;

for k = 1:N-1
    fprintf(' k = %d \n',k);
    omega_curr = omega_traj(:,k);
    q_curr     = q_update(:,k);
    

    [u_opt, omega_next, q_next, alp, att_const] = cvx_sdp2(w, v_b, cos_th, dt, J, q_N_reordered, q_curr, omega_N, omega_curr);

    q_next = q_next / norm(q_next);   % normalised 

    % u_opt
    % omega_next
    % q_next
    % alp
   
    % Store values

    u_traj(:,k) = u_opt;
    omega_traj(:,k+1) = omega_next;
    q_update(:,k+1) = q_next;      % sdp xyzw order
    alp_traj(k) = alp;
    att_const_traj(k+1) = att_const;
    att_const_traj;
    % Show progress at every 10% 
    if mod(k, round(N/10)) == 0 || k == 1
        elapsed = toc;
        percent_done = (k / (N-1)) * 100;
        fprintf('Progress: %.0f%% |alpha = %.4f| Step %d of %d | Elapsed Time: %.2f sec\n', percent_done, alp, k, N-1, elapsed);
        % fprintf("k = %d | alpha = %.4f ", k, alp);
    end
end


q_traj = [q_update(4, :); q_update(1:3, :)];

total_time = toc;
fprintf('Trajectory generation complete in %.2f seconds.\n', total_time);


% Save the results
save('SDP_OfflineResults.mat', 'time', 'q_traj', 'omega_traj', 'u_traj', 'alp_traj','att_const_traj');

%% 
load("SDP_OfflineResults.mat")
u_ref = timeseries(u_traj', time');
q_ref = timeseries(q_traj', time');
omega_ref = timeseries(omega_traj', time');
alp_ref = timeseries(alp_traj,time);
att_const_ref = timeseries(att_const_traj',time);

%% -------------------- Controller Gains --------------------
% 
% k_p = 0.32;
% k_d = 0.8;

% k_p = 0;
% k_d = 0;


% Best so far 
k_p = 1.0;
k_d = 0.02;
% k_d = 0.015;
% k_p = 0.07;
% k_d = 0.0008;

% simOut = sim('Copy_of_SDP_only_Dynamics_test.slx');   
% 
% plot_boresight_cone
%% 3.  RUN THE MODEL  -----------------------------------------------------
% tic
simOut = sim('Copy_of_SDP_only_Dynamics_test.slx');   
% toc

%% Plots
% plot_boresight_cone(theta_min, v_b,w,initialAttitude,finalAttitude)

plot_boresight_cone

plot_SDPsim_results