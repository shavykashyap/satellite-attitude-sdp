function [u_opt, omega_opt, q_opt, alpha] = solveSDPControl(u)
    % ============================================================
    % solveSDPControl: Solves an SDP-based control optimization 
    % for quaternion-based spacecraft control with constraints.
    %
    % INPUTS:
    %   - w                 : Sun vector (3x1) (inertial coordinates)
    %   - v_b               : Boresight vector (3x1) (body coordinates)
    %   - cos_th            : Minimum angular separation (degrees)
    %   - d_t               : Sampling time (seconds)
    %   - J                 : Spacecraft inertia matrix (3x3)
    %   - q_d               : Desired quaternion (4x1)
    %   - q                 : Current quaternion (4x1)
    %   - omega_N           : Desired Angular Velocity (3x1)
    %   - omega             : Angular Velocity (3x1)    
    %   - omega_discrete    : Discrete Angular Velocity (3x1)
    %   - mu_i              : Maximum Eigenvalue of the -A_tilde (1x1)
    %   - M                 : M = mu_i * eye(4) + A_tilde (4x4)
    %   - H                 : (4x10)
    %
    % OUTPUTS:
    %   - u_opt             : Optimal control torque (3x1)
    %   - omega_opt         : Optimal angular velocity (3x1)
    %   - q_opt             : Optimal quaternion (4x1)
    %   - alpha             : Optimal Slack Variable (1x1)
    % ============================================================

    %% Ensure `u` is a column vector
    u = reshape(u, [31, 1]);

    % Extract Required Variables from `u`
    w       = u(1:3);               % Sun vector(inertial) (3x1)
    v_b     = u(4:6);               % Boresight vector(body) (3x1)
    cos_th  = u(7);                 % Cosine of Minimum angular separation (scalar)
    d_t     = u(8);                % Sampling time (scalar)
    J       = reshape(u(9:17), 3, 3); % Spacecraft inertia matrix (3x3)
    q_N     = u(18:21);          % Desired quaternion (4x1)
    q       = u(22:25);            % Current quaternion (4x1)
    omega_N = u(26:28);      % Desired Angular Velocity (3x1)
    omega   = u(29:31);        % Current Angular Velocity (3x1)

    % Previous timestep 
    % u_k_1 = u(32:34);
    % omega_k_1 = u(35:37);
    % q_k_1 = u(38:41);
    % alpha_k_1 = u(42);


    %% Constants 

    par.w = w;
    par.v_b = v_b;

    %% Initialize YALMIP

    yalmip('clear');

    %% Define SDP Decision Variables

    u_opt     = sdpvar(3, 1, 'full');               % Control torques
    omega_opt = sdpvar(3, 1, 'full');           % Angular velocities (body rates)
    q_opt     = sdpvar(4, 1, 'full');               % Quaternions (orientation states)
    alpha     = sdpvar(1, 1);                       % Slack variable for constraint minimization

    % Define decision variable x(k)
    x_k = [u_opt; 
           omega_opt; 
           q_opt];

    %% Quaternion-Based LMI Constraint

    % Matrix to Isolate Quaternion in x_k
    H = [zeros(4,6), eye(4,4)];  

    % Construct A_tilde Matrix
    A = v_b * w' + w * v_b' - (v_b' * w + cos_th) * eye(3);
    b = cross(w, v_b);
    d = v_b' * w - cos_th;
    A_tilde = [A,  b; 
               b', d];

    % Compute Mu_i (Largest Eigenvalue of -A_tilde)
    neg_A_tilde = -1 * A_tilde;
    eig_neg_A_tilde = eig(neg_A_tilde);
    max_eig_neg_A_tilde = max(eig_neg_A_tilde);
    mu_i = max_eig_neg_A_tilde; 

    % Construct Matrix M
    M = mu_i * eye(4) + A_tilde;

    % Check if M is PSD (Positive Semi-Definite)
    % if isequal(M, M')  % Check Symmetry
    %     if min(eig(M)) >= 0
    %         disp('M is symmetric and PSD.');
    %     else
    %         disp('M is symmetric but not PSD.');
    %     end
    % else
    %     disp('M is not symmetric.');
    % end

    % check constraint violation
    % att_const = q'*A_tilde*q;   % (must be negative all the time)

    % LMI Constraint for Boresight Constraint
    Boresight_LMI_Constraint = [mu_i, (H * x_k)'; 
                                (H * x_k), inv(M)] >= 0;

    %% Define Angular Velocity 

    % Angular Velocity Constraint: |ω| ≤ γ_1
    G_1 = [zeros(3,3), eye(3), zeros(3,4)];

    % angular velocity is in deg/s as is in the simulink model 
    gamma_1 = 10 ;                      % Max Angular Velocity (10 degrees/sec)
    AngularVelocityConstraint = abs(G_1 * x_k) <= gamma_1 * [1;1;1];

    %% Define Torque Constraints

    % Torque Constraint: |u| ≤ γ_2
    G_2 = [eye(3), zeros(3,7)];

    % Torque is in N-m as is in the simulink model 
    gamma_2 = 2e-4;                             % Max Control Torque(2e-4 Newton-meters)
    TorqueConstraint = abs(G_2 * x_k) <= gamma_2 * [1;1;1];

    %% Define the Desired Quaternion Constraint

    Q_N = [ q_N(1),   q_N(4),  -q_N(3),  -q_N(2);
           -q_N(4),   q_N(1),   q_N(2),  -q_N(3);
            q_N(3),  -q_N(2),   q_N(1),  -q_N(4);
            q_N(2),   q_N(3),   q_N(4),   q_N(1)]; % (modified since matlab has different q-representation)


    % Compute the E_k Matrix (11x11)
    % This is also formed from final quaternions and final angular
    % velocities
    E_k = [zeros(3,3), zeros(3,3), zeros(3,4), zeros(3,1);
           zeros(3,3), eye(3),     zeros(3,4), -omega_N;
           zeros(4,3), zeros(4,3), zeros(4,4), zeros(4,1);
           zeros(1,3), -omega_N',  zeros(1,4), omega_N' * omega_N] ...
        + [zeros(6,6), zeros(6,4), zeros(6,1);
           zeros(4,6), Q_N' * Q_N, zeros(4,1);
           zeros(1,6), zeros(1,4), zeros(1,1)];

    % Reformulated Quaternion LMI Constraint
    quaternion_LMI_constraint = [alpha, (sqrtm(E_k) * [x_k;1])'; 
                                  sqrtm(E_k) * [x_k;1], E_k] >= 0;

    %% Kinematic Constraints   
    
    R_k = (d_t/2)* ...   
         [ -q(1),   q(4),  -q(3);
           -q(4),  -q(1),   q(2);
            q(3),  -q(2),  -q(1);
            q(2),   q(3),   q(4) ]; % (modified since matlab has differnet q-representation)

    F_k = [-eye(3), J, zeros(3,4);
            zeros(4,3), R_k , eye(4)];

    
    y_k = [J(1,1)* omega(1) + d_t * (J(2,2)-J(3,3) * omega(2) * omega(3));
           J(2,2)* omega(2) + d_t * (J(3,3)-J(1,1) * omega(3) * omega(1));
           J(3,3)* omega(3) + d_t * (J(1,1)-J(2,2) * omega(1) * omega(2));
           q];
    
    % Define Kinematic Constraint as an LMI
    Kinematic_LMI_Constraint = [ eye(7),                F_k * x_k - y_k;
                                (F_k * x_k - y_k)',     eye(1)          ] >= 0; % Without the LMI, the constraint will cause a singularity in the simulation.
    
    %% Solve SDP Optimization Problem

    % Objective = alpha;
    Constraints = [Boresight_LMI_Constraint,... 
                   TorqueConstraint,...
                   AngularVelocityConstraint,...
                   quaternion_LMI_constraint,...
                   Kinematic_LMI_Constraint];

    options = sdpsettings('solver', 'mosek', 'verbose', 0);
    sol = optimize(Constraints, alpha, options);

    %% Extract Optimal Values (Only if Solver Succeeds)
    if sol.problem == 0
        u_opt = value(u_opt);
        omega_opt = value(omega_opt);
        q_opt = value(q_opt);
        q_opt = q_opt/norm(q_opt);
        alpha = value(alpha);

    else
        % If solver fails, return NaN or zero values as fallbacks
        warning("SDP Solver failed: %s", sol.info);
        epsilon = 1e-6; 
        u_opt = epsilon * eye(3, 1);
        omega_opt = omega;
        q_opt = q;
        alpha = NaN;
        % u_opt = u_k_1;
        % omega_opt = omega_k_1;
        % q_opt = q_k_1;
        % alpha = alpha_k_1;
    end
end