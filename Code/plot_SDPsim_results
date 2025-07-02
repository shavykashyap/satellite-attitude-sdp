%% Build a time vector that matches ALL logged signals
dt        = time(2) - time(1);
time_plot = [time , time(end)+dt];   % now length = 61

%% 1) Guidance quaternion  q_ref  (4×61)  -------------------------------
q_ref = simOut.q_ref.Data;           % 61×4 ?

figure(1); clf
plot(time_plot, q_ref)
xlabel('Time [s]');  ylabel('Quaternion component');
legend({'q_0','q_1','q_2','q_3'},'Location','best');
grid on;  title('Quaternion guidance history');
saveas(gcf,'sdp_outputs/guidance_quaternion_vs_time.png')

%% 2) Simulated (“real”) quaternion  q_meas ------------------------------
q_meas = simOut.sat_q.Data;          % 61×4

figure(2); clf
plot(time_plot, q_meas)
xlabel('Time [s]');  ylabel('Quaternion component');
legend({'q_0','q_1','q_2','q_3'},'Location','best');
grid on;  title('Quaternion time history');
saveas(gcf,'sdp_outputs/sim_quaternion_vs_time.png')

%% 3) Quaternion tracking error q_error  ---------------------------------
q_error = squeeze(simOut.Ref_q_error.Data).';   % 61×4

figure(3); clf
plot(time_plot, q_error)
xlabel('Time [s]');  ylabel('Quaternion error component');
legend({'q_{e0}','q_{e1}','q_{e2}','q_{e3}'},'Location','best');
grid on;  title('Quaternion-error time history');
saveas(gcf,'sdp_outputs/quaternion_error_vs_time.png')

%% 4) Measured body-rate ω_meas (rad/s) ----------------------------------
omega_meas = simOut.sat_omega.Data;  % 61×3

figure(4); clf
plot(time_plot, omega_meas)
xlabel('Time [s]');  ylabel('\omega  [rad/s]');
legend({'\omega_x','\omega_y','\omega_z'},'Location','best');
grid on;  title('Body angular-rate (measured)');
saveas(gcf,'sdp_outputs/angular_velocity_meas_vs_time.png')

%% 5) Guidance body-rate ω_ref (rad/s) -----------------------------------
omega_ref = simOut.w_ref.Data;       % 61×3

figure(5); clf
plot(time_plot, omega_ref)
xlabel('Time [s]');  ylabel('\omega  [rad/s]');
legend({'\omega_x^{ref}','\omega_y^{ref}','\omega_z^{ref}'},'Location','best');
grid on;  title('Body angular-rate (reference)');
saveas(gcf,'sdp_outputs/angular_velocity_ref_vs_time.png')

%% 6) Measured control torque  u_meas (N·m) ------------------------------
torque_meas = simOut.u_total.Data;   % 61×3

figure(6); clf
plot(time_plot, torque_meas)
xlabel('Time [s]');  ylabel('\tau  [N·m]');
legend({'\tau_x','\tau_y','\tau_z'},'Location','best');
grid on;  title('Control torque (measured)');
saveas(gcf,'sdp_outputs/torque_meas_vs_time.png')

%% 7) Reference / commanded torque  u_ref (N·m) --------------------------
torque_ref = simOut.u_ref.Data;      % 61×3

figure(7); clf
plot(time_plot, torque_ref)
xlabel('Time [s]');  ylabel('\tau  [N·m]');
legend({'\tau_x^{ref}','\tau_y^{ref}','\tau_z^{ref}'},'Location','best');
grid on;  title('Control torque (reference)');
saveas(gcf,'sdp_outputs/torque_ref_vs_time.png')

%% 8) Slack / cost variable  alpha ---------------------------------------
alpha = squeeze(simOut.alp_ref.Data);   % 1×61 → 61×1
alpha = alpha(:);                       % ensure column

figure(8); clf
plot(time_plot, alpha,'LineWidth',1.3)
xlabel('Time [s]');  ylabel('\alpha');
grid on;  title('SDP slack / cost variable \alpha');
saveas(gcf,'sdp_outputs/alpha_vs_time.png')

%% 9) Attitude-constraint value  qᵀAq (should stay ≤0) -------------------
att = squeeze(simOut.att_const_ref.Data);   % 1×61 → 61×1
att = att(:);                              % ensure column

figure(9); clf
plot(time_plot, att,'LineWidth',1.3)
xlabel('Time [s]');
ylabel('q^{T}Aq');
grid on
title('Attitude-constraint history');
saveas(gcf,'sdp_outputs/attitude_constraint_vs_time.png')