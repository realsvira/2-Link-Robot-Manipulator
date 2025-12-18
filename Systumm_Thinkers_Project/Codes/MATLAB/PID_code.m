close all; clear; clc;

m_1 = 5;
m_2 = 3;
l_1 = 0.25;
l_2 = 0.15;
g = 9.81;
q_initial = [0.2; 0.15; 0; 0];
q_desired = [0; 0];
tspan = [0 10];
e_int_prev = [0; 0];
options = odeset('RelTol', 1e-6, 'AbsTol', 1e-6);

disp('Running PI Controller Simulation...');
kp_pi = 250; ki_pi = 3;
Kp_pi = [kp_pi, kp_pi]; Ki_pi = [ki_pi, ki_pi]; Kd_pi = [0, 0];
[t_pi, q_pi] = ode45(@(t, q) Dynamics(t, q, m_1, m_2, l_1, l_2, g, Kp_pi, Ki_pi, Kd_pi, q_desired, e_int_prev, tspan), tspan, q_initial, options);
e1_pi = q_desired(1) - q_pi(:,1);
e2_pi = q_desired(2) - q_pi(:,2);
e_pi = [e1_pi, e2_pi]';
e_dot_pi = [zeros(size(q_pi,1),1) - q_pi(:,3), zeros(size(q_pi,1),1) - q_pi(:,4)]';
e_int_pi = cumtrapz(t_pi, e_pi, 2);
tau_pi = diag(Kp_pi) * e_pi + diag(Ki_pi) * e_int_pi + diag([0,0]) * e_dot_pi;

disp('Running PD Controller Simulation...');
kp_pd = 900; kd_pd = 80;
Kp_pd = [kp_pd, kp_pd]; Kd_pd = [kd_pd, kd_pd]; Ki_pd = [0, 0];
[t_pd, q_pd] = ode45(@(t, q) Dynamics(t, q, m_1, m_2, l_1, l_2, g, Kp_pd, Ki_pd, Kd_pd, q_desired, e_int_prev, tspan), tspan, q_initial, options);
e1_pd = q_desired(1) - q_pd(:,1);
e2_pd = q_desired(2) - q_pd(:,2);
e_pd = [e1_pd, e2_pd]';
e_dot_pd = [zeros(size(q_pd,1),1) - q_pd(:,3), zeros(size(q_pd,1),1) - q_pd(:,4)]';
e_int_pd = cumtrapz(t_pd, e_pd, 2);
tau_pd = diag(Kp_pd) * e_pd + diag([0,0]) * e_int_pd + diag(Kd_pd) * e_dot_pd;

disp('Running PID Controller Simulation...');
kp_pid = 50; kd_pid = 50; ki_pid = 300;
Kp_pid = [kp_pid, kp_pid]; Kd_pid = [kd_pid, kd_pid]; Ki_pid = [ki_pid, ki_pid];
[t_pid, q_pid] = ode45(@(t, q) Dynamics(t, q, m_1, m_2, l_1, l_2, g, Kp_pid, Ki_pid, Kd_pid, q_desired, e_int_prev, tspan), tspan, q_initial, options);
e1_pid = q_desired(1) - q_pid(:,1);
e2_pid = q_desired(2) - q_pid(:,2);
e_pid = [e1_pid, e2_pid]';
e_dot_pid = [zeros(size(q_pid,1),1) - q_pid(:,3), zeros(size(q_pid,1),1) - q_pid(:,4)]';
e_int_pid = cumtrapz(t_pid, e_pid, 2);
tau_pid = diag(Kp_pid) * e_pid + diag(Ki_pid) * e_int_pid + diag(Kd_pid) * e_dot_pid;

disp('Generating consolidated plots...');

figure('Name', 'Joint Angle Analysis (Per Controller)');
sgtitle('Joint Angle Trajectories for Each Controller');
subplot(3,2,1); plot(t_pi, q_pi(:, 1), 'r'); grid on; title("PI: q1 (Kp=" + kp_pi + ", Ki=" + ki_pi + ")"); ylabel('q1 (rad)');
subplot(3,2,2); plot(t_pi, q_pi(:, 2), 'b'); grid on; title("PI: q2 (Kp=" + kp_pi + ", Ki=" + ki_pi + ")"); ylabel('q2 (rad)');
subplot(3,2,3); plot(t_pd, q_pd(:, 1), 'r'); grid on; title("PD: q1 (Kp=" + kp_pd + ", Kd=" + kd_pd + ")"); ylabel('q1 (rad)');
subplot(3,2,4); plot(t_pd, q_pd(:, 2), 'b'); grid on; title("PD: q2 (Kp=" + kp_pd + ", Kd=" + kd_pd + ")"); ylabel('q2 (rad)');
subplot(3,2,5); plot(t_pid, q_pid(:, 1), 'r'); grid on; title("PID: q1 (Kp=" + kp_pid + ", Ki=" + ki_pid + ", Kd=" + kd_pid + ")"); ylabel('q1 (rad)'); xlabel('Time (s)');
subplot(3,2,6); plot(t_pid, q_pid(:, 2), 'b'); grid on; title("PID: q2 (Kp=" + kp_pid + ", Ki=" + ki_pid + ", Kd=" + kd_pid + ")"); ylabel('q2 (rad)'); xlabel('Time (s)');

figure('Name', 'Error Analysis (Per Controller)');
sgtitle('Joint Angle Errors for Each Controller');
subplot(3,2,1); plot(t_pi, e1_pi, 'r'); grid on; title("PI Error: e1 (Kp=" + kp_pi + ", Ki=" + ki_pi + ")"); ylabel('e1 (rad)');
subplot(3,2,2); plot(t_pi, e2_pi, 'g'); grid on; title("PI Error: e2 (Kp=" + kp_pi + ", Ki=" + ki_pi + ")"); ylabel('e2 (rad)');
subplot(3,2,3); plot(t_pd, e1_pd, 'r'); grid on; title("PD Error: e1 (Kp=" + kp_pd + ", Kd=" + kd_pd + ")"); ylabel('e1 (rad)');
subplot(3,2,4); plot(t_pd, e2_pd, 'g'); grid on; title("PD Error: e2 (Kp=" + kp_pd + ", Kd=" + kd_pd + ")"); ylabel('e2 (rad)');
subplot(3,2,5); plot(t_pid, e1_pid, 'r'); grid on; title("PID Error: e1 (Kp=" + kp_pid + ", Ki=" + ki_pid + ", Kd=" + kd_pid + ")"); ylabel('e1 (rad)'); xlabel('Time (s)');
subplot(3,2,6); plot(t_pid, e2_pid, 'g'); grid on; title("PID Error: e2 (Kp=" + kp_pid + ", Ki=" + ki_pid + ", Kd=" + kd_pid + ")"); ylabel('e2 (rad)'); xlabel('Time (s)');

figure('Name', 'Control Torque Analysis (Per Controller)');
sgtitle('Control Torques for Each Controller');
subplot(3,2,1); plot(t_pi, tau_pi(1,:), 'r'); grid on; title("PI Torque: \tau_1 (Kp=" + kp_pi + ", Ki=" + ki_pi + ")"); ylabel('\tau_1 (Nm)');
subplot(3,2,2); plot(t_pi, tau_pi(2,:), 'b'); grid on; title("PI Torque: \tau_2 (Kp=" + kp_pi + ", Ki=" + ki_pi + ")"); ylabel('\tau_2 (Nm)');
subplot(3,2,3); plot(t_pd, tau_pd(1,:), 'r'); grid on; title("PD Torque: \tau_1 (Kp=" + kp_pd + ", Kd=" + kd_pd + ")"); ylabel('\tau_1 (Nm)');
subplot(3,2,4); plot(t_pd, tau_pd(2,:), 'b'); grid on; title("PD Torque: \tau_2 (Kp=" + kp_pd + ", Kd=" + kd_pd + ")"); ylabel('\tau_2 (Nm)');
subplot(3,2,5); plot(t_pid, tau_pid(1,:), 'r'); grid on; title("PID Torque: \tau_1 (Kp=" + kp_pid + ", Ki=" + ki_pid + ", Kd=" + kd_pid + ")"); ylabel('\tau_1 (Nm)'); xlabel('Time (s)');
subplot(3,2,6); plot(t_pid, tau_pid(2,:), 'b'); grid on; title("PID Torque: \tau_2 (Kp=" + kp_pid + ", Ki=" + ki_pid + ", Kd=" + kd_pid + ")"); ylabel('\tau_2 (Nm)'); xlabel('Time (s)');

figure('Name', 'Comparative Controller Performance');
sgtitle('Direct Comparison of PI, PD, and PID Controllers');
subplot(2,2,1);
plot(t_pi, q_pi(:,1), 'r--', 'DisplayName', 'PI'); hold on;
plot(t_pd, q_pd(:,1), 'g-.', 'DisplayName', 'PD');
plot(t_pid, q_pid(:,1), 'b-', 'DisplayName', 'PID');
grid on; legend; ylabel('q1 (rad)'); title('Joint 1 Trajectories'); hold off;
subplot(2,2,2);
plot(t_pi, q_pi(:,2), 'r--', 'DisplayName', 'PI'); hold on;
plot(t_pd, q_pd(:,2), 'g-.', 'DisplayName', 'PD');
plot(t_pid, q_pid(:,2), 'b-', 'DisplayName', 'PID');
grid on; legend; ylabel('q2 (rad)'); title('Joint 2 Trajectories'); hold off;
subplot(2,2,3);
plot(t_pi, tau_pi(1,:), 'r--', 'DisplayName', 'PI'); hold on;
plot(t_pd, tau_pd(1,:), 'g-.', 'DisplayName', 'PD');
plot(t_pid, tau_pid(1,:), 'b-', 'DisplayName', 'PID');
grid on; legend; ylabel('\tau_1 (Nm)'); xlabel('Time (s)'); title('Control Torques for Joint 1'); hold off;
subplot(2,2,4);
plot(t_pi, tau_pi(2,:), 'r--', 'DisplayName', 'PI'); hold on;
plot(t_pd, tau_pd(2,:), 'g-.', 'DisplayName', 'PD');
plot(t_pid, tau_pid(2,:), 'b-', 'DisplayName', 'PID');
grid on; legend; ylabel('\tau_2 (Nm)'); xlabel('Time (s)'); title('Control Torques for Joint 2'); hold off;

disp('Calculating performance metrics...');
metrics = struct();
controllers = {'PI', 'PD', 'PID'};
data = {{t_pi, q_pi, e1_pi, e2_pi}, {t_pd, q_pd, e1_pd, e2_pd}, {t_pid, q_pid, e1_pid, e2_pid}};

for i = 1:length(controllers)
    name = controllers{i};
    t = data{i}{1}; q = data{i}{2}; e1 = data{i}{3}; e2 = data{i}{4};
    metrics.(name).ss_error_q1 = e1(end);
    metrics.(name).ss_error_q2 = e2(end);
    overshoot_q1 = -min(q(:,1));
    overshoot_q2 = -min(q(:,2));
    metrics.(name).overshoot_q1 = max(0, overshoot_q1 / q_initial(1) * 100);
    metrics.(name).overshoot_q2 = max(0, overshoot_q2 / q_initial(2) * 100);
    tolerance_q1 = 0.02 * abs(q_initial(1));
    tolerance_q2 = 0.02 * abs(q_initial(2));
    idx_settle_q1 = find(abs(e1) > tolerance_q1, 1, 'last');
    idx_settle_q2 = find(abs(e2) > tolerance_q2, 1, 'last');
    if isempty(idx_settle_q1), metrics.(name).settling_time_q1 = t(1); else, metrics.(name).settling_time_q1 = t(idx_settle_q1); end
    if isempty(idx_settle_q2), metrics.(name).settling_time_q2 = t(1); else, metrics.(name).settling_time_q2 = t(idx_settle_q2); end
end

fprintf('\n================== Performance Metrics ==================\n');
fprintf('%-15s | %-12s | %-12s | %-12s \n', 'Metric', 'PI', 'PD', 'PID');
fprintf('----------------------------------------------------------\n');
fprintf('%-15s | %-12.4f | %-12.4f | %-12.4f \n', 'SS Error q1', metrics.PI.ss_error_q1, metrics.PD.ss_error_q1, metrics.PID.ss_error_q1);
fprintf('%-15s | %-12.4f | %-12.4f | %-12.4f \n', 'SS Error q2', metrics.PI.ss_error_q2, metrics.PD.ss_error_q2, metrics.PID.ss_error_q2);
fprintf('%-15s | %-12.2f | %-12.2f | %-12.2f \n', 'Overshoot q1 %%', metrics.PI.overshoot_q1, metrics.PD.overshoot_q1, metrics.PID.overshoot_q1);
fprintf('%-15s | %-12.2f | %-12.2f | %-12.2f \n', 'Overshoot q2 %%', metrics.PI.overshoot_q2, metrics.PD.overshoot_q2, metrics.PID.overshoot_q2);
fprintf('%-15s | %-12.3f | %-12.3f | %-12.3f \n', 'Settle Time q1', metrics.PI.settling_time_q1, metrics.PD.settling_time_q1, metrics.PID.settling_time_q1);
fprintf('%-15s | %-12.3f | %-12.3f | %-12.3f \n', 'Settle Time q2', metrics.PI.settling_time_q2, metrics.PD.settling_time_q2, metrics.PID.settling_time_q2);
fprintf('==========================================================\n');

disp('All simulations and analyses complete.');