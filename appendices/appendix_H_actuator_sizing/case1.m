%% CASE 1: FLAT TERRAIN - ACCELERATION ANALYSIS
clear; clc; close all;

%% 1. SYSTEM PARAMETERS
m_total = 6.9;      % [kg] Total Mass (with suspension)
r       = 0.065;    % [m] Wheel Radius
g       = 9.81;
Crr     = 0.06;     % Rolling Resistance Coeff
v_max   = 0.7;      % [m/s] Max Speed
SF      = 1.5;      % Safety Factor

% Wheel Inertia (Estimated for 0.25kg wheel)
I_wheel = 0.5 * 0.25 * r^2; 

%% 2. SWEEP ANALYSIS (Torque vs Accel Time)
% Range from 0.2s (Fast) to 3.0s (Slow)
t_acc_range = linspace(0.2, 3.0, 100);

% Calculate Torque for the whole range
a_range = v_max ./ t_acc_range;
alpha_range = a_range ./ r;
F_roll = m_total * g * Crr;
F_inertia = m_total .* a_range;
Torque_Curve = ((F_roll + F_inertia) .* r) / 4 + (I_wheel .* alpha_range);

%% 3. CALCULATE AT CHOSEN POINT (t = 1.0 s)
t_chosen = 1.0;
a_chosen = v_max / t_chosen;
alpha_chosen = a_chosen / r;

F_total_chosen = (m_total * g * Crr) + (m_total * a_chosen);
Torque_Nominal = (F_total_chosen * r) / 4 + (I_wheel * alpha_chosen);
Torque_Design  = Torque_Nominal * SF;

% Display Results in Command Window
fprintf('=== RESULTS FOR t_acc = 1.0 s ===\n');
fprintf('Required Linear Accel: %.2f m/s^2\n', a_chosen);
fprintf('Nominal Torque:        %.4f N.m\n', Torque_Nominal);
fprintf('Design Torque (SF=1.5): %.4f N.m\n', Torque_Design);

%% 4. PLOTTING
figure('Color','w');
plot(t_acc_range, Torque_Curve, 'LineWidth', 2.5, 'Color', [0 0.447 0.741]);
grid on;
xlabel('Acceleration Time t_{acc} (s)', 'FontSize', 12, 'FontWeight', 'bold');
ylabel('Nominal Torque per Motor (N.m)', 'FontSize', 12, 'FontWeight', 'bold');
title('Case 1: Torque vs. Responsiveness Trade-off');

% Mark the chosen point
hold on;
plot(t_chosen, Torque_Nominal, 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');

% Add Text Box with values
msg = sprintf('  Selected Point (t = 1.0 s)\n  Nominal T = %.3f N.m\n  With S.F. = %.3f N.m', ...
              Torque_Nominal, Torque_Design);
text(t_chosen + 0.1, Torque_Nominal, msg, 'BackgroundColor', 'w', 'EdgeColor', 'k');

% Draw Safety Factor Line (Optional visualization)
yline(Torque_Design, '--r', 'Label', 'Design Torque Limit (w/ Safety Factor)');