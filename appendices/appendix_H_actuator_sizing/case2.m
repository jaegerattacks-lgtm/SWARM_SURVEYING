%% CASE 2: GRADIENT ANALYSIS & MAX ANGLE
% Objective: Find the maximum slope the robot can climb without slipping.
clear; clc; close all;

%% 1. SYSTEM PARAMETERS
m_total = 6.9;      % [kg] Total Mass
r       = 0.065;    % [m] Wheel Radius
g       = 9.81;
Crr     = 0.06;     % Rolling Resistance (Rough terrain)
mu      = 0.7;      % Friction Coefficient (Rubber/Concrete)
a_lin   = 0.7;      % [m/s^2] Acceleration (From Case 1 Design)

%% 2. ANGLE SWEEP (0 to 60 degrees)
theta_deg = linspace(0, 60, 100);
theta_rad = deg2rad(theta_deg);

% --- FORCE CALCULATIONS ---
% 1. Inertial Force (F = ma) - Constant
F_inertia = m_total * a_lin;

% 2. Rolling Resistance (F = C_rr * N) - Decreases as slope increases
% Normal force N changes with cos(theta)
N_force = m_total * g * cos(theta_rad);
F_roll = N_force * Crr;

% 3. Gravitational Force (F = mg sin(theta)) - Increases with angle
F_gravity = m_total * g * sin(theta_rad);

% TOTAL REQUIRED TRACTION FORCE
F_total_req = F_inertia + F_roll + F_gravity;

% --- TRACTION LIMIT CHECK ---
% Max Force the tires can output before slipping (F = mu * N)
F_traction_limit = mu * N_force; 

%% 3. FIND MAX CLIMBABLE ANGLE
% Find the point where Required Force > Traction Limit
fail_index = find(F_total_req > F_traction_limit, 1);

if ~isempty(fail_index)
    max_angle = theta_deg(fail_index - 1);
    F_max_needed = F_total_req(fail_index - 1);
else
    max_angle = 60;
    F_max_needed = F_total_req(end);
end

fprintf('=== SLOPE ANALYSIS RESULTS ===\n');
fprintf('Acceleration Used: %.2f m/s^2\n', a_lin);
fprintf('Max Feasible Angle (before slip): %.1f degrees\n', max_angle);
fprintf('Total Traction Force at Max Angle: %.2f N\n', F_max_needed);

% Calculate Torque at this Max Angle (Just for your reference)
T_at_max = (21 * r) / 4; 
fprintf('Torque Required at Max Angle: %.3f N.m per motor\n', T_at_max);

%% 4. PLOTTING THE PHYSICS
figure('Color','w');
plot(theta_deg, F_total_req, 'LineWidth', 2.5, 'Color', [0 0.447 0.741]); % Blue Line
hold on;
plot(theta_deg, F_traction_limit, '--r', 'LineWidth', 2.5); % Red Dashed Line

% Mark the intersection
plot(max_angle, F_max_needed, 'ko', 'MarkerSize', 10, 'MarkerFaceColor', 'k');
text(max_angle+2, F_max_needed, sprintf('  Max Angle: %.1f^o', max_angle), 'FontSize', 10);

grid on;
legend('Required Traction Force', 'Friction Limit (Slip)', 'Location', 'NorthWest');
xlabel('Slope Angle (degrees)', 'FontSize', 12, 'FontWeight', 'bold');
ylabel('Force (Newtons)', 'FontSize', 12, 'FontWeight', 'bold');
title('Case 2: Gradient Capability Analysis', 'FontSize', 14);
subtitle('Intersection determines the physical limit of the robot');