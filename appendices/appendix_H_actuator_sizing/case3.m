%% CASE 3: OBSTACLE CLIMBING ANALYSIS (Wada 2006 Model)
clear; clc; close all;

%% 1. SYSTEM PARAMETERS
m_total = 6.9;        % [kg] Total Mass
r       = 0.065;      % [m] Wheel Radius (65mm)
g       = 9.81;
mu      = 0.7;        % Friction Coeff (Rubber/Concrete)
N_motors = 4;

% Motor Capacity (From Case 1 Design)
T_rated_per_motor = 0.22; % [N.m] (Approx. from your S.F. choice)
F_max_total = (T_rated_per_motor * N_motors) / r; 

%% 2. SWEEP STEP HEIGHT (h)
% We test from 0 to 65mm (Max radius)
h_range = linspace(0, 0.065, 100); 

% Calculate Collision Angle theta for each height
% cos(theta) = (r - h) / r
cos_theta = (r - h_range) ./ r;
theta_rad = acos(cos_theta);
theta_deg = rad2deg(theta_rad);

%% 3. CALCULATE REQUIREMENTS (Wada Eq. 7 & 11)
W_total = m_total * g;

% Required Total Force (Eq 7)
% F >= W * sin(theta) / (1 + cos(theta))
F_req_total = W_total .* (sin(theta_rad) ./ (1 + cos_theta));

% Required Torque per Motor
T_req_per_motor = (F_req_total .* r) ./ 4;

% Required Friction to avoid slip (Eq 11)
% mu >= (1 - cos(theta)) / sin(theta)
mu_req = (1 - cos_theta) ./ sin(theta_rad);

%% 4. FIND LIMITS
% A. Friction Limit (Where mu_req > 0.7)
idx_slip = find(mu_req > mu, 1);
if isempty(idx_slip)
    h_max_friction = 0.065;
else
    h_max_friction = h_range(idx_slip);
end

% B. Motor Limit (Where T_req > Rated Torque)
idx_stall = find(T_req_per_motor > T_rated_per_motor, 1);
if isempty(idx_stall)
    h_max_motor = 0.065;
else
    h_max_motor = h_range(idx_stall);
end

% C. The Real Limit (Minimum of the two)
[h_max_real, reason_idx] = min([h_max_friction, h_max_motor]);
limit_reasons = {'Friction (Slip)', 'Motor Torque'};
limit_reason = limit_reasons{reason_idx};

fprintf('=== OBSTACLE CLIMBING RESULTS ===\n');
fprintf('Max Possible Step Height: %.1f mm\n', h_max_real * 1000);
fprintf('Limiting Factor: %s\n', limit_reason);
fprintf('Collision Angle at Limit: %.1f degrees\n', theta_deg(find(h_range>=h_max_real,1)));

%% 5. PLOTTING
figure('Color','w');
yyaxis left
plot(h_range*1000, T_req_per_motor, 'LineWidth', 2.5);
ylabel('Required Torque (N.m)');
yline(T_rated_per_motor, '--b', 'Label', 'Rated Torque Limit');
xlabel('Step Height (mm)');

yyaxis right
plot(h_range*1000, mu_req, 'LineWidth', 2.5, 'Color', [0.85 0.32 0.09]);
ylabel('Required Friction (\mu)');
yline(mu, '--r', 'Label', 'Friction Limit (0.7)');
ylim([0 1.5]);

title('Case 3: Obstacle Negotiation Limits');
subtitle(sprintf('Max Height = %.1f mm (Limited by %s)', h_max_real*1000, limit_reason));
grid on;