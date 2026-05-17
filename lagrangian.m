function [h_ddot, F_thrust, F_grav, F_fict, F_fric] = ...
    lagrangian(h, h_dot, throttle, params)
% Lagrangian EOM for the duo-copter cart on a vertical rail.
%
%   M(h)*h_ddot = T(throttle) - [M_g(h) + dM_g/dh * h]*g
%                              - 0.5*(dM/dh)*h_dot^2 + F_fric
%
% Inputs
%   h        - cart height [m]
%   h_dot    - cart velocity [m/s]
%   throttle - throttle command [%]
%   params   - struct from rig()
%
% Outputs
%   h_ddot   - vertical acceleration [m/s^2]
%   F_thrust, F_grav, F_fict, F_fric - force components [N]

g = params.g;

% --- Cable-dependent masses ---
m_cable = params.cable_slope * h + params.cable_intercept;  % [kg]
dMdh    = params.cable_slope;   % [kg/m]  dM/dh = dM_g/dh (same coefficient)

% Inertial mass: counterweight accelerated through pulley → adds to KE
M   = params.m_fixed + m_cable + params.m_counterweight;

% Gravitational mass: counterweight falls while cart rises → subtracted
M_g = (params.m_fixed - params.m_counterweight) + m_cable;

% --- Thrust (two motors, interpolated from lookup) ---
throttle_clamped = max(0, min(throttle, params.throttle_max));
F_thrust = interp1(params.throttle_grid, params.thrust_grid, ...
    throttle_clamped, 'linear', 'extrap');
F_thrust = max(0, F_thrust);

% --- Gravity term (includes variable-mass correction) ---
F_grav = (M_g + dMdh * h) * g;

% --- Fictitious force from variable inertial mass ---
F_fict = 0.5 * dMdh * h_dot^2;   % always opposes motion (sign handled below)

% --- Friction model (Coulomb stick-slip with abs-value normal force) ---
% Normal force from moment balance about rail pads
N = abs(F_thrust * params.x_thrust - M * g * params.x_cg) / params.L_pads;

% Net non-friction drive force (what friction must resist)
F_drive = F_thrust - F_grav - F_fict;

if abs(h_dot) < params.eps_velocity
    if abs(F_drive) <= params.F_static
        F_fric = -F_drive;                          % stuck
    else
        F_fric = -sign(F_drive) * params.mu_k * N; % breaking free
    end
else
    F_fric = -sign(h_dot) * params.mu_k * N;       % kinetic
end

% --- Equation of motion ---
h_ddot = (F_thrust - F_grav - F_fict + F_fric) / M;
end