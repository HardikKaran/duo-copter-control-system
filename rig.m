function params = rig()
    params.g = 9.81;

    m_cart = 0.15; % [kg] cart
    m_motor = 0.114; % [kg] each (motor 35-36)
    m_esc = 0.040; % [kg] each
    m_propeller = 0.010; % [kg] each
    m_structure = 0.0565; % [kg] CAD design EDIT

    params.m_fixed = m_cart + 2*(m_motor + m_esc + m_propeller + m_structure);
    
    params.m_counterweight = 0.150; % [kg]

    params.cable_slope = 0.190; % [kg/m]
    params.cable_intercept = 0.107; % [kg]  at h = 0

    % --- Throttle-to-thrust lookup (1-D) ---
    % Replace these with your measured curve. The points below are an
    % indicative shape matching the motor-28-35 plateau near 79% throttle.
    params.throttle_max       = 79;          % [%], saturate here, not at 100
    params.throttle_grid      = [0   10   20   30   40   50   60   70   79   100];
    % Thrust in Newtons. Last value clamped at plateau.
    params.thrust_grid        = [0  0.4  1.1  2.0  3.1  4.3  5.6  6.8  7.5  7.5];

    % --- Friction geometry & coefficients ---
    params.L_pads             = 0.1835;      % [m], rail pad separation (G10)
    params.x_thrust           = 0.0480;      % [m], thrust line offset (G17)
    params.x_cg               = 0.0350;      % [m], CG offset to rail centreline
    params.mu_k               = 0.15;        % [-], kinetic friction (mid of 0.11-0.17)
    params.F_static           = 1.275;       % [N], static breakaway (brief: 130 g)
    params.eps_velocity       = 1e-3;        % [m/s], zero-velocity tolerance

    % --- Travel limits (informational; clamping happens in integrator) ---
    params.h_max              = 1.44;        % [m]
    params.h_min              = 0.00;        % [m]

end