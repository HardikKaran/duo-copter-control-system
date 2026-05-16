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

    % throttle to thrust

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