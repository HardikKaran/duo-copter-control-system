function params = rig()
    params.g = 9.81;

    m_cart = 0.15; % [kg] 
    m_motor = 0.114; % [kg] each (motor 35-36)
    m_esc = 0.040; % [kg] each
    m_propeller = 0.010; % [kg] each
    m_structure = 0.02825; % [kg] CAD design EDIT - ONLY 1 arm

    params.m_fixed = m_cart + 2*(m_motor + m_esc + m_propeller + m_structure);

    params.cable_slope = 0.190; % [kg/m]
    params.cable_intercept = 0.107; % [kg] at h = 0

    % Motor throttle (from CSV)
    [tg, fg, tmax] = motor_performance('Motor Performance Test 3536 15V.csv');
    params.throttle_grid = tg; % [%]  
    params.thrust_grid = fg * 2; % [N] both motors
    params.throttle_max = tmax;

    % Sample rate
    params.dt = 0.05; % [s] 20 Hz LiDAR
    params.sigma = 0.007; % [m] LiDAR noise std

    % Derivative filter coefficient
    params.alpha = 0.3; % low-pass weight on error signal


    % Friction geometry & coefficients 
    params.L_pads = 0.1835; % [m], rail pad separation ESTIMATE
    params.x_thrust = 0.03875; % [m], thrust line offset ESTIMATE
    params.x_cg = 0.03888; % [m], CG offset to rail centreline CAD EDIT
    params.mu_k = 0.17; % [-], kinetic friction (mid of 0.11-0.17)
    params.F_static = 1.275; % [N], static breakaway (brief: 130 g)
    params.eps_velocity = 1e-3; % [m/s], zero-velocity tolerance

    % Travel limits
    params.h_max = 1.44; % [m]
    params.h_min = 0.00; % [m]

end