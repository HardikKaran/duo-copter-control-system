function [J, MSD, E] = determine_cost(Kp, Ki, Kd, N, m, x_thrust, x_cg, F_static, mu_k)
% DETERMINE_COST  Run the controller model and return the weighted cost J.

profile = 2;

params = build_params(struct( ...
    'm_structure', m, ...
    'x_thrust',    x_thrust, ...
    'x_cg',        x_cg, ...
    'F_static',    F_static, ...
    'mu_k',        mu_k));

sim_output = run_with_params('controller', Kp, Ki, Kd, N, params, profile);

MSD_manual = 0.002082; % Manually tuned MSD
E_manual = 12.9;    % Manually tuned energy

MSD = sim_output.MSD.Data(end);
E = sim_output.energy.Data(end);

J = (3/7) * (E / E_manual) + (4/7) * (MSD / MSD_manual);
end