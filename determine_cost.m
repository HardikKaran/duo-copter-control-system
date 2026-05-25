function [J, MSD, E] = determine_cost(Kp, Ki, Kd, N, m, x_thrust, x_cg, F_static, mu_k)
% DETERMINE_COST  Run the controller model and return the weighted cost J.
%
%   Same interface as before. The model stores `params` in its MODEL workspace
%   (DataSource = 'Model File'), so SimulationInput.setVariable is shadowed -
%   the override must be written into the model workspace. run_with_params does
%   exactly that (and restores the original afterwards). m_fixed is recomputed
%   from m inside build_params so structural mass actually propagates.

profile = 2;

params = build_params(struct( ...
    'm_structure', m, ...
    'x_thrust',    x_thrust, ...
    'x_cg',        x_cg, ...
    'F_static',    F_static, ...
    'mu_k',        mu_k));

sim_output = run_with_params('controller', Kp, Ki, Kd, N, params, profile);

MSD_manual = 0.002926; % Manually tuned MSD
E_manual   = 10.78;    % Manually tuned energy

MSD = sim_output.MSD.Data(end);
E   = sim_output.energy.Data(end);

J = (3/7) * (E / E_manual) + (4/7) * (MSD / MSD_manual);
end