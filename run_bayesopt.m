% Load nominal model parameters
m_structure = 0.0287;
x_thrust = 0.03875;
x_cg = 0.035;
F_static = 1.2753;
mu_k = 0.17;


fprintf('Setting up Bayesian Optimization...\n');

% 1. Define Optimization Variables with bounds
% replace PID gains with Kp, Ki, Kd, N instead
Kp_var = optimizableVariable('Kp', [50, 400], 'Type', 'real');
Ki_var = optimizableVariable('Ki', [50, 400], 'Type', 'real');
Kd_var = optimizableVariable('Kd', [10, 100], 'Type', 'real');
N_var  = optimizableVariable('N', [5, 50], 'Type', 'real');

vars = [Kp_var, Ki_var, Kd_var, N_var];

% 2. Define Objective Function Wrapper
% bayesopt passes optimization variables as a table (t)
obj_fun = @(t) determine_cost_wrapper(t, m_structure, x_thrust, x_cg, F_static, mu_k);

% 3. Run Bayesian Optimization
% MaxObjectiveEvaluations dictates how many Simulink runs occur.
% Expected Improvement (EI) balances exploring new areas and exploiting known good ones.
results = bayesopt(obj_fun, vars, ...
    'MaxObjectiveEvaluations', 300, ... 
    'AcquisitionFunctionName', 'expected-improvement-plus', ...
    'IsObjectiveDeterministic', true, ...
    'PlotFcn', {@plotObjectiveModel, @plotMinObjective});

% 4. Extract and Display Results
best_vars = results.XAtMinObjective;
best_cost = results.MinObjective;

fprintf('\n=== Optimization Complete ===\n');
fprintf('Optimized Gains:\n');
fprintf(' Kp: %.2f\n', best_vars.Kp);
fprintf(' Ki: %.2f\n', best_vars.Ki);
fprintf(' Kd: %.2f\n', best_vars.Kd);
fprintf(' N:  %.2f\n', best_vars.N);
fprintf('Minimum Cost (J): %.5f\n', best_cost);

% Helper Function
function J = determine_cost_wrapper(t, m, Lt, Xcg, F_s, mu_d)
    % Unpack the table variables and pass to your simulation runner
    [J, ~, ~] = determine_cost(t.Kp, t.Ki, t.Kd, t.N, m, Lt, Xcg, F_s, mu_d);
end