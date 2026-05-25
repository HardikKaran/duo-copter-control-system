function out = run_with_params(model, Kp, Ki, Kd, N, params, profile)
%RUN_WITH_PARAMS  Simulate the controller with `params` injected into the
%   MODEL workspace - which is where the Parameters block actually resolves it.
%
%   The model stores `params` in its Model Workspace (DataSource = 'Model File'),
%   so SimulationInput.setVariable('params',...) is shadowed and ignored. This
%   helper writes the override straight into the model workspace, runs, then
%   restores the original value so the .slx on disk is never mutated.
%
%   Gains (Kp,Ki,Kd,N) and `profile` are still passed via the base workspace,
%   which is correct if those resolve from base (they were varying under
%   bayesopt, so they do).
%
% See also: DETERMINE_COST, MONTECARLO, BUILD_PARAMS

if nargin < 7 || isempty(profile); profile = 2; end

if ~bdIsLoaded(model); load_system(model); end
mw = get_param(model, 'ModelWorkspace');

% Snapshot the current model-workspace params so we can restore it.
hadParams = mw.hasVariable('params');
if hadParams
    params_backup = mw.getVariable('params');
end

cleanup = onCleanup(@() restore(mw, hadParams, ...
    ternary(hadParams, @() params_backup, @() [])));

% Inject the override where the block reads it.
mw.assignin('params', params);

% Gains + profile via base workspace (matches how the model resolves them).
assignin('base', 'Kp', Kp);  assignin('base', 'Ki', Ki);
assignin('base', 'Kd', Kd);  assignin('base', 'N',  N);
assignin('base', 'profile', profile);

out = sim(model);
end


function restore(mw, hadParams, getBackup)
if hadParams
    mw.assignin('params', getBackup());
elseif mw.hasVariable('params')
    mw.clear('params');   % we created it; remove it again
end
end


function out = ternary(c, a, b)
if c; out = a; else; out = b; end
end