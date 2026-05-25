function results = montecarlo(Kp, Ki, Kd, N, varargin)
% MONTECARLO  Monte Carlo robustness analysis for the duo-copter PID controller.
%
%   Holds the (already-tuned) controller gains FIXED and randomly perturbs the
%   uncertain plant parameters about nominal - one group at a time and then all
%   together - to see how the closed-loop cost J, MSD and energy are distributed.
%
%   Perturbations are applied by building the rig() params STRUCT and overriding
%   fields (then injecting it), because the EOM block reads params.* - loose
%   scalar variables are ignored by the model.
%
%   Scenario -> what is perturbed:
%       Mass            -> total fixed mass m_fixed (scaled)   [physically meaningful]
%       DynamicFriction -> params.mu_k
%       StaticFriction  -> params.F_static
%       ThrustLine      -> params.x_thrust
%       CentreOfGravity -> params.x_cg
%       AllCombined     -> all of the above simultaneously
%
% USAGE
%   results = montecarlo(Kp, Ki, Kd, N)
%   results = montecarlo(Kp, Ki, Kd, N, 'Name', Value, ...)
%
% KEY OPTIONS  (see code for the full list)
%   'NumSamples'   (300)        Samples per scenario.
%   'Distribution' ('normal')   'normal' (sigma = fractional 1-sigma) or
%                               'uniform' (sigma = +/- fractional half-width).
%   'Sigma'        (struct)     Fractional spread per parameter. Scalar applies
%                               to all. Default m=0.10, mu_k=0.20, F_static=0.20,
%                               x_thrust=0.10, x_cg=0.10. (m scales total mass.)
%   'Scenarios'    (all six)    Subset by name.
%   'Seed' (0) | 'Model' ('controller') | 'UseParallel' (auto)
%   'EnergyManual' (10.78) | 'MSDManual' (0.002926) | 'Plot' (true)
%
% See also: DETERMINE_COST, BUILD_PARAMS, RUN_BAYESOPT, RIG

% ------------------------------------------------------------------------
% 1. Parse inputs
% ------------------------------------------------------------------------
nominalGeom = struct('mu_k', 0.17, 'F_static', 1.275, ...
                     'x_thrust', 0.03875, 'x_cg', 0.03888);

defaultSigma = struct('m', 0.10, 'mu_k', 0.20, 'F_static', 0.20, ...
                      'x_thrust', 0.10, 'x_cg', 0.10);

hasPCT = license('test', 'Distrib_Computing_Toolbox') && ~isempty(ver('parallel'));

p = inputParser;
p.addParameter('NumSamples', 300, @(x) isnumeric(x) && isscalar(x) && x > 0);
p.addParameter('Distribution', 'normal', @(s) any(strcmpi(s, {'normal','uniform'})));
p.addParameter('Sigma', defaultSigma);
p.addParameter('Scenarios', {}, @(c) iscell(c) || ischar(c) || isstring(c));
p.addParameter('Seed', 0, @(x) isnumeric(x) && isscalar(x));
p.addParameter('Model', 'controller', @(s) ischar(s) || isstring(s));
p.addParameter('UseParallel', hasPCT, @(x) islogical(x) || ismember(x, [0 1]));
p.addParameter('EnergyManual', 10.78, @isnumeric);
p.addParameter('MSDManual', 0.002926, @isnumeric);
p.addParameter('Plot', true, @(x) islogical(x) || ismember(x, [0 1]));
p.parse(varargin{:});
opt = p.Results;

Ns          = round(opt.NumSamples);
distType    = lower(opt.Distribution);
model       = char(opt.Model);
useParallel = logical(opt.UseParallel) && hasPCT;
E_manual    = opt.EnergyManual;
MSD_manual  = opt.MSDManual;

% Resolve sigma
sigma = defaultSigma;
if isstruct(opt.Sigma)
    f = fieldnames(opt.Sigma);
    for k = 1:numel(f)
        if isfield(sigma, f{k}); sigma.(f{k}) = opt.Sigma.(f{k}); end
    end
elseif isscalar(opt.Sigma) && isnumeric(opt.Sigma)
    f = fieldnames(sigma);
    for k = 1:numel(f); sigma.(f{k}) = opt.Sigma; end
end

% Canonical sampling parameters. 'mass_scale' multiplies total mass (nominal 1).
sampleNom = struct('mass_scale', 1.0, 'mu_k', nominalGeom.mu_k, ...
                   'F_static', nominalGeom.F_static, ...
                   'x_thrust', nominalGeom.x_thrust, 'x_cg', nominalGeom.x_cg);
sampleSig = struct('mass_scale', sigma.m, 'mu_k', sigma.mu_k, ...
                   'F_static', sigma.F_static, ...
                   'x_thrust', sigma.x_thrust, 'x_cg', sigma.x_cg);
canon = {'mass_scale', 'mu_k', 'F_static', 'x_thrust', 'x_cg'};

% Scenario catalogue: {name, {canonical params to perturb}}
catalogue = { ...
    'Mass',            {'mass_scale'}; ...
    'DynamicFriction', {'mu_k'}; ...
    'StaticFriction',  {'F_static'}; ...
    'ThrustLine',      {'x_thrust'}; ...
    'CentreOfGravity', {'x_cg'}; ...
    'AllCombined',     {'mass_scale','mu_k','F_static','x_thrust','x_cg'} };

if ~isempty(opt.Scenarios)
    want = cellstr(opt.Scenarios);
    keep = ismember(catalogue(:,1), want);
    if ~any(keep)
        error('montecarlo:badScenario', ...
            'No requested scenario matched. Valid: %s', strjoin(catalogue(:,1)', ', '));
    end
    catalogue = catalogue(keep, :);
end
nScen = size(catalogue, 1);

fprintf('Monte Carlo robustness test\n');
fprintf('  Gains : Kp=%.3g  Ki=%.3g  Kd=%.3g  N=%.3g\n', Kp, Ki, Kd, N);
fprintf('  %d samples/scenario, %s perturbations, %s execution\n', ...
    Ns, distType, ternary(useParallel, 'parallel', 'serial'));

if ~bdIsLoaded(model)
    try, load_system(model);
    catch ME
        warning('montecarlo:loadModel', 'Could not load "%s": %s', model, ME.message);
    end
end

% ------------------------------------------------------------------------
% 2. Baseline run at nominal
% ------------------------------------------------------------------------
params_nom = build_params([]);   % pure nominal rig()
[J_nom, MSD_nom, E_nom] = run_inputs(model, Kp, Ki, Kd, N, {params_nom}, ...
                                     false, E_manual, MSD_manual);
J_nom = J_nom(1); MSD_nom = MSD_nom(1); E_nom = E_nom(1);
if isnan(J_nom)
    warning('montecarlo:nominalFailed', 'Nominal run failed - check the model first.');
else
    fprintf('  Nominal cost J = %.5f  (MSD=%.4g, E=%.4g)\n\n', J_nom, MSD_nom, E_nom);
end

% ------------------------------------------------------------------------
% 3. Run each scenario
% ------------------------------------------------------------------------
rng(opt.Seed);
scenarios = struct('name', {}, 'perturbed', {}, 'samples', {}, 'stats', {});

for s = 1:nScen
    name      = catalogue{s, 1};
    perturbed = catalogue{s, 2};
    fprintf('[%d/%d] %s  (perturbing: %s)\n', s, nScen, name, strjoin(perturbed, ', '));

    % --- draw samples: every canonical param at nominal, perturbed ones jitter
    smp = struct();
    for k = 1:numel(canon)
        smp.(canon{k}) = repmat(sampleNom.(canon{k}), Ns, 1);
    end
    for k = 1:numel(perturbed)
        pn = perturbed{k};
        smp.(pn) = draw(sampleNom.(pn), sampleSig.(pn), Ns, distType);
    end

    % --- build the params struct for each sample, then evaluate
    paramSet = cell(Ns, 1);
    for i = 1:Ns
        paramSet{i} = build_params(struct( ...
            'm_fixed_scale', smp.mass_scale(i), 'mu_k', smp.mu_k(i), ...
            'F_static', smp.F_static(i), 'x_thrust', smp.x_thrust(i), ...
            'x_cg', smp.x_cg(i)));
    end
    [J, MSD, E] = run_inputs(model, Kp, Ki, Kd, N, paramSet, ...
                             useParallel, E_manual, MSD_manual);
    nFail = sum(isnan(J));

    smp.J = J; smp.MSD = MSD; smp.E = E;
    scenarios(s).name      = name;
    scenarios(s).perturbed = perturbed;
    scenarios(s).samples   = smp;
    scenarios(s).stats     = stat_block(J, MSD, E, J_nom, nFail, Ns);

    st = scenarios(s).stats;
    fprintf('      J: mean=%.4f  std=%.4g  p95=%.4f  max=%.4f  fails=%d/%d\n\n', ...
        st.J.mean, st.J.std, st.J.p95, st.J.max, nFail, Ns);
end

% ------------------------------------------------------------------------
% 4. Outputs
% ------------------------------------------------------------------------
results.gains     = struct('Kp', Kp, 'Ki', Ki, 'Kd', Kd, 'N', N);
results.nominal   = sampleNom;
results.J_nominal = J_nom;
results.scenarios = scenarios;
results.settings  = struct('NumSamples', Ns, 'Distribution', distType, 'Sigma', sigma, ...
                           'Seed', opt.Seed, 'Model', model, 'UseParallel', useParallel, ...
                           'EnergyManual', E_manual, 'MSDManual', MSD_manual);
results.summary   = summary_table(scenarios);

fprintf('=== Summary (cost J) ===\n');
disp(results.summary);

if logical(opt.Plot)
    make_plots(scenarios, J_nom);
end
end % ===== montecarlo =====


% ========================================================================
% Helpers
% ========================================================================
function [J, MSD, E] = run_inputs(model, Kp, Ki, Kd, N, paramSet, useParallel, E_manual, MSD_manual)
% Evaluate a set of params structs. Because `params` lives in the MODEL
% workspace, each run must write it there (via run_with_params) - plain
% SimulationInput.setVariable is shadowed. Serial is the reliable path; the
% parallel path uses parsim with each worker writing its own model workspace.
    Ns  = numel(paramSet);
    J   = nan(Ns, 1); MSD = nan(Ns, 1); E = nan(Ns, 1);

    if useParallel
        % Each worker loads the model once and writes params into its own
        % model workspace per iteration. Requires Parallel Computing Toolbox.
        try
            Jp = nan(Ns,1); Mp = nan(Ns,1); Ep = nan(Ns,1);
            parfor i = 1:Ns
                try
                    o = run_with_params(model, Kp, Ki, Kd, N, paramSet{i}, 2);
                    Mp(i) = local_end(o, 'MSD');
                    Ep(i) = local_end(o, 'energy');
                    Jp(i) = (3/7)*(Ep(i)/E_manual) + (4/7)*(Mp(i)/MSD_manual);
                catch
                    % leave NaN
                end
            end
            J = Jp; MSD = Mp; E = Ep;
            return;
        catch ME
            warning('montecarlo:parsimFailed', 'parallel failed (%s); serial fallback.', ME.message);
        end
    end

    % Serial path
    for i = 1:Ns
        try
            o = run_with_params(model, Kp, Ki, Kd, N, paramSet{i}, 2);
            MSD(i) = local_end(o, 'MSD');
            E(i)   = local_end(o, 'energy');
            J(i)   = (3/7)*(E(i)/E_manual) + (4/7)*(MSD(i)/MSD_manual);
        catch
            % leave NaN
        end
    end
end


function v = local_end(out, name)
    sig = get(out, name);
    if isa(sig, 'timeseries'); d = sig.Data;
    elseif isstruct(sig) && isfield(sig, 'Data'); d = sig.Data;
    else; d = sig; end
    v = d(end);
end


function v = draw(mu, frac, n, distType)
    if strcmp(distType, 'normal')
        v = mu .* (1 + frac .* randn(n, 1));
    else
        v = mu .* (1 + frac .* (2 * rand(n, 1) - 1));
    end
    v = max(v, eps);
end


function st = stat_block(J, MSD, E, J_nom, nFail, Ns)
    st.J = metric_stats(J); st.MSD = metric_stats(MSD); st.E = metric_stats(E);
    st.failures = nFail; st.failRate = nFail / Ns;
    if ~isnan(J_nom) && J_nom ~= 0
        st.J.meanRatio  = st.J.mean / J_nom;
        st.J.worstRatio = st.J.max  / J_nom;
    else
        st.J.meanRatio = NaN; st.J.worstRatio = NaN;
    end
end


function m = metric_stats(x)
    x = x(~isnan(x));
    if isempty(x)
        [m.mean, m.std, m.min, m.median, m.p95, m.max] = deal(NaN); return;
    end
    m.mean = mean(x); m.std = std(x); m.min = min(x);
    m.median = median(x); m.p95 = prctile_local(x, 95); m.max = max(x);
end


function q = prctile_local(x, p)
    x = sort(x(:)); n = numel(x);
    if n == 1; q = x; return; end
    pos = p/100 * n + 0.5;
    if pos <= 1, q = x(1);
    elseif pos >= n, q = x(end);
    else, lo = floor(pos); q = x(lo) + (pos - lo) * (x(lo+1) - x(lo)); end
end


function T = summary_table(scenarios)
    n = numel(scenarios);
    Scenario = strings(n,1);
    [J_mean, J_std, J_p95, J_max, MeanRatio, WorstRatio, FailRate] = deal(zeros(n,1));
    for s = 1:n
        st = scenarios(s).stats;
        Scenario(s)   = scenarios(s).name;
        J_mean(s) = st.J.mean; J_std(s) = st.J.std;
        J_p95(s)  = st.J.p95;  J_max(s) = st.J.max;
        MeanRatio(s) = st.J.meanRatio; WorstRatio(s) = st.J.worstRatio;
        FailRate(s)  = st.failRate;
    end
    T = table(Scenario, J_mean, J_std, J_p95, J_max, MeanRatio, WorstRatio, FailRate);
end


function make_plots(scenarios, J_nom)
    n = numel(scenarios);

    % Figure 1: histograms
    figure('Name', 'Monte Carlo - cost distributions', 'Color', 'w');
    rows = ceil(sqrt(n)); cols = ceil(n / rows);
    for s = 1:n
        subplot(rows, cols, s);
        J = scenarios(s).samples.J; J = J(~isnan(J));
        if ~isempty(J), histogram(J, 'FaceColor', [0.2 0.4 0.8], 'EdgeColor', 'none'); end
        hold on;
        if ~isnan(J_nom), xline(J_nom, 'r--', 'LineWidth', 1.5); end
        title(scenarios(s).name, 'Interpreter', 'none');
        xlabel('J'); ylabel('count'); grid on;
    end
    sgtitle('Cost J distribution by scenario (red = nominal)');

    % Figure 2: boxchart
    figure('Name', 'Monte Carlo - cost comparison', 'Color', 'w'); hold on;
    if exist('boxchart', 'file')
        groups = []; vals = [];
        for s = 1:n
            J = scenarios(s).samples.J; J = J(~isnan(J));
            groups = [groups; repmat(s, numel(J), 1)]; %#ok<AGROW>
            vals   = [vals; J(:)];                      %#ok<AGROW>
        end
        boxchart(categorical(groups, 1:n, {scenarios.name}), vals); ylabel('J');
    else
        mu = arrayfun(@(x) x.stats.J.mean, scenarios);
        sd = arrayfun(@(x) x.stats.J.std,  scenarios);
        bar(mu); errorbar(1:n, mu, sd, 'k', 'LineStyle', 'none');
        set(gca, 'XTick', 1:n, 'XTickLabel', {scenarios.name}); ylabel('mean J (+/- std)');
    end
    if ~isnan(J_nom)
        yl = yline(J_nom, 'r--', 'nominal', 'LineWidth', 1.5);
        yl.Annotation.LegendInformation.IconDisplayStyle = 'off';
    end
    title('Cost J across robustness scenarios'); grid on;
    set(gca, 'TickLabelInterpreter', 'none'); xtickangle(30);

    % Figure 3: single-parameter sensitivity
    isSingle = arrayfun(@(x) numel(x.perturbed) == 1, scenarios);
    if any(isSingle)
        idx = find(isSingle);
        mr  = arrayfun(@(s) scenarios(s).stats.J.meanRatio,  idx);
        [~, order] = sort(mr, 'descend'); idx = idx(order);
        figure('Name', 'Monte Carlo - parameter sensitivity', 'Color', 'w');
        b = bar([arrayfun(@(s) scenarios(s).stats.J.meanRatio,  idx)', ...
                 arrayfun(@(s) scenarios(s).stats.J.worstRatio, idx)']);
        b(1).FaceColor = [0.2 0.4 0.8]; b(2).FaceColor = [0.85 0.4 0.2];
        legend(b, {'mean J / nominal', 'worst J / nominal'}, 'Location', 'best');
        set(gca, 'XTick', 1:numel(idx), 'XTickLabel', {scenarios(idx).name});
        set(gca, 'TickLabelInterpreter', 'none'); xtickangle(30);
        ylabel('cost ratio vs nominal');
        yl = yline(1, 'k:'); yl.Annotation.LegendInformation.IconDisplayStyle = 'off';
        title('Controller sensitivity by parameter (higher = less robust)'); grid on;
    end
end


function out = ternary(cond, a, b)
    if cond, out = a; else, out = b; end
end