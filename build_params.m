function params = build_params(overrides)
%BUILD_PARAMS  Nominal rig() parameters with optional plant-parameter overrides.
%
%   params = build_params()           returns the nominal rig() struct.
%   params = build_params(overrides)  returns rig() with selected fields
%                                      replaced, recomputing derived quantities.
%
%   This is what the controller model actually consumes (lagrangian.m reads
%   params.mu_k, params.x_thrust, params.x_cg, params.F_static, params.m_fixed).
%   Injecting this struct - rather than loose scalar variables - is what makes
%   parameter perturbations reach the dynamics.
%
%   Recognised override fields:
%       m_structure   - structural mass of ONE arm [kg]; m_fixed is recomputed
%       m_fixed_scale - multiply the total fixed mass m_fixed by this factor
%                       (the physically meaningful "total mass" perturbation)
%       m_fixed       - set the total fixed mass directly [kg]
%       mu_k, F_static, x_thrust, x_cg - direct scalar overrides
%
%   The first rig() call (CSV read + thrust curves) is cached in a persistent,
%   so calling this thousands of times in a Monte Carlo loop is cheap.
%
% See also: RIG, DETERMINE_COST, MONTECARLO

persistent base
if isempty(base)
    base = rig();   % expensive call done once
end
params = base;

if nargin < 1 || isempty(overrides)
    return;
end

% --- mass: recompute the derived total mass m_fixed correctly ---------
if isfield(overrides, 'm_structure') && ~isempty(overrides.m_structure)
    dStruct        = overrides.m_structure - base.m_structure; % per arm
    params.m_structure = overrides.m_structure;
    params.m_fixed     = base.m_fixed + 2 * dStruct;           % two arms
end
if isfield(overrides, 'm_fixed_scale') && ~isempty(overrides.m_fixed_scale)
    params.m_fixed = params.m_fixed * overrides.m_fixed_scale;
end
if isfield(overrides, 'm_fixed') && ~isempty(overrides.m_fixed)
    params.m_fixed = overrides.m_fixed;
end

% --- direct scalar overrides -----------------------------------------
direct = {'mu_k', 'F_static', 'x_thrust', 'x_cg'};
for k = 1:numel(direct)
    fn = direct{k};
    if isfield(overrides, fn) && ~isempty(overrides.(fn))
        params.(fn) = overrides.(fn);
    end
end
end