function [throttle_grid, thrust_grid, throttle_max] = motor_performance(filename)
    data = readtable(filename);

    % Use column indices to avoid µ encoding issues in variable names
    time      = data{:,1};   % Time (s)
    esc_signal = data{:,2};  % ESC signal (µs)
    thrust_raw = data{:,6};  % Thrust (gf)
    power_raw  = data{:,11}; % Electrical Power (W)

    % Min-max normalisation of ESC signal → throttle (%)
    esc_min = min(esc_signal);
    esc_max = max(esc_signal);
    throttle_raw = (esc_signal - esc_min) * 100 / (esc_max - esc_min);
    thrust_N_raw = thrust_raw * 9.81 / 1000;

    % --- Deduplicate: average thrust and power for identical ESC signal values ---
    [esc_unique, ~, ic] = unique(esc_signal);   % ic maps each row back to its unique ESC group
    throttle = (esc_unique - esc_min) * 100 / (esc_max - esc_min);
    thrust   = accumarray(ic, thrust_N_raw, [], @mean);
    power    = accumarray(ic, power_raw,    [], @mean);
    % -------------------------------------------------------------------------

    % Outputs
    throttle_grid = throttle;
    thrust_grid   = thrust;
    throttle_max  = max(throttle);

    % Plots (all against throttle %, not time — length is consistent now)
    figure;
    subplot(3,1,1); plot(throttle, power,   'o-b'); xlabel('Throttle (%)'); ylabel('Power (W)');    title('Power vs Throttle');
    subplot(3,1,2); plot(throttle, thrust,  'o-r'); xlabel('Throttle (%)'); ylabel('Thrust (N)');   title('Thrust vs Throttle');
    subplot(3,1,3); plot(throttle, thrust ./ power, 'o-g'); xlabel('Throttle (%)'); ylabel('gf/W'); title('Efficiency vs Throttle');
end