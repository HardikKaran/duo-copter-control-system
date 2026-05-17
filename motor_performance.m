function [throttle_grid, thrust_grid, throttle_max] = motor_performance(filename)
    data = readtable(filename);

    esc_signal = data{:,2}; % ESC signal (µs)
    thrust_gf = data{:,6}; % Thrust (gf)
    power_W  = data{:,11}; % Electrical Power (W)

    % Deduplicate: average thrust and power for identical ESC signal values 
    [esc_unique, ~, ic] = unique(esc_signal);   % ic maps each row back to its unique ESC group
    thrust_grid = accumarray(ic, thrust_gf, [], @mean) * 9.81 / 1000;
    power_grid = accumarray(ic, power_W, [], @mean);

    % ESC signal to throttle via min-max normalisation
    esc_min = min(esc_signal);
    esc_max = max(esc_signal);
    throttle_grid = (esc_unique - esc_min) * 100 / (esc_max - esc_min);
    throttle_max = max(throttle_grid);

    % Plots
    figure;
    subplot(3,1,1);
    plot(throttle_grid, power_grid, 'o-b');
    xlabel('Throttle (%)'); 
    ylabel('Power (W)'); 
    title('Power vs Throttle');

    subplot(3,1,2);
    plot(throttle_grid, thrust_grid, 'o-r');
    xlabel('Throttle (%)'); 
    ylabel('Thrust (N)'); 
    title('Thrust vs Throttle');

    subplot(3,1,3);
    plot(throttle_grid, thrust_grid ./ power_grid, 'o-g');
    xlabel('Throttle (%)'); 
    ylabel('N/W'); 
    title('Efficiency vs Throttle');
end