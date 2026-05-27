t_desired = time;
h_desired = (out.step_height.Data)';
h_actual = out.noisy_height;
t_actual = linspace(0,74.65,301);

figure;
% Plot desired height response (red dashed)
plot(t_desired, h_desired(1:1494), 'r--', 'LineWidth', 1.5);
hold on;

% Plot actual height response (solid blue)
plot(t_actual, h_actual, 'b-', 'LineWidth', 1.5);

% Axis labels
xlabel('Time (s)');
ylabel('Height (m)');

% Legend
legend('Desired Height', 'Actual Height', 'Location', 'best');

grid on;