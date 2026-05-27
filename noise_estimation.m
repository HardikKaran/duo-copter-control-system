% Load and extract sample data
data = load('sampleH.mat');
sampleH = data.smplH;
t = sampleH(1, :);
h = sampleH(2, :);

% Keep only samples after t = 68 s
mask = t > 67;
t = t(mask);
h = h(mask);

% Smooth the signal and estimate residual noise
windowSize = 8;
hSmooth = movmean(h, windowSize);
residual = h - hSmooth;
noiseVar = var(residual);

fprintf('Noise Variance: %.3e\n', noiseVar);

% Compare raw vs smoothed signals
figure;
plot(t, h, 'b', 'LineWidth', 0.8, 'DisplayName', 'Unfiltered');
hold on;
plot(t, hSmooth, 'r', 'LineWidth', 1.5, 'DisplayName', 'Filtered');
hold off;
xlabel('Time (s)');
ylabel('Height');
title('Unfiltered vs Filtered Height Data');
legend('show');
grid on;