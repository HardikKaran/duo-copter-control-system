S = load("sampleH.mat");
f = fieldnames(S);
sample = S.(f{1});
time = sample(1,:);
height = sample(2,:);

mission_height = timeseries(height(:), time(:));

% plot(time, height);