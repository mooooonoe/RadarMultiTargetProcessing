clear; clc; close all;

% Initial setup
t = linspace(0, 10, 256);  
dt = t(2) - t(1);

x1 = linspace(0, 3, 256); 
y1 = zeros(size(t));
x2 = linspace(0, 3, 256);  
y2 = zeros(size(t));

for i = 1:length(t)
    if (t(i) >= 3 && t(i) <=8)
        y1(i) = y1(i-1);
        y2(i) = y2(i-1);
    elseif t(i) > 8
        y1(i) = y1(i-1) - 0.001 * t(i);
        y2(i) = y2(i-1) + 0.001 * t(i);
    else
        y1(i) = 2 - 0.2 * t(i);  
        y2(i) = 1 + 0.2 * t(i);
    end
end

err_sig = 0.07;

x1_detect = x1 + err_sig * randn(size(x1));
y1_detect = y1 + err_sig * randn(size(y1));
x2_detect = x2 + err_sig * randn(size(x2));
y2_detect = y2 + err_sig * randn(size(y2));

figure('Position', [300,100, 1200, 800]);
tiledlayout(2,2);
nexttile;
plot(x1, y1, 'DisplayName', 'True position Obj 1', 'LineWidth', 1, 'Color', 'r'); hold on;
scatter(x1_detect, y1_detect, 10, 'r', 'filled', 'DisplayName', 'Detect Obj 1');
plot(x2, y2, 'DisplayName', 'True position Obj 2', 'LineWidth', 1, 'Color', 'b');
scatter(x2_detect, y2_detect, 10, 'b', 'filled', 'DisplayName', 'Detect Obj 2');
ylim([0.5,2.5])
xlabel('x (km)');
ylabel('y (km)');
title('Position');
legend;
grid on;
hold off;

%% sensor detect data
for cnt = 1:length(x1)
    x = [x1_detect(cnt), x2_detect(cnt)];
    y = [y1_detect(cnt), y2_detect(cnt)];
    data_struct.target_x{cnt} = [sort(x)];
    data_struct.target_y{cnt} = [sort(y)];
end

%% data association
x1_reconstructed = zeros(size(data_struct.target_x));
y1_reconstructed = zeros(size(data_struct.target_y));
x2_reconstructed = zeros(size(data_struct.target_x));
y2_reconstructed = zeros(size(data_struct.target_y));

x1_reconstructed(1) = data_struct.target_x{1}(1);
y1_reconstructed(1) = data_struct.target_y{1}(1);
x2_reconstructed(1) = data_struct.target_x{1}(2);
y2_reconstructed(1) = data_struct.target_y{1}(2);

for cnt = 2:length(data_struct.target_x)
    prev_x1 = x1_reconstructed(cnt-1);
    prev_y1 = y1_reconstructed(cnt-1);
    prev_x2 = x2_reconstructed(cnt-1);
    prev_y2 = y2_reconstructed(cnt-1);
    
    x = data_struct.target_x{cnt};
    y = data_struct.target_y{cnt};
    
    dist1_1 = sqrt((prev_x1 - x(1))^2 + (prev_y1 - y(1))^2);
    dist1_2 = sqrt((prev_x1 - x(2))^2 + (prev_y1 - y(2))^2);
    dist2_1 = sqrt((prev_x2 - x(1))^2 + (prev_y2 - y(1))^2);
    dist2_2 = sqrt((prev_x2 - x(2))^2 + (prev_y2 - y(2))^2);
    
    if dist1_1 + dist2_2 < dist1_2 + dist2_1
        x1_reconstructed(cnt) = x(1);
        y1_reconstructed(cnt) = y(1);
        x2_reconstructed(cnt) = x(2);
        y2_reconstructed(cnt) = y(2);
    else
        x1_reconstructed(cnt) = x(2);
        y1_reconstructed(cnt) = y(2);
        x2_reconstructed(cnt) = x(1);
        y2_reconstructed(cnt) = y(1);
    end
end

nexttile;
scatter(x1_reconstructed, y1_reconstructed,  '.r'); hold on;
scatter(x2_reconstructed, y2_reconstructed,  '.b');
ylim([0.5,2.5])
xlabel('x (km)');
ylabel('y (km)');
title('Reconstructed Positions');
legend;
grid on;
hold off;

%% imm tracking of multi opject
nexttile;
for i = 1:length(x1)
    for j = 1: length(data_struct.target_x{2}(:))
        plot(data_struct.target_x{i}(j), data_struct.target_y{i}(j), 'DisplayName', ['True position Obj ', num2str(ceil(i/length(x1)))], 'LineWidth', 1);
        scatter(data_struct.target_x{i}(j), data_struct.target_y{i}(j), '.', 'MarkerEdgeColor', [0, 114/255, 189/255]);
        hold on;
    end
end
ylim([0.5,2.5])
xlabel('x (km)');
ylabel('y (km)');
title('sensor detect data input');
grid on;
hold off;

measPos = [x1_reconstructed; y1_reconstructed; zeros(size(x1))];

positionSelector = [1 0 0 0 0 0; 0 0 1 0 0 0; 0 0 0 0 1 0]; % Position from state
initialState = positionSelector' * measPos(:,1);
initialCovariance = diag([1, 1e4, 1, 1e4, 1, 1e4]); % Velocity is not measured

% Create a constant-velocity trackingEKF
cvekf = trackingEKF(@constvel, @cvmeas, initialState, ...
    'StateTransitionJacobianFcn', @constveljac, ...
    'MeasurementJacobianFcn', @cvmeasjac, ...
    'StateCovariance', initialCovariance, ...
    'HasAdditiveProcessNoise', false, ...
    'ProcessNoise', eye(3));

% Track using the constant-velocity filter
numSteps = numel(t);
dist = zeros(1, numSteps);
estPos = zeros(3, numSteps);
for i = 2:numSteps
    predict(cvekf, dt);
    dist(i) = distance(cvekf, measPos(:,i)); % Distance from true position
    estPos(:,i) = positionSelector * correct(cvekf, measPos(:,i));
end

% hold on;
% plot(estPos(1,:), estPos(2,:), '.g', 'DisplayName', 'CV Low PN');
% title('True and Estimated Positions with CV Filter');
% axis equal;
% legend;

% Increase the process noise for the constant-velocity filter
cvekf2 = trackingEKF(@constvel, @cvmeas, initialState, ...
    'StateTransitionJacobianFcn', @constveljac, ...
    'MeasurementJacobianFcn', @cvmeasjac, ...
    'StateCovariance', initialCovariance, ...
    'HasAdditiveProcessNoise', false, ...
    'ProcessNoise', diag([50, 50, 1])); % Large uncertainty in the horizontal acceleration

dist = zeros(1, numSteps);
estPos = zeros(3, numSteps);
for i = 2:numSteps
    predict(cvekf2, dt);
    dist(i) = distance(cvekf2, measPos(:,i)); % Distance from true position
    estPos(:,i) = positionSelector * correct(cvekf2, measPos(:,i));
end

hold on;
plot(estPos(1,:), estPos(2,:), '.c', 'DisplayName', 'CV High PN');
title('True and Estimated Positions with Increased Process Noise');

% Use an interacting multiple-model (IMM) filter
imm = trackingIMM('TransitionProbabilities', 0.99); % Default IMM with three models
initialize(imm, initialState, initialCovariance);

% Track using the IMM filter
dist = zeros(1, numSteps);
estPos = zeros(3, numSteps);
modelProbs = zeros(3, numSteps);
modelProbs(:,1) = imm.ModelProbabilities;
for i = 2:numSteps
    predict(imm, dt);
    dist(i) = distance(imm, measPos(:,i)); % Distance from true position
    estPos(:,i) = positionSelector * correct(imm, measPos(:,i));
    modelProbs(:,i) = imm.ModelProbabilities;
end
ylim([0.5,2.5]);
legend('radar detect point', 'IMM filter');

% hold on;
% plot(estPos(1,:), estPos(2,:), '.m', 'DisplayName', 'IMM');
% title('True and Estimated Positions with IMM Filter');
% axis equal;
% legend;

% % Plot normalized distance
% hold on;
% plot((1:numSteps)*dt, dist, 'g', 'DisplayName', 'CV Low PN');
% title('Normalized Distance from Estimated Position to True Position');
% xlabel('Time (s)');
% ylabel('Normalized Distance');
% legend;

% % Plot normalized distance
% hold on;
% 
% plot((1:numSteps)*dt, dist, 'm', 'DisplayName', 'IMM');
% title('Normalized Distance from Estimated Position to True Position');
% xlabel('Time (s)');
% ylabel('Normalized Distance');
% legend;

% % Plot model probabilities
% figure;
% plot((1:numSteps)*dt, modelProbs);
% title('Model Probabilities vs. Time');
% xlabel('Time (s)');
% ylabel('Model Probabilities');
% legend('IMM-CV', 'IMM-CA', 'IMM-CT');