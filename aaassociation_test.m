% load('aaaasociation_data_twomen');

close all;
figure();

eps = 0.5;
MinPts = 2;

% 최종 data
cnt = 0;
%data_struct % .target_x .target_y 로 각각 담았으며 [ , ] 데이터 두개 를 담음 % 한개인 경우 프레임
%번호는 data_one 에 담겨있으며 배열 형태가 아니라 타겟 하나만 담겨있을 것
data_no = []; % 해당 프레임 번호
data_one = []; % 해당 프레임 번호

for frame_number = 5:256
    cnt = cnt + 1;

    clf;
    loc = zeros(length(target_x_dynamic_cell{frame_number}),2);
    loc(:,1) = target_x_dynamic_cell{frame_number};
    loc(:,2) = target_y_dynamic_cell{frame_number};
    
    if isempty(loc)
        continue;
    end
    
    D = pdist2(loc,loc);
    
    if isempty(D)
        continue;
    else
        [idx, corepts] = dbscan(D,eps,MinPts,'Distance','precomputed');
        if isempty(idx)
            continue;
        end
        numGroups = length(unique(idx));
        disp(numGroups);

        % Ensure the number of groups matches the length of idx
        hold on;
        if (numGroups == 1)
            scatter(loc(:,1),loc(:,2), 500, '.');
        else
            gscatter(loc(:,1),loc(:,2),idx,hsv(numGroups));
        end
        grid
        
        if numGroups == 1
            sorted_unique_values = unique(sort(idx));
            idmin = sorted_unique_values(1);
            dat = find(idx==idmin, 1);
            x1_detect(cnt) = loc(dat,1);
            y1_detect(cnt) = loc(dat,2);
            
            if cnt == 1
                x2_detect(cnt) = x1_detect(cnt);
                y2_detect(cnt) = y1_detect(cnt);
                x = [x1_detect(cnt), x2_detect(cnt)];
                y = [y1_detect(cnt), y2_detect(cnt)];
                data_struct.target_x{cnt} = [sort(x)];
                data_struct.target_y{cnt} = [sort(y)];
            else 
                x2_detect(cnt) = x1_detect(cnt-1);
                y2_detect(cnt) = y1_detect(cnt-1);
                x = [x1_detect(cnt), x2_detect(cnt)];
                y = [y1_detect(cnt), y2_detect(cnt)];
                data_struct.target_x{cnt} = [sort(x)];
                data_struct.target_y{cnt} = [sort(y)];
            end

            data_one = [data_one; frame_number];
        elseif numGroups == 0 
            % data_struct.target_x{frame_number} = [];
            % data_struct.target_y{frame_number} = [];

            cnt = cnt -1;
            data_no = [data_no; frame_number];
        else
            sorted_unique_values = unique(sort(idx)); 
            smallest_two_values = sorted_unique_values(1:2); 
            
            idmin = smallest_two_values(1);
            idmin2 = smallest_two_values(2);
    
            dat = find(idx==idmin, 1);
            x1_detect(cnt) = loc(dat,1);
            y1_detect(cnt) = loc(dat,2);
            dat2 = find(idx==idmin2,1);
            x2_detect(cnt) = loc(dat2,1);
            y2_detect(cnt) = loc(dat2,2);
    
            x = [x1_detect(cnt), x2_detect(cnt)];
            y = [y1_detect(cnt), y2_detect(cnt)];
    
            data_struct.target_x{cnt} = [sort(x)];
            data_struct.target_y{cnt} = [sort(y)];
        end
    end

    axis([x_min, x_max, y_min, y_max]);
    xlabel('X (m)');
    ylabel('Y (m)');
    title( [num2str(frame_number),' frame'],'Target position estimation Results');
    grid on;
    hold off;
    pause(0.001);
end

%% detect two data plot
figure();
for i = 1:cnt-1
    scatter(data_struct.target_x{i}, data_struct.target_y{i});
    axis([x_min, x_max, y_min, y_max]);
    xlabel('X (m)');
    ylabel('Y (m)');
    title( [num2str(i),' frame'],'Target position estimation Results');
    grid on;
    hold off;
    pause(0.001);
end

%% data association
% Improved data association
x1_reconstructed = zeros(size(data_struct.target_x));
y1_reconstructed = zeros(size(data_struct.target_y));
x2_reconstructed = zeros(size(data_struct.target_x));
y2_reconstructed = zeros(size(data_struct.target_y));

% Improved Initialization
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
    
    % Calculate distances
    dist1_1 = sqrt((prev_x1 - x(1))^2 + (prev_y1 - y(1))^2);
    dist1_2 = sqrt((prev_x1 - x(2))^2 + (prev_y1 - y(2))^2);
    dist2_1 = sqrt((prev_x2 - x(1))^2 + (prev_y2 - y(1))^2);
    dist2_2 = sqrt((prev_x2 - x(2))^2 + (prev_y2 - y(2))^2);
    
    % Improved association logic
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

% Plotting the results
figure();
scatter(x1_reconstructed, y1_reconstructed,  '.r'); hold on;
scatter(x2_reconstructed, y2_reconstructed,  '.b');
xlabel('x (km)');
ylabel('y (km)');
title('Reconstructed Positions');
legend('data1', 'data2');
grid on;
hold off;


% 
% 
% %nexttile;
% figure();
% scatter(x1_reconstructed, y1_reconstructed,  '.r'); hold on;
% scatter(x2_reconstructed, y2_reconstructed,  '.b');
% %ylim([0.5,2.5])
% xlabel('x (km)');
% ylabel('y (km)');
% title('Reconstructed Positions');
% legend;
% grid on;
% hold off;

%% imm tracking of multi opject
%nexttile;
figure();
for i = 1:length(data_struct.target_x)
    for j = 1: length(data_struct.target_x{2}(:))

        plot(data_struct.target_x{i}(j), data_struct.target_y{i}(j), 'DisplayName', ['True position Obj ', num2str(ceil(i/length(data_struct.target_x)))], 'LineWidth', 1);
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

measPos = [x1_reconstructed; y1_reconstructed; zeros(size(data_struct.target_x))];

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
for i = 2:(numSteps-1)
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
for i = 2:(numSteps-1)
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
for i = 2:(numSteps-1)
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