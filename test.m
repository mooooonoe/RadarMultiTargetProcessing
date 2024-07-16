
for frame_number = 5:256
    x1 = 0; y1 = 0; x2 = 0; y2 = 0;
        if numGroups > 2      
        cnt = 1;
        id = find(idx == 1);
        for i = 1: length(id)
            x1 = x1 + loc(i,1);
            y1 = y1 + loc(i,2);
            cnt = cnt + 1;
        end
        x1_detect(frame_number) = x1/cnt;
        y1_detect(frame_number) = y1/cnt;
    
        cnt = 1;
        id = find(idx == 2);
        for i = 1: length(id)
            x2 = x2 + loc(i,1);
            y2 = y2 + loc(i,2);
            cnt = cnt + 1;
        end
        
        x2_detect(frame_number) = x2/cnt;
        y2_detect(frame_number) = y2/cnt;
        
        x = [x1_detect(frame_number), x2_detect(frame_number)];
        y = [y1_detect(frame_number), y2_detect(frame_number)];
        data_struct.target_x{frame_number} = [sort(x)];
        data_struct.target_y{frame_number} = [sort(y)];
    else
        for id = find(idx == 1)
            disp(id)
        end
    end

end

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

% %% imm tracking of multi opject
% if numGroups < 2
% 
% end
% 
% for i = 1:length(x1)
%     for j = 1: length(data_struct.target_x{2}(:))
%         plot(data_struct.target_x{i}(j), data_struct.target_y{i}(j), 'DisplayName', ['True position Obj ', num2str(ceil(i/length(x1)))], 'LineWidth', 1);
%         scatter(data_struct.target_x{i}(j), data_struct.target_y{i}(j), '.', 'MarkerEdgeColor', [0, 114/255, 189/255]);
%         hold on;
%     end
% end
% ylim([0.5,2.5])
% xlabel('x (km)');
% ylabel('y (km)');
% title('sensor detect data input');
% grid on;
% hold off;
% 
% measPos = [x1_reconstructed; y1_reconstructed; zeros(size(x1))];
% 
% positionSelector = [1 0 0 0 0 0; 0 0 1 0 0 0; 0 0 0 0 1 0]; % Position from state
% initialState = positionSelector' * measPos(:,1);
% initialCovariance = diag([1, 1e4, 1, 1e4, 1, 1e4]); % Velocity is not measured
% 
% % Create a constant-velocity trackingEKF
% cvekf = trackingEKF(@constvel, @cvmeas, initialState, ...
%     'StateTransitionJacobianFcn', @constveljac, ...
%     'MeasurementJacobianFcn', @cvmeasjac, ...
%     'StateCovariance', initialCovariance, ...
%     'HasAdditiveProcessNoise', false, ...
%     'ProcessNoise', eye(3));
% 
% % Track using the constant-velocity filter
% numSteps = numel(t);
% dist = zeros(1, numSteps);
% estPos = zeros(3, numSteps);
% for i = 2:numSteps
%     predict(cvekf, dt);
%     dist(i) = distance(cvekf, measPos(:,i)); % Distance from true position
%     estPos(:,i) = positionSelector * correct(cvekf, measPos(:,i));
% end
% 
% % hold on;
% % plot(estPos(1,:), estPos(2,:), '.g', 'DisplayName', 'CV Low PN');
% % title('True and Estimated Positions with CV Filter');
% % axis equal;
% % legend;
% 
% % Increase the process noise for the constant-velocity filter
% cvekf2 = trackingEKF(@constvel, @cvmeas, initialState, ...
%     'StateTransitionJacobianFcn', @constveljac, ...
%     'MeasurementJacobianFcn', @cvmeasjac, ...
%     'StateCovariance', initialCovariance, ...
%     'HasAdditiveProcessNoise', false, ...
%     'ProcessNoise', diag([50, 50, 1])); % Large uncertainty in the horizontal acceleration
% 
% dist = zeros(1, numSteps);
% estPos = zeros(3, numSteps);
% for i = 2:numSteps
%     predict(cvekf2, dt);
%     dist(i) = distance(cvekf2, measPos(:,i)); % Distance from true position
%     estPos(:,i) = positionSelector * correct(cvekf2, measPos(:,i));
% end
% 
% hold on;
% plot(estPos(1,:), estPos(2,:), '.c', 'DisplayName', 'CV High PN');
% title('True and Estimated Positions with Increased Process Noise');
% 
% % Use an interacting multiple-model (IMM) filter
% imm = trackingIMM('TransitionProbabilities', 0.99); % Default IMM with three models
% initialize(imm, initialState, initialCovariance);
% 
% % Track using the IMM filter
% dist = zeros(1, numSteps);
% estPos = zeros(3, numSteps);
% modelProbs = zeros(3, numSteps);
% modelProbs(:,1) = imm.ModelProbabilities;
% for i = 2:numSteps
%     predict(imm, dt);
%     dist(i) = distance(imm, measPos(:,i)); % Distance from true position
%     estPos(:,i) = positionSelector * correct(imm, measPos(:,i));
%     modelProbs(:,i) = imm.ModelProbabilities;
% end
% ylim([0.5,2.5]);
% legend('radar detect point', 'IMM filter');