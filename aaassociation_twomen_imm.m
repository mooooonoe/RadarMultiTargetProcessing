% load('aaaasociation_data_twomen');

close all;
figure();

eps = 0.5;
MinPts = 2;

for frame_number = 5:256
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
        numGroups = length(unique(idx)                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          );
        disp(numGroups);

        % Ensure the number of groups matches the length of idx
        hold on;
        if (numGroups == 1)
            scatter(loc(:,1),loc(:,2), 500, '.');
        else
            gscatter(loc(:,1),loc(:,2),idx,hsv(numGroups));
        end
        grid
    end

    % if numGroups > 2
    %     for 
    % end
    % if numGroups >3
    %     break;
    % end
    % 
    % 

    axis([x_min, x_max, y_min, y_max]);
    xlabel('X (m)');
    ylabel('Y (m)');
    title( [num2str(frame_number),' frame'],'Target position estimation Results');
    grid on;
    hold off;
    pause(0.001);
end


%% dbscan
% eps = 0.5;
% MinPts = 2;
% 
% loc(:,1) = target_x_dynamic_cell{frame_number};
% loc(:,2) = target_y_dynamic_cell{frame_number};
% scatter(loc(:,1),loc(:,2),'.');
% 
% D = pdist2(loc,loc);
% 
% [idx, corepts] = dbscan(D,2,50,'Distance','precomputed');
% numGroups = length(unique(idx));
% gscatter(loc(:,1),loc(:,2),idx,hsv(numGroups));
% annotation('ellipse',[0.54 0.41 .07 .07],'Color','red')
% grid
% 
% % [object_idx, corepts] = dbscan(target_x_dynamic_cell{frame_number}, eps, MinPts);
% 
% % for frame_number = 1:256
% %     if(isempty(target_x_dynamic_cell{frame_number}))
% %         data_struct.target_x{frame_number} = NaN;
% %         data_struct.target_y{frame_number} = NaN;
% %     else
% %         data_struct.target_x{frame_number} = dbscan(target_x_dynamic_cell{frame_number}, eps, MinPts);
% %         data_struct.target_y{frame_number} = dbscan(target_y_dynamic_cell{frame_number}, eps, MinPts);
% %     end
% % end
% 
% %scatter(data_struct.target_x(:), data_struct.target_y(:),  '.r'); hold on;
% 
% 
% %% data association
% 
% x1_reconstructed = zeros(size(data_struct.target_x));
% y1_reconstructed = zeros(size(data_struct.target_y));
% x2_reconstructed = zeros(size(data_struct.target_x));
% y2_reconstructed = zeros(size(data_struct.target_y));
% 
% x1_reconstructed(1) = data_struct.target_x{1}(1);
% y1_reconstructed(1) = data_struct.target_y{1}(1);
% x2_reconstructed(1) = data_struct.target_x{1}(2);
% y2_reconstructed(1) = data_struct.target_y{1}(2);
% 
% 
% 
% for cnt = 2:length(data_struct.target_x)
%     prev_x1 = x1_reconstructed(cnt-1);
%     prev_y1 = y1_reconstructed(cnt-1);
%     prev_x2 = x2_reconstructed(cnt-1);
%     prev_y2 = y2_reconstructed(cnt-1);
% 
%     x = data_struct.target_x{cnt};
%     y = data_struct.target_y{cnt};
% 
%     dist1_1 = sqrt((prev_x1 - x(1))^2 + (prev_y1 - y(1))^2);
%     dist1_2 = sqrt((prev_x1 - x(2))^2 + (prev_y1 - y(2))^2);
%     dist2_1 = sqrt((prev_x2 - x(1))^2 + (prev_y2 - y(1))^2);
%     dist2_2 = sqrt((prev_x2 - x(2))^2 + (prev_y2 - y(2))^2);
% 
%     if dist1_1 + dist2_2 < dist1_2 + dist2_1
%         x1_reconstructed(cnt) = x(1);
%         y1_reconstructed(cnt) = y(1);
%         x2_reconstructed(cnt) = x(2);
%         y2_reconstructed(cnt) = y(2);
%     else
%         x1_reconstructed(cnt) = x(2);
%         y1_reconstructed(cnt) = y(2);
%         x2_reconstructed(cnt) = x(1);
%         y2_reconstructed(cnt) = y(1);
%     end
% end
% 
% nexttile;
% scatter(x1_reconstructed, y1_reconstructed,  '.r'); hold on;
% scatter(x2_reconstructed, y2_reconstructed,  '.b');
% ylim([0.5,2.5])
% xlabel('x (km)');
% ylabel('y (km)');
% title('Reconstructed Positions');
% legend;
% grid on;
% hold off;