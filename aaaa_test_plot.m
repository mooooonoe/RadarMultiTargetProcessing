close all;
%% Detection & Angle estimation Results plot

figure;
for frame_number = 1:256
clf;
hold on;
% dynamic target position plot
p1 = plot(target_x_dynamic_cell{frame_number}, target_y_dynamic_cell{frame_number}, 'bo', 'MarkerFaceColor', 'b', 'MarkerSize', 5);
hold on;
% static target position plot
p2 = plot(target_x_static_cell{frame_number}, target_y_static_cell{frame_number}, 'go', 'MarkerFaceColor', 'g', 'MarkerSize', 5);
% radar position plot
hold on;
p3 = plot(0, 0, '^', 'MarkerFaceColor', 'r');
legend([p3 p1 p2],{'Radar','Dynamic','Static'})

% plot 범위 설정
axis([x_min, x_max, y_min, y_max]);
xlabel('X (m)');
ylabel('Y (m)');
title( [num2str(frame_number),' frame'],'Target position estimation Results');
grid on;
hold off;
pause(0.001);
end


% for frames = 1:Nframe
%     if isempty(target_x_dynamic_cell{frames})
%         continue;
%     end
% x_radar(1,frames) = target_x_dynamic_cell{frames}(1);
% y_radar(1,frames) = target_y_dynamic_cell{frames}(1);
% end
% 
% % time axis
% t = linspace(0, 2*pi, length(x_radar));
% dt = t(2) - t(1);
% 
% % 튀는 값 제거
% mean_x = mean(x_radar);
% std_x = std(x_radar);
% mean_y = mean(y_radar);
% std_y = std(y_radar);
% 
% % 3 표준편차를 기준으로 이상치 제거
% threshold = 1.73; 
% valid_indices = (abs(x_radar - mean_x) < threshold * std_x) & (abs(y_radar - mean_y) < threshold * std_y);
% 
% x_radar_filtered = x_radar(valid_indices);
% y_radar_filtered = y_radar(valid_indices);
% 
% % Plotting the position
% figure;
% scatter(x_radar_filtered, y_radar_filtered, 5, 'red', 'filled', 'DisplayName', 'Radar detected');
% xlabel('x');
% ylabel('y');
% title('Position');
% legend;
% hold on;

%% Micro doppler plot

% MTIfiltering = 1;
% % plot
% figure;
% for Range_Bin = 1:256
% clf;
% Ranges = Range_Bin*range_resolution; 
% if MTIfiltering
%     imagesc(time_axis,velocityBin,sdb_mti{Range_Bin});
%     title([num2str(Ranges),' Range'],'Micro Doppler (MTI)');
% else
%     imagesc(time_axis,velocityBin,sdb{Range_Bin});
%     title([num2str(Ranges),' Range'],'Micro Doppler (not MTI)');
% end
% xlabel('times (s)');
% ylabel('Velocity (m/s)');
% % colormap('gray');
% axis xy
% pause(0.05);
% end

% % plot
% figure;
% if MTIfiltering
%     imagesc(time_axis,velocityBin,sdb_mti{20});
%     title('Micro Doppler (MTI)');
% else
%     imagesc(time_axis,velocityBin,sdb{27});
%     title('Micro Doppler (not MTI)');
% end
% xlabel('times (s)');
% ylabel('Velocity (m/s)');
% % colormap('gray');
% axis xy

%% Range Doppler FFT plot - RDM

% % plot Range Doppler Map
% MTIfiltering = 1;
% figure;
% for frame_number = 1:256
% clf;
% if MTIfiltering
%     imagesc(velocityBin,rangeBin,db_doppler_mti_cell{frame_number});
%     title([num2str(frame_number),' frame'], 'Range-Doppler Map (MTI)');
% else
%     imagesc(velocityBin,rangeBin,db_doppler_cell{frame_number});
%     title([num2str(frame_number),' frame'], 'Range-Doppler Map (not MTI)');
% end
% xlabel('Velocity (m/s)');
% ylabel('Range (m)');
% colorbar;
% axis xy
% pause(0.005);
% end

%% 2D RDM CA-OS CFAR plot

% % plot 2D CAOS-CFAR
% MTIfiltering = 1;
% figure;
% for frame_number = 1:256
% clf;
% if MTIfiltering
%     imagesc(velocityBin,rangeBin,detected_points_2D_cell{frame_number});
%     title([num2str(frame_number),' frame'],'RDM 2D CFAR Target Detect (MTI)');
% else
%     imagesc(velocityBin,rangeBin,detected_points_2D_static_cell{frame_number});
%     title([num2str(frame_number),' frame'],'RDM 2D CFAR Target Detect (not MTI)');
% end
% xlabel('Velocity (m/s)');
% ylabel('Range (m)');
% yticks(0:2:max(rangeBin));
% colorbar;
% axis xy
% pause(0.005);
% end


