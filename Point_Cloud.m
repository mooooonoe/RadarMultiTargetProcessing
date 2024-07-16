function [target_x_cell, target_y_cell, save_det_data_cell] = Point_Cloud(Nframe, rangeBin, velocityBin, angleBin, objOut_cell, Resel_agl_cell)

% preallocation
save_det_data_cell = cell(1, Nframe);
target_x_cell = cell(1, Nframe);
target_y_cell = cell(1, Nframe);

for frames = 1:Nframe
 % dynamic 타겟이 detecting되지 않았을 때 즉, objOut 데이터가 아무것도 없는 경우
 if isempty(objOut_cell{frames})
     objOut_cell{frames} = nan(3,1);
     save_det_data = nan(1, 7);
     target_x = [];
     target_y = [];  
 % deecting된 target이 있는 경우 
 else 
 % detecting for point cloud
 Resel_agl_deg = angleBin(1, Resel_agl_cell{frames});
 Resel_vel = velocityBin(1, objOut_cell{frames}(1,:));
 Resel_rng = rangeBin(1, objOut_cell{frames}(2,:));

 save_det_data = [objOut_cell{frames}(2,:)', objOut_cell{frames}(1,:)', Resel_agl_cell{frames}', objOut_cell{frames}(3,:)', Resel_rng', Resel_vel', Resel_agl_deg'];

 % Detection & Angle estimation data
 % 중심축이 90도가 되도록 target angle 변환 
   target_angle_deg = 90 - Resel_agl_deg;

 % target x, y좌표 구하기 
   % dynamic
   target_x = Resel_rng.*cosd(target_angle_deg);
   target_y = Resel_rng.*sind(target_angle_deg);
 end
% cell에 저장
save_det_data_cell{frames} = save_det_data;
target_x_cell{frames} = target_x;
target_y_cell{frames} = target_y;
end
