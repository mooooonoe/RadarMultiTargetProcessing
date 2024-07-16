%% DBSCAN
function [clusterGrid_cell, corepts, object_idx] = DBSCAN(eps, MinPts, NSample, NChirp, Nframe, detected_points_2D_cell)

% preallocation
clusterGrid_cell = cell(1, Nframe);

% frame마다 반복
for frames = 1:Nframe
 [row_2d, col_2d] = find(detected_points_2D_cell{frames} ~= 0);
 sz_data = size(row_2d);
 size_data = sz_data(1);
 data = zeros(size_data, 2);
 cnt = 1;
 for idx_cl = 1:size_data
       data(cnt, 1) = col_2d(idx_cl);
       data(cnt, 2) = row_2d(idx_cl);
       cnt = cnt + 1;
 end
  % data가 모두 0일 때 dbscan이 실행이 안됨 
  if isempty(data)
      object_idx = [];
      corepts = [];
  else
      [object_idx, corepts] = dbscan(data, eps, MinPts);
  end

 clusterGrid= zeros(NSample, NChirp);
 
 for i = 1:size_data
     % idx랑 corepts 번갈아가며 보는 중
     clusterGrid(data(i,2), data(i,1)) = object_idx(i);
 end

 % negative value  length(data)는 data의 차원 중 가장 큰 차원의 크기 반환
 for i = 1:size(data,1)
     if clusterGrid(data(i,2), data(i,1)) <0
         clusterGrid(data(i,2), data(i,1)) = 0;
     end
 end
clusterGrid_cell{frames} = clusterGrid;
end