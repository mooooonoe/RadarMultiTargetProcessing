%% 2D OS CFAR Algorithm
function [detected_points_2D] = OS_CFAR_2D(sz_r, sz_c, Nt, Ng, Nframe, db_doppler_cell)

% pre allocation
Pfa = 1e-3;
total_cells = (2 * Nt + 2 * Ng + 1)^2 - (2 * Ng + 1)^2;
rank = ceil(total_cells * Pfa); 
detected_points_2D_cell = cell(1, Nframe);
detected_points_2D = zeros(size(db_doppler_cell{1},1),size(db_doppler_cell{1},2));

% frame마다 반복
for frames = 1:Nframe
 data = db_doppler_cell{frames};
 th = zeros(size(data));
 % zero padding
 pad_size = Nt + Ng;
 padded_data = padarray(data, [pad_size pad_size], 0); 

 % 2D CFAR processing
 for i = 1:size(data, 1)
     for j = 1:size(data, 2)
         % 현재 셀 주변의 참조 셀 가져오기
         sub_matrix = padded_data(i:i + 2 * pad_size, j:j + 2 * pad_size);
         % 보호 셀 제거
         sub_matrix(Nt + 1:Nt + 1 + 2 * Ng, Nt + 1:Nt + 1 + 2 * Ng) = NaN;
         
         % 1D 벡터로 변환하여 정렬
         reference_cells = sub_matrix(:);
         reference_cells = reference_cells(~isnan(reference_cells));
         sorted_cells = sort(reference_cells);
        
         % 순위에 해당하는 임계값 선택
         if length(sorted_cells) >= rank
             th(i, j) = sorted_cells(rank);
         else
             th(i, j) = max(sorted_cells); % 만약 순위에 해당하는 셀이 부족하면 최대값 사용
         end
        
%         % 신호 검출: 현재 셀이 임계값을 초과하면 신호로 간주
%         if data(i, j) > threshold(i, j)
%             CFAR_output(i, j) = data(i, j);
%         end
     end
 end

 % detecting points
 for cutRIdx = 1:sz_r
     for cutCIdx = 1:sz_c
         cut = db_doppler_cell{frames}(cutRIdx, cutCIdx);
         compare = th(cutRIdx, cutCIdx);
         if(cut > compare)
             detected_points_2D(cutRIdx, cutCIdx) = 1;
         end
         if(cut <= compare)
             detected_points_2D(cutRIdx, cutCIdx) = 0;
         end
     end
 end
detected_points_2D_cell{frames} = detected_points_2D;
end