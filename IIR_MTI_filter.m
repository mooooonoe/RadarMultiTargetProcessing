%% IIR 필터 설계 (예: 고역통과 필터)
function [radarCubeData_IIR_mti_cell] = IIR_MTI_filter(NChan, NSample, Nframe, radarCubeData_cell)

radarCubeData_IIR_mti_cell = cell(1,Nframe);

% 2차 Butterworth 고역통과 필터
[b, a] = butter(2, 0.089, 'high'); % 0.07

% MTI 필터링을 위한 빈 배열을 초기화
mtiFilteredCube = zeros(size(radarCubeData_cell{1}));

for frames = 1:Nframe
 % 각 채널과 샘플에 대해 IIR 필터 적용
 for channel = 1:NChan
     for sample = 1:NSample
         % 각 chirp에 대해 필터 적용
         mtiFilteredCube(:, channel, sample) = filter(b, a, squeeze(radarCubeData_cell{frames}(:, channel, sample)));
     end
 end
radarCubeData_IIR_mti_cell{frames} = mtiFilteredCube;
end

