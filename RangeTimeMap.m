% Range Time Map
function [range_time_cell, sdb_rangetime_cell] = RangeTimeMap(NSample, NChan, Nframe, NChirp, chanIdx, radarCubeData_mti_cell)

% pre allocation
range_time_cell = cell(1, NChirp);
sdb_rangetime_cell = cell(1, NChirp);
% time axis
% max_time = pri * NChan * NChirp * Nframe;  %frame period = 39.17ms
% max_time = 0.04 * Nframe;  %frame period = 40ms
% time_axis = linspace(0,max_time,Nframe);

% chirp마다 반복
for chirps = 1:NChirp
 % chirp idx를 선택하고 3d data (frames, channels, range)로 생성
 % range FFT 된 데이터를 :를 통해 frame에 대해 accumulation해야 함
 for frames = 1:Nframe
    range_time_cell{chirps}(frames,:,:) = squeeze(radarCubeData_mti_cell{frames}(chirps,:,:));
 end
 % power of range_time
 sdb_rangetime_cell{chirps} = squeeze(10*log10((abs(range_time_cell{chirps}(:,chanIdx,:)))));
end
