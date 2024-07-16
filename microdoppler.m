%% Micro doppler
function [time_axis, micro_doppler_mti, micro_doppler, sdb_mti, sdb] = microdoppler(chanIdx, NChirp, NChan, Nframe, NSample, radarCubeData_mti_cell, radarCubeData_cell)

% pre allocation
micro_doppler = cell(1, NSample);
micro_doppler_mti = cell(1, NSample);
frame_data = zeros(NChirp, NChan, Nframe);
frame_data_mti = zeros(NChirp, NChan, Nframe);
sdb = cell(1, NSample);
sdb_mti = cell(1, NSample);

% time axis
% max_time = pri * NChan * NChirp * Nframe;  %frame period = 39.17ms
max_time = 0.04 * Nframe;  %frame period = 40ms
time_axis = linspace(0,max_time,Nframe);

% frame마다 반복
for Ranges = 1:NSample
 % 거리를 RangeBinIdx로 설정하고 3d data (slow time, channels, frames)로 생성
 for frames = 1:Nframe
 frame_data_mti(:,:,frames) = squeeze(radarCubeData_mti_cell{frames}(:,:,Ranges));
 frame_data(:,:,frames) = squeeze(radarCubeData_cell{frames}(:,:,Ranges));
 end

 % micro_doppler = permute(micro_doppler,[3 2 1]);
 % 시간 축이 존재하므로 frame이 accumulation돼야 함.
 % 따라서 frame에 대해 반복하면 안되고 :를 통해 accumulation해야 함
 for chIdx = 1:NChan
   micro_doppler_mti{Ranges}(:,chIdx,:) = fftshift(fft(squeeze(frame_data_mti(:,chIdx,:)).*hann(NChirp),NChirp),1);
   micro_doppler{Ranges}(:,chIdx,:) = fftshift(fft(squeeze(frame_data(:,chIdx,:)).*hann(NChirp),NChirp),1);
 end

 % power of microdoppler
 sdb{Ranges} = squeeze(abs(micro_doppler{Ranges}(:,chanIdx,:)));
 sdb_mti{Ranges} = squeeze(abs(micro_doppler_mti{Ranges}(:,chanIdx,:)));
end