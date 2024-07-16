% 작업공간 지우기
clear;
% 명령창 지우기
clc;
% plot 지우기
close all;
%load("X:\Personals\Subin_Moon\Radar\0_data\walking_and_run_adc_raw_data.mat");
load("Z:\Radar\0_data\walking_and_run_adc_raw_data.mat");

%% parameters
chirpsIdx=100;
chanIdx=1;
frame_number=70;

numrangeBins=256;
NChirp=128;
NChan=4;
NSample=256;
Rx = 4;
Tx = 1;
Nframe = 256;                        % 프레임 수
c = 3e8;                             % 빛의 속도 (미터/초)

%% 바꿔야되는 파라미터들 mmwave.json file에 있음
ramp_end_tiem = 58.42e-6;
idle_time = 5e-6;
start_frequency = 77e9;              % 시작 주파수 (Hz)
slope = 50.018;                      % 슬로프 (MHz/us)
samples_per_chirp = 256;             % 하나의 칩에서의 샘플 수
chirps_per_frame = 128;              % 프레임 당 chirps 수
sampling_rate = 5e9;                 % 샘플링 속도 (Hz)

%% 위의 파라미터를 통해 계산한 파라미터
pri=63.42e-6;                                     % ramp end tiem + idle time
prf=1/pri;
wavelength = c / start_frequency;                 % 파장(lambda)
bandwidth = ramp_end_tiem * (slope*1e+12);        % 대역폭(Hz)=ramp end time*frequency slope
sampling_time = NSample/sampling_rate;            % 샘플링 타임 (s)

% Range Parameter
freq_res = sampling_rate*(1e-3)/NSample;
freqBin= (0:NSample-1).'*freq_res;
rangeBin = freqBin'*c/(slope*1e+12)/2;            % Range Bin
range_resolution = rangeBin(2);                   % 거리 해상도
max_range = (NSample-1)*range_resolution;         % 최대 거리

% Velocity Parameter
vel_resolution =wavelength/(2*pri*NChirp);        % 속도 해상도 (m/s) = wavelength/(2*pri*Nchirp)
max_vel = wavelength/(4*pri);                     % 최대 속도
velocityBin = -max_vel:vel_resolution:max_vel;    % Velocity Bin
% DopplerBin = fftshiftfreqgrid(NChirp,prf);
% velocityBin = DopplerBin'*wavelength/2;         % Velocity Bin

% Angle Parameter
w = linspace(-1,1,256);
angleBin = asin(w)*180/pi;                        % Angle Bin

% 전체 걸린 시간 frame periodicity : 40ms -> 40ms*256 = 10.24s
frame_periodicity = 4e-2;

% MTI parameter(이때 chirpsIdx가 1보다 커야 함.)
MTIfiltering = 1;

% Range Azimuth FFT parameter
minRangeBinKeep = 0;
rightRangeBinDiscard = 1;
log_plot = 0;
angleFFTSize = 256;
ratio = 0.5;
DopplerCorrection = 0;
d = 1;

% 1D CA-CFAR parameter
window_sz = 33;          % total window size
no_tcell = 24;           % # of training window
no_gcell = 8;            % # of guard window
scale_factor_1D = 2;  % threshold scale factor
object_number = 2;

% CAOS_CFAR_2D parameter
Nt = 24;                 % # of training window
Ng = 8;                  % # of guard window
scale_factor_2D = 1.19;  % threshold scale factor 1.21

Nt_static = 48;          % # of training window
Ng_static = 64;          % # of guard window
scale_factor_2D_static = 2.06; % threshold scale factor 1.02

% FindPeakValue parameter
minPeakHeight = 10;
peak_th = 0.4;

% Clustering parameter
% DBSCAN
eps = 2.5;
MinPts = 4; 
eps_static = 2.5;
MinPts_static = 5; 

% microdoppler parameter
RangeBinIdx = 22;

% Range Azimuth Map 2D CA-CFAR parameter
scale_factor_2D_ram = 1.07;

%% Reshape Data
[frameComplex_cell] = ReshapeData(NChirp, NChan, NSample, Nframe, adcRawData);

%% Time domain output
% frame마다 저장할 필요없음
[currChDataQ, currChDataI, t] = TimeDomainOutput(NSample, sampling_time, chirpsIdx, chanIdx, frame_number, frameComplex_cell);

%% FFT Range Profile
[rangeProfileData, radarCubeData_cell, channelData] = RangeFFT(NChirp, NChan, NSample, Nframe, ...
    chirpsIdx, chanIdx, frame_number, frameComplex_cell);

% MTI filter
[radarCubeData_mti_cell, rangeProfileData_mti, channelData_mti] = MTI_filter(NChirp, NChan, NSample, Nframe,...
    chirpsIdx, chanIdx, frame_number,radarCubeData_cell);

% IIR MTI filter - tracking할 때 제대로 안나옴
[radarCubeData_IIR_mti_cell] = IIR_MTI_filter(NChan, NSample, Nframe, radarCubeData_cell);

%% Range Doppler FFT 
[doppler_cell, doppler_mti_cell, db_doppler_cell, db_doppler_mti_cell] = RangeDopplerFFT(NChirp, NChan, NSample, Nframe, ...
    chanIdx, radarCubeData_cell, radarCubeData_IIR_mti_cell);

%% Range Azimuth FFT
[y_axis, x_axis, angleFFT_output, angleFFT_output_mti, ram_output, ram_output_mti, mag_data, mag_data_mti] = RangeAzimuthFFT(range_resolution, ...
    d, minRangeBinKeep, rightRangeBinDiscard, angleFFTSize, frame_number, doppler_cell, doppler_mti_cell);

%% Find Target
[peak_locs, peaks, target_Idx, target] = FindTarget(minPeakHeight, peak_th, channelData_mti);

%% 1D CA-CFAR
[detected_points_1D, cfarData_mti, th, scale_factor_1D] = CA_CFAR_1D(window_sz, ...
    object_number, scale_factor_1D, no_tcell, no_gcell, rangeProfileData_mti);

%% 2D RDM CFAR
% MTI
[detected_points_2D_cell] = CAOS_CFAR_2D(NSample, NChirp, Nt, Ng, Nframe, scale_factor_2D, db_doppler_mti_cell);
% not MTI
[detected_points_2D_static_cell] = OS_CFAR_2D_static(NSample, NChirp, Nt_static, Ng_static, Nframe, scale_factor_2D_static, db_doppler_cell);
% %
% [detected_points_2D] = OS_CFAR_2D(NSample, NChirp, 32, 12, Nframe, db_doppler_cell);

%% Clustering
% DBSCAN
 % MTI
[clusterGrid_cell, corepts, object_idx_dynamic] = DBSCAN(eps, MinPts, NSample, NChirp, Nframe, detected_points_2D_cell);
 % not MTI
[clusterGrid_static_cell, corepts_static, object_idx_static] = DBSCAN(eps_static, MinPts_static, NSample, NChirp, Nframe, detected_points_2D_static_cell);

%% Micro doppler
% mti 안된거 쓰는게 나을듯
[time_axis, micro_doppler_mti, micro_doppler, sdb_mti, sdb] = microdoppler(chanIdx, NChirp, NChan, Nframe, NSample, radarCubeData_mti_cell, radarCubeData_cell);

%% Range Time Map
[range_time_cell, sdb_rangetime_cell] = RangeTimeMap(NSample, NChan, Nframe, NChirp, chanIdx, radarCubeData_mti_cell);

%% Angle FFT 이게 더 사용 가능성 높을 듯 - 아직 cell로 안만듦
% input: doppler이랑 range_profile이랑 차이가 없는 것 같음.
% AngData_mti : Range Angle Doppler tensor
[AngData, AngData_mti, ram_output2, ram_output2_mti] = AngleFFT(NChirp, ...
    NChan, NSample, angleFFTSize, frame_number, radarCubeData_cell, radarCubeData_mti_cell);
% Angle Cropping
num_crop = 3;
max_value = 1e+04;
Angdata_crop = AngData_mti(:, :, num_crop + 1:NSample - num_crop);
[Angdata_crop] = Normalize(Angdata_crop, max_value);

%% Range Azimuth Map 2D CA-CFAR
Nt = 48;
Ng = 48;
[detected_points_2D_ram] = RAM_CA_CFAR_2D(NSample, angleFFTSize, Nt, Ng, scale_factor_2D_ram, 10*log10(ram_output2_mti));

%% Peak Grouping RDM 
% dynamic data
[objOut_dynamic_cell] = peakGrouping(Nframe, clusterGrid_cell, db_doppler_cell);
% static data
[objOut_static_cell] = peakGrouping(Nframe, clusterGrid_static_cell, db_doppler_cell);

% % Peak Grouping RAM - RAM에 대해서 하는 것은 딱히 필요없음
% [objOut_RAM] = peakGrouping(detected_points_2D_ram, ram_output2);

%% Angle estimation dets
% RDM의 값을 넣었을 때는 Resel_agl이 제대로 나오지 않음
% RAM값을 넣었을 때는 objOut과 objOut_RAM의 차원이 맞지 않아 point cloud에서 오류 발생
% dynamic data
[Resel_agl_dynamic_cell, vel_ambg_list_dynamic_cell, rng_excd_list_dynamic_cell] = angle_estim_dets(objOut_dynamic_cell, Nframe, ...
       doppler_mti_cell, NChirp, angleFFTSize, Rx, Tx, num_crop);
% static data
[Resel_agl_static_cell, vel_ambg_list_static_cell, rng_excd_list_static_cell] = angle_estim_dets(objOut_static_cell, Nframe, ...
       doppler_cell, NChirp, angleFFTSize, Rx, Tx, num_crop);


%% Point Cloud
% dynamic data
[target_x_dynamic_cell, target_y_dynamic_cell, save_det_data_dynamic_cell] = Point_Cloud(Nframe, rangeBin, velocityBin, angleBin, objOut_dynamic_cell, Resel_agl_dynamic_cell);
% static data
[target_x_static_cell, target_y_static_cell, save_det_static_cell] = Point_Cloud(Nframe, rangeBin, velocityBin, angleBin, objOut_static_cell, Resel_agl_static_cell);

% x, y축 범위 설정
x_min = -max_range;
x_max = max_range;
y_min = 0;
y_max = max_range;
