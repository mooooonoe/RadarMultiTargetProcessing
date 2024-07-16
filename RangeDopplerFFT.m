%% Range Doppler FFT
function [doppler_cell, doppler_mti_cell, db_doppler_cell, db_doppler_mti_cell] = RangeDopplerFFT(NChirp, NChan, NSample, Nframe, ...
    chanIdx, radarCubeData_cell, radarCubeData_mti_cell)

doppler_cell = cell(1,Nframe);
doppler_mti_cell = cell(1,Nframe);
db_doppler_cell = cell(1,Nframe);
db_doppler_mti_cell = cell(1,Nframe);

% pre allocation
doppler = zeros(NChirp,NChan,NSample);
doppler_mti = zeros(NChirp,NChan,NSample);
win_dop = hann(NChirp);

for frames = 1:Nframe
 % range doppelr FFT - not MTI filtered data
 for rangebin_size = 1:NSample
    for chIdx = 1:NChan
        DopData1 = squeeze(radarCubeData_cell{frames}(:, chIdx, rangebin_size));
        DopData = fftshift(fft(DopData1 .* win_dop, NChirp));
        doppler(:, chIdx, rangebin_size) = DopData;
    end
 end
doppler_cell{frames} = doppler;

% range doppelr FFT- MTI filtered data 
 for rangebin_size = 1:NSample
    for chIdx = 1:NChan
        DopData1_mti = squeeze(radarCubeData_mti_cell{frames}(:, chIdx, rangebin_size));
        DopData_mti = fftshift(fft(DopData1_mti .* win_dop, NChirp));
        doppler_mti(:, chIdx, rangebin_size) = DopData_mti;
    end
 end
doppler_mti_cell{frames} = doppler_mti;

% power
 % MTI
 doppler1_mti =  doppler_mti_cell{frames}(:,chanIdx,:);
 doppler1_128x256_mti = squeeze(doppler1_mti);
 db_doppler_mti_cell{frames} = 10*log10(abs(doppler1_128x256_mti'));
 % not MTI
 doppler1 = doppler_cell{frames}(:,chanIdx,:);
 doppler1_128x256 = squeeze(doppler1);
 db_doppler_cell{frames} = 10*log10(abs(doppler1_128x256'));
end



% %가장 큰 값의 인덱스
% [maxValue, linearIndex] = max(db_doppler(:));
% [max_row, max_col] = ind2sub(size(db_doppler), linearIndex);

