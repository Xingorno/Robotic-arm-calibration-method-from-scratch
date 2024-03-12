clc
clear all


dir = "E:\PROGRAM\Project_PhD\Calibration\Gynecology_system\Data\"
encoder_filename = "TrackingData.txt"
encoder_fileNAME = strcat(dir, encoder_filename);
[encoder_readings_total, num_encoder] = ReadEncoderData(encoder_fileNAME);
encoder_saveNAME = strcat(dir, "encoder_real.txt");
save(encoder_saveNAME, "encoder_readings_total",'-ascii')

NDItracker_filename = "20231018_Optical_NDI.txt"
NDItracker_fileNAME = strcat(dir, NDItracker_filename);
[NDItracker_readings_total, num_NDItracker] = ReadNDITrackerData(NDItracker_fileNAME);
NDItracker_saveNAME = strcat(dir, "NDItracker_real.txt")
save(NDItracker_saveNAME, "NDItracker_readings_total", '-ascii')


function [encoder_readings_total, num] = ReadEncoderData(encoder_fileNAME)
    encoder_lines = readlines(encoder_fileNAME);
    num = floor(size(encoder_lines, 1)/4);
    encoder_readings_total = [];
    for i = 1:num
        encoder_readings = encoder_lines((i-1)*4 + 2);
        new_encoder_readings = split(encoder_readings, ' ');
        new_encoder_readings_temp = [];
        for j = 1: size(new_encoder_readings, 1)
            new_encoder_readings_temp = [new_encoder_readings_temp, str2num(new_encoder_readings(j))];
        end
        encoder_readings_total = [encoder_readings_total; new_encoder_readings_temp];
    
    end
    
end

function [NDItracker_readings_total, num] = ReadNDITrackerData(NDItracker_fileNAME)
    NDItracker_lines = readlines(NDItracker_fileNAME);
    num = floor(size(NDItracker_lines, 1)/6);
    NDItracker_readings_total = [];
    for i = 1:num
        new_NDItracker_readings_temp = [];
        for j = 1: 4
            NDItracker_readings = NDItracker_lines((i-1)*6 + j+1);
            new_NDItracker_readings = split(NDItracker_readings, ' ');
            
            new_NDItracker_readings_temp_ = [str2num(new_NDItracker_readings(1)), str2num(new_NDItracker_readings(2)), str2num(new_NDItracker_readings(3)), str2num(new_NDItracker_readings(4))];
            new_NDItracker_readings_temp = [new_NDItracker_readings_temp; new_NDItracker_readings_temp_];
        end
        NDItracker_readings_total = [NDItracker_readings_total; new_NDItracker_readings_temp];
    
    end
    
end