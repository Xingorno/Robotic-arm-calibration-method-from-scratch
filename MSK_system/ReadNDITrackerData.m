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