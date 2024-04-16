function [encoder_readings_total, num] = ReadEncoderDataForTesting(encoder_fileNAME)
    encoder_lines = readlines(encoder_fileNAME);
    num = floor(size(encoder_lines, 1)/4);
    encoder_readings_total = [];
    for i = 1:num
        encoder_readings = encoder_lines((i-1)*4 + 4);
        new_encoder_readings = split(encoder_readings, ' ');
        
        new_encoder_readings_temp = [];
        for j = 1: size(new_encoder_readings, 1)
            new_encoder_readings_temp = [new_encoder_readings_temp, str2num(new_encoder_readings(j))];

        end
        new_encoder_readings_temp_4x4 = reshape(new_encoder_readings_temp, [4, 4])';
        encoder_readings_total = [encoder_readings_total; new_encoder_readings_temp_4x4];
    
    end
end