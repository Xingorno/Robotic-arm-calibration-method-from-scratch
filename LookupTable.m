


function mapping_angle = LookupTable(encoder_angle, lookup_table_unique)
    
    index_same = find(lookup_table_unique(1,:) == encoder_angle);
    index_flag = size(index_same,2);
    if index_flag == 1
        mapping_angle = lookup_table_unique(2, index_same);
    else
        index_right = min(find(lookup_table_unique(1,:) > encoder_angle));
        index_left = max(find(lookup_table_unique(1,:) < encoder_angle));
        if size(index_right,2)== 0
            mapping_angle = encoder_angle;
%             mapping_angle = encoder_angle - 360*30/4096;
        elseif size(index_left, 2) == 0
            mapping_angle = encoder_angle;
%             mapping_angle = encoder_angle - 360*30/4096;
        else
        
        delta = lookup_table_unique(1, index_right) - lookup_table_unique(1, index_left);
        mapping_angle = (lookup_table_unique(1, index_right) - encoder_angle)*lookup_table_unique(2, index_left)/delta + (encoder_angle - lookup_table_unique(1, index_left))*lookup_table_unique(2, index_right)/delta;
        end
    end
    
end    