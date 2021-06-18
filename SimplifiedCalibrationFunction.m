function F = SimplifiedCalibrationFunction(X, THETA, v_NDI_tip2ref, mode, correction_flag, encoder1_LUT, encoder2_LUT, encoder5_LUT)
    N = size(THETA, 2);
    
    switch mode
        % THE FULL JOINTS  
        case 1 
            LENGTH = X(1:7);
            PHI = X(8:14);
            t_base2ref = X(19:21);
            angle_base2ref = X(22:24);
        % THE LAST JOINTS
        case 2
            
            LENGTH = [254;64.04;254; X(1); X(2); X(3); X(4)]; %length
            PHI =[0; 0; 0; 0; X(5); X(6); X(7)];

            t_base2ref = X(8:10);
            angle_base2ref = X(11:13);
        % THE FIRST JONITS
        case 3
            
            LENGTH = [X(1);X(2);X(3); 135.7122; 0; 355.4; 0]; %length
            PHI =[X(4); X(5); X(6); X(7); 0; 0; 0];

            t_base2ref = X(8:10);
            angle_base2ref = X(11:13);
    end
     
    
    coeff = X(15:18);
    

    v_tip2ref = [];
    m_tip2base = SimplifiedKinematics(LENGTH, THETA, PHI,coeff,correction_flag, encoder1_LUT, encoder2_LUT, encoder5_LUT);
    m_base2ref = Intermediate_T_base2ref(t_base2ref, angle_base2ref);
    for i = 1: N
%         v_tip2ref(:,i) = m_tip2base(:,:,i)*m_base2ref*[0 0 0 1]';
        v_tip2ref(:,i) = m_base2ref*m_tip2base(:,:,i)*[0 0 0 1]';
%         v_tip2ref(:,i) = m_tip2base(:,:,i)*[0 0 0 1]';
    end
    
    ref1 = v_tip2ref(1:3,1);
    ref2 = v_NDI_tip2ref(1:3,1);
    
    for i = 1:N
%       % relative metric
%         temp1_arm = v_tip2ref(1:3,i) - ref1;
%         temp1_NDI = v_NDI_tip2ref(1:3,i) - ref2;
%         F(i) = vecnorm(temp1_arm) - vecnorm(temp1_NDI);
        % global metric
        F(3*i - 2) = v_tip2ref(1,i) - v_NDI_tip2ref(1,i);
        F(3*i - 1) = v_tip2ref(2,i) - v_NDI_tip2ref(2,i);
        F(3*i - 0) = v_tip2ref(3,i) - v_NDI_tip2ref(3,i);
          % each point as an unit metric 
%         temp1 = v_tip2ref(1,i) - v_NDI_tip2ref(1,i);
%         temp2 = v_tip2ref(2,i) - v_NDI_tip2ref(2,i);
%         temp3 = v_tip2ref(3,i) - v_NDI_tip2ref(3,i);
%         F(i) = sqrt(temp1^2 + temp2^2 + temp3^2);
    end
    
    
    function m_base2ref = Intermediate_T_base2ref(t_base2ref, angle_base2ref)
        
          Rz = [cos(angle_base2ref(1)) -sin(angle_base2ref(1)) 0;
              sin(angle_base2ref(1)) cos(angle_base2ref(1)) 0;
              0 0 1];
          Ry = [cos(angle_base2ref(2)) 0 sin(angle_base2ref(2));
              0 1 0;
              -sin(angle_base2ref(2)) 0 cos(angle_base2ref(2))];
          Rx = [1 0 0;
              0 cos(angle_base2ref(3)) -sin(angle_base2ref(3));
              0 sin(angle_base2ref(3)) cos(angle_base2ref(3))];
          t = t_base2ref';
          R = Rz*Ry*Rx;
          m_base2ref = [R t; [0 0 0 1]];
        
        
        
        
%         r11 = cos(angle_base2ref(1))*cos(angle_base2ref(2));
%         r12 = cos(angle_base2ref(1))*sin(angle_base2ref(2))*sin(angle_base2ref(3))- sin(angle_base2ref(1))*cos(angle_base2ref(3));
%         r13 = cos(angle_base2ref(1))*sin(angle_base2ref(2))*cos(angle_base2ref(3))+ sin(angle_base2ref(1))*sin(angle_base2ref(3));
%         r21 = sin(angle_base2ref(1))*cos(angle_base2ref(2));
%         r22 = sin(angle_base2ref(1))*sin(angle_base2ref(2))*sin(angle_base2ref(3)) + cos(angle_base2ref(1))*cos(angle_base2ref(3));
%         r23 = sin(angle_base2ref(1))*sin(angle_base2ref(2))*cos(angle_base2ref(3)) - cos(angle_base2ref(1))*sin(angle_base2ref(3));
%         r31 = -sin(angle_base2ref(2));
%         r32 = cos(angle_base2ref(2))*sin(angle_base2ref(3));
%         r33 = cos(angle_base2ref(2))*cos(angle_base2ref(3));
%         m_base2ref(1,1) = r11;
%         m_base2ref(1,2) = r12;
%         m_base2ref(1,3) = r13;
%         m_base2ref(1,4) = t_base2ref(1);% translation tx
%         m_base2ref(2,1) = r21;
%         m_base2ref(2,2) = r22;
%         m_base2ref(2,3) = r23;
%         m_base2ref(2,4) = t_base2ref(2);
%         m_base2ref(3,1) = r31;
%         m_base2ref(3,2) = r32;
%         m_base2ref(3,3) = r33;
%         m_base2ref(3,4) = t_base2ref(3);
%         m_base2ref(4,:) = [0. 0, 0, 1];
    end
end