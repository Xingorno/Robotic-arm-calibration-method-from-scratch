% Simplified Kinematics based on Jeff's work
% INPUT:
% THETA: 7*N radian information read from each encoder
% LENGTH: 7*1 （mm） the length of each link and translation on the end
% PHI: 7*1 the residual angle compensate for the inaccurate encoder
% information

% OUTPUT: 4*4*N the matrix from the tip to the virtual arm base

% function [v_tip2base_matrix, v_tip2base_separate] = SimplifiedKinematics(LENGTH,THETA, A_coeff,B_coeff, PHI)
    function [m_tip2base] = SimplifiedKinematics(LENGTH, THETA, PHI, Coeff, correction_flag, encoder1_LUT, encoder2_LUT, encoder5_LUT)

    N = size(THETA,2);
    m_tip2base = zeros(4, 4, N);
    v_tip2base_separate = zeros(4, N);
    v_tip2base_matrix = zeros(4, N);
    
    for i = 1:N
        Theta = THETA(:,i);
        [theta1, theta2, theta3, theta4, theta5, theta6, theta7] = THETA_Transform(Theta, PHI, Coeff, correction_flag, encoder1_LUT, encoder2_LUT, encoder5_LUT);
        T1 = Transform1(theta1, LENGTH(1), Coeff(1), Coeff(2));
        T2 = Transform2(theta2, LENGTH(2), Coeff(3), Coeff(4));
        T3 = Transform3(theta3, LENGTH(3));
        T4 = Transform4(theta4, LENGTH(4));%, Coeff(5), Coeff(6));
        T5 = Transform5(theta5);
        T6 = Transform6(theta6);
        T7 = Transform7(theta7,LENGTH(6));
        T8 = Transform8(LENGTH(5),LENGTH(7));%, Coeff(1), Coeff(2));
        m_tip2base(:,:,i) = T1*T2*T3*T4*T5*T6*T7*T8;
%         v_tip2base_separate(:,i) = Intermediate_V_tip2base(Theta, PHI);
        v_tip2base_matrix(:,i) = m_tip2base(:,:,i)*[0 0 0 1]';
        
    end

% Theta: 7*1
    function [theta1, theta2, theta3, theta4, theta5, theta6, theta7] = THETA_Transform(Theta, PHI, Coeff, correction_flag, encoder1_LUT, encoder2_LUT, encoder5_LUT) 
%         theta1 = A_coeff(1)*Theta(1)*Theta(1) +B_coeff(1)*Theta(1) + PHI(1);
%         theta2 = A_coeff(2)*Theta(2)*Theta(2) +B_coeff(2)*Theta(2) + PHI(2);
%         theta3 = A_coeff(3)*Theta(3)*Theta(3) +B_coeff(3)*Theta(3) + PHI(3);
% %         theta4 = A_coeff(4)*THETA(4)*THETA(4) +B_coeff(4)*THETA(4) + PHI(4);
%         theta4 = Theta(4) + PHI(4);
%         theta5 = A_coeff(4)*Theta(5)*Theta(5) +B_coeff(4)*Theta(5) + PHI(5);
%         theta6 = A_coeff(5)*Theta(6)*Theta(6) +B_coeff(5)*Theta(6) + PHI(6);
%         theta7 = A_coeff(6)*Theta(7)*Theta(7) +B_coeff(6)*Theta(7) + PHI(7);
        
% corrected theta1
        theta1 = 0;
        theta2 = 0;
        theta3 = 0;
        theta4 = 0;
        theta5 = 0;
        theta6 = 0;
        theta7 = 0;
        
        switch correction_flag(1)
            case 2
%                 lookup_table_unique = load('lookup_table_encoder1_p.mat');
                theta1 = LookupTable(Theta(1)*180/pi, encoder1_LUT.lookup_table_unique);
                theta1 = theta1*pi/180;
            case 1
                % % correction from kinematics, probably including residual
%                 from other joints
%                 if Theta(1)< -0.0175
%                     theta1 = -0.0059 + 0.9147*Theta(1) + 0.7669*Theta(1)*Theta(1) + 2.4076*Theta(1)*Theta(1)*Theta(1) + 1.1718*Theta(1)*Theta(1)*Theta(1)*Theta(1);
% 
% %                     theta1 = corrected_coeff(1) + corrected_coeff(2)*Theta(1) + corrected_coeff(3)*Theta(1)*Theta(1) + corrected_coeff(4)*Theta(1)*Theta(1)*Theta(1) +corrected_coeff(5)*Theta(1)*Theta(1)*Theta(1)*Theta(1);% + corrected_coeff(6)*Theta(1)*Theta(1)*Theta(1)*Theta(1)*Theta(1) + corrected_coeff(7)*Theta(1)*Theta(1)*Theta(1)*Theta(1)*Theta(1)*Theta(1);
%                 else
%                     theta1 = -0.0056 + 1.0266*Theta(1) - 0.1931*Theta(1)*Theta(1) + 0.3678*Theta(1)*Theta(1)*Theta(1);
% 
% %                     theta1 = corrected_coeff(6) + corrected_coeff(7)*Theta(1) + corrected_coeff(8)*Theta(1)*Theta(1) + corrected_coeff(9)*Theta(1)*Theta(1)*Theta(1);% + corrected_coeff(12)*Theta(1)*Theta(1)*Theta(1)*Theta(1) + corrected_coeff(13)*Theta(1)*Theta(1)*Theta(1)*Theta(1)*Theta(1);
%                 end
%                  theta1 =  Coeff(1)*Theta(1) + Coeff(2)*Theta(1)*Theta(1) +  Coeff(3)*Theta(1)*Theta(1)*Theta(1);  
%               %% correction just on joint 1, need to add a constant
%               residual on this joint, since we set 0 redian as the ideal
%               one, in practice, some residual exists
                  if Theta(1) < 0
                      theta1 = -0.0013 + 0.7254*Theta(1) -0.1350*Theta(1)*Theta(1) - 0.0371*Theta(1)*Theta(1)*Theta(1) - 0.8662*Theta(1)*Theta(1)*Theta(1)*Theta(1);
                  else
                      theta1 = 0.0005 + 1.0047*Theta(1) -0.3723*Theta(1)*Theta(1) + 0.5297*Theta(1)*Theta(1)*Theta(1);
                  end
            case 0
                theta1 = Theta(1);
        end
        theta1 = theta1 + PHI(1);%+ theta1*theta1*Coeff(1);
%         theta1 = theta1 + 0.0443;
        switch correction_flag(2)
            case 2
                
%                 lookup_table_unique = load('lookup_table_encoder2_p.mat');
%                 theta2 = LookupTable(Theta(2)*180/pi, lookup_table_unique.lookup_table_unique);
                theta2 = LookupTable(Theta(2)*180/pi, encoder2_LUT.lookup_table_unique);
                theta2 = theta2*pi/180;
%                 
            case 1
                  theta2 = -0.0002 + 1.0004*Theta(2); % corrected by joint 2 verification
%                   theta2 = -0.0010 + 0.9987*Theta(2) + 0.0090*Theta(2)*Theta(2); % not sure if this would work
                  
%                   theta2 = -0.0629 + 0.9917*Theta(2); % correct by only joint2 movement
%                 theta2 = -0.0046 + 1.1233*Theta(2);
            case 0
                theta2 = Theta(2);
        end
        theta2 = theta2 + PHI(2);%+ theta2*theta2*Coeff(2);
%         theta2 = theta2 -0.0268;
        switch correction_flag(3)
            case 1
                theta3 = 0.0033 + 0.9818*Theta(3);
            case 0
                theta3 = Theta(3);
        end
        theta3 = theta3 + PHI(3);
%         theta4 = theta4 -0.0195;
        switch correction_flag(4)
            case 1
                theta4 = Theta(4);
            case 0
                theta4 = Theta(4);
        end
        theta4 = theta4 + PHI(4);
        
        switch correction_flag(5)
            case 2
                theta5 = LookupTable(Theta(5)*180/pi, encoder5_LUT.lookup_table_unique);
                theta5 = theta5*pi/180;
                
            case 1
                theta5 = -0.0028 + Theta(5) + 0.0341*sin(2.5616*Theta(5) + 0.0091); % corrected by protractor readings without load
%                 theta5 = 0.0009 + 1.0639*Theta(5) -0.0260*Theta(5)*Theta(5) -0.0268*Theta(5)*Theta(5)*Theta(5); % corrected by only variable joint 5
%                 theta5 = -0.0031 + 0.8121*Theta(5);
%                 theta5 = -0.0808 + 0.8169*Theta(5); %Later
            case 0
                theta5 = Theta(5);
        end
        theta5 = theta5 + PHI(5);
        
        switch correction_flag(6)
            case 1
%                 theta6 = -0.0217 + 0.8088*Theta(6);
                theta6 = Theta(6);
            case 0
                theta6 = Theta(6) - 13.6*pi/180;
        end
        theta6 = theta6 + PHI(6);% + Coeff(1)*sin(Coeff(2)*theta6 + Coeff(3));
        switch correction_flag(7)
            case 1
                theta7 = Theta(7);
            case 0
                theta7 = Theta(7);
        end
        theta7 = theta7 + PHI(7);
% %         theta1 = PHI+ Coeff(1)*Theta(1) + Coeff(2)*Theta(1)*Theta(1)+ Coeff(3)*Theta(1)*Theta(1)*Theta(1)+ Coeff(4)*Theta(1)*Theta(1)*Theta(1)*Theta(1);%+ Coeff(5)*Theta(1)*Theta(1)*Theta(1)*Theta(1)*Theta(1)+ Coeff(6)*Theta(1)*Theta(1)*Theta(1)*Theta(1)*Theta(1)*Theta(1);
%         theta1 = Theta(1);
%         theta2 = Theta(2);
% %          theta2 = -0.0034 + 1.0914*Theta(2);
%         theta3 = Theta(3);
% %         theta3 = 0.0031 + 0.9830*Theta(3);
% %         theta3 = PHI+ Coeff(1)*Theta(3) + Coeff(2)*Theta(3)*Theta(3)+ Coeff(3)*Theta(3)*Theta(3)*Theta(3)+ Coeff(4)*Theta(3)*Theta(3)*Theta(3)*Theta(3);%+ Coeff(5)*Theta(1)*Theta(1)*Theta(1)*Theta(1)*Theta(1)+ Coeff(6)*Theta(1)*Theta(1)*Theta(1)*Theta(1)*Theta(1)*Theta(1);
% 
%         theta4 = Theta(4);
% %         theta5 = -0.0015 + 0.8362*Theta(5);
%         theta5 = Theta(5);
%         theta6 = Theta(6);
%         theta7 = Theta(7);
        
    end
    function T1 = Transform1(theta1, length1, alpha1, omega1)%,Coeff)
%         C1 = Coeff(1) + Coeff(2)*sin(theta1 + Coeff(3));
%         S1 = Coeff(4) + Coeff(5)*cos(theta1);
%          m_rotation_y = [C1 0 S1 0; 
%             0 1 0 0;
%             -S1 0 C1 0;
%             0 0 0 1];
        switch nargin
            case 2 
                alpha1 = 0;
                omega1 = 0;
            case 3
                omega1 = 0;
            case 4
                alpha1 = alpha1;
                omega1 = omega1;
        end
        m_rotation_y = [cos(theta1) 0 sin(theta1) 0; 
            0 1 0 0;
            -sin(theta1) 0 cos(theta1) 0;
            0 0 0 1];
        m_translation_x = [1 0 0 length1;
            0 1 0 0;
            0 0 1 0;
            0 0 0 1];
        m_rotation_x = [1 0 0 0;
            0 cos(alpha1) -sin(alpha1) 0;
            0 sin(alpha1) cos(alpha1) 0;
            0 0 0 1];
        m_rotation_z = [cos(omega1) -sin(omega1) 0 0;
            sin(omega1) cos(omega1) 0 0;
            0 0 1 0;
            0 0 0 1];
%         m_rotation_y*m_translation_x
        T1 = m_rotation_y*m_translation_x*m_rotation_x*m_rotation_z;
    end

    function T2 = Transform2(theta2, length2, alpha2, omega2)
        switch nargin
            case 2
                alpha2 = 0;
                omega2 = 0;
            case 3
                omega2 = 0;
            case 4
                alpha2 = alpha2;
                omega2 = omega2;
        end
        m_rotation_y = [cos(theta2) 0 sin(theta2) 0; 
            0 1 0 0;
            -sin(theta2) 0 cos(theta2) 0;
            0 0 0 1];
        m_translation_x = [1 0 0 length2;
            0 1 0 0;
            0 0 1 0;
            0 0 0 1];
        m_rotation_x = [1 0 0 0;
            0 cos(alpha2) -sin(alpha2) 0;
            0 sin(alpha2) cos(alpha2) 0;
            0 0 0 1];
        m_rotation_z = [cos(omega2) -sin(omega2) 0 0;
            sin(omega2) cos(omega2) 0 0;
            0 0 1 0;
            0 0 0 1];
        T2 = m_rotation_y*m_translation_x*m_rotation_x*m_rotation_z;
    end
    
    function T3 = Transform3(theta3, length3)
        
        T3 = [1 0 0 length3*cos(theta3);
            0 1 0 length3*sin(theta3);
            0 0 1 0;
            0 0 0 1];
        
    end
    
    function T4 = Transform4(theta4, length4, theta4_x, theta4_z)
        
        switch nargin
            case 2
                
                theta4_x = 0;
                theta4_z = 0;
            case 3
                
                theta4_x = theta4_x;
                theta4_z = 0;
            case 4
                theta4_x = theta4_x;
                theta4_z = theta4_z;

        end
        
       
        m_rotation_y = [cos(theta4) 0 sin(theta4) 0; 
            0 1 0 0;
            -sin(theta4) 0 cos(theta4) 0;
            0 0 0 1];
        m_translation_x = [1 0 0 length4;
            0 1 0 0;
            0 0 1 0;
            0 0 0 1];
        m_rotation_x = [1 0 0 0;
            0 cos(theta4_x) -sin(theta4_x) 0;
            0 sin(theta4_x) cos(theta4_x) 0;
            0 0 0 1];
        m_rotation_z = [cos(theta4_z) -sin(theta4_z) 0 0;
            sin(theta4_z) cos(theta4_z) 0 0;
            0 0 1 0;
            0 0 0 1];
        T4 = m_rotation_y*m_translation_x*m_rotation_x*m_rotation_z;
    end

    function T5 = Transform5(theta5, theta5_y)
        
        switch nargin
            case 1
                theta5_y = 0;
            case 2
                theta5_y = theta5_y;
        end
        
        m_rotation_y = [cos(theta5_y) 0 sin(theta5_y) 0; 
            0 1 0 0;
            -sin(theta5_y) 0 cos(theta5_y) 0;
            0 0 0 1];
        m_rotation_x = [1 0 0 0;
            0 cos(-theta5) -sin(-theta5) 0;
            0 sin(-theta5) cos(-theta5) 0;
            0 0 0 1];
        
        T5 = m_rotation_y*m_rotation_x;
%         % Jeff's matrix
%         T5 = [1 0 0 0;
%             0 cos(theta5) -sin(theta5) 0;
%             0 sin(theta5) cos(theta5) 0;
%             0 0 0 1];
    end

    function T6 = Transform6(theta6, theta6_x)
        switch nargin
            case 1
                theta6_x = 0;
            case 2
                theta6_x = theta6_x;
        end
        m_rotation_x = [1 0 0 0;
            0 cos(theta6_x) -sin(theta6_x) 0;
            0 sin(theta6_x) cos(theta6_x) 0;
            0 0 0 1];
        m_rotation_z = [cos(theta6) -sin(theta6) 0 0;
            sin(theta6) cos(theta6) 0 0;
            0 0 1 0;
            0 0 0 1];
        T6 = m_rotation_z*m_rotation_x;
    end
    function T7 = Transform7(theta7, length6)
        m_translation_y = [1 0 0 0;
            0 1 0 length6;
            0 0 1 0;
            0 0 0 1];
        
        m_rotation_y = [cos(-theta7) 0 sin(-theta7) 0;
            0 1 0 0;
            -sin(-theta7) 0 cos(-theta7) 0;
            0 0 0 1];
        T7 = m_translation_y*m_rotation_y;
        %Jeff's matrix
%         T7 = [cos(theta7) 0 sin(theta7) 0;
%             0 1 0 0;
%             -sin(theta7) 0 cos(theta7) 0;
%             0 0 0 1];
        
    end
    function T8 = Transform8(length5, length7, alpha, omega)
        switch nargin
            case 2
                alpha = 0;
                omega = 0;
            case 3
                omega = 0;
                alpha = alpha;
            case 4
                alpha = alpha;
                omega = omega;
        end
        
        
        m_translation_x = [1 0 0 length5;
            0 1 0 0; 
            0 0 1 0;
            0 0 0 1];
        m_translation_z = [1 0 0 0;
            0 1 0 0; 
            0 0 1 length7;
            0 0 0 1];
        m_rotation_x = [1 0 0 0;
            0 cos(alpha) -sin(alpha) 0;
            0 sin(alpha) cos(alpha) 0;
            0 0 0 1];
        m_rotation_z = [cos(omega) -sin(omega) 0 0;
            sin(omega) cos(omega) 0 0;
            0 0 1 0;
            0 0 0 1];
        T8 = m_translation_x*m_rotation_x*m_translation_z*m_rotation_z;
    end



    %% Second way do kinematics
     
%     function v_tip2base = Intermediate_V_tip2base(THETA, PHI)
%         [S1, S2, S3, S4, S5, S6, S7, C1, C2, C3, C4, C5, C6, C7] = SinCosSimply(THETA, PHI);
%         [variable_A, variable_B, variable_C] = Intermediate_variable(THETA, PHI);
%         
%         
%         temp1 = (C1*C2 - S1*S2)*(C4*variable_A + S4*variable_C + LENGTH(4)*C4 + LENGTH(3)*C3 + LENGTH(2)) + (C1*S2 + S1*C2)*(-S4*variable_A + C4*variable_C - LENGTH(4)*S4) + C1*LENGTH(1);
%         temp2 = variable_B + LENGTH(3)*S3;
%         temp3 = (-S1*C2 -C1*S2)*(C4*variable_A + S4*variable_C + LENGTH(4)*C4 + LENGTH(3)*C3 + LENGTH(2)) + (-S1*S2 + C1*C2)*(-S4*variable_A + C4*variable_C - LENGTH(4)*S4) - S1*LENGTH(1);
%         temp4 = 1;
%         v_tip2base = [temp1; temp2; temp3; temp4];
%     end
%     function [variable_A, variable_B, variable_C] = Intermediate_variable(THETA, PHI)
%         [S1, S2, S3, S4, S5, S6, S7, C1, C2, C3, C4, C5, C6, C7] = SinCosSimply(THETA, PHI);
%         
%         variable_A = C6*(LENGTH(5)*C7 + LENGTH(7)*S7) - LENGTH(6)*S6;
%         variable_B = C5*(LENGTH(5)*S6*C7 + LENGTH(7)*S6*S7 + LENGTH(6)*C6) - S5*(-LENGTH(5)*S7 + LENGTH(7)*C7);
%         variable_C = S5*(LENGTH(5)*S6*C7 + LENGTH(7)*S6*S7 + LENGTH(6)*C6) + C5*(-LENGTH(5)*S7 + LENGTH(7)*C7);
% %         variable_A = S4*S5*(S6*C7*A(3) - C6*D(3)) + S4*C5*S7*A(3) + C4*(C6*C7*A(3) + S6*D(3)) + C4*A(2);
% %         variable_B = C4*S5*(S6*C7*A(3) - C6*D(3)) + C4*C5*S7*A(3) - S4*(C6*C7*A(3) + S6*D(3)) - S4*A(2);
%     end
%     function [S1, S2, S3, S4, S5, S6, S7, C1, C2, C3, C4, C5, C6, C7] = SinCosSimply(THETA, PHI)
%         
%         theta1 = A_coeff(1)*THETA(1)*THETA(1) +B_coeff(1)*THETA(1) + PHI(1);
%         theta2 = A_coeff(2)*THETA(2)*THETA(2) +B_coeff(2)*THETA(2) + PHI(2);
%         theta3 = A_coeff(3)*THETA(3)*THETA(3) +B_coeff(3)*THETA(3) + PHI(3);
% %         theta4 = A_coeff(4)*THETA(4)*THETA(4) +B_coeff(4)*THETA(4) + PHI(4);
%         theta4 = THETA(4) + PHI(4);
%         theta5 = A_coeff(4)*(-THETA(5))*(-THETA(5)) +B_coeff(4)*(-THETA(5)) + PHI(5);
%         theta6 = A_coeff(5)*THETA(6)*THETA(6) +B_coeff(5)*THETA(6) + PHI(6);
%         theta7 = A_coeff(6)*(-THETA(7))*(-THETA(7)) +B_coeff(6)*(-THETA(7)) + PHI(7);
%         
%        S1 = sin(theta1);
%        S2 = sin(theta2);
%        S3 = sin(theta3);
%        S4 = sin(theta4);
%        S5 = sin(theta5);
%        S6 = sin(theta6);
%        S7 = sin(theta7);
%        
%        C1 = cos(theta1);
%        C2 = cos(theta2);
%        C3 = cos(theta3);
%        C4 = cos(theta4);
%        C5 = cos(theta5);
%        C6 = cos(theta6);
%        C7 = cos(theta7);
%     end
    
end
