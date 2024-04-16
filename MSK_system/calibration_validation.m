clc
clear all
close all

%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Step1: Load data (encoder readings and NDI tracking data)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% encoder_data = load('E:\PROGRAM\Project_PhD\Calibration\MSK\dataset\encoder_real_20240416.txt');
% NDI_tracking_data = load('E:\PROGRAM\Project_PhD\Calibration\MSK\dataset\NDItracker_real_20240410.txt');

encoder_fileNAME = "E:\PROGRAM\Project_PhD\Calibration\MSK\dataset\TrackingData_20240416.txt";
[arm_readings_total, num_encoder] = ReadEncoderDataForTesting(encoder_fileNAME);

NDItracker_fileNAME = "E:\PROGRAM\Project_PhD\Calibration\MSK\dataset\slicertrackdata_20240416.txt";
[NDI_tracking_data, num_NDItracker] = ReadNDITrackerData(NDItracker_fileNAME);

N = num_encoder; % the number of data

% Encoder data (using the original angle instead of Jeff's
% correction)

encoder_data = [encoder_data(:, 1:3) zeros(N,1) encoder_data(:, 4:6)];
THETA = encoder_data(:,1:7)';

THETA = THETA*pi/180; % Degree to radians
THETA_degree = THETA*180/pi; % Degree

% NDI tracking data
v_NDI_tip2ref_temp = NDI_tracking_data * [0 0 0 1]';
v_NDI_tip2ref = reshape(v_NDI_tip2ref_temp, [4, N]);

% arm tracking data
v_arm_tip2ref_temp = arm_readings_total*[0, 0, 0, 1]';
v_arm_tip2ref = reshape(v_arm_tip2ref_temp, [4, N]);
%%
% 3.1 Display 3D points distribution (qualitatively evaluation)

% Unified to NDI coordinate system
[R, t] = computeRigidTransform(v_arm_tip2ref(1:3,1:end)',v_NDI_tip2ref(1:3,1:end)');
t_alignment(1:3,1:3) = R;
t_alignment(1:3,4) = t;
t_alignment(4,4) = 1;
v_arm_tip2ref_aligned = t_alignment*v_arm_tip2ref;

% Unified to coordinate system of the arm base
[R, t] = computeRigidTransform(v_NDI_tip2ref(1:3,1:end)', v_arm_tip2ref(1:3,1:end)');
t_alignment(1:3,1:3) = R;
t_alignment(1:3,4) = t;
t_alignment(4,4) = 1;
v_NDI_tip2ref_aligned = t_alignment*v_NDI_tip2ref;

figure (1)
scatter3(v_NDI_tip2ref_aligned(1,:), v_NDI_tip2ref_aligned(2,:), v_NDI_tip2ref_aligned(3,:),'filled');
hold on
scatter3(v_arm_tip2ref(1,:), v_arm_tip2ref(2,:), v_arm_tip2ref(3,:),'filled');
hold off
legend('points in NDI', 'points in arm')
xlabel('X')
ylabel('Y')
zlabel('Z')
title('3D point distribution')

% 3.2 Evaluate relative distance error
abs_mean_relative_distance_pre =[];
signed_mean_relative_distance_pre = [];

for reference_index = 1:N % chose one point as the reference point for computing the relative distance
    temp1 = v_arm_tip2ref_aligned(:,reference_index) - v_arm_tip2ref_aligned; 
    delta_Arm = vecnorm(temp1);

    temp2 = v_NDI_tip2ref(:, reference_index) - v_NDI_tip2ref;
    delta_NDI = vecnorm(temp2);

    delta = delta_NDI- delta_Arm;
    
    abs_mean_relative_distance_pre = [abs_mean_relative_distance_pre mean(abs(delta))];
    signed_mean_relative_distance_pre = [signed_mean_relative_distance_pre mean(delta)];
end

disp_signed = ['MEAN(signed): ', num2str(mean(signed_mean_relative_distance_pre)), '   STD(signed): ', num2str(std(signed_mean_relative_distance_pre))];
disp_abs    = ['MEAN(abs)   : ', num2str(mean(abs_mean_relative_distance_pre)), '     STD(abs)   : ', num2str(std(abs_mean_relative_distance_pre))];
disp('NDI as reference')
disp(disp_signed)
disp(disp_abs)

figure(2)
plot(abs_mean_relative_distance_pre)
title('Absolute mean realtive distance','LineWidth',3)
text(150,20, strcat('MEAN:  ',num2str(mean(abs_mean_relative_distance_pre)), 'mm'),'FontSize',12)
text(150,19, strcat('STD   : ', num2str(std(abs_mean_relative_distance_pre)), 'mm'),'FontSize',12)

figure(3)
plot(signed_mean_relative_distance_pre)
title('Signed mean relative distance', 'LineWidth',3)
text(150,-15, strcat('MEAN:  ',num2str(mean(signed_mean_relative_distance_pre)), 'mm'),'FontSize',12)
text(150,-17, strcat('STD   : ', num2str(std(signed_mean_relative_distance_pre)), 'mm'),'FontSize',12)

%%

correction_flag = [0 0 0 0 0 0 0]'; % using which correction method to correct each encoder
LENGTH = load('msk_length_total.mat');
LENGTH = LENGTH.length_total
%LENGTH = [254.0; 64.04; 254.0; 135.7122; 0; 355.4; 0];
PHI = load('msk_phi_total.mat');
%PHI = [0 0 0 0 0 0 0]
PHI = PHI.phi_total
coeff = load('msk_coeff_total.mat');
Coeff = coeff.coeff_total

encoder1_LUT = load('lookup_table_encoder1_mean_with_extension.mat')
encoder2_LUT = load('lookup_table_encoder2_0625_mean.mat');
encoder3_LUT = load('lookup_table_encoder3_0625_mean.mat');
encoder5_LUT = load('lookup_table_encoder5_0625_mean.mat');
encoder1_LUT_ = encoder1_LUT.lookup_table_unique;
% encoder1_LUT_1 = encoder1_LUT_ + 370*360/4096;
encoder2_LUT_ = encoder2_LUT.lookup_table_unique;
encoder3_LUT_ = encoder3_LUT.lookup_table_unique;
encoder5_LUT_ = encoder5_LUT.lookup_table_unique;

[m_tip2base] = SimplifiedKinematics(LENGTH, THETA, PHI, Coeff, correction_flag, encoder1_LUT, encoder2_LUT, encoder3_LUT, encoder5_LUT);
% m_base2ref = EulerAngle2Transform(x(10:12), x(13:15));

for i = 1:N
    v_tip2base_pre(:,i) = m_tip2base(:,:,i)*[0 0 0 1]';
end

% 3.1 Display 3D points distribution (qualitatively evaluation)

% Unified to NDI coordinate system
[R, t] = computeRigidTransform(v_tip2base_pre(1:3,1:end)',v_NDI_tip2ref(1:3,1:end)');
t_alignment(1:3,1:3) = R;
t_alignment(1:3,4) = t;
t_alignment(4,4) = 1;
v_tip2base_pre_aligned = t_alignment*v_tip2base_pre;

% Unified to coordinate system of the arm base
[R, t] = computeRigidTransform(v_NDI_tip2ref(1:3,1:end)', v_tip2base_pre(1:3,1:end)');
t_alignment(1:3,1:3) = R;
t_alignment(1:3,4) = t;
t_alignment(4,4) = 1;
v_NDI_tip2ref_aligned = t_alignment*v_NDI_tip2ref;

figure (1)
scatter3(v_NDI_tip2ref_aligned(1,:), v_NDI_tip2ref_aligned(2,:), v_NDI_tip2ref_aligned(3,:),'filled');
hold on
scatter3(v_tip2base_pre(1,:), v_tip2base_pre(2,:), v_tip2base_pre(3,:),'filled');
hold off
legend('points in NDI', 'points in arm')
xlabel('X')
ylabel('Y')
zlabel('Z')
title('3D point distribution before optimization')

% 3.2 Evaluate relative distance error
abs_mean_relative_distance_pre =[];
signed_mean_relative_distance_pre = [];

for reference_index = 1:N % chose one point as the reference point for computing the relative distance
    temp1 = v_tip2base_pre_aligned(:,reference_index) - v_tip2base_pre_aligned; 
    delta_Arm = vecnorm(temp1);

    temp2 = v_NDI_tip2ref(:, reference_index) - v_NDI_tip2ref;
    delta_NDI = vecnorm(temp2);

    delta = delta_NDI- delta_Arm;
    
    abs_mean_relative_distance_pre = [abs_mean_relative_distance_pre mean(abs(delta))];
    signed_mean_relative_distance_pre = [signed_mean_relative_distance_pre mean(delta)];
end

disp_signed = ['MEAN(signed): ', num2str(mean(signed_mean_relative_distance_pre)), '   STD(signed): ', num2str(std(signed_mean_relative_distance_pre))];
disp_abs    = ['MEAN(abs)   : ', num2str(mean(abs_mean_relative_distance_pre)), '     STD(abs)   : ', num2str(std(abs_mean_relative_distance_pre))];
disp('NDI as reference')
disp(disp_signed)
disp(disp_abs)

figure(2)
plot(abs_mean_relative_distance_pre)
title('Absolute mean realtive distance','LineWidth',3)
text(150,20, strcat('MEAN:  ',num2str(mean(abs_mean_relative_distance_pre)), 'mm'),'FontSize',12)
text(150,19, strcat('STD   : ', num2str(std(abs_mean_relative_distance_pre)), 'mm'),'FontSize',12)

figure(3)
plot(signed_mean_relative_distance_pre)
title('Signed mean relative distance ', 'LineWidth',3)
text(150,-15, strcat('MEAN:  ',num2str(mean(signed_mean_relative_distance_pre)), 'mm'),'FontSize',12)
text(150,-17, strcat('STD   : ', num2str(std(signed_mean_relative_distance_pre)), 'mm'),'FontSize',12)

