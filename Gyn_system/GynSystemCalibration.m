% Calibration algorithm
% By Shuwei Xing 


clc
clear all
close all

%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Step1: Load data (encoder readings and NDI tracking data)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

encoder_data = load('E:\PROGRAM\Project_PhD\Calibration\Gynecology_system\Data\20230726_Gyne_real_Encoder.txt');
NDI_tracking_data = load('E:\PROGRAM\Project_PhD\Calibration\Gynecology_system\Data\20230726_Gyne_real_NDI.txt');
N = size(encoder_data, 1); % the number of data


% Encoder data (using the original angle instead of Jeff's
% correction)
encoder_data = [encoder_data(:,1:3), zeros(N, 3), encoder_data(:,4)];



THETA = encoder_data(:,1:7)';
THETA(2,:) = THETA(2,:);
THETA = THETA*pi/180; % Degree to radians
THETA_degree = THETA*180/pi; % Degree

% NDI tracking data
v_NDI_tip2ref_temp = NDI_tracking_data * [0 0 0 1]';
v_NDI_tip2ref = reshape(v_NDI_tip2ref_temp, [4, N]);


noisy_index = 1:190

v_NDI_tip2ref(:, noisy_index) = [];
% update encoder data
THETA(:,noisy_index) = [];
THETA_degree(:,noisy_index) = [];

% update the number of data points
N = max(size(THETA))

%%
% % filter data
% filter_index = 1:50; % 21, 5, 4, 
% v_NDI_tip2ref(:, filter_index) = [];
% % update encoder data
% THETA(:,filter_index) = [];
% THETA_degree(:,filter_index) = [];
% 
% % update the number of data points
% N = max(size(THETA))
% %%
% % filter data
% filter_index = [193, 192, 191, 190, 189, 188, 187, 186, 165, 35];
% v_NDI_tip2ref(:, filter_index) = [];
% % update encoder data
% THETA(:,filter_index) = [];
% THETA_degree(:,filter_index) = [];
% 
% % update the number of data points
% N = max(size(THETA))
%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Step2: Data pre-processing (clean noisy data and shuffle valid data)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%
% 2.2 Shuffle data
shuffle_index = randperm(N);
num_testset = 0;
THETA_degree_verify = THETA_degree(:,shuffle_index(1:num_testset));
v_NDI_tip2ref_verify = v_NDI_tip2ref(:, shuffle_index(1:num_testset));
THETA_verify = THETA(:,shuffle_index(1:num_testset));

THETA_degree = THETA_degree(:,shuffle_index(num_testset+1:end));
v_NDI_tip2ref = v_NDI_tip2ref(:, shuffle_index((num_testset+1:end)));
THETA = THETA(:,shuffle_index((num_testset+1:end)));
N = max(size(THETA))
%%
% save Gyn_training0726_THETA THETA
% save Gyn_training0726_tip2ref v_NDI_tip2ref
% save Gyn_verify0726_THETA THETA_verify
% save Gyn_verify0726_tip2ref v_NDI_tip2ref_verify
%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Step3: Evaluate the accuray before our algorithm
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


% FENSTER PROBE
% LENGTH = load('CALIBRATION_PARA_LENGTH_Kelowna_real_20220724.mat');
% LENGTH = LENGTH.length_corrected'
LENGTH = [203.2; 68.098; 215.9; 0; 20; 0; 0];
% PHI = load('CALIBRATION_PARA_PHI_RADIAN_Kelowna_real_20220724.mat');
% PHI = PHI.phi_corrected_radian'
PHI = [0 0 0 0 0 0 0]
% coeff = load('CALIBRATION_PARA_COEFF_RADIAN_Kelowna_real_20220724.mat');
% Coeff = coeff.coeff_corrected_radian';
Coeff = [0 0 0 0]

correction_flag = [0 0 0 0 0 0 0]';


% correction_flag = [2 2 0 0 2 1 0]'; % using which correction method to correct each encoder


[m_tip2base] = SimplifiedKinematics(LENGTH, THETA, PHI, Coeff, correction_flag, 0, 0, 0, 0);
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

figure
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
disp('NDI as reference (pre optimization)')
disp(disp_signed)
disp(disp_abs)

figure
plot(abs_mean_relative_distance_pre)
title('Absolute mean realtive distance (pre optimization)','LineWidth',3)
text(150,20, strcat('MEAN:  ',num2str(mean(abs_mean_relative_distance_pre)), 'mm'),'FontSize',12)
text(150,19, strcat('STD   : ', num2str(std(abs_mean_relative_distance_pre)), 'mm'),'FontSize',12)

figure
plot(signed_mean_relative_distance_pre)
title('Signed mean relative distance (pre optimization)', 'LineWidth',3)
text(150,-15, strcat('MEAN:  ',num2str(mean(signed_mean_relative_distance_pre)), 'mm'),'FontSize',12)
text(150,-17, strcat('STD   : ', num2str(std(signed_mean_relative_distance_pre)), 'mm'),'FontSize',12)


% calculate FOV

x_range = minmax(v_tip2base_pre(1,:))
y_range = minmax(v_tip2base_pre(2,:))
z_range = minmax(v_tip2base_pre(3,:))

delta_x = x_range(2) - x_range(1)
delta_y = y_range(2)-y_range(1)
delta_z = z_range(2)- z_range(1)

%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Step4: Apply our LM-based alogrithm
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% 4.1: Initialize the calibartion parameters
t_base2ref = [1200, -150, -1470]';
% t_base2ref = [-161, -3, -1900]';
% t_base2ref = [0, 0, 0]';
angle_base2ref = [0, 0, 0]'; % [z y x]

X(1:7) = [203.2; 68.098; 215.9; 0; 10; 0; 0];
X(8:14) = [0.00; 0.00; 0.00; 0.00; 0.00; 0.00; 0.00]; % PHI
X(15:18) = [0;0;0;0]; % Coefficient
X(19:21) = t_base2ref;
X(22:24) = angle_base2ref;

% X(15:20) = [0;0;0;0; 0; 0]; % Coefficient
% X(21:23) = t_base2ref;
% X(24:26) = angle_base2ref;

calibration_mode = 1;
% correction_flag = [2 2 0 0 2 1 0]';
correction_flag = [0 0 0 0 0 0 0]';
% Determinie the lower bound and upper bound


X_lb(1:7) = [180.0; 58.098; 200; -0; -10; -0; -10];
X_ub(1:7) = [230.0; 78.098; 230; 0; 20; 0; 10];
X_lb(8:14) = [-0;-0.1; -0.2; 0; 0;0; -0.2];
X_ub(8:14) = [0; 0.4; 0.1; 0; 0; 0; 0.2];
X_lb(15:18) = [-0; -0; -0; -0];
X_ub(15:18) = [0; 0; 0; 0];



% 4.2: Randomly select part of the dataset (as the ratio) to train
training_ratio =0.9;
num_training = 1;
length_total = zeros(num_training, 7);
phi_total = zeros(num_training, 7);
coeff_total = zeros(num_training, 4);
t_base2ref_total = zeros(num_training, 3);
angle_base2ref_total = zeros(num_training, 3);
residual_total = zeros(num_training, 1);
for i = 1:num_training
    
    index_training_set = randperm(N, floor(N*training_ratio));
    THETA_traing = THETA(:,index_training_set);
    v_NDI_tip2ref_training = v_NDI_tip2ref(:, index_training_set);
    % F = @(X)SimplifiedCalibrationFunction(X, THETA, gold_standard, calibration_mode, correction_flag, encoder1_LUT, encoder2_LUT);
    F = @(X)SimplifiedCalibrationFunction(X, THETA_traing(:,:), v_NDI_tip2ref_training(:,:), calibration_mode, correction_flag, 0, 0,0, 0);

    options = optimoptions('lsqnonlin','Display','iter');
    options.Algorithm = 'levenberg-marquardt';
    options.StepTolerance = 1e-20;
    options.FunctionTolerance = 1e-20;
    options.MaxFunctionEvaluations = 1000;
    [x,resnorm,residual,exitflag,output]= lsqnonlin(F, X, X_lb, X_ub, options);
    
    txyz_optimized = x(1:end);
    length_total(i,:) = x(1:7)
    phi_total(i, :) = x(8:14)
    coeff_total(i,:) = x(15:18)
    t_base2ref_total(i,:) = x(19:21)
    angle_base2ref_total(i,:) = x(22:24)
    
%     coeff_total(i,:) = x(15:20);
%     t_base2ref_total(i,:) = x(21:23);
%     angle_base2ref_total(i,:) = x(24:26);
    
    residual_total(i,1) = residual*residual';
end

%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Step5: Preliminarily evaluate the accuracy
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

LENGTH = x(1:7)
PHI = x(8:14)
coeff = x(15:18)
% coeff = x(15:20)
correction_flag = [0 0 0 0 0 0 0]';
% correction_flag = [1 1 1 1 1 1 1];% correction_flag = [0 0 0 0 0 0 0]';
% correction_flag = [2 2 1 0 2 1 0]';


[m_tip2base_corrected] = SimplifiedKinematics(LENGTH, THETA, PHI,coeff,correction_flag, 0, 0,0, 0);

for i = 1: max(size(THETA))
    
           v_tip2base_corrected(:,i) = m_tip2base_corrected(:,:,i)*[0 0 0 1]';
end

% 5.1 Display the 3D points distribution
[R, t] = computeRigidTransform(v_NDI_tip2ref(1:3,:)', v_tip2base_corrected(1:3,:)');
t_alignment(1:3,1:3) = R;
t_alignment(1:3,4) = t;
t_alignment(4,4) = 1;
v_NDI_tip2ref_corrected_aligned = t_alignment*v_NDI_tip2ref;

figure
scatter3(v_NDI_tip2ref_aligned(1,:), v_NDI_tip2ref_aligned(2,:), v_NDI_tip2ref_aligned(3,:),'filled');
hold on
scatter3(v_tip2base_pre(1,:), v_tip2base_pre(2,:), v_tip2base_pre(3,:),'filled');
hold off
legend('points in NDI', 'points in arm')
xlabel('X')
ylabel('Y')
zlabel('Z')
title('3D point distribution before calibration')

figure 
scatter3(v_NDI_tip2ref_corrected_aligned(1,:), v_NDI_tip2ref_corrected_aligned(2,:), v_NDI_tip2ref_corrected_aligned(3,:),'filled');
hold on
scatter3(v_tip2base_corrected(1,:), v_tip2base_corrected(2,:), v_tip2base_corrected(3,:),'filled');
hold off
legend('points in NDI', 'points in arm')
xlabel('X')
ylabel('Y')
zlabel('Z')
% ylim([190 330])
% yticks([200 230 260 290 320])
title('3D point distribution after calibration')

[R, t] = computeRigidTransform(v_tip2base_corrected(1:3,:)',v_NDI_tip2ref(1:3,:)');
t_alignment(1:3,1:3) = R;
t_alignment(1:3,4) = t;
t_alignment(4,4) = 1;
v_tip2base_corrected_aligned = t_alignment*v_tip2base_corrected;

% 5.2 Evlaute the Global mean distance error
delta_NDI_tip = v_NDI_tip2ref_corrected_aligned - v_tip2base_corrected;

figure
plot(delta_NDI_tip(1,:),'LineWidth', 2)
hold on
plot(delta_NDI_tip(2,:),'LineWidth', 2)
plot(delta_NDI_tip(3,:),'LineWidth', 2)
hold off
title('Distance deviations along X, Y, Z directions')
legend('X', 'Y', 'Z')
xlabel('Point number')
ylabel('Distance deviations(mm)')
ylim([-2 2])

global_abs_value = vecnorm(delta_NDI_tip);
disp('GLOBAL ABSOLUTE MEAN VALUE:')
disp(num2str(mean(vecnorm(delta_NDI_tip))))
disp('GLOBAL ABSOLUTE STD VALUE:')
disp(num2str(std(global_abs_value)))

disp('GLOBAL SIGNED MEAN VALUE (X, Y, Z)')
signed_mean_value = [num2str(mean(delta_NDI_tip(1,:)')), '  ', num2str(mean(delta_NDI_tip(2,:)')), '  ', num2str(mean(delta_NDI_tip(3,:)'))];
disp(signed_mean_value)
disp('GLOBAL SIGNED STD VALUE (X, Y, Z)')
signed_std_value = [num2str(std(delta_NDI_tip(1,:)')), '  ', num2str(std(delta_NDI_tip(2,:)')), '  ', num2str(std(delta_NDI_tip(3,:)'))];
disp(signed_std_value)

% 5.3 Evaluate relative distance error
table_abs_mean_relative_distance =[];
table_signed_mean_relative_distance = [];

for reference_index = 1:N

    temp1 = v_tip2base_corrected(:,reference_index) - v_tip2base_corrected; 
    delta_Arm_corrected = vecnorm(temp1);

    temp2 = v_NDI_tip2ref(:, reference_index) - v_NDI_tip2ref;
    delta_NDI = vecnorm(temp2);

    delta_NDI_arm = delta_NDI- delta_Arm_corrected;
%     delta_plate_arm = delta_plate - delta_Arm_corrected;
%     delta_plate_NDI =  delta_plate - delta_NDI;

    table_abs_mean_relative_distance = [table_abs_mean_relative_distance mean(abs(delta_NDI_arm))];
    table_signed_mean_relative_distance = [table_signed_mean_relative_distance mean(delta_NDI_arm)];
end

figure
plot(table_abs_mean_relative_distance,'LineWidth',3)
title('absolute relative distance')
figure
plot(table_signed_mean_relative_distance,'LineWidth',3)
title('signed relative distance')

disp_signed = ['mean(signed): ', num2str(mean(table_signed_mean_relative_distance)),      '    std(signed): ', num2str(std(table_signed_mean_relative_distance))];
disp_abs    = ['mean(abs)   : ', num2str(mean(table_abs_mean_relative_distance)), '   std(abs)   : ', num2str(std(table_abs_mean_relative_distance))];
disp('NDI as reference')
disp(disp_signed)
disp(disp_abs)

%% before saving, training around 500 times
save length_total_Gyn_real_20230726 length_total
save phi_total_Gyn_real_20230726  phi_total
save coeff_total_Gyn_real_20230726  coeff_total
save residual_total_Gyn_real_20230726  residual_total
save t_base2ref_total_Gyn_real_20230726 t_base2ref_total
save angle_base2ref_total_Gyn_real_20230726 angle_base2ref_total
%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Step6: Determine each parameters according to the data distribution
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

length_total = load('length_total_Gyn_real_20230726.mat')
length_total = length_total.length_total;
phi_total = load('phi_total_Gyn_real_20230726.mat')
phi_total = phi_total.phi_total;
coeff_total = load('coeff_total_Gyn_real_20230726.mat')
coeff_total = coeff_total.coeff_total;
residual_total = load('residual_total_Gyn_real_20230726.mat')
residual_total = residual_total.residual_total;
t_base2ref_total = load('t_base2ref_total_Gyn_real_20230726.mat')
t_base2ref_total = t_base2ref_total.t_base2ref_total;
angle_base2ref_total = load('angle_base2ref_total_Gyn_real_20230726.mat')
angle_base2ref_total = angle_base2ref_total.angle_base2ref_total;

length_corrected = zeros(7,1);
length_corrected(4) = 105.572;
phi_corrected_degree = zeros(7,1);
phi_corrected_degree(7) = 0;
phi_corrected_radian = zeros(7,1);
phi_corrected_radian(7) = 0;

coeff_corrected_degree = zeros(4,1);
coeff_corrected_radian = zeros(4,1);

t_base2ref_corrected = zeros(3,1);
angle_base2ref_corrected = zeros(3,1);
%%
% Filter the unconverged tranining results
figure 
plot(residual_total)

mean_residual = mean(residual_total);
std_residual = std(residual_total);
filter_index = find(residual_total > 270);
residual_total_filterted = residual_total;
residual_total_filterted(filter_index) = [];
%%
figure
plot(residual_total_filterted)

%%
length_total(filter_index, :) = [];
phi_total(filter_index, :) = [];
coeff_total(filter_index,:) = [];

%% LINK 1

link_index = 1
length_temp = length_total(:,link_index);
figure (1)
histogram(length_temp,100,'Normalization','pdf','EdgeAlpha',0.3,'FaceAlpha',0.8,'FaceColor','#0072BD')


% filter_index = find(length_temp > 258)
% 
% length_temp(filter_index) = [];

pd = fitdist(length_temp,'Normal')
x_values = 206:0.01:206.8;
y = pdf(pd,x_values);

figure (1)
histogram(length_temp,100,'Normalization','pdf','EdgeAlpha',0.3,'FaceAlpha',0.8,'FaceColor','#0072BD')
hold on
plot(x_values,y,'LineWidth',4)
hold off
title(strcat('Link', num2str(link_index), ' distribution'))
xlabel('Link length (mm)' )
ylabel('Probabilty density')
set(gca,'FontSize',30);
set(gcf,'Position',[200 200 900 700])
length_corrected(link_index) = pd.mu
% figure (1)
% histogram(length1(:,1),100,'Normalization','pdf','EdgeAlpha',0.3,'FaceAlpha',0.8,'FaceColor','#0072BD')


%% Link 2

link_index = 2

length_temp = length_total(:,link_index);
figure (1)
histogram(length_temp,100,'Normalization','pdf','EdgeAlpha',0.3,'FaceAlpha',0.8,'FaceColor','#0072BD')

% filter_index = find(length_temp > 258)
% 
% length_temp(filter_index) = [];

pd = fitdist(length_temp,'Normal')
x_values = 56:0.01:58.5;
y = pdf(pd,x_values);

figure (1)
histogram(length_temp,100,'Normalization','pdf','EdgeAlpha',0.3,'FaceAlpha',0.8,'FaceColor','#0072BD')
hold on
plot(x_values,y,'LineWidth',4)
hold off
title(strcat('Link', num2str(link_index), ' distribution'))
xlabel('Link length (mm)' )
ylabel('Probabilty density')
set(gca,'FontSize',30);
set(gcf,'Position',[200 200 900 700])
length_corrected(link_index) = pd.mu
% figure (1)
% histogram(length1(:,1),100,'Normalization','pdf','EdgeAlpha',0.3,'FaceAlpha',0.8,'FaceColor','#0072BD')

%% LINK 3

link_index = 3;
length_temp = length_total(:,link_index);
figure (1)
histogram(length_temp,100,'Normalization','pdf','EdgeAlpha',0.3,'FaceAlpha',0.8,'FaceColor','#0072BD')
% 
% filter_index = find(length_temp > 256 | length_temp < 253.5)
% 
% length_temp(filter_index) = [];
pd = fitdist(length_temp,'Normal')
x_values = 214.5:0.01:216.5;
y = pdf(pd,x_values);

figure (2)
histogram(length_temp,100,'Normalization','pdf','EdgeAlpha',0.3,'FaceAlpha',0.8,'FaceColor','#0072BD')
hold on
plot(x_values,y,'LineWidth',4)
hold off
title(strcat('Link', num2str(link_index), ' distribution'))
xlabel('Link length (mm)' )
ylabel('Probabilty density')
set(gca,'FontSize',30);
set(gcf,'Position',[200 200 900 700])
length_corrected(link_index) = pd.mu

%% LINK 4

link_index = 4;
% length_temp = length_total(:,link_index);
% figure (1)
% histogram(length_temp,100,'Normalization','pdf','EdgeAlpha',0.3,'FaceAlpha',0.8,'FaceColor','#0072BD')
% % filter_index = find(length_temp > 140 | length_temp < 135)
% % 
% % length_temp(filter_index) = [];
% pd = fitdist(length_temp,'Normal')
% x_values = 119:0.01:139;
% y = pdf(pd,x_values);
% 
% figure (3)
% histogram(length_temp,100,'Normalization','pdf','EdgeAlpha',0.3,'FaceAlpha',0.8,'FaceColor','#0072BD')
% hold on
% plot(x_values,y,'LineWidth',4)
% hold off
% title(strcat('Link', num2str(link_index), ' distribution'))
% xlabel('Link length (mm)' )
% ylabel('Probabilty density')
% set(gca,'FontSize',30);
% set(gcf,'Position',[200 200 900 700])
length_corrected(link_index) = 105.572

%% Tip offset X
link_index = 5;

length_temp = length_total(:,link_index);
figure (1)
histogram(length_temp,100,'Normalization','pdf','EdgeAlpha',0.3,'FaceAlpha',0.8,'FaceColor','#0072BD')
% filter_index = find(length_temp > -0.8 | length_temp < -2.3)
% 
% length_temp(filter_index) = [];
pd = fitdist(length_temp,'Normal')
x_values = 52.5:0.01:53.5;
y = pdf(pd,x_values);

figure (3)
histogram(length_temp,100,'Normalization','pdf','EdgeAlpha',0.3,'FaceAlpha',0.8,'FaceColor','#0072BD')
hold on
plot(x_values,y,'LineWidth',4)
hold off
title('Tip offset X distribution')
xlabel('Offset length (mm)' )
ylabel('Probabilty density')
set(gca,'FontSize',30);
set(gcf,'Position',[200 200 900 700])
length_corrected(link_index) = pd.mu

%% Link 5 (including tip offset Y)
link_index = 6;

length_temp = length_total(:,link_index);
figure (1)
histogram(length_temp,100,'Normalization','pdf','EdgeAlpha',0.3,'FaceAlpha',0.8,'FaceColor','#0072BD')

% filter_index = find(length_temp < 286)
% 
% length_temp(filter_index) = [];

pd = fitdist(length_temp,'Normal')
x_values = 220.5:0.01:222;
y = pdf(pd,x_values);

figure (3)
histogram(length_temp,100,'Normalization','pdf','EdgeAlpha',0.3,'FaceAlpha',0.8,'FaceColor','#0072BD')
hold on
plot(x_values,y,'LineWidth',4)
hold off
title(strcat('Link', num2str(5), ' distribution (including tip offset Y)'))
xlabel('Link length (mm)' )
ylabel('Probabilty density')
set(gca,'FontSize',30);
set(gcf,'Position',[200 200 900 700])
length_corrected(link_index) = pd.mu

%% Tip offset Z
link_index = 7;

length_temp = length_total(:,link_index);
figure (1)
histogram(length_temp,100,'Normalization','pdf','EdgeAlpha',0.3,'FaceAlpha',0.8,'FaceColor','#0072BD')
% filter_index = find(length_temp > 2 | length_temp < -0.50)
% 
% length_temp(filter_index) = [];
pd = fitdist(length_temp,'Normal')
x_values = -4:0.01:-3;
y = pdf(pd,x_values);

figure (3)
histogram(length_temp,100,'Normalization','pdf','EdgeAlpha',0.3,'FaceAlpha',0.8,'FaceColor','#0072BD')
hold on
plot(x_values,y,'LineWidth',4)
hold off
title('Tip offset Z distribution')
xlabel('Offset length (mm)' )
ylabel('Probabilty density')
set(gca,'FontSize',30);
set(gcf,'Position',[200 200 900 700])
length_corrected(link_index) = pd.mu

%%
save CALIBRATION_PARA_LENGTH_Gyn_real_20230726  length_corrected

%%
phi_index = 2;
phi_temp = phi_total(:,phi_index)*180/pi;
figure (1)
histogram(phi_temp,100,'Normalization','pdf','EdgeAlpha',0.3,'FaceAlpha',0.8,'FaceColor','#0072BD')

% filter_index = find(phi_temp > 256 | phi_temp < 253.5)
% 
% phi_temp(filter_index) = [];

pd = fitdist(phi_temp,'Normal')
x_values = 0.2:0.001:0.55;
y = pdf(pd,x_values);

figure (2)
histogram(phi_temp,100,'Normalization','pdf','EdgeAlpha',0.3,'FaceAlpha',0.8,'FaceColor','#0072BD')
hold on
plot(x_values,y,'LineWidth',4)
hold off
title(strcat('\phi_', num2str(phi_index), ' distribution'))
xlabel('Angle({\circ})' )
ylabel('Probabilty density')
set(gca,'FontSize',30);
set(gcf,'Position',[200 200 900 700])
phi_corrected_degree(phi_index) = pd.mu
phi_corrected_radian(phi_index) = pd.mu*pi/180
%%
phi_index = 3;
phi_temp = phi_total(:,phi_index)*180/pi;
figure (1)
histogram(phi_temp,100,'Normalization','pdf','EdgeAlpha',0.3,'FaceAlpha',0.8,'FaceColor','#0072BD')

% filter_index = find( phi_temp < -1)
% 
% phi_temp(filter_index) = [];

pd = fitdist(phi_temp,'Normal')
x_values = -3.5:0.01:-1.5;
y = pdf(pd,x_values);

figure (2)
histogram(phi_temp,100,'Normalization','pdf','EdgeAlpha',0.3,'FaceAlpha',0.8,'FaceColor','#0072BD')
hold on
plot(x_values,y,'LineWidth',4)
hold off
title(strcat('\phi_', num2str(phi_index), ' distribution'))
xlabel('Angle({\circ})' )
ylabel('Probabilty density')
set(gca,'FontSize',30);
set(gcf,'Position',[200 200 900 700])
phi_corrected_degree(phi_index) = pd.mu
phi_corrected_radian(phi_index) = pd.mu*pi/180

%%
phi_index = 4;
phi_temp = phi_total(:,phi_index)*180/pi;
figure (1)
histogram(phi_temp,100,'Normalization','pdf','EdgeAlpha',0.3,'FaceAlpha',0.8,'FaceColor','#0072BD')

% filter_index = find( phi_temp > -0.8)
% 
% phi_temp(filter_index) = [];

pd = fitdist(phi_temp,'Normal')
x_values = -0.2:0.01:0.5;
y = pdf(pd,x_values);

figure (2)
histogram(phi_temp,100,'Normalization','pdf','EdgeAlpha',0.3,'FaceAlpha',0.8,'FaceColor','#0072BD')
hold on
plot(x_values,y,'LineWidth',4)
hold off
title(strcat('\phi_', num2str(phi_index), ' distribution'))
xlabel('Angle({\circ})' )
ylabel('Probabilty density')
set(gca,'FontSize',30);
set(gcf,'Position',[200 200 900 700])
phi_corrected_degree(phi_index) = pd.mu
phi_corrected_radian(phi_index) = pd.mu*pi/180
%%
phi_index = 5;
phi_temp = phi_total(:,phi_index)*180/pi;
figure (1)
histogram(phi_temp,100,'Normalization','pdf','EdgeAlpha',0.3,'FaceAlpha',0.8,'FaceColor','#0072BD')

% filter_index = find( phi_temp > -0.8)
% 
% phi_temp(filter_index) = [];

pd = fitdist(phi_temp,'Normal')
x_values = -2:0.01:-1.4;
y = pdf(pd,x_values);

figure (2)
histogram(phi_temp,100,'Normalization','pdf','EdgeAlpha',0.3,'FaceAlpha',0.8,'FaceColor','#0072BD')
hold on
plot(x_values,y,'LineWidth',4)
hold off
title(strcat('\phi_', num2str(phi_index), ' distribution'))
xlabel('Angle({\circ})' )
ylabel('Probabilty density')
set(gca,'FontSize',30);
set(gcf,'Position',[200 200 900 700])
phi_corrected_degree(phi_index) = pd.mu
phi_corrected_radian(phi_index) = pd.mu*pi/180
%%

phi_index = 6;
phi_temp = phi_total(:,phi_index)*180/pi;
figure (1)
histogram(phi_temp,100,'Normalization','pdf','EdgeAlpha',0.3,'FaceAlpha',0.8,'FaceColor','#0072BD')

% filter_index = find( phi_temp < 1)
% 
% phi_temp(filter_index) = [];

pd = fitdist(phi_temp,'Normal')
x_values = -2.5:0.01:-1;
y = pdf(pd,x_values);

figure (2)
histogram(phi_temp,100,'Normalization','pdf','EdgeAlpha',0.3,'FaceAlpha',0.8,'FaceColor','#0072BD')
hold on
plot(x_values,y,'LineWidth',4)
hold off
title(strcat('\phi_', num2str(phi_index), ' distribution'))
xlabel('Angle({\circ})' )
ylabel('Probabilty density')
set(gca,'FontSize',30);
set(gcf,'Position',[200 200 900 700])
phi_corrected_degree(phi_index) = pd.mu
phi_corrected_radian(phi_index) = pd.mu*pi/180
%%

phi_index = 7;
phi_temp = phi_total(:,phi_index)*180/pi;
figure (1)
histogram(phi_temp,100,'Normalization','pdf','EdgeAlpha',0.3,'FaceAlpha',0.8,'FaceColor','#0072BD')

% filter_index = find( phi_temp < 1)
% 
% phi_temp(filter_index) = [];

pd = fitdist(phi_temp,'Normal')
x_values = 9:0.01:14;
y = pdf(pd,x_values);

figure (2)
histogram(phi_temp,100,'Normalization','pdf','EdgeAlpha',0.3,'FaceAlpha',0.8,'FaceColor','#0072BD')
hold on
plot(x_values,y,'LineWidth',4)
hold off
title(strcat('\phi_', num2str(phi_index), ' distribution'))
xlabel('Angle({\circ})' )
ylabel('Probabilty density')
set(gca,'FontSize',30);
set(gcf,'Position',[200 200 900 700])
phi_corrected_degree(phi_index) = pd.mu
phi_corrected_radian(phi_index) = pd.mu*pi/180


%%
save CALIBRATION_PARA_PHI_DEGREE_Gyn_real_20230726  phi_corrected_degree
save CALIBRATION_PARA_PHI_RADIAN_Gyn_real_20230726  phi_corrected_radian

%%

coeff_index = 1;
coeff_temp = coeff_total(:,coeff_index)*180/pi;
figure (1)
histogram(coeff_temp,100,'Normalization','pdf','EdgeAlpha',0.3,'FaceAlpha',0.8,'FaceColor','#0072BD')

% filter_index = find( coeff_temp > -0.26)
% 
% coeff_temp(filter_index) = [];

pd = fitdist(coeff_temp,'Normal')
x_values = -0.45:0.0001:-0.38;
y = pdf(pd,x_values);


figure (2)
histogram(coeff_temp,100,'Normalization','pdf','EdgeAlpha',0.3,'FaceAlpha',0.8,'FaceColor','#0072BD')
hold on
plot(x_values,y,'LineWidth',4)
hold off
title(strcat('\beta_', num2str(coeff_index), ' distribution'))
xlabel('Angle({\circ})' )
ylabel('Probabilty density')
set(gca,'FontSize',30);
set(gcf,'Position',[200 200 900 700])
coeff_corrected_degree(coeff_index) = pd.mu
coeff_corrected_radian(coeff_index) = pd.mu*pi/180

%%

coeff_index = 2;
coeff_temp = coeff_total(:,coeff_index)*180/pi;
figure (1)
histogram(coeff_temp,100,'Normalization','pdf','EdgeAlpha',0.3,'FaceAlpha',0.8,'FaceColor','#0072BD')

% filter_index = find( coeff_temp > 0.6)
% 
% coeff_temp(filter_index) = [];

pd = fitdist(coeff_temp,'Normal')
x_values = 1.1:0.0001:1.25;
y = pdf(pd,x_values);

figure (2)
histogram(coeff_temp,100,'Normalization','pdf','EdgeAlpha',0.3,'FaceAlpha',0.8,'FaceColor','#0072BD')
hold on
plot(x_values,y,'LineWidth',4)
hold off
title(strcat('\beta_', num2str(3), ' distribution'))
xlabel('Angle({\circ})' )
ylabel('Probabilty density')
set(gca,'FontSize',30);
set(gcf,'Position',[200 200 900 700])
coeff_corrected_degree(coeff_index) = pd.mu
coeff_corrected_radian(coeff_index) = pd.mu*pi/180

%%
coeff_index = 3;
coeff_temp = coeff_total(:,coeff_index)*180/pi;
figure (1)
histogram(coeff_temp,100,'Normalization','pdf','EdgeAlpha',0.3,'FaceAlpha',0.8,'FaceColor','#0072BD')

% filter_index = find( coeff_temp > 0.6)
% 
% coeff_temp(filter_index) = [];

pd = fitdist(coeff_temp,'Normal')
x_values = -0.45:0.0001:0.1;
y = pdf(pd,x_values);

figure (2)
histogram(coeff_temp,100,'Normalization','pdf','EdgeAlpha',0.3,'FaceAlpha',0.8,'FaceColor','#0072BD')
hold on
plot(x_values,y,'LineWidth',4)
hold off
title(strcat('\beta_', num2str(2), ' distribution'))
xlabel('Angle({\circ})' )
ylabel('Probabilty density')
set(gca,'FontSize',30);
set(gcf,'Position',[200 200 900 700])
coeff_corrected_degree(coeff_index) = pd.mu
coeff_corrected_radian(coeff_index) = pd.mu*pi/180
%%

coeff_index = 4;
coeff_temp = coeff_total(:,coeff_index)*180/pi;
figure (1)
histogram(coeff_temp,100,'Normalization','pdf','EdgeAlpha',0.3,'FaceAlpha',0.8,'FaceColor','#0072BD')

pd = fitdist(coeff_temp,'Normal')
x_values = 1:0.001:4;
y = pdf(pd,x_values);

figure (2)
histogram(coeff_temp,100,'Normalization','pdf','EdgeAlpha',0.3,'FaceAlpha',0.8,'FaceColor','#0072BD')
hold on
plot(x_values,y,'LineWidth',4)
hold off
title(strcat('\beta_', num2str(coeff_index), ' distribution'))
xlabel('Angle deviation ({\circ})' )
ylabel('Probabilty density')
set(gca,'FontSize',30);
set(gcf,'Position',[200 200 900 700])
coeff_corrected_degree(coeff_index) = pd.mu
coeff_corrected_radian(coeff_index) = pd.mu*pi/180

%%
save CALIBRATION_PARA_COEFF_DEGREE_Gyn_real_20230726 coeff_corrected_degree
save CALIBRATION_PARA_COEFF_RADIAN_Gyn_real_20230726 coeff_corrected_radian

%%
t_base2ref_index = 2;
t_base2ref_temp = t_base2ref_total(:,t_base2ref_index);
figure (1)
histogram(t_base2ref_temp,100,'Normalization','pdf','EdgeAlpha',0.3,'FaceAlpha',0.8,'FaceColor','#0072BD')
%%

angle_base2ref_index = 2;
angle_base2ref_temp = angle_base2ref_total(:,angle_base2ref_index);
figure (1)
histogram(angle_base2ref_temp,100,'Normalization','pdf','EdgeAlpha',0.3,'FaceAlpha',0.8,'FaceColor','#0072BD')

%%
figure
plot(residual_total_filterted)
%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Step7: Evaluate the final results using our optimized calibration parameters
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

LENGTH = load('CALIBRATION_PARA_LENGTH_Gyn_real_20230726.mat');
LENGTH = LENGTH.length_corrected'
%LENGTH = [254.0; 64.04; 254.0; 135.7122; 0; 355.4; 0];
PHI = load('CALIBRATION_PARA_PHI_RADIAN_Gyn_real_20220726.mat');
PHI = PHI.phi_corrected_radian'
%PHI = [0 0 0 0 0 0 0]
coeff = load('CALIBRATION_PARA_COEFF_RADIAN_Gyn_real_20230726.mat');
coeff = coeff.coeff_corrected_radian';
%coeff = [0 0 0 0]

correction_flag = [0 0 0 0 0 0 0]';

%THETA = [-19.2480 -4.5703 1.8457 -45.7910 2.1094 -12.3047 -59.0625]'*pi/180;
theta_degree = table2array(Encoderreading)
THETA = theta_degree*pi/180;
THETA = [THETA(:,1:3) zeros(70, 1), THETA(:,4:6)]';
[m_tip2base_corrected] = SimplifiedKinematics(LENGTH, THETA, PHI,coeff,correction_flag, 0, 0,0, 0);

for i = 1: max(size(THETA))
    
      v_tip2base_corrected(:,i) = m_tip2base_corrected(:,:,i)*[0 0 0 1]';
end


figure (5)
for i = 1:14
scatter3(v_tip2base_corrected(1,i), v_tip2base_corrected(2,i), v_tip2base_corrected(3,i),'filled');
hold on
text(v_tip2base_corrected(1,i), v_tip2base_corrected(2,i), v_tip2base_corrected(3,i), num2str(i));
end

for i = 15:28
scatter3(v_tip2base_corrected(1,i), v_tip2base_corrected(2,i), v_tip2base_corrected(3,i),'filled');
hold on
text(v_tip2base_corrected(1,i), v_tip2base_corrected(2,i), v_tip2base_corrected(3,i), num2str(i));
end

for i = 14*2+1:14*3
scatter3(v_tip2base_corrected(1,i), v_tip2base_corrected(2,i), v_tip2base_corrected(3,i),'filled');
hold on
text(v_tip2base_corrected(1,i), v_tip2base_corrected(2,i), v_tip2base_corrected(3,i), num2str(i));
end

for i = 14*3+1:14*4
scatter3(v_tip2base_corrected(1,i), v_tip2base_corrected(2,i), v_tip2base_corrected(3,i),'filled');
hold on
text(v_tip2base_corrected(1,i), v_tip2base_corrected(2,i), v_tip2base_corrected(3,i), num2str(i));
end


for i = 14*4+1:14*5
scatter3(v_tip2base_corrected(1,i), v_tip2base_corrected(2,i), v_tip2base_corrected(3,i),'filled');
hold on
text(v_tip2base_corrected(1,i), v_tip2base_corrected(2,i), v_tip2base_corrected(3,i), num2str(i));
end
% legend('points in NDI', 'points in arm')
xlabel('X')
ylabel('Y')
zlabel('Z')
title('3D point distribution before optimization')
%%

% 7.1 Display the 3D points distribution
[R, t] = computeRigidTransform(v_NDI_tip2ref(1:3,:)', v_tip2base_corrected(1:3,:)');
t_alignment(1:3,1:3) = R;
t_alignment(1:3,4) = t;
t_alignment(4,4) = 1;
v_NDI_tip2ref_corrected_aligned = t_alignment*v_NDI_tip2ref;

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

figure (2)
scatter3(v_NDI_tip2ref_corrected_aligned(1,:), v_NDI_tip2ref_corrected_aligned(2,:), v_NDI_tip2ref_corrected_aligned(3,:),'filled');
hold on
scatter3(v_tip2base_corrected(1,:), v_tip2base_corrected(2,:), v_tip2base_corrected(3,:),'filled');
hold off
legend('points in NDI', 'points in arm')
xlabel('X')
ylabel('Y')
zlabel('Z')
% ylim([190 330])
% yticks([200 230 260 290 320])
title('3D point distribution after optimization')

[R, t] = computeRigidTransform(v_tip2base_corrected(1:3,:)',v_NDI_tip2ref(1:3,:)');
t_alignment(1:3,1:3) = R;
t_alignment(1:3,4) = t;
t_alignment(4,4) = 1;
v_tip2base_corrected_aligned = t_alignment*v_tip2base_corrected;

% 7.2 Evlaute the Global mean distance error
delta_NDI_tip = v_NDI_tip2ref_corrected_aligned - v_tip2base_corrected;

figure (3)
plot(delta_NDI_tip(1,:),'LineWidth', 2)
hold on
plot(delta_NDI_tip(2,:),'LineWidth', 2)
plot(delta_NDI_tip(3,:),'LineWidth', 2)
hold off
title('Distance deviation along X, Y, Z direction')
legend('X', 'Y', 'Z')
xlabel('Point number')
ylabel('Distance deviation(mm)')
ylim([-3 3])

global_abs_value = vecnorm(delta_NDI_tip);
disp('GLOBAL ABSOLUTE MEAN VALUE:')
disp(num2str(mean(vecnorm(delta_NDI_tip))))
disp('GLOBAL ABSOLUTE STD VALUE:')
disp(num2str(std(global_abs_value)))

disp('GLOBAL SIGNED MEAN VALUE (X, Y, Z)')
signed_mean_value = [num2str(mean(delta_NDI_tip(1,:)')), '  ', num2str(mean(delta_NDI_tip(2,:)')), '  ', num2str(mean(delta_NDI_tip(3,:)'))];
disp(signed_mean_value)
disp('GLOBAL SIGNED STD VALUE (X, Y, Z)')
signed_std_value = [num2str(std(delta_NDI_tip(1,:)')), '  ', num2str(std(delta_NDI_tip(2,:)')), '  ', num2str(std(delta_NDI_tip(3,:)'))];
disp(signed_std_value)

% 7.3 Evaluate relative distance error
table_abs_mean_relative_distance =[];
table_signed_mean_relative_distance = [];

for reference_index = 1:N

    temp1 = v_tip2base_corrected(:,reference_index) - v_tip2base_corrected; 
    delta_Arm_corrected = vecnorm(temp1);

    temp2 = v_NDI_tip2ref(:, reference_index) - v_NDI_tip2ref;
    delta_NDI = vecnorm(temp2);

    delta_NDI_arm = delta_NDI- delta_Arm_corrected;
%     delta_plate_arm = delta_plate - delta_Arm_corrected;
%     delta_plate_NDI =  delta_plate - delta_NDI;

    table_abs_mean_relative_distance = [table_abs_mean_relative_distance mean(abs(delta_NDI_arm))];
    table_signed_mean_relative_distance = [table_signed_mean_relative_distance mean(delta_NDI_arm)];
end

figure (4)
plot(table_abs_mean_relative_distance,'LineWidth',3)
title('absolute relative distance')
figure (5)
plot(table_signed_mean_relative_distance,'LineWidth',3)
title('signed relative distance')

disp_signed = ['mean(signed): ', num2str(mean(table_signed_mean_relative_distance)),      '    std(signed): ', num2str(std(table_signed_mean_relative_distance))];
disp_abs    = ['mean(abs)   : ', num2str(mean(table_abs_mean_relative_distance)), '   std(abs)   : ', num2str(std(table_abs_mean_relative_distance))];
disp('NDI as reference')
disp(disp_signed)
disp(disp_abs)

%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Step7: Verify the accuray if necessary 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
encoder_data_verify = load('E:\PROGRAM\Project_PhD\Calibration\MyCode\Data\20210625\Verification\20210626_fulljoints_verification_arm.txt');

% using the original angle instead of Jeff's correction
THETA_verify = encoder_data_verify(:,1:7)';
THETA_verify(5,:) = encoder_data_verify(:,8)';
THETA_verify = THETA_verify*pi/180; % Degree to radians
THETA_degree_verify = THETA_verify*180/pi;
N = size(encoder_data_verify, 1);
NDI_tracking_data_verify = load('E:\PROGRAM\Project_PhD\Calibration\MyCode\Data\20210625\Verification\20210626_fulljoints_verification_NDI.txt');

v_NDI_tip2ref_temp = NDI_tracking_data_verify * [0 0 0 1]';
v_NDI_tip2ref_verify = reshape(v_NDI_tip2ref_temp, [4, N]);
%%
% KELOWNA probe
% LENGTH = load('CALIBRATION_PARA_LENGTH_Kelowna_real_20220724.mat');
% LENGTH = LENGTH.length_corrected'
% %LENGTH = [254.0; 64.04; 254.0; 135.7122; 0; 355.4; 0];
% % PHI = load('CALIBRATION_PARA_PHI_RADIAN_Kelowna_real_20220724.mat');
% % PHI = PHI.phi_corrected_radian'
% % % %PHI = [0 0 0 0 0 0 0]
% % % coeff = load('CALIBRATION_PARA_COEFF_RADIAN_Kelowna_real_20220724.mat');
% % % coeff = coeff.coeff_corrected_radian'
% % 
% % % FENSTER PROBE
% % LENGTH = load('CALIBRATION_PARA_LENGTH_Kelowna_20220713.mat');
% % LENGTH = LENGTH.length_corrected'
% % %LENGTH = [254.0; 64.04; 254.0; 135.7122; 0; 355.4; 0];
% % PHI = load('CALIBRATION_PARA_PHI_RADIAN_Kelowna_20220713.mat');
% % PHI = PHI.phi_corrected_radian'
% % %PHI = [0 0 0 0 0 0 0]
% % coeff = load('CALIBRATION_PARA_COEFF_RADIAN_Kelowna_20220713.mat');
% % coeff = coeff.coeff_corrected_radian'
% % %coeff = [0 0 0 0]
% % encoder1_LUT = load('lookup_table_encoder1_mean_with_extension.mat');
% % % encoder2_LUT = load('lookup_table_encoder2_p.mat');
% % encoder2_LUT = load('lookup_table_encoder2_0625_mean.mat');
% % encoder3_LUT = load('lookup_table_encoder3_0625_mean.mat');
% % encoder5_LUT = load('lookup_table_encoder5_0625_mean.mat');
% % correction_flag = [0 0 0 0 0 0 0]';

LENGTH = load('CALIBRATION_PARA_LENGTH_Gyn_real_20230726.mat');
LENGTH = LENGTH.length_corrected'
%LENGTH = [254.0; 64.04; 254.0; 135.7122; 0; 355.4; 0];
PHI = load('CALIBRATION_PARA_PHI_RADIAN_Gyn_real_20230726.mat');
PHI = PHI.phi_corrected_radian'
%PHI = [0 0 0 0 0 0 0]
coeff = load('CALIBRATION_PARA_COEFF_RADIAN_Gyn_real_20230726.mat');
coeff = coeff.coeff_corrected_radian';
%coeff = [0 0 0 0]

correction_flag = [0 0 0 0 0 0 0]';

% THETA_verify = [13.79; -9.05; -12.48; 0; -2.83; 2.55; 7.29]*pi/180;
% THETA_verify = [37.7930; -23.3789; -0.6152; 0; 1.3092; -25.6641; -13.7988]*pi/180;
[m_tip2base_corrected] = SimplifiedKinematics(LENGTH, THETA_verify, PHI,coeff,correction_flag, 0, 0,0, 0)

% v_tip2base_corrected_verify(:,1) = m_tip2base_corrected(:,:)*[0 0 0 1]'
%%
for i = 1: max(size(THETA_verify))
    
      v_tip2base_corrected_verify(:,i) = m_tip2base_corrected(:,:,i)*[0 0 0 1]';
end
%%
% Evaulation metric 1: visualize the 3D points distribution
[R, t] = computeRigidTransform(v_NDI_tip2ref_verify(1:3,:)', v_tip2base_corrected_verify(1:3,:)');
t_alignment(1:3,1:3) = R;
t_alignment(1:3,4) = t;
t_alignment(4,4) = 1;
v_NDI_tip2ref_corrected_aligned_verify = t_alignment*v_NDI_tip2ref_verify;

figure (2)
scatter3(v_NDI_tip2ref_corrected_aligned_verify(1,:), v_NDI_tip2ref_corrected_aligned_verify(2,:), v_NDI_tip2ref_corrected_aligned_verify(3,:),'filled');
hold on
scatter3(v_tip2base_corrected_verify(1,:), v_tip2base_corrected_verify(2,:), v_tip2base_corrected_verify(3,:),'filled');
hold off
legend('Tip in NDI', 'Arm tip')
xlabel('X')
ylabel('Y')
zlabel('Z')

title('3D point distribution after optimization')


delta_NDI_tip = v_NDI_tip2ref_corrected_aligned_verify - v_tip2base_corrected_verify;

figure (3)
plot(delta_NDI_tip(1,:),'LineWidth', 2)
hold on
plot(delta_NDI_tip(2,:),'LineWidth', 2)
plot(delta_NDI_tip(3,:),'LineWidth', 2)
hold off
title('Distance deviation along X, Y, Z direction')
legend('X', 'Y', 'Z')
xlabel('Point number')
ylabel('Distance deviation(mm)')


global_abs_value = vecnorm(delta_NDI_tip);
disp('GLOBAL ABSOLUTE MEAN VALUE:')
disp(num2str(mean(vecnorm(delta_NDI_tip))))
disp('GLOBAL ABSOLUTE STD VALUE:')
disp(num2str(std(global_abs_value)))


disp('GLOBAL SIGNED MEAN VALUE (X, Y, Z)')
signed_mean_value = [num2str(mean(delta_NDI_tip(1,:)')), '  ', num2str(mean(delta_NDI_tip(2,:)')), '  ', num2str(mean(delta_NDI_tip(3,:)'))];
disp(signed_mean_value)
disp('GLOBAL SIGNED STD VALUE (X, Y, Z)')
signed_std_value = [num2str(std(delta_NDI_tip(1,:)')), '  ', num2str(std(delta_NDI_tip(2,:)')), '  ', num2str(std(delta_NDI_tip(3,:)'))];
disp(signed_std_value)

%%
table_abs_mean_relative_distance =[];
table_signed_mean_relative_distance = [];

for reference_index = 1:40

 temp1 = v_tip2base_corrected_verify(:,reference_index) - v_tip2base_corrected_verify; 
 delta_Arm_corrected = vecnorm(temp1);

 temp2 = v_NDI_tip2ref_verify(:, reference_index) - v_NDI_tip2ref_verify;
 delta_NDI = vecnorm(temp2);

 delta_NDI_arm = delta_NDI- delta_Arm_corrected;
%     delta_plate_arm = delta_plate - delta_Arm_corrected;
%     delta_plate_NDI =  delta_plate - delta_NDI;

 table_abs_mean_relative_distance = [table_abs_mean_relative_distance mean(abs(delta_NDI_arm))];
 table_signed_mean_relative_distance = [table_signed_mean_relative_distance mean(delta_NDI_arm)];

end

figure (3)
plot(table_abs_mean_relative_distance,'LineWidth',3)
title('absolute relative distance')

figure (4)
plot(table_signed_mean_relative_distance,'LineWidth',3)
title('signed relative distance')

disp_signed = ['mean(signed): ', num2str(mean(table_signed_mean_relative_distance)),      '    std(signed): ', num2str(std(table_signed_mean_relative_distance))];
disp_abs    = ['mean(abs)   : ', num2str(mean(table_abs_mean_relative_distance)), '   std(abs)   : ', num2str(std(table_abs_mean_relative_distance))];
disp('NDI as reference')
disp(disp_signed)
disp(disp_abs)







