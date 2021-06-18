% Calibration algorithm


clc
clear all
close all
% Step1: Load data

% encoder_data = load('E:\PROGRAM\Project_PhD\Calibration\MyCode\Data\20210608\20210608_fulljoints_arm_data.txt');
encoder_data = load('E:\PROGRAM\Project_PhD\Calibration\MyCode\Data\20210613\20210613_fulljoints_encoder_data.txt');
% using the original angle instead of Jeff's correction
THETA = encoder_data(:,1:7)';
THETA(5,:) = encoder_data(:,8)';

THETA = THETA*pi/180; % Degree to radians
THETA_degree = THETA*180/pi;

N = size(encoder_data, 1);

% NDI_tracking_data = load('E:\PROGRAM\Project_PhD\Calibration\MyCode\Data\20210608\20210608_fulljoints_NDI_data.txt');
NDI_tracking_data = load('E:\PROGRAM\Project_PhD\Calibration\MyCode\Data\20210613\20210613_fulljoints_NDI_data.txt');

v_NDI_tip2ref_temp = NDI_tracking_data * [0 0 0 1]';
v_NDI_tip2ref = reshape(v_NDI_tip2ref_temp, [4, N]);

% protractor_data = load('E:\PROGRAM\Project_PhD\Calibration\MyCode\Data\20210528\verification\20210528_fulljoints_plate_data.txt');
% gold_standard_layer1 = [protractor_data(1:150, :)*20 zeros(150,1)];
% gold_standard_layer2 = [protractor_data(151:300, :)*20 zeros(150,1)-32.5262];
% gold_standard_layer3 = [protractor_data(301:314,:)*20 zeros(14,1)-47];
% gold_standard = [gold_standard_layer1; gold_standard_layer2; gold_standard_layer3]';
% gold_standard = [gold_standard; ones(1,314)];


%% Step2: filter data if necessary

% find the filter index
% filter_index = 231:250;
filter_index = 1:133;
% update NDI tracking data
v_NDI_tip2ref(:, filter_index) = [];
% update encoder data
THETA(:,filter_index) = [];
THETA_degree(:,filter_index) = [];
% update the number of data points
N = max(size(THETA));
%%

figure
plot(THETA_degree(3,:))
mean(THETA_degree(3,:))

filter_index = find(THETA_degree(6,:)<-10)

v_NDI_tip2ref(:, filter_index) = [];
% update encoder data
THETA(:,filter_index) = [];
THETA_degree(:,filter_index) = [];
% update the number of data points
N = max(size(THETA));
%% Step3: evaluate the accuray before LM algorithm. 

LENGTH = [254.0; 64.04; 254.0; 135.7122; 0; 355.4; 0]; % x y z
PHI = [0; 0; 0; 0; 0; 0; 0];
Coeff = [0 0 0 0];
correction_flag = [2 2 1 0 2 1 0]';

% encoder1_LUT = load('lookup_table_encoder1_p.mat');
encoder1_LUT = load('lookup_table_encoder1_new_mean.mat')
% encoder2_LUT = load('lookup_table_encoder2_p.mat');
encoder2_LUT = load('lookup_table_encoder2_new_mean.mat');
encoder5_LUT = load('lookup_table_encoder5_new2_mean.mat');
encoder1_LUT_ = encoder1_LUT.lookup_table_unique;
encoder2_LUT_ = encoder2_LUT.lookup_table_unique;
encoder5_LUT_ = encoder5_LUT.lookup_table_unique;

[m_tip2base] = SimplifiedKinematics(LENGTH, THETA, PHI, Coeff, correction_flag,encoder1_LUT, encoder2_LUT, encoder5_LUT);
% m_base2ref = EulerAngle2Transform(x(10:12), x(13:15));
for i = 1:N
    v_tip2base_pre(:,i) = m_tip2base(:,:,i)*[0 0 0 1]';
%     v_tip2base_pre(:,i) = m_base2ref*m_tip2base(:,:,i)*[0 0 0 1]';

end

% Evaluation metric 1: visualize 3D points distribution

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


% figure
% scatter3(v_NDI_tip2ref(1,:), v_NDI_tip2ref(2,:), v_NDI_tip2ref(3,:),'filled');
% hold on
% scatter3(v_tip2base_pre_aligned(1,:), v_tip2base_pre_aligned(2,:), v_tip2base_pre_aligned(3,:),'filled');
% hold off
% legend('points in NDI', 'points in arm')
% xlabel('X')
% ylabel('Y')
% zlabel('Z')
% title('3D point distribution before optimization')

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





% Evaulation metric 2: compute relative distance error
abs_mean_relative_distance_pre =[];
signed_mean_relative_distance_pre = [];

for reference_index = 1:N
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


figure(2)
plot(abs_mean_relative_distance_pre)
title('Absolute mean realtive distance (pre optimization)','LineWidth',3)

text(150,20, strcat('MEAN:  ',num2str(mean(abs_mean_relative_distance_pre)), 'mm'),'FontSize',12)
text(150,19, strcat('STD   : ', num2str(std(abs_mean_relative_distance_pre)), 'mm'),'FontSize',12)


figure(3)
plot(signed_mean_relative_distance_pre)
title('Signed mean relative distance (pre optimization)', 'LineWidth',3)

text(150,-15, strcat('MEAN:  ',num2str(mean(signed_mean_relative_distance_pre)), 'mm'),'FontSize',12)
text(150,-17, strcat('STD   : ', num2str(std(signed_mean_relative_distance_pre)), 'mm'),'FontSize',12)



%%

filter_index = find(abs_mean_relative_distance_pre>11)

v_NDI_tip2ref(:, filter_index) = [];
% update encoder data
THETA(:,filter_index) = [];
THETA_degree(:,filter_index) = [];
% update the number of data points
N = max(size(THETA));

%% Step3: optimize using LM algorithm
%
% Step3.1: Initialize the calibartion parameters
%
t_base2ref = [0, 0, 0]';
angle_base2ref = [0, 0, 0]'; % Z y x

X(1:7) = [254.0; 64.04; 254.0; 135.7122; 0; 280; 0]; %length
X(8:14) = [0.00; 0.00; 0.00; 0.00; 0.00; 0.00; 0.00]; % PHI
X(15:18) = [0;0;0;0]; 
X(19:21) = t_base2ref;
X(22:24) = angle_base2ref;

calibration_mode = 1;

% X_lb = [244.0; 64.04; 244.0; 125.7122; -20; 200; -50; 0;-0.1; -0.2; -0.1; -0.1;-0.2; 0; -0; -0; -0; -0]; 
% X_ub = [264.0; 64.04; 264.0; 165.7122; 20; 370; 20; 0; 0.1; 0.1; 0.1; 0.1; 0.2; 0; 0; 0; 0; 0];

X_lb = [244.0; 64.04; 244.0; 125.7122; -20; 200; -50; 0;-0.1; -0.1; -0.1; -0.1;-0.1; 0; -0.1; -0.1; -0.1; -0]; 
X_ub = [264.0; 64.04; 264.0; 165.7122; 20; 370; 20; 0; 0.1; 0.1; 0.1; 0.1; 0.2; 0; 0.1; 0.1; 0.1; 0];
% X_lb  = [254.0; 64.04; 254.0; 135.7122; -20; 280; -20; -0.1]; %length
% X_ub = [254.0; 64.04; 254.0; 135.7122; 20; 350; 20; 0.1]; %length

% encoder1_LUT = load('lookup_table_encoder1_p.mat');
encoder1_LUT = load('lookup_table_encoder1_new_mean.mat');
% encoder2_LUT = load('lookup_table_encoder2_p.mat');
encoder2_LUT = load('lookup_table_encoder2_new_mean.mat');
encoder5_LUT = load('lookup_table_encoder5_new2_mean.mat');
correction_flag = [2 2 1 0 2 1 0]';

%
% Step 3.2: Randomly select the dataset to training with a ratio and
% train the dataset
%
training_ratio = 0.9;
num_training = 500;
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
    
%     THETA_traing = THETA;
%     v_NDI_tip2ref_training = v_NDI_tip2ref;
    
    %
    % F = @(X)SimplifiedCalibrationFunction(X, THETA, gold_standard, calibration_mode, correction_flag, encoder1_LUT, encoder2_LUT);
    F = @(X)SimplifiedCalibrationFunction(X, THETA_traing(:,:), v_NDI_tip2ref_training(:,:), calibration_mode, correction_flag, encoder1_LUT, encoder2_LUT, encoder5_LUT);

    options = optimoptions('lsqnonlin','Display','iter');
    options.Algorithm = 'levenberg-marquardt';
    options.StepTolerance = 1e-20;
    options.FunctionTolerance = 1e-20;
    options.MaxFunctionEvaluations = 1000;
    % [x,resnorm,residual,exitflag,output]= lsqnonlin(F, X, [], [], options);
    [x,resnorm,residual,exitflag,output]= lsqnonlin(F, X, X_lb, X_ub, options);
    txyz_optimized = x(1:end)

    length_total(i,:) = x(1:7);
    phi_total(i, :) = x(8:14);
    coeff_total(i,:) = x(15:18);
    t_base2ref_total(i,:) = x(19:21);
    angle_base2ref_total(i,:) = x(22:24);
    residual_total(i,1) = residual*residual';

end
%%
save 1st_length_total length_total
save 1st_phi_total = phi_total
save 1st_coeff_total = coeff_total
save 1st_residual_total = residual_total

%% Distribution of parameters
% Filter the unconverged results
figure 
plot(residual_total)

mean_residual = mean(residual_total);
std_residual = std(residual_total);

filter_index = find(residual_total > mean_residual+std_residual*0.9);

residual_total_filterted = residual_total;
residual_total_filterted(filter_index) = [];
figure
plot(residual_total_filterted)

length_total(filter_index, :) = [];
phi_total(filter_index, :) = [];
coeff_total(filter_index,:) = [];


%%

length_corrected = zeros(7,1);
length_corrected(2) = 64.04;

%% LINK 1

link_index = 1

length_temp = length_total(:,link_index);
figure (1)
histogram(length_temp,100,'Normalization','pdf','EdgeAlpha',0.3,'FaceAlpha',0.8,'FaceColor','#0072BD')


filter_index = find(length_temp > 258)

length_temp(filter_index) = [];



pd = fitdist(length_temp,'Normal')
x_values = 256:0.01:258;
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
length_corrected(link_index) = pd.mu;
% figure (1)
% histogram(length1(:,1),100,'Normalization','pdf','EdgeAlpha',0.3,'FaceAlpha',0.8,'FaceColor','#0072BD')


%% LINK 3

link_index = 3;
length_temp = length_total(:,link_index);
figure (1)
histogram(length_temp,100,'Normalization','pdf','EdgeAlpha',0.3,'FaceAlpha',0.8,'FaceColor','#0072BD')


filter_index = find(length_temp > 256 | length_temp < 253.5)

length_temp(filter_index) = [];


pd = fitdist(length_temp,'Normal')
x_values = 253.5:0.01:256;
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

length_temp = length_total(:,link_index);
figure (1)
histogram(length_temp,100,'Normalization','pdf','EdgeAlpha',0.3,'FaceAlpha',0.8,'FaceColor','#0072BD')


filter_index = find(length_temp > 140 | length_temp < 135)

length_temp(filter_index) = [];


pd = fitdist(length_temp,'Normal')
x_values = 135:0.01:140;
y = pdf(pd,x_values);


figure (3)
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

%% Tip offset X
link_index = 5;

length_temp = length_total(:,link_index);
figure (1)
histogram(length_temp,100,'Normalization','pdf','EdgeAlpha',0.3,'FaceAlpha',0.8,'FaceColor','#0072BD')


% filter_index = find(length_temp > -0.8 | length_temp < -2.3)
% 
% length_temp(filter_index) = [];


pd = fitdist(length_temp,'Normal')
x_values = -2.4:0.01:-0.6;
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

% 
% filter_index = find(length_temp > 140 | length_temp < 135)
% 
% length_temp(filter_index) = [];


pd = fitdist(length_temp,'Normal')
x_values = 290:0.01:295;
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


filter_index = find(length_temp > 2 | length_temp < -0.50)

length_temp(filter_index) = [];


pd = fitdist(length_temp,'Normal')
x_values = -1:0.01:2;
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
save CALIBRATION_PARA_LENGTH  length_corrected
%%

phi_corrected_degree = zeros(7,1);
phi_corrected_degree(7) = 0;

phi_corrected_radian = zeros(7,1);
phi_corrected_radian(7) = 0;
%%
phi_index = 2;
phi_temp = phi_total(:,phi_index)*180/pi;
figure (1)
histogram(phi_temp,100,'Normalization','pdf','EdgeAlpha',0.3,'FaceAlpha',0.8,'FaceColor','#0072BD')


% filter_index = find(phi_temp > 256 | phi_temp < 253.5)
% 
% phi_temp(filter_index) = [];


pd = fitdist(phi_temp,'Normal')
x_values = -0.75:0.001:-0.3;
y = pdf(pd,x_values);


figure (2)
histogram(phi_temp,100,'Normalization','pdf','EdgeAlpha',0.3,'FaceAlpha',0.8,'FaceColor','#0072BD')
hold on
plot(x_values,y,'LineWidth',4)
hold off
title(strcat('\phi_', num2str(phi_index), ' distribution'))
xlabel('Angle deviation ({\circ})' )
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


filter_index = find( phi_temp < -1)

phi_temp(filter_index) = [];


pd = fitdist(phi_temp,'Normal')
x_values = -1:0.01:5.5;
y = pdf(pd,x_values);


figure (2)
histogram(phi_temp,100,'Normalization','pdf','EdgeAlpha',0.3,'FaceAlpha',0.8,'FaceColor','#0072BD')
hold on
plot(x_values,y,'LineWidth',4)
hold off
title(strcat('\phi_', num2str(phi_index), ' distribution'))
xlabel('Angle deviation ({\circ})' )
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
x_values = -1.7:0.01:-0.8;
y = pdf(pd,x_values);


figure (2)
histogram(phi_temp,100,'Normalization','pdf','EdgeAlpha',0.3,'FaceAlpha',0.8,'FaceColor','#0072BD')
hold on
plot(x_values,y,'LineWidth',4)
hold off
title(strcat('\phi_', num2str(phi_index), ' distribution'))
xlabel('Angle deviation ({\circ})' )
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
x_values = 1.32:0.01:2.3;
y = pdf(pd,x_values);


figure (2)
histogram(phi_temp,100,'Normalization','pdf','EdgeAlpha',0.3,'FaceAlpha',0.8,'FaceColor','#0072BD')
hold on
plot(x_values,y,'LineWidth',4)
hold off
title(strcat('\phi_', num2str(phi_index), ' distribution'))
xlabel('Angle deviation ({\circ})' )
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


filter_index = find( phi_temp < 1)

phi_temp(filter_index) = [];


pd = fitdist(phi_temp,'Normal')
x_values = 1.5:0.01:8.5;
y = pdf(pd,x_values);


figure (2)
histogram(phi_temp,100,'Normalization','pdf','EdgeAlpha',0.3,'FaceAlpha',0.8,'FaceColor','#0072BD')
hold on
plot(x_values,y,'LineWidth',4)
hold off
title(strcat('\phi_', num2str(phi_index), ' distribution'))
xlabel('Angle deviation ({\circ})' )
ylabel('Probabilty density')
set(gca,'FontSize',30);
set(gcf,'Position',[200 200 900 700])
phi_corrected_degree(phi_index) = pd.mu
phi_corrected_radian(phi_index) = pd.mu*pi/180

%%
save CALIBRATION_PARA_PHI_DEGREE  phi_corrected_degree
save CALIBRATION_PARA_PHI_RADIAN  phi_corrected_radian
%%
coeff_corrected_degree = zeros(4,1);
coeff_corrected_radian = zeros(4,1);

%%



coeff_index = 1;
coeff_temp = coeff_total(:,coeff_index)*180/pi;
figure (1)
histogram(coeff_temp,100,'Normalization','pdf','EdgeAlpha',0.3,'FaceAlpha',0.8,'FaceColor','#0072BD')


filter_index = find( coeff_temp > -0.26)

coeff_temp(filter_index) = [];


pd = fitdist(coeff_temp,'Normal')
x_values = -0.28:0.0001:-0.256;
y = pdf(pd,x_values);


figure (2)
histogram(coeff_temp,100,'Normalization','pdf','EdgeAlpha',0.3,'FaceAlpha',0.8,'FaceColor','#0072BD')
hold on
plot(x_values,y,'LineWidth',4)
hold off
title(strcat('\theta_', num2str(coeff_index), ' distribution'))
xlabel('Angle deviation ({\circ})' )
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


filter_index = find( coeff_temp > 0.6)

coeff_temp(filter_index) = [];


pd = fitdist(coeff_temp,'Normal')
x_values = 0.31:0.0001:0.6;
y = pdf(pd,x_values);


figure (2)
histogram(coeff_temp,100,'Normalization','pdf','EdgeAlpha',0.3,'FaceAlpha',0.8,'FaceColor','#0072BD')
hold on
plot(x_values,y,'LineWidth',4)
hold off
title(strcat('\theta_', num2str(coeff_index), ' distribution'))
xlabel('Angle deviation ({\circ})' )
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
x_values = 0.49:0.0001:1.5;
y = pdf(pd,x_values);


figure (2)
histogram(coeff_temp,100,'Normalization','pdf','EdgeAlpha',0.3,'FaceAlpha',0.8,'FaceColor','#0072BD')
hold on
plot(x_values,y,'LineWidth',4)
hold off
title(strcat('\theta_', num2str(coeff_index), ' distribution'))
xlabel('Angle deviation ({\circ})' )
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


filter_index = find( coeff_temp > 2)

coeff_temp(filter_index) = [];


pd = fitdist(coeff_temp,'Normal')
x_values = -6:0.001:2;
y = pdf(pd,x_values);


figure (2)
histogram(coeff_temp,100,'Normalization','pdf','EdgeAlpha',0.3,'FaceAlpha',0.8,'FaceColor','#0072BD')
hold on
plot(x_values,y,'LineWidth',4)
hold off
title(strcat('\theta_', num2str(coeff_index), ' distribution'))
xlabel('Angle deviation ({\circ})' )
ylabel('Probabilty density')
set(gca,'FontSize',30);
set(gcf,'Position',[200 200 900 700])
coeff_corrected_degree(coeff_index) = pd.mu
coeff_corrected_radian(coeff_index) = pd.mu*pi/180

%%
save CALIBRATION_PARA_COEFF_DEGREE coeff_corrected_degree
save CALIBRATION_PARA_COEFF_RADIAN coeff_corrected_radian


%%
t_base2ref_corrected = zeros(3,1);


t_base2ref_index = 3;
t_base2ref_temp = t_base2ref_total(:,t_base2ref_index);
figure (1)
histogram(t_base2ref_temp,100,'Normalization','pdf','EdgeAlpha',0.3,'FaceAlpha',0.8,'FaceColor','#0072BD')
%%
angle_base2ref_corrected = zeros(3,1);


angle_base2ref_index = 3;
angle_base2ref_temp = angle_base2ref_total(:,angle_base2ref_index);
figure (1)
histogram(angle_base2ref_temp,100,'Normalization','pdf','EdgeAlpha',0.3,'FaceAlpha',0.8,'FaceColor','#0072BD')


%%
figure
plot(residual_total_filterted)
% save 20210608_calibration_para x
%% Step 4: evaluation after optimization



% LENGTH = load('CALIBRATION_PARA_LENGTH.mat');
% LENGTH = LENGTH.length_corrected
% 
% PHI = load('CALIBRATION_PARA_PHI_RADIAN.mat');
% PHI = PHI.phi_corrected_radian
% PHI = [PHI 0]
% 
% coeff = load('CALIBRATION_PARA_COEFF_RADIAN.mat');
% coeff = coeff.coeff_corrected_radian
% 


LENGTH = x(1:7)
PHI = x(8:14);
coeff = x(15:18);
% correction_flag = [1 1 1 1 1 1 1];
% correction_flag = [0 0 0 0 0 0 0]';
correction_flag = [2 2 1 0 2 1 0]';
% encoder1_LUT = load('lookup_table_encoder1_p.mat');
encoder1_LUT = load('lookup_table_encoder1_new_mean.mat');
% encoder2_LUT = load('lookup_table_encoder2_p.mat');
encoder2_LUT = load('lookup_table_encoder2_new_mean.mat');
encoder5_LUT = load('lookup_table_encoder5_new2_mean.mat');
[m_tip2base_corrected] = SimplifiedKinematics(LENGTH, THETA, PHI,coeff,correction_flag, encoder1_LUT, encoder2_LUT, encoder5_LUT);
% m_base2ref = EulerAngle2Transform(x(19:21), x(22:24));
for i = 1: max(size(THETA))

%            v_tip2ref_corrected(:,i) = m_base2ref*m_tip2base_corrected(:,:,i)*[0 0 0 1]';
           v_tip2base_corrected(:,i) = m_tip2base_corrected(:,:,i)*[0 0 0 1]';

end

% Evaulation metric 1: visualize the 3D points distribution
[R, t] = computeRigidTransform(v_NDI_tip2ref(1:3,:)', v_tip2base_corrected(1:3,:)');
t_alignment(1:3,1:3) = R;
t_alignment(1:3,4) = t;
t_alignment(4,4) = 1;
v_NDI_tip2ref_corrected_aligned = t_alignment*v_NDI_tip2ref;


figure (2)
scatter3(v_NDI_tip2ref_corrected_aligned(1,:), v_NDI_tip2ref_corrected_aligned(2,:), v_NDI_tip2ref_corrected_aligned(3,:),'filled');
hold on
scatter3(v_tip2base_corrected(1,:), v_tip2base_corrected(2,:), v_tip2base_corrected(3,:),'filled');
hold off
legend('Tip in NDI', 'Arm tip')
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

figure
scatter3(v_NDI_tip2ref(1,:), v_NDI_tip2ref(2,:), v_NDI_tip2ref(3,:),'filled');
hold on
% scatter3(v_tip2ref_corrected(1,:), v_tip2ref_corrected(2,:), v_tip2ref_corrected(3,:),'filled');
scatter3(v_tip2base_corrected_aligned(1,:), v_tip2base_corrected_aligned(2,:), v_tip2base_corrected_aligned(3,:),'filled');
hold off
legend('Tip in NDI', 'Arm tip')
xlabel('X')
ylabel('Y')
zlabel('Z')


%%

% delta_NDI_tip = vecnorm(v_tip2base_corrected_aligned - v_tip2ref_corrected);
% mean(delta_NDI_tip)

delta_NDI_tip = v_NDI_tip2ref_corrected_aligned - v_tip2base_corrected;

figure (3)
plot(delta_NDI_tip(1,:),'LineWidth', 2)
hold on
plot(delta_NDI_tip(2,:),'LineWidth', 2)
plot(delta_NDI_tip(3,:),'LineWidth', 2)
hold off
title('distance deviation along X, Y, Z direction')
legend('X', 'Y', 'Z')

global_abs_value = vecnorm(delta_NDI_tip);
disp('GLOBAL ABSOLUTE MEAN VALUE:')
disp(num2str(mean(vecnorm(delta_NDI_tip))))
disp('GLOBAL SIGNED MEAN VALUE (X, Y, Z)')
signed_mean_value = [num2str(mean(delta_NDI_tip(1,:)')), '  ', num2str(mean(delta_NDI_tip(2,:)')), '  ', num2str(mean(delta_NDI_tip(3,:)'))];
disp(signed_mean_value)
disp('GLOBAL SIGNED STD VALUE (X, Y, Z)')
signed_std_value = [num2str(std(delta_NDI_tip(1,:)')), '  ', num2str(std(delta_NDI_tip(2,:)')), '  ', num2str(std(delta_NDI_tip(3,:)'))];
disp(signed_std_value)




%%
% Evaluation metric 2: compute relative distance error
%69
table_abs_mean_relative_distance =[];
table_signed_mean_relative_distance = [];

for reference_index = 1:247

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
%%










for i = 1:300
    
    table_NDI_arm(floor(i/15)+1, mod(i,15)+1) = delta_NDI_arm(i);
   
end





figure (4)
xvalues = {'10','30','50','70','90', '110','130','150','170','190','210','230','250','270','290' };
yvalues = {'10','30','50','70','90', '110','130','150','170','190'};
h = heatmap(xvalues,yvalues,abs(table_NDI_arm(1:10,:)))
title('Absolute relative distance (plate Z=0)')
xlabel('X direction of plate (mm)')
ylabel('Y direction of plate (mm)')
figure (5)
xvalues = {'10','30','50','70','90', '110','130','150','170','190','210','230','250','270','290' };
yvalues = {'10','30','50','70','90', '110','130','150','170','190'};
h = heatmap(xvalues, yvalues,table_NDI_arm(1:10,:))
title('Signed relative distance (plate Z=0)')
xlabel('X direction of plate (mm)')
ylabel('Y direction of plate (mm)')

figure (6)
xvalues = {'10','30','50','70','90', '110','130','150','170','190','210','230','250','270','290' };
yvalues = {'10','30','50','70','90', '110','130','150','170','190'};
h = heatmap(xvalues, yvalues,abs(table_NDI_arm(11:20,:)))
title('Absolute relative distance (plate Z=-32 mm)')
xlabel('X direction of plate (mm)')
ylabel('Y direction of plate (mm)')
figure (7)
xvalues = {'10','30','50','70','90', '110','130','150','170','190','210','230','250','270','290' };
yvalues = {'10','30','50','70','90', '110','130','150','170','190'};
h = heatmap(xvalues, yvalues,table_NDI_arm(11:20,:))
title('Signed relative distance (plate Z=-32 mm)')
xlabel('X direction of plate (mm)')
ylabel('Y direction of plate (mm)')


mean(mean(table_abs_mean_relative_distance));

mean(mean(table_signed_mean_relative_distance));
% disp_signed = ['mean(signed): ', num2str(mean(delta_plate_arm)),      '    std(signed): ', num2str(std(delta_plate_arm))];
% disp_abs    = ['mean(abs)   : ', num2str(mean(abs(delta_plate_arm))), '   std(abs)   : ', num2str(std(abs(delta_plate_arm)))];
% disp('Plate as reference')
% disp(disp_signed)
% disp(disp_abs)



vector_abs_mean_relative_distancereshape = reshape(table_abs_mean_relative_distance, [], 1);
vector_signed_mean_relative_distance = reshape(table_signed_mean_relative_distance, [], 1);




disp_signed = ['mean(signed): ', num2str(mean(mean(table_signed_mean_relative_distance))),      '    std(signed): ', num2str(std(vector_signed_mean_relative_distance))];
disp_abs    = ['mean(abs)   : ', num2str(mean(mean(table_abs_mean_relative_distance))), '   std(abs)   : ', num2str(std(vector_abs_mean_relative_distancereshape))];
disp('NDI as reference')
disp(disp_signed)
disp(disp_abs)






%%
LENGTH = x(1:7)
PHI = x(8:14)
% PHI = x(7:9);
% coeff = x(7:end);
coeff = x(21:22)
THETA1 = [8.789 -7.8222 -10.9863 1.9336 -0.3460 14.3262 -91.8457;
    26.455 -40.2539 -13.2715 1.9336 -0.3460 14.4141 -91.8457;
    25.4883 -25.8498 -7.9101 1.9336 -4.0877 23.2031 -85.3418]'*pi/180;



[m_tip2base_corrected] = SimplifiedKinematics(LENGTH, THETA1, PHI,coeff,correction_flag, encoder1_LUT, encoder2_LUT, encoder5_LUT)
save POC_LING_Parameters x