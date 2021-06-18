%% Encoder 5
clc
clear all
%%
encoder_data_p_1 = load('E:\PROGRAM\Project_PhD\Calibration\MyCode\Data\20210526\20210526_joint5_encoder_data_no_load.txt');
THETA_p_1 = encoder_data_p_1'; % counter clockwise should be positive, but our collected data is opposite
THETA_p_1 = THETA_p_1*pi/180; % Degree to radians
THETA_p_1_degree = THETA_p_1*180/pi;
Theta5_p_1_degree = THETA_p_1_degree(5,:)';
Theta5_p_1_redian = THETA_p_1(5,:)';
Theta6_p_1_degree = THETA_p_1_degree(6,:)';
Theta6_p_1_redian = THETA_p_1(6,:)';


protractor_data_1 = load('E:\PROGRAM\Project_PhD\Calibration\MyCode\Data\20210526\20210516_joint5_protractor_no_load.txt');
protractor_data_1(1:51) = protractor_data_1(1:51)*(-1);

encoder_data_p_2 = load('E:\PROGRAM\Project_PhD\Calibration\MyCode\Data\20210606\Encoder5\20210606_encoder5_data.txt');
THETA_p_2 = encoder_data_p_2'; % counter clockwise should be positive, but our collected data is opposite
THETA_p_2 = THETA_p_2*pi/180; % Degree to radians
THETA_p_2_degree = THETA_p_2*180/pi;
Theta5_p_2_degree = THETA_p_2_degree(5,:)';
Theta5_p_2_redian = THETA_p_2(5,:)';
Theta6_p_2_degree = THETA_p_2_degree(6,:)';
Theta6_p_2_redian = THETA_p_2(6,:)';


protractor_data_2 = load('E:\PROGRAM\Project_PhD\Calibration\MyCode\Data\20210606\Encoder5\0606_encoder5.txt');
% protractor_data_2 = 180-protractor_data_2;

encoder_data_p_3 = load('E:\PROGRAM\Project_PhD\Calibration\MyCode\Data\20210607\Encoder5\20210607_encoder5_data.txt');
THETA_p_3 = encoder_data_p_3'; % counter clockwise should be positive, but our collected data is opposite
THETA_p_3 = THETA_p_3*pi/180; % Degree to radians
THETA_p_3_degree = THETA_p_3*180/pi;
Theta5_p_3_degree = THETA_p_3_degree(5,:)';
Theta5_p_3_redian = THETA_p_3(5,:)';
Theta6_p_3_degree = THETA_p_3_degree(6,:)';
Theta6_p_3_redian = THETA_p_3(6,:)';

protractor_data_3 = load('E:\PROGRAM\Project_PhD\Calibration\MyCode\Data\20210607\Encoder5\Encoder5_0607_total.txt');
% protractor_data_1(1:51) = protractor_data_1(1:51)*(-1);

protractor_data_2(168) = [];
Theta5_p_2_degree(168) = [];
Theta5_p_2_redian(168) = [];
Theta5_p_2_original_degree(168) = [];

delta1 = protractor_data_1 - Theta5_p_1_degree;
delta2 = protractor_data_2 - Theta5_p_2_degree;
delta3 = protractor_data_3 - Theta5_p_3_degree;
%%
delta1_original = protractor_data_1 - Theta5_p_1_original_degree;
delta2_original = protractor_data_2 - Theta5_p_2_original_degree;
delta3_original = protractor_data_3 - Theta5_p_3_original_degree;

%%
x = [];
for i = 1: 92
   temp = find(protractor_data_2 == protractor_data_1(i)) 
   x =  [x; temp]
end

% bias = Theta1_p_1_degree(17) - Theta1_p_2_degree(39)
% bias = Theta1_p_1_degree(20) - Theta1_p_2_degree(48)
% bias = Theta1_p_1_degree(21) - Theta1_p_2_degree(51)
% bias = Theta1_p_1_degree(26) - Theta1_p_2_degree(66)
% bias = Theta1_p_1_degree(31) - Theta1_p_2_degree(81)
% bias = Theta1_p_1_degree(32) - Theta1_p_2_degree(84)
% bias = Theta1_p_1_degree(35) - Theta1_p_2_degree(97)
% bias = Theta1_p_1_degree(37) - Theta1_p_2_degree(102)
% bias = Theta1_p_1_degree(38) - Theta1_p_2_degree(105)
% bias = Theta1_p_1_degree(43) - Theta1_p_2_degree(120)
bias = Theta2_p_1_degree(5) - Theta2_p_2_degree(19)
bias = Theta2_p_1_degree(6) - Theta2_p_2_degree(23)
bias/(360/4096)
%%

figure (1) 
scatter(Theta5_p_1_degree(:), delta1(:),'filled')
hold on
scatter(Theta5_p_2_degree(:), delta2(:), 'filled')
scatter(Theta5_p_3_degree(:), delta3(:),'filled')
hold off
xlabel('encoder angle (degree)')
ylabel('angle by protractor (degree)')
title('Joint 1: difference between encoder angle and angle measured by protractor')
legend('1st', '2nd','3rd')



figure (2) 
scatter(Theta5_p_1_original_degree(:), delta1_original(:),'filled')
hold on
scatter(Theta5_p_2_original_degree(:), delta2_original(:), 'filled')
scatter(Theta5_p_3_original_degree(:), delta3_original(:),'filled')
hold off
xlabel('encoder angle (degree)')
ylabel('angle by protractor (degree)')
title('Joint 1: difference between encoder angle and angle measured by protractor')
legend('1st', '2nd','3rd')

figure (3) 
scatter(Theta5_p_3_original_degree(:), protractor_data_3(:),'filled')
% hold on
% scatter(Theta5_p_2_original_degree(:), delta2_original(:), 'filled')
% scatter(Theta5_p_3_original_degree(:), delta3_original(:),'filled')
% hold off
xlabel('encoder angle (degree)')
ylabel('angle by protractor (degree)')
title('Joint 1: difference between encoder angle and angle measured by protractor')
legend('1st', '2nd','3rd')



%%


figure (3) 
plot(Theta2_p_1_degree(:), delta1(:),'LineWidth',2)
hold on
plot(Theta2_p_2_degree(:), delta2(:)+ 360*27/4096, 'LineWidth',2)
hold off
xlabel('encoder angle (degree)')
ylabel('angle by protractor (degree)')
title('Joint 1: difference between encoder angle and angle measured by protractor')
legend('1st', '2nd')






%% Compute two lookup table

table1 = [encoder_data_p_3(1:101,8)'; protractor_data_3(1:101)'];
table2 = [encoder_data_p_3(102:end,8)'; protractor_data_3(102:end)'];

figure (4)
plot(encoder_data_p_3(1:101,8), delta3_original(1:101),'LineWidth',2)
hold on
plot(encoder_data_p_3(102:end,8), delta3_original(102:end), 'LineWidth',2)
hold off
legend('1st','2nd')

table_up = load('lookup_table_encoder5_new2_up.mat');

table_down = load('lookup_table_encoder5_new2_down.mat');

table_up = table_up.lookup_table_unique;
table_down = table_down.lookup_table_unique;

mapping_angle_up = [];
mapping_angle_down = [];
encoder_readings = -38:0.5:38;
for i = -38:0.5:38
    encoder_angle = i;
    temp_up = LookupTable(encoder_angle, table_up);
    mapping_angle_up = [mapping_angle_up temp_up];
    temp_down = LookupTable(encoder_angle, table_down);
    mapping_angle_down =  [mapping_angle_down temp_down];
end

delta_interp_up = mapping_angle_up - encoder_readings;
delta_interp_down = mapping_angle_down - encoder_readings;


mean_interp = (mapping_angle_up + mapping_angle_down)/2;
table_mean1 = [encoder_readings; mean_interp];
table_mean = load('lookup_table_encoder5_new2_mean.mat');
table_mean = table_mean.lookup_table_unique;


% mean__delta_interp = (delta_interp_up + delta_interp_down)/2;
mean_delta_interp = table_mean(2,:) - encoder_readings;
figure (4)
% plot(encoder_data_p_3(1:101,8), delta3_original(1:101),'LineWidth',2)
% hold on
% plot(encoder_data_p_3(102:end,8), delta3_original(102:end), 'LineWidth',2)
plot(encoder_readings, delta_interp_up,'LineWidth',2)
hold on
plot(encoder_readings, delta_interp_down,'LineWidth',2)
plot(encoder_readings, mean_delta_interp,'LineWidth',2)
hold off
% legend('1st','2nd','Interp up','Interp down','Interp mean')
legend('50{\circ}-> -50{\circ}','-50{\circ}-> 50{\circ}','correction curve')
xlabel('Encoder reading ({\circ})')
ylabel('Correction angle ({\circ})')
title('5th encoder correction')
% ylim([-1.6 1])
set(gcf,'Position',[200 200 1000 700])
% table_mean = [table_up(:,1:14) table_mean];


% save('encoder5_LUT_mean2.txt','table_mean','-ascii')



figure
plot(table_up(1,:), table_up(2,:))
hold on
plot(table_down(1,:), table_down(2,:))
plot(table_mean(1,:), table_mean(2,:))
hold off
legend('up','down','mean')

%%



