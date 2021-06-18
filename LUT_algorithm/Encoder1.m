%% Encoder 1
clc
clear all

encoder_data_p_1 = load('E:\PROGRAM\Project_PhD\Calibration\MyCode\Data\20210528\Encoder1_calib\20210528_joint1_encoder_data.txt');
THETA_p_1 = encoder_data_p_1'; % counter clockwise should be positive, but our collected data is opposite
THETA_p_1 = THETA_p_1*pi/180; % Degree to radians
THETA_p_1_degree = THETA_p_1*180/pi;
Theta1_p_1_degree = THETA_p_1_degree(1,:)';
Theta1_p_1_redian = THETA_p_1(1,:)';

protractor_data_1 = load('E:\PROGRAM\Project_PhD\Calibration\MyCode\Data\20210528\Encoder1_calib\20210528_joint1_angle_protractor.txt');
protractor_data_1 = 180-protractor_data_1;

encoder_data_p_2 = load('E:\PROGRAM\Project_PhD\Calibration\MyCode\Data\20210605\20210605_encoder1_data.txt');
THETA_p_2 = encoder_data_p_2'; % counter clockwise should be positive, but our collected data is opposite
THETA_p_2 = THETA_p_2*pi/180; % Degree to radians
THETA_p_2_degree = THETA_p_2*180/pi;
Theta1_p_2_degree = THETA_p_2_degree(1,:)';
Theta1_p_2_redian = THETA_p_2(1,:)';

protractor_data_2 = load('E:\PROGRAM\Project_PhD\Calibration\MyCode\Data\20210605\20210605_encoder1_protractor.txt');
protractor_data_2 = 180-protractor_data_2;



delta1 = protractor_data_1 - Theta1_p_1_degree;
delta2 = protractor_data_2 - Theta1_p_2_degree;


x = [];
for i = 1: 105
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
% bias = Theta1_p_1_degree(48) - Theta1_p_2_degree(135)
% bias/(360/4096)
%%

figure (1) 
scatter(Theta1_p_1_degree(:), delta1(:),'filled')
hold on
scatter(Theta1_p_2_degree(:), delta2(:), 'filled')

hold off
xlabel('encoder angle (degree)')
ylabel('angle by protractor (degree)')
title('Joint 1: difference between encoder angle and angle measured by protractor')
legend('1st', '2nd')

figure (3) 
plot(Theta1_p_1_degree(:), delta1(:),'LineWidth',2)
hold on
plot(Theta1_p_2_degree(:), delta2(:)+ 360*30/4096, 'LineWidth',2)
hold off
xlabel('encoder angle (degree)')
ylabel('angle by protractor (degree)')
title('Joint 1: difference between encoder angle and angle measured by protractor')
legend('1st', '2nd')






%% Compute two lookup table

table1 = [Theta1_p_2_degree(1:183)'; protractor_data_2(1:183)'];
table2 = [Theta1_p_2_degree(184:end)'; protractor_data_2(184:end)'];

figure (4)
plot(Theta1_p_2_degree(1:183), delta2(1:183),'LineWidth',2)
hold on
plot(Theta1_p_2_degree(184:end), delta2(184:end), 'LineWidth',2)
hold off
legend('1st','2nd')

table_up = load('lookup_table_encoder1_new_up.mat');

table_down = load('lookup_table_encoder1_new_down.mat');

table_up = table_up.lookup_table_unique;
table_down = table_down.lookup_table_unique;

mapping_angle_up = [];
mapping_angle_down = [];
encoder_readings = -90:0.5:90;
for i = -90:0.5:90
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
table_mean = load('lookup_table_encoder1_new_mean.mat');
table_mean = table_mean.lookup_table_unique;
% mean__delta_interp = (delta_interp_up + delta_interp_down)/2;
mean_delta_interp = table_mean(2,:) - encoder_readings;
figure (4)
% plot(Theta1_p_2_degree(1:183), delta2(1:183),'LineWidth',2)
% hold on
% plot(Theta1_p_2_degree(184:end), delta2(184:end), 'LineWidth',2)
plot(encoder_readings, delta_interp_up,'LineWidth',2)
hold on
plot(encoder_readings(2:361), delta_interp_down(2:361),'LineWidth',2)
plot(encoder_readings, mean_delta_interp,'LineWidth',2)
% legend('1st','2nd','Interp up','Interp down','Interp mean')
legend('90{\circ}-> -90{\circ}','-90{\circ}-> 90{\circ}','correction curve')
xlabel('Encoder reading ({\circ})')
ylabel('Correction angle ({\circ})')
title('1st encoder correction')
set(gcf,'Position',[200 200 1000 700])


% save('encoder1_LUT_mean.txt','table_mean','-ascii')



%%



