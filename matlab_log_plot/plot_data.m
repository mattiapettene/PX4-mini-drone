%% Import data
clc
clear all
close all

%% Set LaTeX as default interpreter for axis labels, ticks and legends
set(0,'defaulttextinterpreter','latex')
set(groot, 'defaultAxesTickLabelInterpreter','latex');
set(groot, 'defaultLegendInterpreter','latex');

set(0,'defaultAxesFontSize',22)
set(0,'DefaultLegendFontSize',22)
set(0,'DefaultFigureWindowStyle','docked')
set(0,'DefaultUicontrolFontsize', 14)
set(0, 'DefaultLineLineWidth', 3);

data = readtable("log_sitl_indoor/uwb_2.csv");

%% Create new table to avoid Nan

data_to_plot = struct();

data_to_plot.setpoint_x = set_value(data.vehicle_local_position_setpoint_x);
data_to_plot.est_x = set_value(data.vehicle_local_position_x);
data_to_plot.mocap_x = set_value(data.vehicle_visual_odometry_position_00);
data_to_plot.ground_x = set_value(data.vehicle_local_position_groundtruth_x);

data_to_plot.setpoint_y = set_value(data.vehicle_local_position_setpoint_y);
data_to_plot.est_y = set_value(data.vehicle_local_position_y);
data_to_plot.mocap_y = set_value(data.vehicle_visual_odometry_position_01);
data_to_plot.ground_y = set_value(data.vehicle_local_position_groundtruth_y);

data_to_plot.setpoint_z = set_value(data.vehicle_local_position_setpoint_z);
data_to_plot.est_z = set_value(data.vehicle_local_position_z);
data_to_plot.mocap_z = set_value(data.vehicle_visual_odometry_position_02);
data_to_plot.ground_z = set_value(data.vehicle_local_position_groundtruth_z);

%% Plot

figure("Name","X coordinate"), hold on
plot(data_to_plot.setpoint_x,'-')
plot(data_to_plot.est_x,'--')
plot(data_to_plot.mocap_x,'-.')
legend({'Setpoint','Estimated','Motion Capture'})
title("X coordinate")
xlabel('sample')
ylabel('[m]')

figure("Name","Y coordinate"), hold on
plot(data_to_plot.setpoint_y,'-')
plot(data_to_plot.est_y,'--')
plot(data_to_plot.mocap_y,'-.')
legend({'Setpoint','Estimated','Motion Capture'})
title("Y coordinate")
xlabel('sample')
ylabel('[m]')

figure("Name","Z coordinate"), hold on
plot(data_to_plot.setpoint_z,'-')
plot(data_to_plot.est_z,'--')
plot(data_to_plot.mocap_z,'-.')
legend({'Setpoint','Estimated','Motion Capture'})
title("Z coordinate")
xlabel('sample')
ylabel('[m]')

%%

horizon = 3000:length(data.x__time);

figure("Name","(X,Y,Z) coordinate"), hold on
plot(data.x__time(horizon),data_to_plot.setpoint_x(horizon),'-')
plot(data.x__time(horizon),data_to_plot.est_x(horizon),'--')
plot(data.x__time(horizon),data_to_plot.mocap_x(horizon),':')
plot(data.x__time(horizon),data_to_plot.ground_x(horizon),'-.')
plot(data.x__time(horizon),data_to_plot.setpoint_y(horizon),'-')
plot(data.x__time(horizon),data_to_plot.est_y(horizon),'--')
plot(data.x__time(horizon),data_to_plot.mocap_y(horizon),':')
plot(data.x__time(horizon),data_to_plot.ground_y(horizon),'-.')
% plot(data.x__time(horizon),data_to_plot.setpoint_z(horizon),'-')
% plot(data.x__time(horizon),data_to_plot.est_z(horizon),'--')
% plot(data.x__time(horizon),data_to_plot.mocap_z(horizon),':')
% plot(data.x__time(horizon),data_to_plot.ground_z(horizon),'-.')
legend({'X Setpoint','X Estimated','X UWB','X Ground Truth','Y Setpoint','Y Estimated','Y UWB','Y Ground Truth'})%,'Z Setpoint','Z Estimated','Z UWB','Z Ground Truth'})
% legend({'X Setpoint','X Estimated','X Ground Truth','Y Setpoint','Y Estimated','Y Ground Truth','Z Setpoint','Z Estimated','Z Ground Truth'})
title("(X,Y,Z) coordinate")
xlim([15 45])
xlabel('time [s]')
ylabel('[m]')

%%

figure("Name","(X,Y) coordinate"), hold on
plot(data_to_plot.setpoint_x,data_to_plot.setpoint_y,'-')
plot(data_to_plot.est_x,data_to_plot.est_y,'--')
plot(data_to_plot.mocap_x,data_to_plot.mocap_y,'-.')
plot(data_to_plot.ground_x,data_to_plot.ground_y,'-.')
legend({'Setpoint','Estimated','UWB','Ground Truth'})
title("(X,Y) coordinate")
xlabel('x [m]')
ylabel('y [m]')

figure
plot3(data_to_plot.setpoint_x,data_to_plot.setpoint_y,-data_to_plot.setpoint_z,'-',...
    data_to_plot.est_x,data_to_plot.est_y,-data_to_plot.est_z,'--',...
    data_to_plot.ground_x,data_to_plot.ground_y,-data_to_plot.ground_z,'-.',...
    data_to_plot.mocap_x,data_to_plot.mocap_y,-data_to_plot.mocap_z,'-.')
legend({'Setpoint','Estimated','Ground Truth','UWB'})
title("(X,Y,Z) coordinate")
xlabel('x [m]')
ylabel('y [m]')
zlabel('z [m]')

%% Histograms

hist_x = data_to_plot.ground_x - data_to_plot.est_x;
hist_y = data_to_plot.ground_y - data_to_plot.est_y;
hist_z = data_to_plot.ground_z - data_to_plot.est_z;

figure('Name','Histogram X coordinate')
histogram(hist_x)
title('Histogram X coordinate')

figure('Name','Histogram Y coordinate')
histogram(hist_y)
title('Histogram Y coordinate')

figure('Name','Histogram Z coordinate')
histogram(hist_z)
title('Histogram Z coordinate')


