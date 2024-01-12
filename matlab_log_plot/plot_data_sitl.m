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

data = readtable("log_sitl_gps/log_0_2024-1-6-13-45-32.csv");

%% Create new table to avoid Nan

data_to_plot = struct();

data_to_plot.setpoint_x = set_value(data.vehicle_local_position_setpoint_x);
data_to_plot.est_x = set_value(data.vehicle_local_position_x);
data_to_plot.ground_x = set_value(data.vehicle_local_position_groundtruth_x);

data_to_plot.setpoint_y = set_value(data.vehicle_local_position_setpoint_y);
data_to_plot.est_y = set_value(data.vehicle_local_position_y);
data_to_plot.ground_y = set_value(data.vehicle_local_position_groundtruth_y);

data_to_plot.setpoint_z = set_value(data.vehicle_local_position_setpoint_z);
data_to_plot.est_z = set_value(data.vehicle_local_position_z);
data_to_plot.ground_z = set_value(data.vehicle_local_position_groundtruth_z);

%% Plot

figure("Name","X coordinate"), hold on
plot(data_to_plot.setpoint_x,'-')
plot(data_to_plot.est_x,'--')
plot(data_to_plot.ground_x,'--')
legend({'Setpoint','Estimated','Ground Truth'})
title("X coordinate")
xlabel('sample')
ylabel('[m]')

figure("Name","Y coordinate"), hold on
plot(data_to_plot.setpoint_y,'-')
plot(data_to_plot.est_y,'--')
plot(data_to_plot.ground_y,'--')
legend({'Setpoint','Estimated','Ground Truth'})
title("Y coordinate")
xlabel('sample')
ylabel('[m]')

figure("Name","Z coordinate"), hold on
plot(data_to_plot.setpoint_z,'-')
plot(data_to_plot.est_z,'--')
plot(data_to_plot.ground_z,'--')
legend({'Setpoint','Estimated','Ground Truth'})
title("Z coordinate")
xlabel('sample')
ylabel('[m]')

%%

figure("Name","(X,Y,Z) coordinate"), hold on
plot(data.x__time,data_to_plot.setpoint_x,'-')
plot(data.x__time,data_to_plot.est_x,'--')
plot(data.x__time,data_to_plot.ground_x,'-.')
plot(data.x__time,data_to_plot.setpoint_y,'-')
plot(data.x__time,data_to_plot.est_y,'--')
plot(data.x__time,data_to_plot.ground_y,'-.')
plot(data.x__time,data_to_plot.setpoint_z,'-')
plot(data.x__time,data_to_plot.est_z,'--')
plot(data.x__time,data_to_plot.ground_z,'-.')
legend({'X Setpoint','X Estimated','X Ground Truth','Y Setpoint','Y Estimated','Y Ground Truth','Z Setpoint','Z Estimated','Z Ground Truth'})
title("(X,Y,Z) coordinate")
xlabel('time [s]')
ylabel('[m]')

%%

figure("Name","(X,Y) coordinate"), hold on
plot(data_to_plot.setpoint_x,data_to_plot.setpoint_y,'-')
plot(data_to_plot.est_x,data_to_plot.est_y,'--')
plot(data_to_plot.ground_x,data_to_plot.ground_y,'--')
legend({'Setpoint','Estimated','Ground Truth'})
title("(X,Y) coordinate")
xlabel('x [m]')
ylabel('y [m]')
axis equal

figure
plot3(data_to_plot.setpoint_x,data_to_plot.setpoint_y,-data_to_plot.setpoint_z,'-',...
    data_to_plot.est_x,data_to_plot.est_y,-data_to_plot.est_z,'--',...
    data_to_plot.ground_x,data_to_plot.ground_y,-data_to_plot.ground_z,'-.')
legend({'Setpoint','Estimated','Ground Truth'})
title("(X,Y,Z) coordinate")
xlabel('x [m]')
ylabel('y [m]')
zlabel('z [m]')
axis equal