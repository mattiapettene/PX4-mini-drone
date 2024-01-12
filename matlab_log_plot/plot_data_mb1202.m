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

data = readtable("dati_read_sensor_2.csv");

%% Create new table to avoid Nan

data_to_plot = struct();

x_mocap = set_value(data.y_mocap);
y_mocap = -set_value(data.x_mocap);
x_uwb = set_value(data.x_uwb);
y_uwb = set_value(data.y_uwb);

%% Outliers rejection
window = 3;
outliers_x = isoutlier(x_uwb, "movmedian", window);
out_index_x = find(outliers_x);
x_uwb_new = x_uwb;
for i = out_index_x
    if i > 1
        x_uwb_new(i) = x_uwb(i - 1); 
        x_uwb(i) = x_uwb(i - 1);
    end
end
outliers_y = isoutlier(y_uwb, "movmedian", window);
out_index_y = find(outliers_y);
y_uwb_new = y_uwb;
for i = out_index_x
    if i > 1
        y_uwb_new(i) = y_uwb(i - 1);
        y_uwb(i) = y_uwb(i - 1);
    end
end


%% Plot

figure("Name","Original"), hold on
plot(x_mocap, y_mocap,'-')
plot(x_uwb_new, y_uwb_new, '.')
legend({'Motion Capture', 'UWB'})
title("Original")
xlabel('x')
ylabel('y')


% %% Histograms
% z_moc_hist = z_mocap(42:end);
% z_mb_hist = z_mb1202_new(42:end);
% hist_z = z_moc_hist - (-z_mb_hist);
% hist_true = z_mocap - (-z_mb1202_new);
% mean = mean(hist_true);
% std = std(hist_true);
% figure('Name','Histogram Z coordinate')
% histogram(hist_true)
% title('Histogram Z coordinate')
