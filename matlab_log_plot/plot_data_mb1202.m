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

data = readtable("data_collected_with_lab/dati_read_sensor_2.csv");

%% Create new table to avoid Nan

data_to_plot = struct();

z_mocap = set_value(data.z_mocap(1:217));
z_mb1202 = set_value(data.z_mb1202(1:217));

outliers = isoutlier(z_mb1202, "movmedian", 10);
out_index = find(outliers);
z_mb1202_new = z_mb1202;
for i = out_index
    if i > 1
        z_mb1202_new(i) = z_mb1202(i - 1); % Sostituisci con il valore precedente
        z_mb1202(i) = z_mb1202(i - 1);
    end
end


%% Plot

figure("Name","Z coordinate"), hold on
plot(-z_mb1202_new,'-')
plot(z_mocap,'--')
legend({'mb1202','Motion Capture'})
title("Z coordinate")
xlabel('sample')
ylabel('[m]')


%% Histograms
z_moc_hist = z_mocap(42:end);
z_mb_hist = z_mb1202_new(42:end);
hist_z = z_moc_hist - (-z_mb_hist);
hist_true = z_mocap - (-z_mb1202_new);
mean = mean(hist_true);
std = std(hist_true);
figure('Name','Histogram Z coordinate')
histogram(hist_true)
title('Histogram Z coordinate')
