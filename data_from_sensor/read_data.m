%% Initialization and import table

clear all
close all
clc

data_px4 = readtable("dati_read_sensor_2.csv");

%% reject outlier 

for i = 2:length(data_px4.x_uwb)
    if(data_px4.x_uwb(i) > 3 || data_px4.x_uwb(i) < -1)
        data_px4.x_uwb(i) = data_px4.x_uwb(i-1);
    end
end
for i = 2:length(data_px4.y_uwb)
    if(data_px4.y_uwb(i) > 1 || data_px4.y_uwb(i) < -1)
        data_px4.y_uwb(i) = data_px4.y_uwb(i-1);
    end
end

%% Plots

figure('Name','(X,Y) coordinate'), hold on;
plot(data_px4.x_mocap,data_px4.y_mocap)
plot(data_px4.x_uwb,data_px4.y_uwb)
legend({'Mocap','UWB'})
title('(X,Y) coordinate')
xlabel('X [m]')
ylabel('Y [m]')

figure('Name','(Z) coordinate'), hold on;
plot(data_px4.z_mocap)
plot(-data_px4.z_mb1202)
legend({'Mocap','mb1202'})
title('(Z) coordinate')
xlabel('sample [#]')
ylabel('Z [m]')

%% Histograms

hist_x = data_px4.x_mocap - data_px4.x_uwb;
hist_y = data_px4.y_mocap - data_px4.y_uwb;
hist_z = data_px4.z_mocap - (-data_px4.z_mb1202);

figure('Name','Histogram X coordinate')
histogram(hist_x)
title('Histogram X coordinate')

figure('Name','Histogram Y coordinate')
histogram(hist_y)
title('Histogram Y coordinate')

figure('Name','Histogram Z coordinate')
histogram(hist_z)
title('Histogram Z coordinate')

