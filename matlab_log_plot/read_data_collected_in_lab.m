%% Initialization and import table

clear all
close all
clc

data_px4 = readtable("data_collected_with_lab/dati_read_sensor_2.csv");

% anchors positions
anchors = [0.083, 2.046, -0.074;...
           1.725, 7.830, 0.040;...
           4.730, 9.571, 0.005;...
           5.764, 4.277, 0.103;...
           5.584, 0.098, 0.111;...
           0.0, 0.0, 0.0];

%% Reject outlier outside the laboratory

for i = 2:length(data_px4.x_uwb)
    if(data_px4.x_uwb(i) > 6 || data_px4.x_uwb(i) < -6)
        data_px4.x_uwb(i) = data_px4.x_uwb(i-1);
        data_px4.y_uwb(i) = data_px4.y_uwb(i-1);
    end
end

for i = 2:length(data_px4.y_uwb)
    if(data_px4.y_uwb(i) > 10 || data_px4.y_uwb(i) < -10)
        data_px4.x_uwb(i) = data_px4.x_uwb(i-1);
        data_px4.y_uwb(i) = data_px4.y_uwb(i-1);
    end
end

%% Remove Nan (for plot purpose)

for i = 1:length(data_px4.y_uwb)
    if(isnan(data_px4.y_uwb(i)) || isnan(data_px4.x_uwb(i)))
        data_px4.x_uwb(i) = data_px4.x_uwb(i-1);
        data_px4.y_uwb(i) = data_px4.y_uwb(i-1);
    end
end

%% Plot the anchors position

figure('Name','(X,Y) coordinate'), hold on;
for i = 1:6
    plot(anchors(i,2),anchors(i,1),'.','MarkerSize',50,'Color','r')
    text(anchors(i,2),anchors(i,1) - 0.3,['Anchors ', num2str(i)],'HorizontalAlignment','center','FontWeight','bold','FontSize',20,'Color','red')
end
title('Anchors position')
xlabel('x [m]')
ylabel('y [m]')
axis equal

%% Plots results

figure('Name','(X,Y) coordinate'), hold on;
for i = 1:6
    plot(anchors(i,2),anchors(i,1),'.','MarkerSize',50,'Color','r')
    text(anchors(i,2),anchors(i,1) - 0.3,['Anchors ', num2str(i)],'HorizontalAlignment','center','FontWeight','bold','FontSize',20,'Color','red')
end
plot(5.981 - data_px4.x_mocap,3.565 + data_px4.y_mocap,'LineWidth',5)
plot(5.981 + data_px4.y_uwb,3.565 + data_px4.x_uwb,'.','MarkerSize',10)
legend({'UWB Anchors','','','','','','Mocap','UWB'})
title('(X,Y) coordinate')
xlabel('x [m]')
ylabel('y [m]')
axis equal

figure('Name','(Z) coordinate'), hold on;
plot(data_px4.z_mocap)
plot(-data_px4.z_mb1202)
legend({'Mocap','mb1202'})
title('(Z) coordinate')
xlabel('sample')
ylabel('Z [m]')

%% Statistical analysis

% Histograms

hist_x = -data_px4.x_mocap - data_px4.y_uwb;
hist_y = data_px4.y_mocap - data_px4.x_uwb;
hist_z = data_px4.z_mocap - (-data_px4.z_mb1202);

figure('Name','Histogram X coordinate')
histogram(hist_x)
title('Histogram X coordinate')
xlabel('Error')
ylabel('Sample')

figure('Name','Histogram Y coordinate')
histogram(hist_y)
title('Histogram Y coordinate')
xlabel('Error')
ylabel('Sample')

figure('Name','Histogram Z coordinate')
histogram(hist_z)
title('Histogram Z coordinate')
xlabel('Error')
ylabel('Sample')

% Mean and standard deviation

display("X coordinate")
mean(hist_x)
std(hist_x)

display("Y coordinate")
mean(hist_y)
std(hist_y)