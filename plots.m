close all;
clear;

set(0,'defaulttextinterpreter','latex')
set(groot, 'defaultAxesTickLabelInterpreter','latex');
set(groot, 'defaultLegendInterpreter','latex');
set(0,'DefaultFigureWindowStyle','docked');
set(0,'defaultAxesFontSize',  16)
set(0,'DefaultLegendFontSize', 16)

load dati.csv

x_gps = dati(:,1);
y_gps = dati(:,2);
z_gps = dati(:,3);

x_uwb = dati(:,4);
y_uwb = dati(:,5);
z_uwb = dati(:,6);

figure
hold on
plot(x_gps, 'LineWidth', 2)
plot(x_uwb, 'LineWidth', 2)
legend({'x gps','x uwb'}, 'Location','eastoutside');
xlabel('$t$ [s]')
ylabel('$x$ [m]')

figure
hold on
plot(y_gps, 'LineWidth', 2)
plot(y_uwb, 'LineWidth', 2)
legend({'y gps','y uwb'}, 'Location','eastoutside');
xlabel('$t$ [s]')
ylabel('$y$ [m]')

figure
hold on
plot(z_gps, 'LineWidth', 2)
plot(z_uwb, 'LineWidth', 2)
legend({'z gps','z uwb'}, 'Location','eastoutside');
xlabel('$t$ [s]')
ylabel('$z$ [m]')
