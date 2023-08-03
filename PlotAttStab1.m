clear all;
load('DESKTOP-4J3FS1P_2023_01_13_19_21_50FlapperInQualisys.mat')
DarkRed = [0.7,0.2,0.2];
PatchRed = [0.5,0.25,0.25];
LighGrayRed = [0.9,0.5,0.5];

DarkRed_2 = [0.5,0.3,0.3];
PatchRed_2 = [0.8,0.6,0.6];
LighGrayRed_2 = [0.95,0.55,0.75];

DarkGreen = [0.2,0.7,0.2];
PatchGreen = [0.25,0.5,0.25];
LighGrayGreen = [0.5,0.9,0.5];

DarkGreen_2 = [0.3,0.5,0.3];
PatchGreen_2 = [0.6,0.8,0.6];
LighGrayGreen_2 = [0.55,0.95,0.65];

DarkBlue = [0.2,0.2 ,0.7];
PatchBlue = [0.25,0.25, 0.5];
LighGrayBlue = [0.5,0.5,0.9];

DarkBlue_2 = [0.3,0.3,0.5];
PatchBlue_2 = [0.6,0.6,0.8];
LighGrayBlue_2 = [0.55,0.7,0.95];

LighGray = [0.8,0.8,0.8];


LineWidth = 0.9;
Emph = 1.3;
BasisRotation = [1, 0, 0;...
                 0, 0, 1;...
                 0,-1, 0]';
             
% x_tick = [0 2000 4000 6000 8000 10000 12000];
figure;
subplot(3,1,1);
hold on;
Cut = 500 : 2900;

plot(record_time_stamp(Cut),deg2rad(record_Sensor_data(Cut,4)) ,"-",'LineWidth',LineWidth* Emph,'Color',LighGrayRed * 0.7);
plot(record_time_stamp(Cut),deg2rad(record_Sensor_data(Cut,5)) ,"-",'LineWidth',LineWidth* Emph,'Color',LighGrayGreen * 0.7);
plot(record_time_stamp(Cut),deg2rad(record_Sensor_data(Cut,6)) ,"-",'LineWidth',LineWidth* Emph,'Color',LighGrayBlue* 0.7);

x_tick = record_time_stamp(Cut(1)) + ...
        (record_time_stamp(Cut(end)) - record_time_stamp(Cut(1))) * (0 : 0.2 : 1);
x_tick = roundn(x_tick,-2);
y_tick = [-3.14,  0,  3.14];
set(gca,'xtick',x_tick);
set(gca,'ytick',y_tick);
axis([x_tick(1) x_tick(end) y_tick(1) y_tick(end)]);
grid on;
h = gca;
set(h,'LineWidth',1,'GridLineStyle','--','GridAlpha',1,'GridColor',[0.9,0.9,0.9])
hold off;
legend('Roll','Pitch','Yaw');


idnetical_rot = [1,0,0;...
                 0,1,0;...
                 0,0,1];
G =             [1,0,0;...
                 0,1,0;...
                 0,0,1];
psi = ones(1,size(Cut,2));
for i = 1 : size(Cut,2)
    psi(i) = 0.5 * trace (G * (idnetical_rot ...
        - idnetical_rot' * reshape(record_Flapper_att(Cut(i),:,:), 3, 3))); 
end
subplot(3,1,2);
plot(record_time_stamp(Cut),psi ,"-",'LineWidth',LineWidth* Emph,'Color',LighGray * 0.6);
y_tick = [-0.1,  1,  2.1];
set(gca,'xtick',x_tick);
set(gca,'ytick',y_tick);
axis([x_tick(1) x_tick(end) y_tick(1) y_tick(end)]);
grid on;
h = gca;
set(h,'LineWidth',1,'GridLineStyle','--','GridAlpha',1,'GridColor',[0.9,0.9,0.9])
hold off;

subplot(3,1,3);
hold on;
% for i = 2:4 
%         plot(record_time_stamp(Cut),record_Output_channel_data(Cut,i))
% end

plot(record_time_stamp(Cut),record_com(Cut,2) ,"-",'LineWidth',LineWidth,'Color',LighGrayRed * 0.7);
plot(record_time_stamp(Cut),record_com(Cut,3) ,"-",'LineWidth',LineWidth,'Color',LighGrayGreen * 0.7);
plot(record_time_stamp(Cut),record_com(Cut,4) ,"-",'LineWidth',LineWidth,'Color',LighGrayBlue* 0.7);


y_tick = -1 : 1 : 1;
set(gca,'xtick',x_tick);
set(gca,'ytick',y_tick);
axis([x_tick(1) x_tick(end) y_tick(1) y_tick(end)]);
grid on;
h = gca;
set(h,'LineWidth',1,'GridLineStyle','--','GridAlpha',1,'GridColor',[0.9,0.9,0.9])
hold off;

Cut1 = 501 : 1200;
sample = psi(Cut1 - Cut(1));
max(sample)
rms(sample)