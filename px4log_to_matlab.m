clc;
clear all;
close all;


%% --------- Read .px4log and convert to .csv file then load to Matlab ---------------
log_file = '07_49_41.px4log';

log_file_name = strsplit(log_file,'.');
data_file = strcat(log_file_name{1},'.csv');
delim = ',';
time_field = 'TIME';
csv_null = '';

if not(exist(data_file, 'file'))
    s = system( sprintf('python sdlog2_dump.py "%s" -f "%s" -t"%s" -d"%s" -n"%s"', log_file, data_file, time_field, delim, csv_null) );    
end
sysvector = tdfread(data_file, ',');

%% --------- Convert the GPS time from "ms" to "s"
fconv_timestamp=1E-6; % [microseconds] to [seconds]
time=double(sysvector.TIME_StartTime).*fconv_timestamp;


%% ---------- Get Different Control State ------------
stateManual = find(sysvector.STAT_MainState == 0);

stateAltCtl = find(sysvector.STAT_MainState == 1);
timeAltCtl = time(stateAltCtl);
pxAltCtl = [timeAltCtl', fliplr(timeAltCtl')];

statePosCtl = find(sysvector.STAT_MainState == 2);
timePosCtl = time(statePosCtl);
pxPosCtl = [timePosCtl', fliplr(timePosCtl')];

%% ------- Convert GPS Lat/Lon/Alt to X/Y/Z --------
lat = sysvector.GPS_Lat;
lon = sysvector.GPS_Lon;
h = sysvector.GPS_Alt;
wgs84 = wgs84Ellipsoid('meters');
[x,y,z] = geodetic2ecef(wgs84,lat,lon,h);


%% ---------- Draw converted GPS 3D Position ---------
fig(1) = figure(1)
subplot(2, 4, [1 2 5 6]);
% [x,y,z]=lla2ecef(sysvector.GPS_Lat, sysvector.GPS_Lon, sysvector.GPS_Alt);



% for i=1:size(x)
%     switch sysvector.STAT_MainState(i)
%         case 0
%             plot3(x(i),y(i),z(i),'b.'); drawnow; hold on;
%         case 1
%             plot3(x(i),y(i),z(i),'k.');  drawnow; hold on;
%         case 2
%             plot3(x(i),y(i),z(i),'g.');  drawnow; hold on;
%     end    
% end

plot3(x,y,z);
hold on;

plot3(x(2), y(2), z(2),'rs');
text(x(2), y(2), z(2),'   Start','HorizontalAlignment','left','FontSize',8);
hold on;

plot3(x(end), y(end), z(end), 'ro');
text(x(end), y(end), z(end), '   End','HorizontalAlignment','left','FontSize',8);
grid on;



%% ----------- Draw converted GPS 2D Position ----------
subplot(2, 4, [3 4 7 8]);
axis equal;

% for i=1:size(x)
%     switch sysvector.STAT_MainState(i)
%         case 0
%             plot(x(i),y(i),'b.'); hold on;
%         case 1
%             plot(x(i),y(i),'k.'); hold on;
%         case 2
%             plot(x(i),y(i),'g.'); hold on;
%     end    
% end

s = scatter(x(stateManual),y(stateManual),12,'filled');
s.MarkerEdgeColor = [32 178 170]/255;
s.MarkerFaceColor = [32 178 170]/255;
s.Marker = 'o';

hold on;

s = scatter(x(stateAltCtl),y(stateAltCtl),12,'filled');
s.Marker = 'o';
s.MarkerEdgeColor = [0.7 0.7 0.7];
s.MarkerFaceColor = [0.7 0.7 0.7];
% s.FaceAlpha = 0.4;
hold on;

% s = scatter(x(statePosCtl),y(statePosCtl),12,'filled');
% s.Marker = 'o';
% s.MarkerEdgeColor = [0 139 69]/255;
% s.MarkerFaceColor = [0 139 60]/255;

plot(x(statePosCtl),y(statePosCtl),'-o','color',[0 139 69]/255,...
    'MarkerSize',4,'MarkerFaceColor',[0 139 69]/255, 'MarkerEdgeColor','none');
hold on;

plot(x(2), y(2), 'rs');
text(x(2), y(2),'   Start','HorizontalAlignment','left','FontSize',8);
hold on;

plot(x(end), y(end), 'ro');
text(x(end), y(end), '   End','HorizontalAlignment','left','FontSize',8);

grid on;
hold on;


%% RC Chanel 5 & Main State
fig(2) = figure(2);

figNum = 9;
figIndex = 1;
subplot(figNum, 3, [figIndex figIndex+1 figIndex+2]);
shadingPeriod = find(sysvector.RC_Ch5 > -0.5);

hold on;
[ax h1 h2]=plotyy(time, sysvector.RC_Ch5, time,sysvector.STAT_MainState);
legend('Chanel 5','State');
xlabel('Time(s)');
ylim(ax(2),[-1,2]);
grid on;
hold on;

yAxisMax = max(max([sysvector.RC_Ch5, sysvector.STAT_MainState])) * 1.15;
yAxisMin = min(min([sysvector.RC_Ch5, sysvector.STAT_MainState])) * 1.15;

pyAltCtl = [ones(size(timeAltCtl'))*yAxisMin, ones(size(timeAltCtl'))*yAxisMax];
patch(pxAltCtl, pyAltCtl, 'k','EdgeColor','none');

pyPosCtl = [ones(size(timePosCtl'))*yAxisMin, ones(size(timePosCtl'))*yAxisMax];
patch(pxPosCtl, pyPosCtl, 'g','EdgeColor','none');

ylim([yAxisMin, yAxisMax]);
alpha(.15);


%% -------- Attitude Roll and Pitch -----------
figIndex = figIndex + 3;
subplot(figNum, 3, [figIndex figIndex+1 figIndex+2]);
% subplot(4, 3, [4 5 6]);

plot(time, [sysvector.ATT_Roll, sysvector.ATT_Pitch],'LineWidth',1.5);
legend('roll','pitch');
xlabel('Time(s)');
grid on;
hold on;

yAxisMax = max(max([sysvector.ATT_Roll, sysvector.ATT_Pitch])) * 1.15;
yAxisMin = min(min([sysvector.ATT_Roll, sysvector.ATT_Pitch])) * 1.15;

pyAltCtl = [ones(size(timeAltCtl'))*yAxisMin, ones(size(timeAltCtl'))*yAxisMax];
patch(pxAltCtl, pyAltCtl, 'k','EdgeColor','none');

pyPosCtl = [ones(size(timePosCtl'))*yAxisMin, ones(size(timePosCtl'))*yAxisMax];
patch(pxPosCtl, pyPosCtl, 'g','EdgeColor','none');

ylim([yAxisMin, yAxisMax]);
alpha(.15);

%% ------------ Local Position X ----
% subplot(4, 3, [7 8 9]);
figIndex = figIndex + 3;
subplot(figNum, 3, [figIndex figIndex+1 figIndex+2]);

plot(time, [sysvector.LPOS_X, sysvector.LPSP_X],'LineWidth',1.5);
legend('local X','local SP X');
xlabel('Time(s)');
grid on;
hold on;

yAxisMax = max(max([sysvector.LPOS_X, sysvector.LPSP_X])) * 1.15;
yAxisMin = min(min([sysvector.LPOS_X, sysvector.LPSP_X])) * 1.15;

pyAltCtl = [ones(size(timeAltCtl'))*yAxisMin, ones(size(timeAltCtl'))*yAxisMax];
patch(pxAltCtl, pyAltCtl, 'k','EdgeColor','none');

pyPosCtl = [ones(size(timePosCtl'))*yAxisMin, ones(size(timePosCtl'))*yAxisMax];
patch(pxPosCtl, pyPosCtl, 'g','EdgeColor','none');

ylim([yAxisMin, yAxisMax]);
alpha(.15);

%% ---------- Local Position Y --------
figIndex = figIndex + 3;
subplot(figNum, 3, [figIndex figIndex+1 figIndex+2]);

plot(time, [sysvector.LPOS_Y, sysvector.LPSP_Y],'LineWidth',1.5);
legend('local Y','local SP Y');
xlabel('Time(s)');
grid on;
hold on;

yAxisMax = max(max([sysvector.LPOS_Y, sysvector.LPSP_Y])) * 1.15;
yAxisMin = min(min([sysvector.LPOS_Y, sysvector.LPSP_Y])) * 1.15;

pyAltCtl = [ones(size(timeAltCtl'))*yAxisMin, ones(size(timeAltCtl'))*yAxisMax];
patch(pxAltCtl, pyAltCtl, 'k','EdgeColor','none');

pyPosCtl = [ones(size(timePosCtl'))*yAxisMin, ones(size(timePosCtl'))*yAxisMax];
patch(pxPosCtl, pyPosCtl, 'g','EdgeColor','none');

ylim([yAxisMin, yAxisMax]);
alpha(.15);

%% ---------- Local Position Z --------
figIndex = figIndex + 3;
subplot(figNum, 3, [figIndex figIndex+1 figIndex+2]);

plot(time, [sysvector.LPOS_Z, sysvector.LPSP_Z],'LineWidth',1.5);
legend('local Z','local SP Z');
xlabel('Time(s)');
grid on;
hold on;

yAxisMax = max(max([sysvector.LPOS_Z, sysvector.LPSP_Z])) * 1.15;
yAxisMin = min(min([sysvector.LPOS_Z, sysvector.LPSP_Z])) * 1.15;

pyAltCtl = [ones(size(timeAltCtl'))*yAxisMin, ones(size(timeAltCtl'))*yAxisMax];
patch(pxAltCtl, pyAltCtl, 'k','EdgeColor','none');

pyPosCtl = [ones(size(timePosCtl'))*yAxisMin, ones(size(timePosCtl'))*yAxisMax];
patch(pxPosCtl, pyPosCtl, 'g','EdgeColor','none');

ylim([yAxisMin, yAxisMax]);
alpha(.15);

%% ------------ IMU Acc--------------
figIndex = figIndex + 3;
subplot(figNum, 3, [figIndex figIndex+1 figIndex+2]);

plot(time, [sysvector.IMU_AccX, sysvector.IMU_AccY, sysvector.IMU_AccZ],'LineWidth',1.5);
legend('IMU.AccX','IMU.AccY','IMU.AccZ');
xlabel('Time(s)');
grid on;
hold on;

yAxisMax = max(max([sysvector.IMU_AccX, sysvector.IMU_AccY, sysvector.IMU_AccZ])) * 1.15;
yAxisMin = min(min([sysvector.IMU_AccX, sysvector.IMU_AccY, sysvector.IMU_AccZ])) * 1.15;

pyAltCtl = [ones(size(timeAltCtl'))*yAxisMin, ones(size(timeAltCtl'))*yAxisMax];
patch(pxAltCtl, pyAltCtl, 'k','EdgeColor','none');

pyPosCtl = [ones(size(timePosCtl'))*yAxisMin, ones(size(timePosCtl'))*yAxisMax];
patch(pxPosCtl, pyPosCtl, 'g','EdgeColor','none');

ylim([yAxisMin, yAxisMax]);
alpha(.15);

%% ------- IMU Gyro -----------------
figIndex = figIndex + 3;
subplot(figNum, 3, [figIndex figIndex+1 figIndex+2]);

plot(time, [sysvector.IMU_GyroX, sysvector.IMU_GyroY, sysvector.IMU_GyroZ],'LineWidth',1.5);
legend('IMU.GyroX','IMU.GyroY','IMU.GyroZ');
xlabel('Time(s)');
grid on;
hold on;

yAxisMax = max(max([sysvector.IMU_GyroX, sysvector.IMU_GyroY, sysvector.IMU_GyroZ])) * 1.15;
yAxisMin = min(min([sysvector.IMU_GyroX, sysvector.IMU_GyroY, sysvector.IMU_GyroZ])) * 1.15;

pyAltCtl = [ones(size(timeAltCtl'))*yAxisMin, ones(size(timeAltCtl'))*yAxisMax];
patch(pxAltCtl, pyAltCtl, 'k','EdgeColor','none');

pyPosCtl = [ones(size(timePosCtl'))*yAxisMin, ones(size(timePosCtl'))*yAxisMax];
patch(pxPosCtl, pyPosCtl, 'g','EdgeColor','none');

ylim([yAxisMin, yAxisMax]);
alpha(.15);

%% ------- IMU Gyro -----------------
figIndex = figIndex + 3;
subplot(figNum, 3, [figIndex figIndex+1 figIndex+2]);

plot(time, [sysvector.IMU_MagX, sysvector.IMU_MagY, sysvector.IMU_MagZ],'LineWidth',1.5);
legend('IMU.MagX','IMU.MagY','IMU.MagZ');
xlabel('Time(s)');
grid on;
hold on;

yAxisMax = max(max([sysvector.IMU_MagX, sysvector.IMU_MagY, sysvector.IMU_MagZ])) * 1.15;
yAxisMin = min(min([sysvector.IMU_MagX, sysvector.IMU_MagY, sysvector.IMU_MagZ])) * 1.15;

pyAltCtl = [ones(size(timeAltCtl'))*yAxisMin, ones(size(timeAltCtl'))*yAxisMax];
patch(pxAltCtl, pyAltCtl, 'k','EdgeColor','none');

pyPosCtl = [ones(size(timePosCtl'))*yAxisMin, ones(size(timePosCtl'))*yAxisMax];
patch(pxPosCtl, pyPosCtl, 'g','EdgeColor','none');

ylim([yAxisMin, yAxisMax]);
alpha(.15);



%% ---------- GPS fix/EPH/EPV -----------
figIndex = figIndex + 3;
subplot(figNum, 3, [figIndex figIndex+1 figIndex+2]);


plot(time, [sysvector.GPS_EPH, sysvector.GPS_EPV, sysvector.GPS_Fix],'LineWidth',1.5);
legend('GPS.EPH','GPS.EPV','GPS.Fix');
xlabel('Time(s)');
grid on;
hold on;

yAxisMax = max(max([sysvector.GPS_EPH, sysvector.GPS_EPV, sysvector.GPS_Fix])) * 1.15;
yAxisMin = min(min([sysvector.GPS_EPH, sysvector.GPS_EPV, sysvector.GPS_Fix])) * 0.85;

pyAltCtl = [ones(size(timeAltCtl'))*yAxisMin, ones(size(timeAltCtl'))*yAxisMax];
patch(pxAltCtl, pyAltCtl, 'k','EdgeColor','none');

pyPosCtl = [ones(size(timePosCtl'))*yAxisMin, ones(size(timePosCtl'))*yAxisMax];
patch(pxPosCtl, pyPosCtl, 'g','EdgeColor','none');

ylim([yAxisMin, yAxisMax]);
alpha(.15);

%% ----- IMU 1 -------
fig(3) = figure(3);
figNum = 2;
figIndex = 1;
subplot(figNum, 3, [figIndex figIndex+1 figIndex+2]);

plot(time, [sysvector.IMU_AccX, sysvector.IMU_AccY, sysvector.IMU_AccZ],'LineWidth',1.5);
legend('IMU.AccX','IMU.AccY','IMU.AccZ');
xlabel('Time(s)');
grid on;
hold on;

yAxisMax = max(max([sysvector.IMU_AccX, sysvector.IMU_AccY, sysvector.IMU_AccZ])) * 1.15;
yAxisMin = min(min([sysvector.IMU_AccX, sysvector.IMU_AccY, sysvector.IMU_AccZ])) * 1.15;

pyAltCtl = [ones(size(timeAltCtl'))*yAxisMin, ones(size(timeAltCtl'))*yAxisMax];
patch(pxAltCtl, pyAltCtl, 'k','EdgeColor','none');

pyPosCtl = [ones(size(timePosCtl'))*yAxisMin, ones(size(timePosCtl'))*yAxisMax];
patch(pxPosCtl, pyPosCtl, 'g','EdgeColor','none');

ylim([yAxisMin, yAxisMax]);
alpha(.15);

%% ------- IMU1 Gyro -----------------
figIndex = figIndex + 3;
subplot(figNum, 3, [figIndex figIndex+1 figIndex+2]);

plot(time, [sysvector.IMU1_GyroX, sysvector.IMU1_GyroY, sysvector.IMU1_GyroZ],'LineWidth',1.5);
legend('IMU.GyroX','IMU.GyroY','IMU.GyroZ');
xlabel('Time(s)');
grid on;
hold on;

yAxisMax = max(max([sysvector.IMU1_GyroX, sysvector.IMU1_GyroY, sysvector.IMU1_GyroZ])) * 1.15;
yAxisMin = min(min([sysvector.IMU1_GyroX, sysvector.IMU1_GyroY, sysvector.IMU1_GyroZ])) * 1.15;

pyAltCtl = [ones(size(timeAltCtl'))*yAxisMin, ones(size(timeAltCtl'))*yAxisMax];
patch(pxAltCtl, pyAltCtl, 'k','EdgeColor','none');

pyPosCtl = [ones(size(timePosCtl'))*yAxisMin, ones(size(timePosCtl'))*yAxisMax];
patch(pxPosCtl, pyPosCtl, 'g','EdgeColor','none');

ylim([yAxisMin, yAxisMax]);
alpha(.15);

figSaveName = strcat(log_file_name{1},'.fig');
savefig(fig, figSaveName);




