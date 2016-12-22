clc;
close all;
clear all;
 
% 
% 
% %% --------- Read .px4log and convert to .csv file then load to Matlab ---------------
% % log_file = '151221.Small_Size_Fixed_Wing_1.px4log';
% % log_file = '160107.Small_Size_Fixed_Wing_1.px4log';
log_file = '161219.LPE_with_VICION.px4log';
% log_file = '161222.UAV_2_Circle.px4log';

% 
% % log_file = '08_48_28.px4log';
% 
log_file_name = strsplit(log_file,'.');
data_file = strcat(log_file_name{1},'.csv');
delim = ',';
time_field = 'TIME';
csv_null = '';

if not(exist(data_file, 'file'))
    s = system( sprintf('python sdlog2_dump.py "%s" -f "%s" -t"%s" -d"%s" -n"%s"', log_file, data_file, time_field, delim, csv_null) );        
    sysvector = tdfread(data_file, ',');
    save(strcat(log_file_name{1}, '.', log_file_name{2},'.mat'), 'sysvector');
end
sysvector = load(strcat(log_file_name{1}, '.', log_file_name{2},'.mat'));
sysvector = sysvector.sysvector;

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
lat = sysvector.GPOS_Lat;
lon = sysvector.GPOS_Lon;
h = sysvector.GPOS_Alt;
wgs84 = wgs84Ellipsoid('meters');
[x,y,z] = geodetic2ecef(wgs84,lat,lon,h);


%% ---------- Draw converted GPS 3D Position ---------
fig(1) = figure(1);
subplot(2, 4, [1 2 5 6]);
% [x,y,z]=lla2ecef(sysvector.GPOS_Lat, sysvector.GPOS_Lon, sysvector.GPOS_Alt);



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
fig = figure(2);
p = get(fig,'position');
p(3) = p(3)*1.8;
p(4) = p(4)*2.0; % Add 10 percent to height
set(fig, 'position', p);
figNum = 9;
figIndex = 1;
subplot(figNum, 3, [figIndex figIndex+1 figIndex+2]);
shadingPeriod = find(sysvector.RC_C8 > -0.5);

hold on;
[ax h1 h2]=plotyy(time, sysvector.RC_C8, time,sysvector.STAT_MainState);
legend('Chanel 8','State');
xlabel('Time(s)');
ylim(ax(2),[-1,2]);
grid on;
hold on;

yAxisMax = max(max([sysvector.RC_C8, sysvector.STAT_MainState]));
yAxisMin = min(min([sysvector.RC_C8, sysvector.STAT_MainState]));
yAxisScale = yAxisMax - yAxisMin;
yAxisMax = yAxisMax  +  yAxisScale*0.2;
yAxisMin = yAxisMin  -  yAxisScale*0.2;


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

yAxisMax = max(max([sysvector.ATT_Roll, sysvector.ATT_Pitch]));
yAxisMin = min(min([sysvector.ATT_Roll, sysvector.ATT_Pitch]));
yAxisScale = yAxisMax - yAxisMin;
yAxisMax = yAxisMax  +  yAxisScale*0.2;
yAxisMin = yAxisMin  -  yAxisScale*0.2;


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

yAxisMax = max(max([sysvector.LPOS_X, sysvector.LPSP_X]));
yAxisMin = min(min([sysvector.LPOS_X, sysvector.LPSP_X]));
yAxisScale = yAxisMax - yAxisMin;
yAxisMax = yAxisMax  +  yAxisScale*0.2;
yAxisMin = yAxisMin  -  yAxisScale*0.2;

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

yAxisMax = max(max([sysvector.LPOS_Y, sysvector.LPSP_Y]));
yAxisMin = min(min([sysvector.LPOS_Y, sysvector.LPSP_Y]));
yAxisScale = yAxisMax - yAxisMin;
yAxisMax = yAxisMax  +  yAxisScale*0.2;
yAxisMin = yAxisMin  -  yAxisScale*0.2;

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

yAxisMax = max(max([sysvector.LPOS_Z, sysvector.LPSP_Z]));
yAxisMin = min(min([sysvector.LPOS_Z, sysvector.LPSP_Z]));
yAxisScale = yAxisMax - yAxisMin;
yAxisMax = yAxisMax  +  yAxisScale*0.2;
yAxisMin = yAxisMin  -  yAxisScale*0.2;


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

yAxisMax = max(max([sysvector.IMU_AccX, sysvector.IMU_AccY, sysvector.IMU_AccZ]));
yAxisMin = min(min([sysvector.IMU_AccX, sysvector.IMU_AccY, sysvector.IMU_AccZ]));
yAxisScale = yAxisMax - yAxisMin;
yAxisMax = yAxisMax  +  yAxisScale*0.2;
yAxisMin = yAxisMin  -  yAxisScale*0.2;


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

yAxisMax = max(max([sysvector.IMU_GyroX, sysvector.IMU_GyroY, sysvector.IMU_GyroZ]));
yAxisMin = min(min([sysvector.IMU_GyroX, sysvector.IMU_GyroY, sysvector.IMU_GyroZ]));
yAxisScale = yAxisMax - yAxisMin;
yAxisMax = yAxisMax  +  yAxisScale*0.2;
yAxisMin = yAxisMin  -  yAxisScale*0.2;


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

yAxisMax = max(max([sysvector.IMU_MagX, sysvector.IMU_MagY, sysvector.IMU_MagZ]));
yAxisMin = min(min([sysvector.IMU_MagX, sysvector.IMU_MagY, sysvector.IMU_MagZ]));
yAxisScale = yAxisMax - yAxisMin;
yAxisMax = yAxisMax  +  yAxisScale*0.2;
yAxisMin = yAxisMin  -  yAxisScale*0.2;

pyAltCtl = [ones(size(timeAltCtl'))*yAxisMin, ones(size(timeAltCtl'))*yAxisMax];
patch(pxAltCtl, pyAltCtl, 'k','EdgeColor','none');

pyPosCtl = [ones(size(timePosCtl'))*yAxisMin, ones(size(timePosCtl'))*yAxisMax];
patch(pxPosCtl, pyPosCtl, 'g','EdgeColor','none');

ylim([yAxisMin, yAxisMax]);
alpha(.15);



%% ---------- GPS fix/EPH/EPV -----------
% figIndex = figIndex + 3;
% subplot(figNum, 3, [figIndex figIndex+1 figIndex+2]);
% 
% 
% plot(time, [sysvector.GPOS_EPH, sysvector.GPOS_EPV, sysvector.GPOS_Fix],'LineWidth',1.5);
% legend('GPS.EPH','GPS.EPV','GPS.Fix');
% xlabel('Time(s)');
% grid on;
% hold on;
% 
% yAxisMax = max(max([sysvector.GPOS_EPH, sysvector.GPOS_EPV, sysvector.GPOS_Fix])) * 1.15;
% yAxisMin = min(min([sysvector.GPOS_EPH, sysvector.GPOS_EPV, sysvector.GPOS_Fix])) * 0.85;
% 
% pyAltCtl = [ones(size(timeAltCtl'))*yAxisMin, ones(size(timeAltCtl'))*yAxisMax];
% patch(pxAltCtl, pyAltCtl, 'k','EdgeColor','none');
% 
% pyPosCtl = [ones(size(timePosCtl'))*yAxisMin, ones(size(timePosCtl'))*yAxisMax];
% patch(pxPosCtl, pyPosCtl, 'g','EdgeColor','none');
% 
% ylim([yAxisMin, yAxisMax]);
% alpha(.15);

%% ----- IMU 1 -------
fig  = figure(3);
p = get(fig,'position');
p(3) = p(3)*1.8;
p(4) = p(4)*1.0; % Add 10 percent to height
set(fig, 'position', p);

figNum = 3;
figIndex = 1;
subplot(figNum, 3, [figIndex figIndex+1 figIndex+2]);

plot(time, [sysvector.IMU_AccX, sysvector.IMU_AccY, sysvector.IMU_AccZ],'LineWidth',1.5);
legend('IMU.AccX','IMU.AccY','IMU.AccZ');
xlabel('Time(s)');
grid on;
hold on;

yAxisMax = max(max([sysvector.IMU_AccX, sysvector.IMU_AccY, sysvector.IMU_AccZ]));
yAxisMin = min(min([sysvector.IMU_AccX, sysvector.IMU_AccY, sysvector.IMU_AccZ]));
yAxisScale = yAxisMax - yAxisMin;
yAxisMax = yAxisMax  +  yAxisScale*0.2;
yAxisMin = yAxisMin  -  yAxisScale*0.2;


pyAltCtl = [ones(size(timeAltCtl'))*yAxisMin, ones(size(timeAltCtl'))*yAxisMax];
patch(pxAltCtl, pyAltCtl, 'k','EdgeColor','none');

pyPosCtl = [ones(size(timePosCtl'))*yAxisMin, ones(size(timePosCtl'))*yAxisMax];
patch(pxPosCtl, pyPosCtl, 'g','EdgeColor','none');

ylim([yAxisMin, yAxisMax]);
alpha(.15);

%% ------- IMU1 Gyro -----------------
figIndex = figIndex + 3;
subplot(figNum, 3, [figIndex figIndex+1 figIndex+2]);

plot(time, [sysvector.IMU_GyroX, sysvector.IMU_GyroY, sysvector.IMU_GyroZ],'LineWidth',1.5);
legend('IMU.GyroX','IMU.GyroY','IMU.GyroZ');
xlabel('Time(s)');
grid on;
hold on;


yAxisMax = max(max([sysvector.IMU_GyroX, sysvector.IMU_GyroY, sysvector.IMU_GyroZ]));
yAxisMin = min(min([sysvector.IMU_GyroX, sysvector.IMU_GyroY, sysvector.IMU_GyroZ]));
yAxisScale = yAxisMax - yAxisMin;
yAxisMax = yAxisMax  +  yAxisScale*0.2;
yAxisMin = yAxisMin  -  yAxisScale*0.2;


pyAltCtl = [ones(size(timeAltCtl'))*yAxisMin, ones(size(timeAltCtl'))*yAxisMax];
patch(pxAltCtl, pyAltCtl, 'k','EdgeColor','none');

pyPosCtl = [ones(size(timePosCtl'))*yAxisMin, ones(size(timePosCtl'))*yAxisMax];
patch(pxPosCtl, pyPosCtl, 'g','EdgeColor','none');

ylim([yAxisMin, yAxisMax]);
alpha(.15);

figSaveName = strcat(log_file_name{1},'.fig');
savefig(fig, figSaveName);

figIndex = figIndex + 3;
subplot(figNum, 3, [figIndex figIndex+1 figIndex+2]);

plot(time, [sysvector.IMU_MagX, sysvector.IMU_MagY, sysvector.IMU_MagZ],'LineWidth',1.5);

xlabel('Time(s)');
grid on;
hold on;

IMU_MagNorm = sqrt(sysvector.IMU_MagX.^2 + sysvector.IMU_MagY.^2 + sysvector.IMU_MagZ.^2);
plot(time, IMU_MagNorm,'LineWidth',1.5);
legend('IMU.MagX','IMU.MagY','IMU.MagZ','IMU.MagNorm');
yAxisMax = max(max([sysvector.IMU_MagX, sysvector.IMU_MagY, sysvector.IMU_MagZ, IMU_MagNorm]));
yAxisMin = min(min([sysvector.IMU_MagX, sysvector.IMU_MagY, sysvector.IMU_MagZ, IMU_MagNorm]));
yAxisScale = yAxisMax - yAxisMin;
yAxisMax = yAxisMax  +  yAxisScale*0.2;
yAxisMin = yAxisMin  -  yAxisScale*0.2;

% pyAltCtl = [ones(size(timeAltCtl'))*yAxisMin, ones(size(timeAltCtl'))*yAxisMax];
% patch(pxAltCtl, pyAltCtl, 'k','EdgeColor','none');
% 
% pyPosCtl = [ones(size(timePosCtl'))*yAxisMin, ones(size(timePosCtl'))*yAxisMax];
% patch(pxPosCtl, pyPosCtl, 'g','EdgeColor','none');

% ylim([yAxisMin, yAxisMax]);
% alpha(.15);

%%
fig = figure(12); 
p = get(fig,'position');
p(3) = p(3)*1.8;
p(4) = p(4)*1.0; % Add 10 percent to height
set(fig, 'position', p);

subplot(2,1,1);
plot(time, [sysvector.RC_C1, sysvector.RC_C2, sysvector.RC_C3, sysvector.RC_C4],'LineWidth',1.5);
legend('RC1', 'RC2', 'RC3', 'RC4');
axis([min(time) max(time) -2 2]);
grid on;

subplot(2,1,2);
plot(time, [sysvector.RC_C5, sysvector.RC_C6, sysvector.RC_C7, sysvector.RC_C8],'LineWidth',1.5);
legend('RC5', 'RC6', 'RC7', 'RC8');
axis([min(time) max(time) -2 2]);
grid on;

%%
fig = figure(13);
p = get(fig,'position');
p(3) = p(3)*1.8;
p(4) = p(4)*1.0; % Add 10 percent to height
set(fig, 'position', p);

[hAx,hLine1,hLine2] = plotyy(time, sysvector.LPSP_X, time, sysvector.RC_C8);
hold on;
hLine1.LineWidth = 1.5;
hLine2.LineWidth = 1.5;
hLine1.Marker = 'o';
% hLine2.Marker = 'o';
pl1 = plot(time, sysvector.LPOS_X,'o-r','LineWidth',1.5,'Color',[0.4940    0.1840    0.5560]);

yAxisMax = max(max([sysvector.LPOS_Z, sysvector.LPSP_Z]));
yAxisMin = min(min([sysvector.LPOS_Z, sysvector.LPSP_Z])) ;
yAxisScale = yAxisMax - yAxisMin;
yAxisMax = yAxisMax  +  yAxisScale*0.2;
yAxisMin = yAxisMin  -  yAxisScale*0.2;
set(hAx(1),'yLim',[yAxisMin yAxisMax],'YTick', yAxisMin:yAxisMax);
set(hAx(2),'YLim',[-2 2])
grid on;
legend([hLine1, pl1,hLine2], { 'LPSP_X','LPOS_X','RC 8'});

%%
fig = figure(14);
p = get(fig,'position');
p(3) = p(3)*1.8;
p(4) = p(4)*1.0; % Add 10 percent to height
set(fig, 'position', p);
pl1 = plot(time, sysvector.LPOS_Y,'o-r','LineWidth',1.5,'Color',[0.4940    0.1840    0.5560]);
hold on;
[hAx,hLine1,hLine2] = plotyy(time, sysvector.LPSP_Y, time, sysvector.RC_C8);
hold on;
hLine1.LineWidth = 1.5;
hLine2.LineWidth = 1.5;
hLine1.Marker = 'o';
% hLine2.Marker = 'o';

yAxisMax = max(max([sysvector.LPOS_Y, sysvector.LPSP_Y]));
yAxisMin = min(min([sysvector.LPOS_Y, sysvector.LPSP_Y]));
yAxisScale = yAxisMax - yAxisMin;
yAxisMax = yAxisMax  +  yAxisScale*0.2;
yAxisMin = yAxisMin  -  yAxisScale*0.2;
set(hAx(2),'YLim',[-2 2])
set(hAx(1),'yLim',[yAxisMin yAxisMax],'YTick', yAxisMin:yAxisMax);

grid on;
legend([hLine1, pl1,hLine2], { 'LPSP_Y','LPOS_Y','RC 8'});

%%
fig = figure(15);
p = get(fig,'position');
p(3) = p(3)*1.8;
p(4) = p(4)*1.0; % Add 10 percent to height
set(fig, 'position', p);
[hAx,hLine1,hLine2] = plotyy(time, sysvector.LPSP_Z, time, sysvector.RC_C8);
hold on;
hLine1.LineWidth = 1.5;
hLine2.LineWidth = 1.5;
hLine1.Marker = 'o';
% hLine2.Marker = '-';
pl1 = plot(time, sysvector.LPOS_Z,'o-','LineWidth',1.5,'Color',[0.4940    0.1840    0.5560]);

yAxisMax = max(max([sysvector.LPOS_Z, sysvector.LPSP_Z]));
yAxisMin = min(min([sysvector.LPOS_Z, sysvector.LPSP_Z]));
yAxisScale = yAxisMax - yAxisMin;
yAxisMax = yAxisMax  +  yAxisScale*0.2;
yAxisMin = yAxisMin  -  yAxisScale*0.2;
set(hAx(1),'yLim',[yAxisMin yAxisMax],'YTick', yAxisMin:yAxisMax);
set(hAx(2),'YLim',[-2 2])
grid on;
legend([hLine1, pl1,hLine2], { 'LPSP_Z','LPOS_Z','RC 8'});


