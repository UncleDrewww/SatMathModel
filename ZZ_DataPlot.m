%-----------------------------------------------------------------------------------------------------------------------------------                      
%                                                      画图脚本
%-----------------------------------------------------------------------------------------------------------------------------------
close all;clc;
DEG2RAD = pi/180;RAD2DEG = 180/pi;

%% 单图模式
plotDataCell = {};  % 预先创建一个空 cell
%----------------------------------------------------------------------------------------------------------------------------------------------------------------
% 对所需的数据进行画图（单图），需要传递的参数包括：数据（需做单位处理）、xlabel、ylabel、title、legend、y轴限幅（无限幅就填空矩阵），按照示例语句赋值在后面即可。
% 数据组包顺序参考ProjectModel/TestModel/TMDisplay。
%----------------------------------------------------------------------------------------------------------------------------------------------------------------
plotDataCell = addPlotData(plotDataCell, TM_Mode, '时间(s)', '控制模式', '控制模式',{'模式','阶段'},[]);
plotDataCell = addPlotData(plotDataCell, TM_Ctrl(:,10:12)*RAD2DEG, '时间(s)', '角度(\circ)','控制用姿态角',{'x','y','z'},[-0.1,0.1]);
plotDataCell = addPlotData(plotDataCell, TM_Ctrl(:,13:15)*RAD2DEG, '时间(s)', '角速度(\circ/s)','控制用姿态角速度',{'x','y','z'},[]);
plotDataCell = addPlotData(plotDataCell, TM_Fw(:,1:6), '时间(s)', '转速(rpm)','飞轮指令转速',{'a','b','c','d','e','f'},[]);
plotDataCell = addPlotData(plotDataCell, TM_Mt(:,1:6), '时间(s)', '磁矩(Am^2)','磁棒指令磁矩',{'a','b','c','d','e','f'},[]);
plotDataCell = addPlotData(plotDataCell, TM_SADA(:,19:3:36)*RAD2DEG, '时间(s)', '角度(\circ)','SADA角度',{'+Y A','+Y B','-Y A','-Y B','展开 A','展开 B'},[]);
plotDataCell = addPlotData(plotDataCell, TM_SADA(:,20:3:37)*RAD2DEG, '时间(s)', '角速度(\circ/s)','SADA角速度',{'+Y A','+Y B','-Y A','-Y B','展开 A','展开 B'},[]);
plotDataCell = addPlotData(plotDataCell, TM_Dyn(:,[10,11,12,14,15,18]), '时间(s)', '惯量(kgm^2)','转动惯量',{'Ix','Ixy','Ixz','Iy','Iyz','Iz'},[]);
plotDataCell = addPlotData(plotDataCell, TM_Dyn(:,19:21), '时间(s)', '质心(m)','质心',{'x','y','z'},[]);
plotDataCell = addPlotData(plotDataCell, TM_Dyn(:,22:24), '时间(s)','太阳矢量','本体系太阳矢量',{'x','y','z'},[]);
plotDataCell = addPlotData(plotDataCell, TM_Dyn(:,49:54), '时间(s)','模态坐标','模态坐标',{'1st','2nd','3rd','4th','5th','6th'},[]);
plotDataCell = addPlotData(plotDataCell, TM_Dyn(:,43:45), '时间(s)','力矩(Nm)','惯性系累计干扰力矩',{'x','y','z'},[]);

% 循环画图
for i = 1:length(plotDataCell)
    figure;
    plot(TM_SimuTime,plotDataCell{i}.data,'LineWidth',2);  
    grid on;
    xlabel(plotDataCell{i}.xlabel);
    ylabel(plotDataCell{i}.ylabel);
    title(plotDataCell{i}.title);
    legend(plotDataCell{i}.legend);
    % 调整纵坐标范围，防止看不见曲线
    yData = plotDataCell{i}.data;
    ymin = min(yData(:));
    ymax = max(yData(:));
    if isempty(plotDataCell{i}.ylim)        
        if (ymax == ymin)
            buffer = 0.1;  % 设定一个默认的 buffer
        else
            buffer = 0.1 * (ymax - ymin);
        end
        ylim([ymin - buffer, ymax + buffer]);
    else
        ylim(plotDataCell{i}.ylim);
    end
end

%% 子图模式
figure;
subplot(2,1,1);
plot(TM_SimuTime,TM_Dyn(:,31:33),'LineWidth',2);
grid on;title('重力梯度力矩');xlabel('时间(s)');ylabel('力矩(Nm)');legend('x','y','z');
subplot(2,1,2);
plot(TM_SimuTime,TM_Dyn(:,28:30),'LineWidth',2);
grid on;title('剩磁力矩');xlabel('时间(s)');ylabel('力矩(Nm)');legend('x','y','z');

figure;
subplot(2,1,1);
plot(TM_SimuTime,TM_Dyn(:,34:36),'LineWidth',2);
grid on;title('气动力矩');xlabel('时间(s)');ylabel('力矩(Nm)');legend('x','y','z');
subplot(2,1,2);
plot(TM_SimuTime,TM_Dyn(:,37:39),'LineWidth',2);
grid on;title('光压力矩');xlabel('时间(s)');ylabel('力矩(Nm)');legend('x','y','z');

figure;
subplot(2,1,1);
plot(TM_SimuTime,TM_Dyn(:,40:42),'LineWidth',2);
grid on;title('电推干扰力矩');xlabel('时间(s)');ylabel('力矩(Nm)');legend('x','y','z');
subplot(2,1,2);
plot(TM_SimuTime,TM_Dyn(:,46:48),'LineWidth',2);
grid on;title('电机干扰力矩');xlabel('时间(s)');ylabel('力矩(Nm)');legend('x','y','z');

%% 构建Cell
function plotDataCell = addPlotData(plotDataCell, data, xlabelText, ylabelText, titleText, legendText,ylimText)
    plotDataCell{end+1} = struct('data', data, ...
                      'xlabel', xlabelText, ...
                      'ylabel', ylabelText, ...
                      'title', titleText, ...
                      'legend', {legendText}, ...
                      'ylim',ylimText);     
end