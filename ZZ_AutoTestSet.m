%%%%% 仿真设置，如初始时间、轨道、姿态、模式等  %%%%%%
function [] = ZZ_AutoTestSet(RunCount)
DEG2RAD = pi/180;
%% 动力学初始参数设置
global Reset0 Time0 Orbit0 Agl0 Wbi0 Spread0 Fw_Spd0 SADA_Agl0 QV_Agl0;
if RunCount == 1% 动力学初始化在这里设置

    Time0 = [2026 1 26 04 00 00]';% 年月日时分秒（UTC）
    Orbit0 = [6927.57e3;0.000979676;97.5746*DEG2RAD;215.45*DEG2RAD;261.334*DEG2RAD;98.8067*DEG2RAD];% 初始轨道参数 a/e/i/OMG/omg/f

    Agl0 = 0*[20;-10;10]/180*pi; %
    Wbi0 = [0;-0.0628;0]/180*pi;
    SADA_Agl0 = 1*[0;0;0;0;0;0]; % 动力学SADA角度初值，deg X Y
    QV_Agl0 = [0;0;0;0];
    Spread0(1:2) = 1*[1;1];   %动力学展开标志，阻尼怎么设置？
   
elseif RunCount == 2
    Reset0 = hex2dec('22');
end
%% 软件参数设置
global LcMark SadaParam Tc_StUse_Enable Tc_SadaCtrl_Enable Tc_QVCtrl_Enable Tc_NotchFilter_Enable ...
    Tc_TdCal_Enable Tc_TdFdFwd_Enable Tc_AglAccFdFwd_Enable Tc_Index Tc_SadaCtrl_Cmd Tc_Engine Tc_MagCtrlAlgthm;
global Tc_StareCtrl_Enable Tc_Stare_LLA;%deg/deg/m
global Tc_ZeroDplCtrl_Enable Tc_ZeroDpl_IncidAgl;%deg
global Tc_PLJCtrl_Enable Tc_PLJ_DB_Enable Tc_PLJ_TgtAgl Tc_StChoiceIndex ;
global Tc_Aligned_and_Constrained;
global Tc_QVWork_LLA Tc_QVWork_Time;
if RunCount == 2
     Tc_TdCal_Enable = 1*[1;1;1;1;1;1];  % 力矩计算/前馈开关:1光压/2气动/3重力梯度/4磁控/5陀螺/6 SADA驱动
    Tc_TdFdFwd_Enable = 1*[1;1;1;1;1;1];
    Tc_AglAccFdFwd_Enable = 1;    % 标称系角加速度前馈
    Tc_NotchFilter_Enable = 0;    % 结构滤波器
    LcMark.Mode = 3;
    LcMark.SubMode = 1;
    SadaParam.Spread = 1*[1;1];  %星上展开标志
    Tc_StUse_Enable = 1;
    Tc_SadaCtrl_Enable = 0*[1;1;1;1];%顺序为a1/p1/a2/p2,行云对应就是X1/Y1/X2/Y2
    Tc_QVCtrl_Enable = 0;
    Tc_StareCtrl_Enable   = 0; Tc_Stare_LLA = [-56;-70;45];     % 凝视     经纬高 deg/deg/m
    Tc_ZeroDplCtrl_Enable = 0; Tc_ZeroDpl_IncidAgl = 0;         % 零多普勒 下视角 deg
    Tc_PLJCtrl_Enable     = 0; Tc_PLJ_TgtAgl = [0;0;0];         % 偏流角   侧摆目标角，123转序，deg
    Tc_Aligned_and_Constrained = 0;    % 约束对日开关

elseif RunCount == 40/0.1
    Tc_Index = [6;0];
%     LcMark.Mode = 5; 
%     LcMark.SubMode = 1;
%     Tc_Index = [0;7];%0-轨道系 1-惯性系 2-凝视 3-零多普勒 4-偏流角
%     Spread0(1:2) = [1;0];
    % 0不控、1待机、2停转、3闭环+角度、4开环+角速度、5增量+增量角、6归零+方向？  99星上自主  
%     Tc_SadaCtrl_Cmd=[4 0.6;6 1;6 -1;1 0];% 模式(默认99星上自主1待2)+参数 apap  单位：deg
%     Tc_StChoiceIndex = {'StA'};
end
end