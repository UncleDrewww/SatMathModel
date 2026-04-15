function y = ZZ_SystemApp(u)
persistent RunCount;
if isempty(RunCount)
    RunCount = 0;
    
end
RunCount = RunCount + 1;
if RunCount == 1
    ZZ_SoftwareInit(RunCount); % 软件中全局变量初始化
end

global StParam GyroParam SadaParam MmParam AssParam FwParam CamParam QVParam OrbitElement Tc_F107 Ass_kS SadaCtrlCoeff QVCtrlCoeff Tc_Engine;
global Reset0 Time0 Orbit0 Agl0 Wbi0 Spread0 Fw_Spd0 SADA_Agl0 QV_Agl0 EnvirParam ;

% OrbitElement.Qob = u(1:4);
% Wbo = u(5:7);
Td_dyn = u(5:7);
%StTime:8/9
StParam.StA.Ok = u(10);
StParam.StA.Qis = u(11:14);
StParam.StA.Wsi_s = u(15:17);
StParam.StB.Ok = u(20);
StParam.StB.Qis = u(21:24);
StParam.StB.Wsi_s = u(25:27);
StParam.StC.Ok = u(30);
StParam.StC.Qis = u(31:34);
StParam.StC.Wsi_s = u(35:37);
StParam.StD.Ok = u(40);
StParam.StD.Qis = u(41:44);
StParam.StD.Wsi_s = u(45:47);

%StC/D：28:47
OrbitElement.Qio = u(48:51);
% AscDes_Sun = u(52:53);
OrbitElement.a = u(54);
OrbitElement.e = u(55);
OrbitElement.i = u(56);
OrbitElement.OMG = u(57);
OrbitElement.omg = u(58);
OrbitElement.f = u(59);
OrbitElement.M = u(60);
OrbitElement.E = u(61);
OrbitElement.w0 = u(62);
OrbitElement.Period = u(63);
GyroParam.GyroA.Wgi = u(64:66)/180*pi;
GyroParam.GyroB.Wgi = u(67:69)/180*pi;
%GyroC/D：70:75
% I3 = u(76:78);
t_ctrl = u(79);

SadaParam.Sada1.Agl = u(80:81);  % 【摆动 转动】rad
SadaParam.Sada2.Agl = u(82:83);  %
SadaParam.Sada3.Agl = u(84:85);  % 展开电机
QVParam.QV1.Agl = u(86:87);
QVParam.QV2.Agl = u(88:89);
UTC_1970 = u(90);
OrbitElement.Time = UTC_1970;
MmParam.MmA.Bm = u(91:93);
MmParam.MmB.Bm = u(94:96);
%MmC/D 97:102
OrbitElement.RV_i = u(103:108);
OrbitElement.RV_f = u(109:114);
OrbitElement.Qif = u(115:118);

FwParam.FwSpd = u(120:6:154);
AssParam.AssA.AssVolt = u(155:158)/Ass_kS;  %电压归一化，放在这儿合适不？
AssParam.AssB.AssVolt = u(159:162)/Ass_kS;
AssParam.AssC.AssVolt = u(163:166)/Ass_kS;
AssParam.AssD.AssVolt = u(167:170)/Ass_kS;

%AssE/F:171:178
% Data_Ass = u(155:178);
% Data_Gnss = u(179:197);
SadaParam.Sada1.HLState = [u(198:199);u(204:205)];  %SADA1A HL1 SADA1B HL1 SADA1A HL2 SADA1B HL2
SadaParam.Sada2.HLState = [u(200:201);u(206:207)];  %SADA2A HL1 SADA2B HL1 SADA2A HL2 SADA2B HL2
SadaParam.Sada3.HLState = [u(202:203);u(208:209)];  %SADA3A HL1 SADA2B HL1 SADA2A HL2 SADA2B HL2
OrbitElement.Sunshine = u(210);  %光照阴影区


ZZ_AutoTestSet(RunCount);  % 初始化参数设置，如初始时间、轨道、姿态、模式等
if Reset0 ~= hex2dec('33')
    [EnvirParam.R_Sun,EnvirParam.Asc_Sun, EnvirParam.Dec_Sun,EnvirParam.R_Sun_long] = SunParamCal((UTC_1970-946728000)/86400/36525);% R_Sun为单位矢量
    EnvirParam.Beta = pi/2-acos([0 -1 0]*(min(1,max(-1,quat2dcm(OrbitElement.Qio')*EnvirParam.R_Sun)))); % 太阳beta角（与轨道面夹角，太阳在-Y侧beta角为正）
    EnvirParam.gama_Sun = SunGamaCal(OrbitElement.RV_i(1:3),EnvirParam.R_Sun);
    EnvirParam.AirDensity = AirDensityCal(OrbitElement.RV_i(1:3),EnvirParam.Asc_Sun,EnvirParam.Dec_Sun,Tc_F107);
    EnvirParam.LocalTimeOfDesNode = mod(OrbitElement.OMG-EnvirParam.Asc_Sun,2*pi)*12/pi;  %简化版地方时计算
    % disp(1)
    [Att4Ctrl, FwSpd, FwT, Tw3, Mt_M,M3, NominalCSYS,Td] = ...
        ZZ_Att(UTC_1970,EnvirParam,OrbitElement,StParam,GyroParam,AssParam,MmParam,SadaParam,CamParam,Td_dyn,t_ctrl,RunCount);

    [SadaInstrCmd, SadaParam,Agl_Sada_Tgt] = ZZ_Sada(EnvirParam,NominalCSYS,SadaParam,OrbitElement,RunCount,t_ctrl);

%     QVInstrCmd = zeros(8,1);
    [QVInstrCmd,QVParam,Agl_QV_Tgt,Agl_QV_Ctrl] = ZZ_QV(OrbitElement,RunCount,t_ctrl,QVParam);
    
    %遥测组包
    TM_SystemApp = ZZ_TelemetryData(FwSpd,FwT, Tw3,Mt_M, M3,NominalCSYS,Td,SadaInstrCmd,Agl_Sada_Tgt,Att4Ctrl,QVInstrCmd,Agl_QV_Tgt);
    y = [TM_SystemApp;[Reset0; Time0; Orbit0; Agl0; Wbi0 ;Spread0 ;Fw_Spd0 ;SADA_Agl0; QV_Agl0];FwParam.Mode; FwSpd; FwT; Mt_M;SadaInstrCmd;QVInstrCmd;Tc_Engine;...
         SadaCtrlCoeff.w_max;SadaCtrlCoeff.a_max;QVCtrlCoeff.w_max;QVCtrlCoeff.a_max];
else
    y = [zeros(178,1);[Reset0; Time0; Orbit0; Agl0; Wbi0; Spread0; Fw_Spd0 ;SADA_Agl0; QV_Agl0];zeros(60,1)];
end
end

%% 软件中全局变量初始化
function [] = ZZ_SoftwareInit(~)
global LcMark SatShape AttRef StRef GyroRef AssRef AssMmRef StAttRef AssAttRef GyroAttRef MmRef AssMmAttRef StGyroAttRef AssGyroAttRef AssMmGyroAttRef MagRef OrbitElement SadaModeParam QVModeParam EnvirParam;
LcMark = struct('Mode',0,'SubMode',0,'Mode_last',0,'SubMode_last',0);
SatShape = struct('Now',0,'Tgt',0);  % 卫星构型标志，0-ShakFin;1-OpenBook
AttRef = struct('Ok',0,'Qib',[1;0;0;0],'Qob',[1;0;0;0],'Qnb',[1;0;0;0],'Qz0b',[1;0;0;0],'Agl',[0;0;0],'Wbi',[0;0;0],'Wbo',[0;0;0],'Wbn',[0;0;0],'Wbz0_b',[0;0;0],'Choice','0');
StRef = struct('Ok',0,'Qib',[1;0;0;0],'Wbi',[0;0;0],'tao',0,'Choice','0');
GyroRef = struct('Ok',0,'Qib',[1;0;0;0],'Wbi',[0;0;0],'Choice','0');
AssRef = struct('Ok',0,'Qib',[1;0;0;0],'Wbi',[0;0;0],'Sb',[0;0;0],'tao',0);
AssMmRef = struct('Ok',0,'Qib',[1;0;0;0],'Wbi',[0;0;0],'tao',0); %太敏磁强计双矢量定姿
StAttRef = struct('Ok',0,'Qib',[1;0;0;0],'Wbi',[0;0;0]);%单星敏基准
AssAttRef = struct('Ok',0,'Qib',[1;0;0;0],'Wbi',[0;0;0]);%单太敏基准
GyroAttRef = struct('Ok',0,'Qib',[1;0;0;0],'Wbi',[0;0;0]);%单陀螺基准
AssMmAttRef = struct('Ok',0,'Qib',[1;0;0;0],'Wbi',[0;0;0]);%双矢量基准
StGyroAttRef = struct('Ok',0,'Qib',[1;0;0;0],'Wbi',[0;0;0]);%星敏陀螺基准
AssGyroAttRef = struct('Ok',0,'Qib',[1;0;0;0],'Wbi',[0;0;0]);%太敏陀螺基准
AssMmGyroAttRef = struct('Ok',0,'Qib',[1;0;0;0],'Wbi',[0;0;0]);%双矢量陀螺基准
MmRef = struct('Ok',0,'Prior',[1;2],'Bb',[0;0;0],'dBb',[0;0;0],'Bb_Filter',[0;0;0],'dBb_Filter',[0;0;0],'Bb_TimeSlice',[0;0;0],'dBb_TimeSlice',[0;0;0],'tao',2);
MagRef = struct('Ok',0,'ChoiceResult',0,'Bb',[0;0;0],'Bb_MagEqu',[0;0;0],'Bo_MagEqu',[0;0;0]);
OrbitElement = struct('Ok',1,'Time',0,'a',6378.14e3,'e',0,'i','0','OMG',0,'omg',0,'f',0,'M',0,'E',0,'w0',0,'Period',0,...
    'Qio',[1;0;0;0],'Qif',[1;0;0;0],'RV_i',[0;0;0;0;0;0],'RV_f',[0;0;0;0;0;0],'Sunshine',0);
SadaModeParam = struct('Mode1Cnt1',0,'Mode1Cnt2',0,'Mode2Cnt1',0,'Mode2Cnt2',0,'ModeLevelA',1,'ModeLevelB',1,'ModeLevelC',1,'ModeLevelD',1,'Sada1WorkEnd',0,'Sada2WorkEnd',0,'IncrementTime',[1;1;1;1]);
EnvirParam = struct('R_Sun',[0;0;0],'Asc_Sun',0,'Dec_Sun',0,'Beta',0,'gama_Sun',0,'AirDensity',0,'R_Sun_long',[0;0;0],'LocalTimeOfDesNode',0);%单位太阳矢量/太阳赤经/赤纬/光照因子/大气密度/太阳矢量/地方时

%%%  遥控设置
global Tc_FwUse_Enable Tc_SadaCtrl_Enable Tc_Engine Tc_MtUse_Enable Tc_QVCtrl_Enable;

Tc_FwUse_Enable=[0;0;0;0;0;0];Tc_SadaCtrl_Enable=[0;0;0;0];Tc_Engine=0;Tc_MtUse_Enable=[0;0;0;0;0;0];Tc_QVCtrl_Enable=0;

global Tc_StareCtrl_Enable Tc_Stare_LLA;Tc_StareCtrl_Enable=0;Tc_Stare_LLA=[120;60;45];         % 凝视      地面站，经纬高，deg/deg/m
global Tc_PLJCtrl_Enable Tc_PLJ_DB_Enable Tc_PLJ_TgtAgl;Tc_PLJCtrl_Enable = 0;Tc_PLJ_DB_Enable=0;Tc_PLJ_TgtAgl=[0;0;0];                 % 偏流角    侧摆目标角，123转序，deg
global Tc_ZeroDplCtrl_Enable Tc_ZeroDpl_IncidAgl;Tc_ZeroDplCtrl_Enable=0;Tc_ZeroDpl_IncidAgl=25;% 零多普勒  下视角 deg
global Tc_Aligned_and_Constrained;Tc_Aligned_and_Constrained = 0;  %对齐约束
global Tc_QVWork_LLA Tc_QVWork_Time;Tc_QVWork_LLA=[0;0;0];Tc_QVWork_Time=[0;0];  %QV目标点经纬度、工作区间

global Tc_Index Tc_MagCtrlAlgthm Tc_SadaCtrl_Cmd;
Tc_Index = [9;0];Tc_MagCtrlAlgthm=9;Tc_SadaCtrl_Cmd=[99 0;99 0;99 0;99 0];% 强选标志，默认9表示自主(不强选)
global Tc_TdCal_Enable Tc_TdFdFwd_Enable Tc_NotchFilter_Enable Tc_AglAccFdFwd_Enable;
Tc_TdCal_Enable = [0;0;0;0;0;0];Tc_TdFdFwd_Enable=[0;0;0;0;0;0];%力矩计算/前馈开关:光压/气动/重力梯度/磁控/陀螺/SADA驱动
Tc_NotchFilter_Enable = 0;Tc_AglAccFdFwd_Enable=0;

%%% 配置表中的参数
global FwParam StParam GyroParam SadaParam MmParam MtParam CamParam AssParam QVParam

% 单机相关配置
global StA_Fix_Qsb StB_Fix_Qsb StC_Fix_Qsb StD_Fix_Qsb GyroA_Fix_Abg GyroB_Fix_Abg MmA_Fix_Abm MmB_Fix_Abm ...
    AssA_Fix_Abs AssB_Fix_Abs AssC_Fix_Abs AssD_Fix_Abs Fw_Mode Fw_J Fw_Hmax Fw_Tmax Fw_Abf ...
    Mt_Abm MtAllocationCoeff Mt_max_M Mt_M2Cur Cam_f Cam_d Cam_lizhou Cam_r_L_L Cam_Aba ...
    Aba10 Aba20 Aba30 Aba40 SADA_RotateOrder QV_RotateOrder...
    Ass_n1 Ass_n2 Ass_n3 Ass_n4;

StAParam =struct('Ok',1,'Qis',[1 0 0 0]','Qib',[1 0 0 0]','Wsi_s',[0;0;0],'Qsb',StA_Fix_Qsb');
StBParam =struct('Ok',1,'Qis',[1 0 0 0]','Qib',[1 0 0 0]','Wsi_s',[0;0;0],'Qsb',StB_Fix_Qsb');
StCParam =struct('Ok',1,'Qis',[1 0 0 0]','Qib',[1 0 0 0]','Wsi_s',[0;0;0],'Qsb',StC_Fix_Qsb');
StDParam =struct('Ok',1,'Qis',[1 0 0 0]','Wsi_s',[0;0;0],'Qsb',StD_Fix_Qsb');
St_AB_Param = struct('Ok',1,'Qib',[1 0 0 0]','Wsi_s',[0;0;0]);
St_AC_Param = struct('Ok',1,'Qib',[1 0 0 0]','Wsi_s',[0;0;0]);
St_BC_Param = struct('Ok',1,'Qib',[1 0 0 0]','Wsi_s',[0;0;0]);
GyroAParam = struct('Ok',1,'Wgi',[0;0;0],'Abg',GyroA_Fix_Abg,'Wbi',[0;0;0]);
GyroBParam = struct('Ok',1,'Wgi',[0;0;0],'Abg',GyroB_Fix_Abg,'Wbi',[0;0;0]);
AssAParam = struct('Ok',1,'AssVolt',[0;0;0;0],'Abs',AssA_Fix_Abs,'Ass_n',[Ass_n1 Ass_n2 Ass_n3 Ass_n4]);
AssBParam = struct('Ok',1,'AssVolt',[0;0;0;0],'Abs',AssB_Fix_Abs,'Ass_n',[Ass_n1 Ass_n2 Ass_n3 Ass_n4]);
AssCParam = struct('Ok',1,'AssVolt',[0;0;0;0],'Abs',AssC_Fix_Abs,'Ass_n',[Ass_n1 Ass_n2 Ass_n3 Ass_n4]);
AssDParam = struct('Ok',1,'AssVolt',[0;0;0;0],'Abs',AssD_Fix_Abs,'Ass_n',[Ass_n1 Ass_n2 Ass_n3 Ass_n4]);
MmAParam = struct('Ok',1,'Bm',[0;0;0],'Abm',MmA_Fix_Abm);
MmBParam = struct('Ok',1,'Bm',[0;0;0],'Abm',MmB_Fix_Abm);
CamAParam = struct('f',Cam_f,'d',Cam_d,'lizhou',Cam_lizhou,'r',Cam_r_L_L,'Aba',Cam_Aba,'InteTime',0);
if SADA_RotateOrder == 123
    SADA_RotOdr = 'XYZ';
elseif SADA_RotateOrder == 213
    SADA_RotOdr = 'YXZ';
elseif SADA_RotateOrder == 132
    SADA_RotOdr = 'XZY';
elseif SADA_RotateOrder == 312
    SADA_RotOdr = 'ZXY';
elseif SADA_RotateOrder == 231
    SADA_RotOdr = 'YZX';
elseif SADA_RotateOrder == 321
    SADA_RotOdr = 'ZYX';
end
if QV_RotateOrder == 123
    QV_RotOdr = 'XYZ';
elseif QV_RotateOrder == 213
    QV_RotOdr = 'YXZ';
elseif QV_RotateOrder == 132
    QV_RotOdr = 'XZY';
elseif QV_RotateOrder == 312
    QV_RotOdr = 'ZXY';
elseif QV_RotateOrder == 231
    QV_RotOdr = 'YZX';
elseif QV_RotateOrder == 321
    QV_RotOdr = 'ZYX';
end
Sada1Param = struct('Ok',[1;1],'Agl',[0;0],'Agl4Ctrl',[0;0],'Wc',[0;0],'Ac',[0;0],'Aba0',Aba10,'RotOdr',SADA_RotOdr,'HLState',[0;0;0;0],'WorkEnd',0);
Sada2Param = struct('Ok',[1;1],'Agl',[0;0],'Agl4Ctrl',[0;0],'Wc',[0;0],'Ac',[0;0],'Aba0',Aba20,'RotOdr',SADA_RotOdr,'HLState',[0;0;0;0],'WorkEnd',0);
Sada3Param = struct('Ok',[1;1],'Agl',[0;0],'Agl4Ctrl',[0;0],'Wc',[0;0],'Ac',[0;0],'Aba0',Aba30,'RotOdr',SADA_RotOdr,'HLState',[0;0;0;0],'WorkEnd',0);
QV1Param  = struct('Ok',[1;1],'Agl',[0;0],'Agl4Ctrl',[0;0],'Wc',[0;0],'Ac',[0;0],'Aba0',Aba40,'RotOdr',QV_RotOdr);
QV2Param = struct('Ok',[1;1],'Agl',[0;0],'Agl4Ctrl',[0;0],'Wc',[0;0],'Ac',[0;0],'Aba0',Aba40,'RotOdr',QV_RotOdr);

StParam = struct('StA',StAParam,'StB',StBParam,'StC',StCParam,'StD',StDParam,'StAB',St_AB_Param,'StAC',St_AC_Param,'StBC',St_BC_Param);
GyroParam = struct('GyroA',GyroAParam,'GyroB',GyroBParam);
AssParam = struct('AssA',AssAParam,'AssB',AssBParam,'AssC',AssCParam,'AssD',AssDParam);
MmParam = struct('MmA',MmAParam,'MmB',MmBParam);
FwParam = struct('Jw',Fw_J(1),'Hmax',min(Fw_Hmax),'Tmax',min(Fw_Tmax),'Abf',Fw_Abf,'Mode',Fw_Mode,...%Mode:1-转矩模式；2-转速模式
    'Ok',[1;1;1;1;1;1],'Priority',[1;1;1;1;1;1],'FwSpd',zeros(6,1));%Priority:1-正常；2-性能下降
MtParam = struct('M',Mt_max_M,'M2Cur',Mt_M2Cur,'Abt',Mt_Abm,'AAA',MtAllocationCoeff);
CamParam = struct('CamA',CamAParam);
SadaParam = struct('Spread',[0;0],'Sada1',Sada1Param,'Sada2',Sada2Param,'Sada3',Sada3Param);
QVParam = struct('Spread',[0;0],'QV1',QV1Param,'QV2',QV2Param);
end

%% 太阳矢量计算
function [R_Sun, Asc_Sun, Dec_Sun, R_Sun_long]= SunParamCal(JDC_2000)
M = 357.5256 + 35999.049*JDC_2000;
lamM = 282.94 + M + 1.9144*sind(M) + 0.02*sind(2 * M);
rs = (149.619 - 2.499*cosd(M) - 0.021*cosd(2 * M))*1e9;
m33 = angle2dcm(-23.43929111*pi/180,0,0,'xyz');
% v31 = [rs*cosd(lamM); rs*sind(lamM);0.0];% 距离
v31 = [cosd(lamM); sind(lamM);0.0]; % 单位矢量
R_Sun = m33*v31;
R_Sun_long = rs*R_Sun;
Asc_Sun = atan2(R_Sun(2),R_Sun(1));
Dec_Sun = atan2(R_Sun(3),sqrt(R_Sun(1)^2+R_Sun(2)^2));
end


%% 光照因子计算
function gama_Sun = SunGamaCal(R_i,S_i)
Re = 6371.14e3;
rou_s = 0.26/180*pi;
rou_e = asin(Re/norm(R_i));
alpha = acos(-R_i'*S_i/norm(R_i)/norm(S_i));
if alpha <= rou_e-rou_s
    gama_Sun = 0;
elseif alpha >= rou_e+rou_s
    gama_Sun = 1;
else
    gama_Sun = (alpha-rou_e+rou_s)/2/rou_s;
end
end


%% 大气密度计算
function AirDensity = AirDensityCal(R_i,Asc_Sun,Dec_Sun,F107)
h_km = GeodeticHeightCal(R_i) / 1000;
HPcoeff;
switch F107
    case 65
        h     = hpcoef(1,1:3:end);
        c_min = hpcoef(1,2:3:end);
        c_max = hpcoef(1,3:3:end);
    case 75
        h     = hpcoef(2,1:3:end);
        c_min = hpcoef(2,2:3:end);
        c_max = hpcoef(2,3:3:end);
    case 100
        h     = hpcoef(3,1:3:end);
        c_min = hpcoef(3,2:3:end);
        c_max = hpcoef(3,3:3:end);
    case 125
        h     = hpcoef(4,1:3:end);
        c_min = hpcoef(4,2:3:end);
        c_max = hpcoef(4,3:3:end);
    case 150
        h     = hpcoef(5,1:3:end);
        c_min = hpcoef(5,2:3:end);
        c_max = hpcoef(5,3:3:end);
    case 175
        h     = hpcoef(6,1:3:end);
        c_min = hpcoef(6,2:3:end);
        c_max = hpcoef(6,3:3:end);
    case 200
        h     = hpcoef(7,1:3:end);
        c_min = hpcoef(7,2:3:end);
        c_max = hpcoef(7,3:3:end);
    case 225
        h     = hpcoef(8,1:3:end); 
        c_min = hpcoef(8,2:3:end);
        c_max = hpcoef(8,3:3:end);
    case 250
        h     = hpcoef(9,1:3:end);
        c_min = hpcoef(9,2:3:end);
        c_max = hpcoef(9,3:3:end);
    case 275
        h     = hpcoef(10,1:3:end);
        c_min = hpcoef(10,2:3:end);
        c_max = hpcoef(10,3:3:end);
    otherwise
        R = [65;75;100;125;150;175;200;225;250;275];
        Sheet = interp1(R, hpcoef, F107, 'linear', 'extrap');   % 线性插值，extrap允许外插
        h = Sheet(1,1:3:end);
        c_min = Sheet(1,2:3:end);
        c_max = Sheet(1,3:3:end);            
end
ih = 1;                           % section index reset
for  i=1:59-1                 % loop over N_Coef height regimes
  if ( h_km >= h(i) && h_km < h(i+1) )
    ih = i;                       % ih identifies height section
    break
  end
end
Hm = (h(ih)-h(ih+1))/log(c_min(ih+1)/c_min(ih));
HM = (h(ih)-h(ih+1))/log(c_max(ih+1)/c_max(ih));
rou_m = c_min(ih)*exp((h(ih)-h_km)/Hm);
rou_M = c_max(ih)*exp((h(ih)-h_km)/HM);
lamda = pi/6;
n = 6;
eb = [cos(Dec_Sun)*cos(Asc_Sun+lamda);cos(Dec_Sun)*sin(Asc_Sun+lamda);sin(Dec_Sun)];
CosNPhi = (0.5+R_i'/norm(R_i)*eb/2)^(n/2);
AirDensity = (rou_m+(rou_M-rou_m)*CosNPhi)*1e-9;
end

function TM_SystemApp = ZZ_TelemetryData(FwSpd,FwT, Tw3,Mt_M,M3,NominalCSYS,Td,SadaInstrCmd,Agl_Sada_Tgt,Att4Ctrl,QVInstrCmd,Agl_QV_Tgt)

global LcMark MmRef StRef GyroRef AssRef AttRef EnvirParam MagRef;
StRefChoice = zeros(4,1);
GyroRefChoice= zeros(5,1);
AttRefChoice = zeros(9,1);
StRefChoice(1:length(StRef.Choice)) = double(StRef.Choice);
GyroRefChoice(1:length(GyroRef.Choice)) = double(GyroRef.Choice);
AttRefChoice(1:length(AttRef.Choice)) = double(AttRef.Choice);
TM_Mode = [LcMark.Mode;LcMark.SubMode];          % Output dimensions:2
TM_Ctrl = [Td.Ts;Td.Ta;Td.Tp;Att4Ctrl.Agl;Att4Ctrl.Wbn;Tw3;M3];   % Output dimensions:21
TM_StRef = [StRef.Ok;StRef.Qib;StRef.Wbi;StRefChoice];    % Output dimensions:12
TM_GyroRef = [GyroRef.Ok;GyroRef.Qib;GyroRef.Wbi;GyroRefChoice];    % Output dimensions:13
TM_MmRef = [MagRef.Ok;MmRef.Bb_TimeSlice;MmRef.dBb;MagRef.Bb;MmRef.dBb_TimeSlice];     % Output dimensions:13
TM_AssRef = [AssRef.Ok;AssRef.Qib;AssRef.Wbi;AssRef.Sb];   % Output dimensions:11
TM_Fw = [FwSpd;FwT];   % Output dimensions:12
TM_Mt = Mt_M;   % Output dimensions:6
TM_SADA = [SadaInstrCmd;Agl_Sada_Tgt];    % Output dimensions:18
TM_Att = [AttRef.Qz0b;AttRef.Wbz0_b;AttRefChoice;NominalCSYS.Qin;NominalCSYS.Wni_n;NominalCSYS.Ani_n];   % Output dimensions:26
TM_Orbit = [EnvirParam.Beta;EnvirParam.LocalTimeOfDesNode]; % Output dimensions:2
TM_QV = [QVInstrCmd;Agl_QV_Tgt];   % Output dimensions:12
TM_Reserved = [NominalCSYS.Wzi_z;zeros(27,1)];    %预留遥测：30×1,Output dimensions:30
TM_SystemApp = [TM_Mode;TM_Ctrl;TM_StRef;TM_GyroRef;TM_MmRef;TM_AssRef;TM_Fw;TM_Mt;TM_SADA;TM_Att;TM_Orbit;TM_QV;TM_Reserved]; %Output dimensions:178

end

function h = GeodeticHeightCal(r)
R_equ = 6378136.3 ; 
f     = 0.00335281066474748;

epsRequ = 1e-9*R_equ;      % Convergence criterion
e2      = f*(2-f);        % Square of eccentricity

X = r(1);                 % Cartesian coordinates
Y = r(2);
Z = r(3);
rho2 = X*X + Y*Y;         % Square of distance from z-axis

% Check validity of input data
if (norm(r)==0)
    disp ( ' invalid input in Geodetic constructor\n' );
    lon = 0;
    lat = 0;
    h   = R_equ;
    return
end

% Iteration 
dZ = e2*Z;

while(1)
    ZdZ    = Z + dZ;
    Nh     = sqrt ( rho2 + ZdZ*ZdZ ); 
    SinPhi = ZdZ / Nh;    % Sine of geodetic latitude
    N      = R_equ / sqrt(1-e2*SinPhi*SinPhi);
    dZ_new = N*e2*SinPhi;
    if (abs(dZ-dZ_new) < epsRequ)
        break
    end
    dZ = dZ_new;
end

% Longitude, latitude, altitude
lon = atan2(Y, X);
lat = atan2(ZdZ, sqrt(rho2));
h   = Nh - N;
end