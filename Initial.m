if ~exist('buildcnt','var')
    clear all;clc;close all;   %不能放在initial里，不然清的只有局部变量
end
InitFcn;
TestMode = 1;% 测试模式开关,0:编译，1:数学仿真
global SatClass;  % 卫星类别，用来区分批产型号的不同状态，参数配置时通过SatNum设置多组值。
if TestMode == 1
    SatClass = 1;  % 设置仿真用卫星序号    
end
global Reset0 Time0 Orbit0 Agl0 Wbi0 Spread0 Fw_Spd0 SADA_Agl0 QV_Agl0 q00 dq00;

%%  仿真设置
% T_Simu = 0.0125*TestMode+0.02*(1-TestMode);
% T_Ctrl = 0.25*TestMode+0.5*(1-TestMode); % 控制步长，在飞轮延迟环节中用到
% T_Ctrl_DM = 0.25*TestMode+0.5*(1-TestMode);% 数管软件步长，用于SADA和QV控制
T_Simu = 0.01*TestMode+0.02*(1-TestMode);
T_Ctrl = 0.1*TestMode+0.5*(1-TestMode); % 控制步长，在飞轮延迟环节中用到
T_Ctrl_DM = 0.1*TestMode+0.5*(1-TestMode);% 数管软件步长，用于SADA和QV控制
SimuStopTime = 200; % 仿真时间
Reset0 = hex2dec('33');

%%  初始轨道、姿态
Time0 = [2026 1 26 04 00 00]';% 年月日时分秒（UTC）
Orbit0=[6927.57e3;0.000979676;97.5746*DEG2RAD;215.45*DEG2RAD;261.334*DEG2RAD;98.8067*DEG2RAD];%初始轨道参数 a/e/i/OMG/omg/f beta=35
Agl0 = [-1.2311    0.3649    2.0246]'; % 初始欧拉角（轨道系）rad
Wbi0 = 0*[0.1;0.06;-.1]/180*pi; % 初始角速度（惯性系）
Spread0 = zeros(8,1);%初始帆板展开标志


%% 读取配置文件
filename = 'Satellite Parameters.xlsx';
t1 = xlsread(filename, 1, 'B2:G27', 'basic');
t2 = xlsread(filename, 2, '', 'basic');
t3 = xlsread(filename, 3, '', 'basic');
t4 = xlsread(filename, 4, 'B2:D18', 'basic');
t5 = xlsread(filename, 5, 'B2:D18', 'basic');
t6 = xlsread(filename, 6, 'C1:E16', 'basic');
t7 = xlsread(filename, 7, 'B2:S41', 'basic');
t8 = xlsread(filename, 8, 'B2:G59', 'basic');
t9 = xlsread(filename, 9, '','basic');

[mL_fold, r_L_Lc_b_Fold, I_L_Lc_b_Fold, r_L_b_b_Fold, Abb_Fold] = SetMassProperties(t1(1:8,4:6)); % 星本体展开前
[mL, r_L_Lc_b, I_L_Lc_b, r_L_b_b, Abb] = SetMassProperties(t1(1:8,1:3)); % 星本体展开后

%% 附件
global Aba10 Aba20 Aa10p10 Aa20p20 ...
    Aba30 Aba40 Aa30p30 Aa40p40

% 第一个附件，a1/p1(收拢状态)
[ma1_fold, r_a1_ac1_a1_fold, I_a1_ac1_a1_fold, r_L_a1_b_fold, Aba10_fold] = SetMassProperties(t2(1:8,4:6));
[mp1_fold, r_p1_pc1_p1_fold, I_p1_pc1_p1_fold, r_a1_p1_a1_fold, Aa10p10_fold] = SetMassProperties(t2(10:17,4:6));
% 第一个附件，a1/p1(展开状态)
[ma1, r_a1_ac1_a1, I_a1_ac1_a1, r_L_a1_b, Aba10] = SetMassProperties(t2(1:8,1:3));
[mp1, r_p1_pc1_p1, I_p1_pc1_p1, r_a1_p1_a1, Aa10p10] = SetMassProperties(t2(10:17,1:3));

% 第二个附件，a2/p2(收拢状态)
[ma2_fold, r_a2_ac2_a2_fold, I_a2_ac2_a2_fold, r_L_a2_b_fold, Aba20_fold] = SetMassProperties(t3(1:8,4:6));
[mp2_fold, r_p2_pc2_p2_fold, I_p2_pc2_p2_fold, r_a2_p2_a2_fold, Aa20p20_fold] = SetMassProperties(t3(10:17,4:6));
% 第二个附件，a2/p2(展开状态)
[ma2, r_a2_ac2_a2, I_a2_ac2_a2, r_L_a2_b, Aba20] = SetMassProperties(t3(1:8,1:3));
[mp2, r_p2_pc2_p2, I_p2_pc2_p2, r_a2_p2_a2, Aa20p20] = SetMassProperties(t3(10:17,1:3));

% 第三个附件  QV1
[ma3, r_a3_ac3_a3, I_a3_ac3_a3, r_L_a3_b, Aba30] = SetMassProperties(t4(1:8,1:3));
[mp3, r_p3_pc3_p3, I_p3_pc3_p3, r_a3_p3_a3, Aa30p30] = SetMassProperties(t4(10:17,1:3));

% 第四个附件  QV2
[ma4, r_a4_ac4_a4, I_a4_ac4_a4, r_L_a4_b, Aba40] = SetMassProperties(t5(1:8,1:3));
[mp4, r_p4_pc4_p4, I_p4_pc4_p4, r_a4_p4_a4, Aa40p40] = SetMassProperties(t5(10:17,1:3));

m=mL+ma1+mp1+ma2+mp2;

% 尺寸参数
[Sx_board, n_PX, CenterOfFigPX] = SetStructProperties([t1(9,1:3);t1(15,1:3);t1(21,1:3)]);
[~, n_NX, CenterOfFigNX] = SetStructProperties([t1(10,1:3);t1(16,1:3);t1(22,1:3)]);
[Sy_board, n_PY, CenterOfFigPY] = SetStructProperties([t1(11,1:3);t1(17,1:3);t1(23,1:3)]);
[~, n_NY, CenterOfFigNY] = SetStructProperties([t1(12,1:3);t1(18,1:3);t1(24,1:3)]);
[Szp_board, n_PZ, CenterOfFigPZ] = SetStructProperties([t1(13,1:3);t1(19,1:3);t1(25,1:3)]);
[Szn_board, n_NZ, CenterOfFigNZ] = SetStructProperties([t1(14,1:3);t1(20,1:3);t1(26,1:3)]);

[S_P1, n_P1P, CenterOfFigP1] = SetStructProperties(t2(19:21,1:3));
[S_P2, n_P2P, CenterOfFigP2] = SetStructProperties(t3(19:21,1:3));
n_P1N = -n_P1P;
n_P2N = -n_P2P;

%% 挠性参数
NumMode1 = t2(23,1);
NumMode2 = t3(23,1);


[Choice_q1, Fre0_p1, Ksi0_p1, Bt0_p1, Br0_p1] = SetFlexProperties(t2(24:23+5*NumMode1,1:3),NumMode1);
[Choice_q2, Fre0_p2, Ksi0_p2, Bt0_p2, Br0_p2] = SetFlexProperties(t3(24:23+5*NumMode2,1:3),NumMode2);
q00 = zeros(max(NumMode1,1)+max(NumMode2,1),1);
dq00 = zeros(max(NumMode1,1)+max(NumMode2,1),1);
j = 0;
for i = 1:2% 附件维数
    if eval(['NumMode' num2str(i)]) == 0 % 当不加入挠性时，强制给挠性参数赋1维，否则simulink中参数SatParam结构体赋值会有问题
        eval(['NumMode' num2str(i), '=1;']);
        eval(['Fre_p' num2str(i), '= 0;']);
        eval(['Ksi_p' num2str(i), '= 0;']);
        eval(['Bt_p' num2str(i), '= [0;0;0];']);
        eval(['Br_p' num2str(i), '= [0;0;0];']);
        q00(j+1:j+eval(['NumMode' num2str(i)]),1) = 0;
        dq00(j+1:j+eval(['NumMode' num2str(i)]),1) = 0;
        j = j + eval(['NumMode' num2str(i)]);
    else
        Fre0 = eval(['Fre0_p' num2str(i)]);
        Ksi0 = eval(['Ksi0_p' num2str(i)]);
        list = eval(['Choice_q' num2str(i)]);% 不能在下一句中再嵌套一个eval
        eval(['Fre_p' num2str(i) '= Fre0_p' num2str(i) '(list);']);
        eval(['Ksi_p' num2str(i) '= Ksi0_p' num2str(i) '(list);']);
        eval(['Bt_p' num2str(i) '= Bt0_p' num2str(i) '(:,list);']);
        eval(['Br_p' num2str(i) '= Br0_p' num2str(i) '(:,list);']);
        q00(j+1:j+eval(['NumMode' num2str(i)]),1) = zeros(eval(['NumMode' num2str(i)]),1);
        dq00(j+1:j+eval(['NumMode' num2str(i)]),1) = zeros(eval(['NumMode' num2str(i)]),1);
        j = j + eval(['NumMode' num2str(i)]);
    end
end

%% 单机参数
global StA_Fix_Qsb StB_Fix_Qsb StC_Fix_Qsb StD_Fix_Qsb ...
    GyroA_Fix_Abg GyroB_Fix_Abg GyroC_Fix_Abg GyroD_Fix_Abg ...
    MmA_Fix_Abm MmB_Fix_Abm MmC_Fix_Abm MmD_Fix_Abm ...
    AssA_Fix_Abs AssB_Fix_Abs AssC_Fix_Abs AssD_Fix_Abs ...
    Fw_Mode Fw_J Fw_Hmax Fw_Tmax Fw_Abf ...
    Mt_Abm Mt_max_M Mt_M2Cur ...
    Cam_f Cam_d Cam_lizhou Cam_r_L_L Cam_Aba ...
    SADA_RotateOrder QV_RotateOrder...
    Ass_n1 Ass_n2 Ass_n3 Ass_n4 Ass_kS;
% 星敏
STR_Block = t7(1:11,1:4);
St_SightAxis = STR_Block(1,1:3)'; % 星敏系下的光轴矢量
StA_Fix_Qsb = STR_Block(2:5,1)'; % 测量系到本体系，1*4行向量
StB_Fix_Qsb = STR_Block(2:5,2)';
StC_Fix_Qsb = STR_Block(2:5,3)';
StD_Fix_Qsb = STR_Block(2:5,4)';
StANoise = STR_Block(6:8,1);    % 星敏A噪声
StBNoise = STR_Block(6:8,2);    % 星敏B噪声
StCNoise = STR_Block(6:8,3);    % 星敏C噪声
StDNoise = STR_Block(6:8,4);    % 星敏D噪声
St_SEA = STR_Block(9,1:4)'/180*pi;    % Sun Exclusion Angle
St_EEA = STR_Block(10,1:4)'/180*pi;    % Earth Exclusion Angle
St_EarthAtomAgl = asin(Re/Orbit0(1));% 对应508km
St_TimeStamp_leap = STR_Block(11,1);%默认0输出的时间戳为相对于1970的UTC时间。

% 陀螺
Gyro_Block = t7(13:16,1:12);
GyroA_Fix_Abg = Gyro_Block(1:3,1:3); % 安装矩阵，陀螺到本体，20241022更新至最新
GyroB_Fix_Abg = Gyro_Block(1:3,4:6);
GyroC_Fix_Abg = Gyro_Block(1:3,7:9);
GyroD_Fix_Abg = Gyro_Block(1:3,10:12);
GyroA_ARW = Gyro_Block(4,1:3)';
GyroB_ARW = Gyro_Block(4,4:6)';
GyroC_ARW = Gyro_Block(4,7:9)';
GyroD_ARW = Gyro_Block(4,10:12)';
GyroA_Noise = GyroA_ARW/60*sqrt(1/T_Ctrl)*3; % 3sigma白噪声
GyroB_Noise = GyroB_ARW/60*sqrt(1/T_Ctrl)*3;
GyroC_Noise = GyroC_ARW/60*sqrt(1/T_Ctrl)*3;
GyroD_Noise = GyroD_ARW/60*sqrt(1/T_Ctrl)*3;

% 磁强计
MGM_Block = t7(18:27,1:12);
MmA_Fix_Abm = MGM_Block(1:3,1:3); %磁强计安装矩阵
MmB_Fix_Abm = MGM_Block(1:3,4:6);
MmC_Fix_Abm = MGM_Block(1:3,7:9);
MmD_Fix_Abm = MGM_Block(1:3,10:12);
MmA_Noise = MGM_Block(4,1:3)';%单位：Gs  霍尼韦尔20nT,3sigma  彭宇思锐模拟量采集噪声20mV，+1~+4v对应±0.5Gs
MmB_Noise = MGM_Block(4,4:6)';
MmC_Noise = MGM_Block(4,7:9)';
MmD_Noise = MGM_Block(4,10:12)';
%磁棒对磁强计（本体系测量值）影响系数，3*n，乘性系数，无影响填零。每列为对应磁棒单位磁矩对磁强计的三轴影响，单位Gs/Am2
MmA_MtInflunce_Coff = MGM_Block(5:10,1:3)';
MmB_MtInflunce_Coff = MGM_Block(5:10,4:6)';
MmC_MtInflunce_Coff = MGM_Block(5:10,7:9)';
MmD_MtInflunce_Coff = MGM_Block(5:10,10:12)';
% 太敏
ASS_Block = t7(29:40,1:18);
Ass_n1 = ASS_Block(4,1:3)';   % 太敏系下4个电池片的法线方向
Ass_n2 = ASS_Block(5,1:3)';
Ass_n3 = ASS_Block(6,1:3)';
Ass_n4 = ASS_Block(7,1:3)';
AssA_Fix_Abs = ASS_Block(1:3,1:3);   % 太敏系到本体系的转换矩阵
AssB_Fix_Abs = ASS_Block(1:3,4:6);
AssC_Fix_Abs = ASS_Block(1:3,7:9);
AssD_Fix_Abs = ASS_Block(1:3,10:12);
AssE_Fix_Aps = ASS_Block(1:3,13:15);   % 太敏系到附件系的转换矩阵
AssF_Fix_Aps = ASS_Block(1:3,16:18);
Ass_Resolution = ASS_Block(8,1); % 太敏测量分辨率 0.0957mA/bit
Ass_Noise = ASS_Block(9,1);% 3mV  LC
Ass_kS = ASS_Block(10,1);   % 太阳常数*电池片面积，具体数值不祥，仅知最大电流104mA   20190709由104mA改为100mA
Ass_R = ASS_Block(11,1); % 采样电阻2.5欧姆
Ass_TmOrder = ASS_Block(12,1:6)';%输出给Labview的太敏顺序（假设星体两个本体一个帆板则为[1;2;5;3;4;6]）

% 飞轮
FW_Block = t8(1:8,1:6);
Fw_Mode = FW_Block(8,1);% Mode:1-转矩模式；2-转速模式
Fw_Spd0 = FW_Block(7,1:6)';% 初始转速
Fw_J = FW_Block(6,1:6)';% 没有的飞轮不要写0，会有除0
Fw_Hmax = FW_Block(4,1:6)';
Fw_Tmax = FW_Block(5,1:6)';
Fw_Abf = FW_Block(1:3,1:6);

% 磁力矩器
MGT_Block = t8(10:15,1:6);
Mt_Abm = MGT_Block(1:3,1:6);         % 安装矩阵
Mt_max_M = MGT_Block(4,1:6)';        % 最大磁矩  Am2
Mt_Cur2M = MGT_Block(5,1:6)';% 磁电流（mA）到磁矩(Am2)的转换系数,没有磁棒的填0
Tao_Mt = MGT_Block(6,1); %磁棒上升时间(单位：s)
[Mt_Index, ~] = find(Mt_Cur2M);
Mt_M2Cur = zeros(6,1);
Mt_M2Cur(Mt_Index) = 1./Mt_Cur2M(Mt_Index);
% 剩磁
RemanMag = t9(1,1:3)';

% GNSS
Gnss_Fre = t9(3,1);      % GNSS采样频率，采样从整秒开始
Gnss_Noise_Pos = t9(4,1)*3;   % m  1sigma 
Gnss_Noise_Vel = t9(5,1)*3;  % m/s  1sigma
Gnss_PropgationTime = t9(6,1:2)'; % GNSS的递推功能的时间
T_LeapSec = 0*27;% 起始点为1970年时的闰秒数，星务已处理，因此动力学GNSS伪采集中就不用模拟了

% 相机
Cam_f = 1.15;           % 焦距，m
Cam_d = 4.6e-6;         % 像元尺寸4.6um
Cam_lizhou =  0/180*pi; % 离轴角，rad
Cam_r_L_L = [0 0 0]';   % 相机（物镜）在卫星下的安装位置
Cam_Aba = eye(3)*0+1*angle2dcm(0.01/180*pi,-0.4/180*pi,0.06/180*pi,'XYZ');       % 相机到本体系的安装矩阵，

% SADA(动力学模拟的是不SADA，而是SADA里面那个转动单元，转动单元和SADA之间还有一个极性转换)
SADA_Block = t8(17:35,1:6);
SADA_RotateOrder = SADA_Block(1,1);               % SADA转序，123代表XYZ，213表示YXZ
SADA_A_RotUnitSADA = SADA_Block(2,:)'; % SADA坐标系到转动单元的极性
SADA_A_DriveRotUnit = SADA_Block(3,:)';   % 驱动器指令到转动单元的极性
SADA_StepAgl = SADA_Block(4,:)';         % 步距角 deg
SADA_SpdDownRatio = SADA_Block(5,:)';    % 减速比
SADA_Agl0 = SADA_Block(6,:)';            % SADA系下SADA初始角度（收拢状态下）  deg  +Y的X/Y轴  -Y的X/Y轴
SADA_XbLw_Value = SADA_Block(7,:)';      % 零位对应旋变值 deg 0~360
SADA_Agl_HL1 = SADA_Block(8:9,:);  % 霍尔1单边触发宽度（【左；右】）取值±180°范围内,转动单元坐标系
SADA_Agl_HL2 = SADA_Block(10:11,:);  % 霍尔2单边触发宽度（【左；右】），转动单元坐标系
SADA_FreLimit = SADA_Block(12,:)';   %SADA电机1/2/3/4的频率门限
SADA_PosLocationLimit = SADA_Block(13,:)';   % 驱动器中的位置门限，SADA系角度限幅值-正向，deg(不需限位时正反向门限设同一个值即可)
SADA_NegLocationLimit = SADA_Block(14,:)';   % SADA系角度限幅值-负向，deg
SADA_PosLocationLimit_Real = SADA_Block(15,:)';   % 物理硬限位，SADA系角度限幅值-正向，deg(不需限位时可用±360000，可转约1000轨)
SADA_NegLocationLimit_Real = SADA_Block(16,:)';   % SADA系角度限幅值-负向，deg
SADA_PosStepLimit = SADA_Block(17,:)';  % 正向步数门限。如果是连续转动，设置成正负相等即可；如果是非连续转动，需设置成相应的数（0~360对应的数）
SADA_NegStepLimit = SADA_Block(18,:)';  % 反向步数门限。
SADA_TimeStamp_leap = SADA_Block(19,1);%默认0输出的时间戳为相对于1970的UTC时间。 1136073600为1970UTC到北斗时

%QV
QV_Block = t8(37:55,1:4);
QV_RotateOrder = QV_Block(1,1);               % SADA转序，123代表XYZ，213表示YXZ
QV_A_RotUnitQV = QV_Block(2,1:4)'; % SADA坐标系到转动单元的极性
QV_A_DriveRotUnit = QV_Block(3,1:4)';   % 驱动器指令到转动单元的极性
QV_StepAgl = QV_Block(4,1:4)'; 
QV_SpdDownRatio = QV_Block(5,1:4)';
QV_Agl0 = QV_Block(6,1:4)';         %QV系下QV初始角度（收拢状态下）  deg  +Y的X/Y轴  -Y的X/Y轴
QV_XbLw_Value = QV_Block(7,1:4)';    % 零位对应旋变值 deg 0~360
QV_Agl_HL1 = QV_Block(8:9,1:4); % 霍尔1单边触发宽度（【左；右】），QV不用默认填0即可
QV_Agl_HL2 = QV_Block(10:11,1:4); % 霍尔2单边触发宽度（【左；右】），QV不用默认填0即可
QV_FreLimit = QV_Block(12,1:4)';  %QV电机1/2/3/4的频率门限
QV_PosLocationLimit = QV_Block(13,1:4)';  % 驱动器中的位置门限，QV系角度-正向，deg(不需限位时正反向门限设同一个值即可)
QV_NegLocationLimit = QV_Block(14,1:4)'; 
QV_PosLocationLimit_Real = QV_Block(15,1:4)';   % 物理硬限位，QV系角度-正向，deg(不需限位时可用±360000，可转约1000轨)
QV_NegLocationLimit_Real = QV_Block(16,1:4)'; 
QV_PosStepLimit = QV_Block(17,1:4)';  % QV驱动器真机不设步数门限，正反向门限设同一个值即可
QV_NegStepLimit = QV_Block(18,1:4)';
QV_TimeStamp_leap = QV_Block(19,1);%默认0输出的时间戳为相对于1970的UTC时间。 1136073600为1970UTC到北斗时

% 推进器  Engine 目前只有2个推力器的接口
Eg_r_L_L = [t9(8:10,1);t9(8:10,2)];% 推力器安装位置(机械系)[3N,1]
Eg_Dir = [t9(11:13,1);t9(11:13,2)];% 推力矢量方向（本体系）[3N,1]
Eg_F = t9(14,1:2)';              % 推力大小，单位N
Eg_PrepareTime = t9(15,1:2)';     % 推力器点火准备时间，单位s。收到点火指令后延迟Eg_PrepareTime产生推力。

% 卫星编号
SatID = hex2dec(num2str(t9(16,1)));%卫星唯一标识，参考钉盘卫星序列号命名规则

%% 环境相关参数
% 21阶摄动参数
Orbit_UTC_second_t0=1582689600;%%%UTC 2020-02-26 04:00:00  
Orbit_date0= [2020,2,26];%%%year,month,day
Orbit_time0= [4,0,0];%%%hour,min,second
Orbit_year0 = Orbit_date0(1);
Orbit_hour0 = Orbit_time0(1);
Orbit_min0 = Orbit_time0(2);
Orbit_second0 = Orbit_time0(3);
% 光压模型参数
Crou=1;   % 反射系数 条件恶劣
Cmu=0.2;  % 散射系数
SConst=1395; %太阳常数：W/m?
C = 3e08;   %光速
% 大气模型参数
F107 = 80;% F10.7系数 65/75/100/125/150/175/200/225/250/275
HPcoeff;
ra_lag      = 0.523599; % Right ascension lag [rad]
n_prm       =        3; % Harris-Priester parameter 
                        % 2(6) low(high) inclination
Cd = 2.2;% 大气阻尼系数

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

load aeroCIP2006;%用于计算天球中间极点CIP的一个文件，Afi计算模块中用到，struct类型，simulink不能直接用
aeroCIP2006_Y = aeroCIP2006.Y;
aeroCIP2006_X = aeroCIP2006.X;
aeroCIP2006_S = aeroCIP2006.S;

%% 结构体
BodyParam = struct('m',mL,'Ic',I_L_Lc_b,'rc',r_L_Lc_b,'FixPoint',[0;0;0],'Aba',eye(3));%
PanelA1Param = struct('m',ma1,'Ic',I_a1_ac1_a1,'rc',r_a1_ac1_a1,'FixPoint',r_L_a1_b,'Aba',Aba10);
PanelA2Param = struct('m',ma2,'Ic',I_a2_ac2_a2,'rc',r_a2_ac2_a2,'FixPoint',r_L_a2_b,'Aba',Aba20);
PanelP1Param = struct('m',mp1,'Ic',I_p1_pc1_p1,'rc',r_p1_pc1_p1,'FixPoint',r_a1_p1_a1,'Aba',Aa10p10);
PanelP2Param = struct('m',mp2,'Ic',I_p2_pc2_p2,'rc',r_p2_pc2_p2,'FixPoint',r_a2_p2_a2,'Aba',Aa20p20);
QvA3Param = struct('m',ma3,'Ic',I_a3_ac3_a3,'rc',r_a3_ac3_a3,'FixPoint',r_L_a3_b,'Aba',Aba30);
QvA4Param = struct('m',ma4,'Ic',I_a4_ac4_a4,'rc',r_a4_ac4_a4,'FixPoint',r_L_a4_b,'Aba',Aba40);
QvP3Param = struct('m',mp3,'Ic',I_p3_pc3_p3,'rc',r_p3_pc3_p3,'FixPoint',r_a3_p3_a3,'Aba',Aa30p30);
QvP4Param = struct('m',mp4,'Ic',I_p4_pc4_p4,'rc',r_p4_pc4_p4,'FixPoint',r_a4_p4_a4,'Aba',Aa40p40);
BodyParam_Fold = struct('m',mL_fold,'Ic',I_L_Lc_b_Fold,'rc',r_L_Lc_b_Fold,'FixPoint',[0;0;0],'Aba',eye(3));
PanelA1Param_Fold = struct('m',ma1_fold,'Ic',I_a1_ac1_a1_fold,'rc',r_a1_ac1_a1_fold,'FixPoint',r_L_a1_b_fold,'Aba',Aba10);
PanelA2Param_Fold = struct('m',ma2_fold,'Ic',I_a2_ac2_a2_fold,'rc',r_a2_ac2_a2_fold,'FixPoint',r_L_a2_b_fold,'Aba',Aba20);
PanelP1Param_Fold = struct('m',mp1_fold,'Ic',I_p1_pc1_p1_fold,'rc',r_p1_pc1_p1_fold,'FixPoint',r_a1_p1_a1_fold,'Aba',Aa10p10);
PanelP2Param_Fold = struct('m',mp2_fold,'Ic',I_p2_pc2_p2_fold,'rc',r_p2_pc2_p2_fold,'FixPoint',r_a2_p2_a2_fold,'Aba',Aa20p20);
FlexP1Param = struct('Num',NumMode1,'Fre',Fre_p1,'Ksi',Ksi_p1,'Bt',Bt_p1,'Br',Br_p1);
FlexP2Param = struct('Num',NumMode2,'Fre',Fre_p2,'Ksi',Ksi_p2,'Bt',Bt_p2,'Br',Br_p2);
StructPxParam = struct('n',n_PX,'S',Sx_board,'FigCent',CenterOfFigPX);
StructNxParam = struct('n',n_NX,'S',Sx_board,'FigCent',CenterOfFigNX);
StructPyParam = struct('n',n_PY,'S',Sy_board,'FigCent',CenterOfFigPY);
StructNyParam = struct('n',n_NY,'S',Sy_board,'FigCent',CenterOfFigNY);
StructPzParam = struct('n',n_PZ,'S',Szp_board,'FigCent',CenterOfFigPZ);
StructNzParam = struct('n',n_NZ,'S',Szn_board,'FigCent',CenterOfFigNZ);
StructPp1Param = struct('n',n_P1P,'S',S_P1,'FigCent',CenterOfFigP1);
StructNp1Param = struct('n',n_P1N,'S',S_P1,'FigCent',CenterOfFigP1);
StructPp2Param = struct('n',n_P2P,'S',S_P2,'FigCent',CenterOfFigP2);
StructNp2Param = struct('n',n_P2N,'S',S_P2,'FigCent',CenterOfFigP2);
global SatParam;
SatParam = struct('Body',BodyParam,...
    'A1',PanelA1Param,'P1',PanelP1Param,'A2',PanelA2Param,'P2',PanelP2Param,...
    'A3',QvA3Param,'P3',QvP3Param,'A4',QvA4Param,'P4',QvP4Param,...
    'Body_Fold',BodyParam_Fold,...
    'A1_Fold',PanelA1Param_Fold,'P1_Fold',PanelP1Param_Fold,'A2_Fold',PanelA2Param_Fold,'P2_Fold',PanelP2Param_Fold,...
    'FlexP1',FlexP1Param,'FlexP2',FlexP2Param,...
    'StructPx',StructPxParam,'StructNx',StructNxParam,'StructPy',StructPyParam,'StructNy',StructNyParam,...
    'StructPz',StructPzParam,'StructNz',StructNzParam,'StructPp1',StructPp1Param,'StructNp1',StructNp1Param,...
    'StructPp2',StructPp2Param,'StructNp2',StructNp2Param);

%% 定义总线
% FlexP1Property
FlexP1PropertyNum = busElemCre('Num', 1);
FlexP1PropertyFre = busElemCre('Fre', [NumMode1 1]);
FlexP1PropertyKsi = busElemCre('Ksi', [NumMode1 1]);
FlexP1PropertyBt = busElemCre('Bt',  [3 NumMode1]);
FlexP1PropertyBr = busElemCre('Br',  [3 NumMode1]);
FlexP1Property = Simulink.Bus; 
FlexP1Property.Elements = [FlexP1PropertyNum, FlexP1PropertyFre, FlexP1PropertyKsi, FlexP1PropertyBt, FlexP1PropertyBr];
% FlexP2Property
FlexP2PropertyNum = busElemCre('Num', 1);
FlexP2PropertyFre = busElemCre('Fre', [NumMode2 1]);
FlexP2PropertyKsi = busElemCre('Ksi', [NumMode2 1]);
FlexP2PropertyBt = busElemCre('Bt',  [3 NumMode2]);
FlexP2PropertyBr = busElemCre('Br',  [3 NumMode2]);
FlexP2Property = Simulink.Bus; 
FlexP2Property.Elements = [FlexP2PropertyNum, FlexP2PropertyFre, FlexP2PropertyKsi, FlexP2PropertyBt, FlexP2PropertyBr];
% ObjectProperty
ObjectPropertym = busElemCre('m', 1);
ObjectPropertyIc = busElemCre('Ic', [3 3]);
ObjectPropertyrc = busElemCre('rc', [3 1]);
ObjectPropertyFix = busElemCre('FixPoint',  [3 1]);
ObjectPropertyAba = busElemCre('Aba',  [3 3]);
ObjectProperty = Simulink.Bus; 
ObjectProperty.Elements = [ObjectPropertym, ObjectPropertyIc, ObjectPropertyrc, ObjectPropertyFix, ObjectPropertyAba];
% StructProperty
StructPropertyn = busElemCre('n', [3 1]);
StructPropertyS = busElemCre('S', 1);
StructPropertyFigCent = busElemCre('FigCent', [3 1]);
StructProperty = Simulink.Bus; 
StructProperty.Elements = [StructPropertyn, StructPropertyS, StructPropertyFigCent];
% SatProperty
F1 = busElemCre('Body', 'Bus: ObjectProperty');
F2 = busElemCre('A1', 'Bus: ObjectProperty');
F3 = busElemCre('P1', 'Bus: ObjectProperty');
F4 = busElemCre('A2', 'Bus: ObjectProperty');
F5 = busElemCre('P2', 'Bus: ObjectProperty');
F6 = busElemCre('A3', 'Bus: ObjectProperty');
F7 = busElemCre('P3', 'Bus: ObjectProperty');
F8 = busElemCre('A4', 'Bus: ObjectProperty');
F9 = busElemCre('P4', 'Bus: ObjectProperty');
F10 = busElemCre('Body_Fold', 'Bus: ObjectProperty');
F11 = busElemCre('A1_Fold', 'Bus: ObjectProperty');
F12 = busElemCre('P1_Fold', 'Bus: ObjectProperty');
F13 = busElemCre('A2_Fold', 'Bus: ObjectProperty');
F14 = busElemCre('P2_Fold', 'Bus: ObjectProperty');
F15 = busElemCre('FlexP1', 'Bus: FlexP1Property');
F16 = busElemCre('FlexP2', 'Bus: FlexP2Property');
F17 = busElemCre('StructPx', 'Bus: StructProperty');
F18 = busElemCre('StructNx', 'Bus: StructProperty');
F19 = busElemCre('StructPy', 'Bus: StructProperty');
F20 = busElemCre('StructNy', 'Bus: StructProperty');
F21 = busElemCre('StructPz', 'Bus: StructProperty');
F22 = busElemCre('StructNz', 'Bus: StructProperty');
F23 = busElemCre('StructPp1', 'Bus: StructProperty');
F24 = busElemCre('StructNp1', 'Bus: StructProperty');
F25 = busElemCre('StructPp2', 'Bus: StructProperty');
F26 = busElemCre('StructNp2', 'Bus: StructProperty');
SatProperty = Simulink.Bus; 
SatProperty.Elements = [F1, F2, F3, F4, F5, F6, F7, F8, F9, F10, F11, F12, F13, F14, F15, F16, F17, F18, F19, F20, F21, F22, F23, F24, F25, F26];


%% 软件中的默认参数
global Tc_StUse_Enable Tc_StAbr_Enable Tc_MagEquUse_Enable Tc_SadaCtrlMethod Tc_Bdot_TimeSliceCnt Tc_MagDump_TimeSliceCnt Tc_F107...
       Tc_StChoiceIndex Tc_GyroChoiceIndex Tc_AssChoiceIndex...
       Tc_AttDeterChoiceIndex1 Tc_AttDeterChoiceIndex2 Tc_AttDeterChoiceIndex3;
Tc_StUse_Enable=1;Tc_MagEquUse_Enable=0;Tc_SadaCtrlMethod=[0;0;0;0;0;0];%0-PI,1-滞环
Tc_StAbr_Enable=0;%光行差补偿开关,默认：关
Tc_Bdot_TimeSliceCnt=0;Tc_MagDump_TimeSliceCnt=10;Tc_F107 = F107;  %分时默认关
Tc_StChoiceIndex = {'StAB','StAC','StBC','StA','StB','StC'};%星敏的选择顺序，AB表示双星敏定姿，三星敏的选择顺序可以按需扩展，例如：{'StAB','StAC','StBC','StA','StB','StC'}，仅配置一组值有强选的效果
Tc_GyroChoiceIndex = {'GyroA','GyroB'};%陀螺的选择顺序，仅配置一组值有强选的效果
Tc_AssChoiceIndex = {'AssA','AssB','AssC','AssD'};%星体上参与解算太阳矢量的太敏组合
Tc_AttDeterChoiceIndex1 = {'AssGyro','Ass','Gyro'};%基准规则一：太敏+陀螺，单太敏，单陀螺
Tc_AttDeterChoiceIndex2 = {'StGyro','St','AssGyro','Ass','Gyro','AssMmGyro','AssMm'};%基准规则二：星敏陀螺，单星敏，太敏陀螺，单太敏，单陀螺，双矢量陀螺，双矢量
Tc_AttDeterChoiceIndex3 = {'StGyro','St','Gyro'};%基准规则三：星敏陀螺，单星敏，单陀螺
global FwCtrlCoeff MagCtrlCoeff SadaCtrlCoeff QVCtrlCoeff NotchFiterCoeff MtAllocationCoeff ;
MtAllocationCoeff = [1 0 0;0 1 0;0 0 1;0 0 0;0 0 0;0 0 0];  % 磁棒分配矩阵
PDCoeff1 = struct('lamda',1.5*[0.2;0.2;0.2],'xi',[1;1;1],'ki',[0;0;0],'w_ui_max',[0.02;0.02;0.02],'w_max',[2;2;2]);
PIDCoeff1 = struct('lamda',1.5*[0.2;0.2;0.2],'xi',[1;1;1],'ki',[0.01;0.01;0.01],'w_ui_max',[0.02;0.02;0.02],'w_max',[2;2;2]);
PDTCoeff1 = struct('lamda',[0.1;0.1;0.1]*1+0*[0.0257;0.0257;0.0257],'xi',[0;0;0],'ki',[0;0;0],'w_ui_max',[0.02;0.02;0.02],'w_max',[2;2;2]);
PIDTCoeff1 = struct('lamda',[0.1;0.1;0.1],'xi',[0;0;0],'ki',1*[0.005;0.005;0.005],'w_ui_max',[0.02;0.02;0.02],'w_max',[2;2;2]);% 展开前
PIDTCoeff2 = struct('lamda',[0.15;0.15;0.15]*0+1*[0.1;0.1;0.1],'xi',[0;0;0],'ki',1*[0.005;0.005;0.005],'w_ui_max',[0.02;0.02;0.02],'w_max',[2;2;2]);% 展开后

DTCoeff1 = struct('lamda',[0.1;0.1;0.1],'xi',[0.8;0.8;0.8],'ki',[0;0;0],'w_ui_max',[0;0;0],'w_max',[0.5;0.5;0.5]);
DTCoeff2 = struct('lamda',[0.0257;0.0257;0.0257],'xi',[0.8;0.8;0.8],'ki',[0;0;0],'w_ui_max',[0;0;0],'w_max',[0.5;0.5;0.5]);
PDCoeff = struct('Coeff1',PDCoeff1);
PIDCoeff = struct('Coeff1',PIDCoeff1);
PDTCoeff = struct('Coeff1',PDTCoeff1);
PIDTCoeff = struct('Coeff1',PIDTCoeff1,'Coeff2',PIDTCoeff2);
DTCoeff = struct('Coeff1',DTCoeff1,'Coeff2',DTCoeff2);
FwCtrlCoeff = struct('PDCoeff',PDCoeff,'PIDCoeff',PIDCoeff,'PDTCoeff',PDTCoeff,'PIDTCoeff',PIDTCoeff,'DTCoeff',DTCoeff,'StaticSpd',1000);
SadaCtrlCoeff = struct('Kp',[0.001;0.001;0.001;0.001;0.001],'Ki',[0;0;0;0;0;0],'Kd',[0.05;0.05;0.05;0.05;0.05;0.05],...
    'Ui_max',0*[0.005;0.005;0.005;0.005;0.005;0.005],'w_max_ZL',[0.6;1;0.6;1;1;1],'w_max',[0.6;0.2;0.6;0.2;0.2;0.2],'a_max',[0.04;0.02;0.04;0.02;0.02;0.02],...
    'ZH_L1',[4;4;4;4;4;4],'ZH_L2',[2;2;2;2;2;2],'ZH_W0',[0;0;0;0;0;0],'ZH_dW',[0.002;0.002;0.002;0.002;0.002;0.002],'ZH_Wmax',[0.6;0.6;0.6;0.6;0.6;0.6],...
    'RotType',["No";"Stop";"No";"Stop";"No";"No"],'WorkArea',[-180 -45 -180 -45 -180 -180;180 45 180 45 180 180]');% No/Around/Stop
QVCtrlCoeff = struct('Kp',[0.4;0.4;0.4;0.4],'Ki',[0;0;0;0],'Kd',[1;1;1;1],...
    'Ui_max',[0;0;0;0],'w_max_ZL',[0.6;1;0.6;1],'w_max',[0.6;0.6;0.6;0.6],'a_max',[0.04;0.01;0.04;0.01],...
    'RotType',["Around";"Around";"Around";"Around"],'WorkArea',[-62 -62 -62 -62;62 62 62 62]');
MagCtrlCoeff = struct('WFdbk',-30000000,'Bdot',-80000000,'MagDump',[-0.002;-0.002;-0.002]);
NotchFiterCoeff = struct('lamda',1,'h',0.2,'f',0.035);% f单位Hz
%% 读取gitHash值
git_hash_Project = get_git_hash();
git_hash_BaseLine = get_git_hash('baseline');
%%
%模型编译时TestMode是0，仿真时TestMode是1
load_system('baseline/GS_model.slx');
if TestMode == 1
    GS_Project = find_system('GS_model', 'IncludeCommented', 'on', 'Commented', 'on', 'Name', 'ProjectModel');
    for i = 1:numel(GS_Project)% 将模块设置为“uncomment”状态
        blockHandle = cell2mat(GS_Project(i));
        set_param(blockHandle, 'Commented', 'off');
    end
elseif TestMode == 0
    GS_Project = find_system('GS_model', 'Name', 'ProjectModel');
    for i = 1:numel(GS_Project)% 将模块设置为“commented-out”状态
        blockHandle = cell2mat(GS_Project(i));
        set_param(blockHandle, 'Commented', 'on');
    end
end
load_system('ProjectModel.slx');
if TestMode == 1
    Project_Test = find_system('ProjectModel', 'IncludeCommented', 'on', 'Commented', 'on', 'Name', 'TestModel');
    for i = 1:numel(Project_Test)% 将模块设置为“uncomment”状态
        blockHandle = cell2mat(Project_Test(i));
        set_param(blockHandle, 'Commented', 'off');
    end
elseif TestMode == 0
    Project_Test = find_system('ProjectModel', 'Name', 'TestModel');
    for i = 1:numel(Project_Test)% 将模块设置为“comment-out”状态
        blockHandle = cell2mat(Project_Test(i));
        set_param(blockHandle, 'Commented', 'on');
    end
end

function git_hash = get_git_hash(directory)
    % This function gets the current Git hash of the repository
    if nargin < 1 || isempty(directory) || strcmp(directory, '.')
        % 如果未提供目录或目录为当前目录
        directory = pwd;  % 设置为当前目录
    end
    currentDir = pwd;
    cd(directory);
    % Execute the git command to get the latest commit hash
    [status, cmdout] = system('git rev-parse HEAD');
    cd(currentDir);
    if status == 0
        % If the command executed successfully, return the hash
        git_hash_str= strtrim(cmdout);  % Remove any trailing newlines or spaces
        git_hash_str = git_hash_str(1:8);
        git_hash = hex2dec(git_hash_str);
    else
        % If there was an error, display an error message and return empty
        git_hash = 0;
    end 
end

function InitFcn()
    addpath 'Baseline\';
    UTC_1970to2000 = 946728000;% 1970.1.1 00:00:00 到 2020.1.1  12:00:00的UTC秒数
    UTC_0to1970 = 2440587.5*86400;

    UTC_0to2000 =2451545*86400;
    K_Day2Sec = 86400;
    K_Century2Day = 36525;
    miu = 3.9860044e14;
    DEG2RAD = pi/180;RAD2DEG = 180/pi;
    RPM2RadPerSec = pi/30; % rpm 到 rad/s 的转换系数
    RadPerSec2RPM = 30/pi; % rad/s 到 rpm 的转换系数
    we = 7.2921e-5;%地球自转角速度，rad/s
    Re = 6371.14e+3;
    assignin('base','UTC_1970to2000',UTC_1970to2000);
    assignin('base','UTC_0to1970',UTC_0to1970);
    assignin('base','UTC_0to2000',UTC_0to2000);
    assignin('base','K_Day2Sec',K_Day2Sec);
    assignin('base','K_Century2Day',K_Century2Day);
    assignin('base','miu',miu);
    assignin('base','DEG2RAD',DEG2RAD);
    assignin('base','RAD2DEG',RAD2DEG);
    assignin('base','RPM2RadPerSec',RPM2RadPerSec);
    assignin('base','RadPerSec2RPM',RadPerSec2RPM);
    assignin('base','we',we);
    assignin('base','Re',Re);
end

function [m, r, I, FixPoint, Aba] = SetMassProperties(M)
    m = M(1,1); % 质量
    r = M(2,1:3)'; % 质心
    I = SetMomentofInertia(M(3:4,1:3)); % 转动惯量
    FixPoint = M(5,1:3)'; % 安装位置坐标
    Aba = M(6:8,1:3); % 安装矩阵
end

function [S, n, FigCent] = SetStructProperties(M)
    S = M(1,1); % 面积
    n = M(2,1:3)'; % 法线
    FigCent = M(3,1:3)'; % 形心
end

function [Choice, Fre, Ksi, Bt, Br] = SetFlexProperties(M, NumMode)
    Choice = zeros(NumMode,1); % 挠性选取阶次
    Fre = zeros(NumMode,1);
    Ksi = zeros(NumMode,1);
    Bt = zeros(3,NumMode);
    Br = zeros(3,NumMode);
    for row = 1:NumMode
        Choice(row) = M(row,1);
        Fre(row) = M(row + NumMode,1);
        Ksi(row) = M(row + 2*NumMode,1);
        Bt(:,row) = M(row + 3*NumMode,:)';
        Br(:,row) = M(row + 4*NumMode,:)';
    end
end

function I = SetMomentofInertia(M)
    I = zeros(3,3);
    I(1,1) = M(1,1);
    I(2,2) = M(1,2);
    I(3,3) = M(1,3);
    I(1,2) = M(2,1);
    I(2,1) = M(2,1);
    I(1,3) = M(2,2);
    I(3,1) = M(2,2);
    I(2,3) = M(2,3);
    I(3,2) = M(2,3);
end

%% 辅助函数：创建 BusElement 对象
function elem = busElemCre(name, dims)
    elem = Simulink.BusElement;
    elem.Name = name;    
    if isnumeric(dims)
        elem.Dimensions = dims;
        elem.DataType = 'double';
    else
        elem.Dimensions = 1; % 如果dims是字符型，如 'Bus: FlexP1Property'
        elem.DataType = dims;
    end
end