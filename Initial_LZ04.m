clear all;clc;tic;%bdclose all;
% slCharacterEncoding('ISO-8859-1');
global T_ctrl;
NaoXingOn = 0;
if NaoXingOn == 1
    T=0.01;T_ctrl=0.5;                %仿真步长/s
else
%     T = 0.0125;T_ctrl=1/8;        %控制频率8Hz
    T = 0.125;T_ctrl=0.25;        %控制频率2Hz
end
mu=3.9860044e14;
omegaearth=7.2921e-5;%地球自转角速度，rad/s
J2 = 0.00108263;
Re=6371.14e+3;      
%% GS-1a轨道参数
Sunshine0 = 1;        % f =0 对应阴影区，f=175对应光照区
Year=2024;Month=10 ;Day=1;Hour=04;Min=0;Sec=0;% 02AP星标称轨道 
% Year=2022;Month=11;Day=13;Hour=04;Min=0;Sec=0;% 02AP星标称轨道 
a=6898.14e3;e=1.45695e-15;i=97.4506/180*pi;omg=0/180*pi;OMG=347.55/180*pi;f=359.865/180*pi;
StartTime=[Year Month Day Hour Min Sec]';
w0 = sqrt(mu/a^3);%卫星轨道角速度值
H=a-Re;  % 高度
F107 = 150;% F10.7系数 65/75/100/125/150/175/200/225/250/275
% disp(2*pi/w0);
%% 初始姿态
Att0=0*[80 0 0]'*pi/180;% 磁阻尼
W_bi0=1*[0;-w0;0]+0*[2;-2;2]*pi/180;
MtInfluence = 0;
OpenLoop_Att = 0;% 1开环  0闭环
OpenLoop_Obt = 0;
global Mode SubMode index Mode_last SubMode_last index_last SunDirFace SafeModeStepFlag SunDirFace_last ModeChange SubModeChange SunDirFace_Tgt;
Mode = 3;SubMode = 1;SunDirFace=0;index = [0;1];%   0--Orbit;1--Inertial;     +Y超前[0;2]  +X超前[0;1]
Mode_last=Mode;SubMode_last=SubMode;index_last=index;SafeModeStepFlag=1;SunDirFace_last=1; ModeChange=0;SubModeChange=0;SunDirFace_Tgt=0;

global FwMode FwMode_last;FwMode = [1;1;1;1];FwMode_last=FwMode;
H_Fw0 = [0;0;0;0];
global Allow_AttRefToGyro  Allow_KalmanUse_Tc Allow_KalmanCal_Tc Drift_GyroA_Tc Drift_GyroB_Tc;  % 陀螺相关
Allow_AttRefToGyro=1;Allow_KalmanUse_Tc=0;Allow_KalmanCal_Tc=0;Drift_GyroA_Tc=[0;0;0];Drift_GyroB_Tc=[0;0;0];
global AttRef_Ok Agl_Deter W_Deter AttRefFlag MagRefFlag FwCtrlFlag W_Target Att4CtrlRefFlag MagCtrlFlag Agl4Ctrl W4Ctrl w_Tgt theta_Tgt a_f_Tgt;
AttRef_Ok=0;Agl_Deter=[0;0;0];W_Deter=[0;0;0];AttRefFlag=0;MagRefFlag=0;FwCtrlFlag=[0;0;0];W_Target=[0;0;0];Att4CtrlRefFlag=[0;0;0];MagCtrlFlag=0;Agl4Ctrl=[0;0;0];W4Ctrl=[0;0;0];w_Tgt=0; theta_Tgt=0; a_f_Tgt=0;
global indexForce MagRefForce AttRefForce Att4CtrlRefForce FwCtrlForce W_TargetForce MagCtrlForce;  % 强选基准及控制算法
indexForce=[9;9];MagRefForce=9;AttRefForce=9;Att4CtrlRefForce=9;FwCtrlForce=[9;9;9];W_TargetForce=[0;0;0];MagCtrlForce=9;
global FwModeForce;
FwModeForce=[9;9;9;9];
global MagRefChoice MmPriority;
MagRefChoice=0;MmPriority=1;
global StForceFlag StPriorityFlag Allow_StAlign Allow_DoubleSt Qib_DoubleSt Allow_StUse;   % 星敏相关
StForceFlag=0;StPriorityFlag=1;Allow_StAlign=0;Allow_DoubleSt=1;Qib_DoubleSt=[1;0;0;0];Allow_StUse=1;
Noise_On=1;
Kg_Noise_Mm=Noise_On;Kg_Noise_St=Noise_On;Kg_FixErr_St=0+0*Noise_On;Kg_Noise_Orbit=Noise_On*1;
Kg_Drift_Gyro=Noise_On*0;Kg_Noise_Gyro=Noise_On*1;Kg_Noise_Ass=Noise_On;
global Gyro_Ok_Set StOk_Set Qnb_St Alpha_Tgt;  % --LC  testw
Gyro_Ok_Set=[1;1];StOk_Set=1;Qnb_St=[1;0;0;0];Alpha_Tgt=0;
global RapidManeuver;
RapidManeuver = [0;0;0];
%%
% 计算惯性系下的初始位置和速度
p=a*(1-e^2);
r0=p/(1+e*cos(f));
x0=r0*(cos(omg+f)*cos(OMG)-sin(omg+f)*cos(i)*sin(OMG));
y0=r0*(cos(omg+f)*sin(OMG)+sin(omg+f)*cos(i)*cos(OMG));
z0=r0*sin(omg+f)*sin(i);
sp=[cos(omg)*cos(OMG)-sin(omg)*sin(OMG)*cos(i);cos(omg)*sin(OMG)+sin(omg)*cos(OMG)*cos(i);sin(omg)*sin(i)];
sq=[-sin(omg)*cos(OMG)-cos(omg)*sin(OMG)*cos(i);-sin(omg)*sin(OMG)+cos(omg)*cos(OMG)*cos(i);cos(omg)*sin(i)];
vx0=sqrt(mu/a/(1-e^2))*(-sin(f)*sp(1)+(e+cos(f))*sq(1));
vy0=sqrt(mu/a/(1-e^2))*(-sin(f)*sp(2)+(e+cos(f))*sq(2));
vz0=sqrt(mu/a/(1-e^2))*(-sin(f)*sp(3)+(e+cos(f))*sq(3));
MiSec=696;JDC = (367*Year-floor(7*(Year+floor((Month+9)/12))/4)+floor(275*Month/9)+Day+1721013.5+(Hour+Min/60.0+Sec/3600+MiSec/1000/3600)/24-2451545.0)/36525.0;
Centroi = [0.477;-0.0133;-0.0266];

%% 数据来源：20231226更新
m = 350;
Ibxx=101;Ibyy=109;Ibzz=108;           %拉偏10%
Ibxy=-3.03;Ibxz= 11.24;Ibyz=15.02;
Ib0_0 = [Ibxx Ibxy Ibxz;Ibxy Ibyy Ibyz;Ibxz Ibyz Ibzz];% 帆板展开前转动惯量

global FwTmax FwHmax jw;
jw=0.0064;         % 反作用飞轮转动惯量
FwTmax0 = 0.1;    % 反作用飞轮最大力矩0.1Nm
FwTmax = FwTmax0;
FwHmax = 4;
MaxRWn= FwHmax/jw;%飞轮最大转速，rad/s
Fw_Mode = 1;   %1--转矩模式，2--转速模式
Fw_tao = 0.3;  %飞轮响应时间

% 磁卸载系数
global K_MagUnload Mag_max;
K_MagUnload=2*[10;10;10];%100磁棒用8  70用5  25用2 
Mag_max=25;  % 磁棒的最大磁矩Am^2，阈值
global Matrix_I0 Matrix_I Hw0;
Matrix_I0 = [sin(50/180*pi)*cos(45/180*pi) -sin(50/180*pi)*cos(45/180*pi) -sin(50/180*pi)*cos(45/180*pi) sin(50/180*pi)*cos(45/180*pi);...
          sin(50/180*pi)*sin(45/180*pi) sin(50/180*pi)*sin(45/180*pi) -sin(50/180*pi)*sin(45/180*pi) -sin(50/180*pi)*sin(45/180*pi);...
          cos(50/180*pi) cos(50/180*pi) cos(50/180*pi) cos(50/180*pi)];
Matrix_I=Matrix_I0;
Hw0=1000*[1;-1;1;-1]*pi/30*jw;
H0 = 0*[2;-2;-2;0];
sss = Matrix_I'/(Matrix_I*Matrix_I');
FwFrictionOn = 0;

% 算法1三轴PIT控制指令(对应惯量20)
global T_PID T_Fw H_Fw MagCmd T_Mag;
T_PID=[0;0;0];T_Fw=[0;0;0;0];H_Fw=[0;0;0;0];MagCmd=[0;0;0];T_Mag=[0;0;0];

global K_MagDamp;K_MagDamp = 8e3;
global tao_Mag;tao_Mag = 1*0.5;    % 用于计算磁场强度微分的滤波值
global Kp_Mag Ki_Mag Kt_Mag;
Kp_Mag=[0.00095;0.00074;0.00048];   % lamda = 0.02;ksi =0.8 磁控PIT系数
Ki_Mag=[0.0000064;0.0000049;0.0000032];
Kt_Mag=[0.006;0.006;0.006];
global Kp_Mag1 Kd_Mag1;% 磁控能量法PD系数
Kp_Mag1 = [79.47;61.86;39.58]*0.02^2;
Kd_Mag1 = 2*[79.47;61.86;39.58]*0.02*0.8;

%磁强计
Abm_MmA = [0 -1 0;1 0 0;0 0 1];  % 安装矩阵
Abm_MmB = [0 -1 0;1 0 0;0 0 1];  % 安装矩阵

Noise_Mm = 7e-5*Kg_Noise_Mm;  % 70uGs(3sigma)
global Mag_sys dBb MagCtrl_First Mag_TD dBb_TD;% 磁场微分
Mag_sys=[0;0;0];dBb=[0;0;0];MagCtrl_First=1;Mag_TD=[0;0;0];dBb_TD=[0;0;0];


%星敏感器参数  20190411更新，安装矩阵参见正样极性报告，噪声参见正样设计报告（清华星敏）
global Qbs_StAFix Qbs_StBFix;
Qbs_StAFix=quatmultiply(angle2quat(pi,0,0,'XYZ'),angle2quat(-40/180*pi,20/180*pi,0,'XYZ'));
Qbs_StBFix=quatmultiply(angle2quat(pi,0,0,'XYZ'),angle2quat(40/180*pi,20/180*pi,0,'XYZ'));
Noise_StA=[3;3;30]*Kg_Noise_St;%星敏2a测量误差值，白噪声3σ，单位角秒
Noise_StB=[3;3;30]*Kg_Noise_St;%星敏2b测量误差值，白噪声3σ，单位角秒
FixErr_StA=[15;15;15]*0*Kg_FixErr_St;%星敏2a安装及热变形误差，常值，单位角秒
FixErr_StB=([15;15;15]*0+[10800;0;0])*Kg_FixErr_St;%星敏2b安装及热变形误差，常值，单位角秒
stepOMG=3.7e-3 * Kg_Noise_Orbit;  stepi=6e-4  * Kg_Noise_Orbit;  stepu=0.015  * Kg_Noise_Orbit;%轨道递推六根数误差，常值，单位deg
global tao_St;  % 星敏伪速率滤波时间常数
tao_St = 0;
global St_Ok Agl_St AglRate_St StAttCal_First Qib_StA Qib_StB AssMagAttCal_First
St_Ok=0;Agl_St=[0;0;0];AglRate_St=[0;0;0];StAttCal_First=1;Qib_StA=[1;0;0;0];Qib_StB=[1;0;0;0];AssMagAttCal_First=1;


% 陀螺参数
global Abg_GyroA Abg_GyroB;
Abg_GyroA = [0 0 1;0 1 0;-1 0 0];    % 安装矩阵，陀螺到本体
Abg_GyroB = angle2dcm(-pi*3/4,0,-58/180*pi,'ZYX');
Ngc1=[0.01;-0.02;-0.03]*Kg_Drift_Gyro*1+0*[0.001;-0.001;-0.001];%陀螺常值漂移，单位deg/s
Ngc2=[-0.02;0.01;-0.03]*Kg_Drift_Gyro;
NgwxA=0.005/60*sqrt(1/T_ctrl)*3*Kg_Noise_Gyro;NgwyA=NgwxA;NgwzA=NgwxA;% 光纤陀螺白噪声3σ，单位deg/s   对应0.005deg/sqrt(h)
NgwxB=0.02/60*sqrt(1/T_ctrl)*3*Kg_Noise_Gyro;NgwyB=NgwxB;NgwzB=NgwxB;% MEMS陀螺白噪声3σ，单位deg/s   对应0.02deg/sqrt(h)

global Gyro_Ok Agl_Gyro AglRate_Gyro Qnb_Gyro;
Gyro_Ok=0;Agl_Gyro=[0;0;0];AglRate_Gyro=[0;0;0];Qnb_Gyro=[0;0;0];
global Drift_GyroA Drift_GyroB Drift_GyroC Delta_Drift_GyroA Delta_Drift_GyroB Delta_Drift_GyroC Kalman_GyroA Kalman_GyroB;
Drift_GyroA=[0;0;0];Drift_GyroB=[0;0;0];Drift_GyroC=[0;0;0];Delta_Drift_GyroA=[0;0;0];Delta_Drift_GyroB=[0;0;0];Delta_Drift_GyroC=[0;0;0];
Kalman_GyroA = W_bi0; Kalman_GyroB=W_bi0;
% Kalman_GyroA = [0;0;0]; Kalman_GyroB=[0;0;0];

global AttRefChoice;
AttRefChoice=0;
global StGyroRef_Ok Agl_StGyroRef AglRate_StGyroRef;
StGyroRef_Ok=0;Agl_StGyroRef=[0;0;0];AglRate_StGyroRef=[0;0;0];
global SingleStRef_Ok Agl_SingleStRef AglRate_SingleStRef;
SingleStRef_Ok=0;Agl_SingleStRef=[0;0;0];AglRate_SingleStRef=[0;0;0];
global AssGyroRef_Ok Agl_AssGyroRef AglRate_AssGyroRef;
AssGyroRef_Ok=0;Agl_AssGyroRef=[0;0;0];AglRate_AssGyroRef=[0;0;0];
global SingleAssRef_Ok Agl_SingleAssRef AglRate_SingleAssRef;
SingleAssRef_Ok=0;Agl_SingleAssRef=[0;0;0];AglRate_SingleAssRef=[0;0;0];
global SingleGyroRef_Ok Agl_SingleGyroRef AglRate_SingleGyroRef;
SingleGyroRef_Ok=0;Agl_SingleGyroRef=[0;0;0];AglRate_SingleGyroRef=[0;0;0];


global RunTimer;RunTimer = 1;

global DesNodeTimeArea DesNodeTime_First_Tc;
DesNodeTimeArea=2;DesNodeTime_First_Tc=0;%发射时间降交点地方时9：55~10:05，位于区间2

% global Time0 Att_q0 Hw_Re0 Tset;  %  路径规划用参数
% Time0 = 0;Att_q0=[1;0;0;0];Hw_Re0=[0;0;0;0];Tset=0;

toc