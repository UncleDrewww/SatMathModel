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
% R=Re+H;
Sunshine0 = 1;        % f =0 对应阴影区，f=175对应光照区
% Year=2020;Month=2;Day=26;Hour=04;Min=0;Sec=0;%%%%UTC 2020-02-26 04:00:00   02星标称轨道
% a=7553369.655000004;e=0.002700004503512;i=1.509674896390055;omg=1.579540411752145;OMG=1.745329251994330;f=4.712406433677209-omg;%标称轨道对应顺根(轨控测试用)
Year=2024;Month=10 ;Day=1;Hour=04;Min=0;Sec=0;% 02AP星标称轨道 
% Year=2022;Month=11;Day=13;Hour=04;Min=0;Sec=0;% 02AP星标称轨道 
a=6898.14e3;e=1.45695e-15;i=97.4506/180*pi;omg=0/180*pi;OMG=347.55/180*pi;f=359.865/180*pi;
Qio0 = quatnormalize([-0.961738510;0.034921054;-0.271495046;-0.011401662]'*(Sunshine0==1)+(Sunshine0==0)*[-0.708515992;0.007202676;0.704737974;-0.03602221]')';
AscDec_Sun = [3.647185852 6.074235943];
StartTime=[Year Month Day Hour Min Sec]';
w0 = sqrt(mu/a^3);%卫星轨道角速度值
H=a-Re;  % 高度
F107 = 150;% F10.7系数 65/75/100/125/150/175/200/225/250/275
% disp(2*pi/w0);
%% 初始姿态
Att0=1*[80 0 0]'*pi/180;% 磁阻尼
W_bi0=1*[0;-w0;0]+0*[2;-2;2]*pi/180;
MtInfluence = 0;
OpenLoop_Att = 0;% 1开环  0闭环
OpenLoop_Obt = 0;
global Mode SubMode index Mode_last SubMode_last index_last SunDirFace SafeModeStepFlag SunDirFace_last ModeChange SubModeChange SunDirFace_Tgt;
Mode = 3;SubMode = 1;SunDirFace=0;index = [0;1];%   0--Orbit;1--Inertial;     +Y超前[0;2]  +X超前[0;1]
% Mode = 1;SubMode = 1;SunDirFace=0;index = [1;SunDirFace];%
Mode_last=Mode;SubMode_last=SubMode;index_last=index;SafeModeStepFlag=1;SunDirFace_last=1; ModeChange=0;SubModeChange=0;SunDirFace_Tgt=0;

global FwMode FwMode_last;FwMode = [1;1;1;1];FwMode_last=FwMode;
H_Fw0 = [0;0;0;0];
global OrbitCtrlStartTime OrbitCtrlDuration OrbitCtrlDir;   % 轨控相关
OrbitCtrlStartTime=0*4000;OrbitCtrlDuration=1*600;OrbitCtrlDir=0;     % 时长单位s;轨控方向:0升高,1降低,2+90,3-90；
global AllowMode4StateChange;
AllowMode4StateChange=0;
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
global Allow_KalmanUse Allow_KalmanCal Allow_KalmanCal_last;
Allow_KalmanUse=0;Allow_KalmanCal=1;Allow_KalmanCal_last=0;
global Allow_QVCtrl_Tc Allow_StareCtrl_Tc R_Tgt Allow_FYJJCtrl_Tc Allow_StareCtrl_Tc_last Allow_FYJJCtrl_Tc_last; % 允许对地凝视控制、俯仰渐进控制
Allow_QVCtrl_Tc=0;Allow_StareCtrl_Tc=0;R_Tgt=[0;0;0];Allow_FYJJCtrl_Tc=0;Allow_StareCtrl_Tc_last=0;Allow_FYJJCtrl_Tc_last=0;
global Allow_SADAWTCtrl_Tc Allow_SADACopy_Tc SADA_Priority SADA_GL_State SADA_ZL_State SADA_ACtrlFlag_Tc UnlockSpread0;% SADA相关
Allow_SADAWTCtrl_Tc=[0;0];Allow_SADACopy_Tc=0;SADA_Priority=0;SADA_GL_State=[0;0;0;0];SADA_ZL_State=[0;0;0;0];SADA_ACtrlFlag_Tc=1;UnlockSpread0=0*[1;1];
Noise_On=1;
Kg_Noise_Mm=Noise_On;Kg_Noise_St=Noise_On;Kg_FixErr_St=0+0*Noise_On;Kg_Noise_Orbit=Noise_On*1;
Kg_Drift_Gyro=Noise_On*0;Kg_Noise_Gyro=Noise_On*1;Kg_Noise_Ass=Noise_On;
global Gyro_Ok_Set StOk_Set Qnb_St Alpha_Tgt;  % --LC  testw
Gyro_Ok_Set=[1;1];StOk_Set=1;Qnb_St=[1;0;0;0];Alpha_Tgt=0;
global  theta_r w_r a_r;%俯仰渐进临时用
theta_r=0;w_r=0;a_r=0;
global ImagingMode;   % 相机成像模式
ImagingMode = 1;
global Band_1 Band_2 Band_3;
Band_1 = [0;0;0];Band_2=[0;0;0];Band_3=[0;0;0];
%%
% 计算惯性系下的初始位置和速度
% Coe0=[a e i OMG omg w0 f];
% rv0=RV_from_OrbitalElement(Coe0);

p=a*(1-e^2);
r0=p/(1+e*cos(f));
% v0=sqrt(mu*(2/r0-1/a));
% sg=e*sin(f)/sqrt(1+e^2+2*e*cos(f));
% cg=sqrt(1-sg^2);
x0=r0*(cos(omg+f)*cos(OMG)-sin(omg+f)*cos(i)*sin(OMG));
y0=r0*(cos(omg+f)*sin(OMG)+sin(omg+f)*cos(i)*cos(OMG));
z0=r0*sin(omg+f)*sin(i);
% x0=1184844.6264962;y0=-3166067.5090052;z0=6192267.2136932;
% vx0=v0*(sg*(cos(omg+f)*cos(OMG)-sin(omg+f)*cos(i)*sin(OMG))+cg*(-sin(omg+f)*cos(OMG)-cos(omg+f)*cos(i)*sin(OMG)));% x肖业伦P53
% vy0=v0*(sg*(cos(omg+f)*sin(OMG)+sin(omg+f)*cos(i)*c'os(OMG))+cg*(-sin(omg+f)*sin(OMG)+cos(omg+f)*cos(i)*cos(OMG)));
% vz0=v0*(sg*sin(omg+f)*sin(i)+cg*cos(omg+f)*sin(i));
sp=[cos(omg)*cos(OMG)-sin(omg)*sin(OMG)*cos(i);cos(omg)*sin(OMG)+sin(omg)*cos(OMG)*cos(i);sin(omg)*sin(i)];
sq=[-sin(omg)*cos(OMG)-cos(omg)*sin(OMG)*cos(i);-sin(omg)*sin(OMG)+cos(omg)*cos(OMG)*cos(i);cos(omg)*sin(i)];
vx0=sqrt(mu/a/(1-e^2))*(-sin(f)*sp(1)+(e+cos(f))*sq(1));
vy0=sqrt(mu/a/(1-e^2))*(-sin(f)*sp(2)+(e+cos(f))*sq(2));
vz0=sqrt(mu/a/(1-e^2))*(-sin(f)*sp(3)+(e+cos(f))*sq(3));
MiSec=696;JDC = (367*Year-floor(7*(Year+floor((Month+9)/12))/4)+floor(275*Month/9)+Day+1721013.5+(Hour+Min/60.0+Sec/3600+MiSec/1000/3600)/24-2451545.0)/36525.0;
% rv_J2000 = MatrixFrom84toJ2000([JDC;-3157792.019;6337719.802;29186.581;72.886794;2.508805;7488.112621]);
% x0 = rv_J2000(1);y0 = rv_J2000(2);z0 = rv_J2000(3);% 通过84系下的RV计算J2000下的RV
% vx0 = rv_J2000(4);vy0 = rv_J2000(5);vz0 = rv_J2000(6);
Centroi = [0.477;-0.0133;-0.0266];
%% 数据来源：20231226更新
mb= 345;   % 本体质量 kg
ma1=14.39; % 单翼质量 kg
ma2=14.39;
Ibxx=101;Ibyy=109;Ibzz=108;           %拉偏10%
Ibxy=-3;Ibxz= 9;Ibyz=15;
Ib0=[Ibxx Ibxy Ibxz;Ibxy Ibyy Ibyz;Ibxz Ibyz Ibzz];% 星体相对其自身质心的转动惯量
Cent0_0 = [-91.87; -1.66 ;679.47]/1000;% 帆板展开前质心
Ib0_0 = [117.6 -1.39e-1 -1.14;-1.39e-1 101.9 -8.37e-2;-1.14 -8.37e-2 109.0];% 帆板展开前转动惯量
Ipcxx=2.66;Ipcyy=5.82;Ipczz=8.48;Ipcxy=0.00;Ipcxz=0;Ipcyz=0.000; 
Ipc0=[Ipcxx -Ipcxy -Ipcxz;-Ipcxy Ipcyy -Ipcyz;-Ipcxz -Ipcyz Ipczz];%帆板相对于自身质心的转动惯量（ac系）
rb0=[-91.87 -1.66 679.47]'/1000; % 本体质心坐标（L系）
ra0_PY=[0 603+450 492]'/1000;  % +X帆板a点安装位置坐标（L系） 
ra0_NY=[0 -603-450 492]'/1000;  % -X帆板a点安装位置坐标（L系）
r_a2p_a=[0;0;0]/1000;%  beta安装p点相对于alpha安装a点的位置(a系)%根据工大报告推算的
r_p2pc_p=[1282;-0.55;-10.87]/1000;  % 帆板质心相对于铰链p点的位置（p系）

m=mb+ma1+ma2;
global Aba0_PY Aba0_NY;
Aba0_PY=angle2dcm(-pi/2,0,0,'ZYX');   % 帆板零位安装系到b系的转换矩阵。
Aba0_NY=angle2dcm(pi,0,0,'ZYX')*Aba0_PY;
Agl0_PY = 1*[0;0;0];  % +Y帆板初始角度（+Y帆板的a系），deg,02AP的SADA是XYZ转序
Agl0_NY = Aba0_NY'*Aba0_PY*Agl0_PY;  % -Y帆板初始角度（-Y帆板的a系）
SadaSet=0;SadaSetW=0*[0.06;0;0];%deg/s

Fre0 = [3.482195E-02 5.239632E-02 7.397585E-02 1.107146E-01 1.201705E-01 1.743363E-01 1.783116E-01 2.277798E-01 2.347671E-01 2.569632E-01 2.709669E-01 2.855845E-01 3.409392E-01 3.458544E-01 3.485606E-01 3.679646E-01 4.297235E-01]';% ...  % Hz
Fre=[Fre0;Fre0]*2*pi;
Kexi=0.005*ones(size(Fre,1),1)*1;
% BtBr_HIT = textread('BtBr_HIT.txt');% 工大报告里的（工大SADA系）
% BtBr_HIT = NaoXingOn*BtBr_HIT;
% Bt_PY_0_HIT = BtBr_HIT(1:3,:);Bt_NY_0_HIT = BtBr_HIT(4:6,:);Br_PY_0_HIT = BtBr_HIT(7:9,:);Br_NY_0_HIT = BtBr_HIT(10:12,:);
% k=12;Bt_PY_P20_HIT = BtBr_HIT(1+k:3+k,:);Bt_NY_P20_HIT = BtBr_HIT(4+k:6+k,:);Br_PY_N20_HIT = BtBr_HIT(7+k:9+k,:);Br_NY_N20_HIT = BtBr_HIT(10+k:12+k,:);
% k=24;Bt_PY_P40_HIT = BtBr_HIT(1+k:3+k,:);Bt_NY_P40_HIT = BtBr_HIT(4+k:6+k,:);Br_PY_N40_HIT = BtBr_HIT(7+k:9+k,:);Br_NY_N40_HIT = BtBr_HIT(10+k:12+k,:);
% k=36;Bt_PY_P60_HIT = BtBr_HIT(1+k:3+k,:);Bt_NY_P60_HIT = BtBr_HIT(4+k:6+k,:);Br_PY_N60_HIT = BtBr_HIT(7+k:9+k,:);Br_NY_N60_HIT = BtBr_HIT(10+k:12+k,:);
% k=48;Bt_PY_P70_HIT = BtBr_HIT(1+k:3+k,:);Bt_NY_P70_HIT = BtBr_HIT(4+k:6+k,:);Br_PY_N70_HIT = BtBr_HIT(7+k:9+k,:);Br_NY_N70_HIT = BtBr_HIT(10+k:12+k,:);
% Bt_PY_0 = Aba0_PY'*Bt_PY_0_HIT;Bt_NY_0 = Aba0_NY'*Bt_NY_0_HIT;Br_PY_0 = Aba0_PY'*Br_PY_0_HIT;Br_NY_0 = Aba0_NY'*Br_NY_0_HIT;% 转到02星SADA坐标系
% Bt_PY_P20 = Aba0_PY'*Bt_PY_P20_HIT;Bt_NY_P20 = Aba0_NY'*Bt_NY_P20_HIT;Br_PY_N20 = Aba0_PY'*Br_PY_N20_HIT;Br_NY_N20 = Aba0_NY'*Br_NY_N20_HIT;
% Bt_PY_P40 = Aba0_PY'*Bt_PY_P40_HIT;Bt_NY_P40 = Aba0_NY'*Bt_NY_P40_HIT;Br_PY_N40 = Aba0_PY'*Br_PY_N40_HIT;Br_NY_N40 = Aba0_NY'*Br_NY_N40_HIT;
% Bt_PY_P60 = Aba0_PY'*Bt_PY_P60_HIT;Bt_NY_P60 = Aba0_NY'*Bt_NY_P60_HIT;Br_PY_N60 = Aba0_PY'*Br_PY_N60_HIT;Br_NY_N60 = Aba0_NY'*Br_NY_N60_HIT;
% Bt_PY_P70 = Aba0_PY'*Bt_PY_P70_HIT;Bt_NY_P70 = Aba0_NY'*Bt_NY_P70_HIT;Br_PY_N70 = Aba0_PY'*Br_PY_N70_HIT;Br_NY_N70 = Aba0_NY'*Br_NY_N70_HIT;
% A_temp = angle2dcm(pi,0,0,'XYZ');%用于将+beta的耦合矩阵求解-beta的耦合矩阵
% Bt_PY_N20 = A_temp*Bt_PY_P20;Bt_NY_N20 = A_temp*Bt_NY_P20;Br_PY_P20 = A_temp*Br_PY_N20;Br_NY_P20 = A_temp*Br_NY_N20;
% Bt_PY_N40 = A_temp*Bt_PY_P40;Bt_NY_N40 = A_temp*Bt_NY_P40;Br_PY_P40 = A_temp*Br_PY_N40;Br_NY_P40 = A_temp*Br_NY_N40;
% Bt_PY_N60 = A_temp*Bt_PY_P60;Bt_NY_N60 = A_temp*Bt_NY_P60;Br_PY_P60 = A_temp*Br_PY_N60;Br_NY_P60 = A_temp*Br_NY_N60;
% Bt_PY_N70 = A_temp*Bt_PY_P70;Bt_NY_N70 = A_temp*Bt_NY_P70;Br_PY_P70 = A_temp*Br_PY_N70;Br_NY_P70 = A_temp*Br_NY_N70;
BT0 = NaoXingOn*[-0.000422160948646960,3.21288716309082e-05,-0.000967710111096349,8.99854484599996e-05,-0.00186928558623977,0.00366654854343285,2.98174651899886e-05,-0.00277789709998643,0.00737106495631112,0.000302029178972377,0.00187883581922922,0.0118371453502183,0.0126792389864778,-4.88152200184937e-05,0.00106923118338347,-0.00182311265540875,0.0109805146416419;
    0.000708786352033866,0.000691876113939007,-0.0102168093865983,0.00176642382037461,-0.00104760270788394,0.0333745126211306,0.00186057093908390,-0.0187142514504022,0.0508856272710306,0.00883713533333538,0.0230694502887720,0.164259377680308,0.147981189059340,0.00467935122297515,0.0246374218098831,-1.14744072527038,-7.38693127989290;
    6.02804980958906,-0.163472149281597,-0.197228293614597,-0.0489684301658894,2.42647027216970,-0.176334711005518,-0.0742405513301037,0.815386485774605,-2.15188015740648,-0.0111135884441606,-0.179748596512154,-1.20370521424980,-2.43862076551646,0.00542563235398500,-0.103272261252196,-0.228611757373795,-0.0118644369368558];
% BR0=NaoXingOn*[-1.31843E-01	6.33100E+01	-5.16534E-03;
% -5.38694E-01	2.69559E-03	6.41962E+01;
% -3.57024E-02	-7.30280E+00	1.34789E-03;
% -4.32019E+00	1.09047E-04	-8.57385E-02;
% 3.02266E-02	1.59631E+00	8.98944E-04;
% -5.61154E-03	-3.03256E-05	-5.78864E-02]';
BR0 = NaoXingOn*[0.102664433814176,4.09660667611959,0.0643946439055142,-0.287154895137960,0.0413855373807166,0.0375286888316232,1.27516813471463,0.0130392894361858,-0.0289319230594890,-0.260604163182274,0.0117568203366441,0.0330878255197276,0.0106855510252894,0.728990105856417,0.00158934321081155,-0.0713380280385102,-0.451110014151861;
    -29.2824578738051,1.03297930059334,-12.5510469117503,0.612187373098869,-10.9243000996932,9.23489941990979,0.133068123271961,-4.57297804609369,11.4867812564108,0.397418721522950,2.17371953333050,12.4857342619592,14.2398816943960,-0.0343849099631519,0.932796816891522,-2.11950339597841,0.810880849173128;
    -0.00759506420255279,0.00304235760976562,-0.0261785464035069,0.00725431800012363,-0.0411869138446107,0.116798412571128,0.0100448934853457,-0.105463442591491,0.291238843703822,0.0375205303836375,0.103300323195343,0.752912276154520,0.755062207338391,0.0206969437290969,0.116322900863394,-5.44317264409932,-35.3971318551142];

Bt_PY = Aba0_PY'*BT0;
% Br_PY = BR0 - xw(ra0_PY-[   -1.0552 ;   0.0193  ;  0.0398])*BT0;
Br_PY = Aba0_PY'*BR0;
Bt_NY = 0 * Bt_PY;
Br_NY = 0 * Br_PY;

Psa=zeros(3,3);          % 帆板对安装点的静矩（a系）
% % 临时 计算相对于a点的Bt和Br
% Apa = angle2dcm(0,1*pi/3,0,'XYZ');
% Bt_PY_a = Apa'*Bt_PY;
% Br_PY_a = Apa'*Br_PY+xw(r_a2p_a)*Apa'*Bt_PY;
% disp(Bt_PY_a)
% disp(Br_PY_a)
%% 初始姿态设置
Qob0 = angle2quat(Att0(3),Att0(2),Att0(1),'ZYX');
% [Qin0,~,~,Qon0] = Nominal_CSYS_Deter(Qio0,AscDec_Sun(1),AscDec_Sun(2),w0);
qib0=[-0.7647;0.1142;-0.5699;-0.2781];
qob0=[1;0;0;0];
Lx_panel= 15.6*0+12*0+1;                      % 基帆板的长度   改基板长度时记得改帆板质心位置
Ly_panel= 1;                                  % 基板的宽度，帆板长度为3.2×(1.7522*4)   
S_panel=Lx_panel*Ly_panel*3;     % 帆板的面积，共3块帆板
P_sun=1395/(3e+8);
Lx_board=1;                             % 卫星主体在  x  轴方向的长度
Ly_board=1;                             % 卫星主体在  y  轴方向的长度
Lz_board=1;                            % 卫星主体在  z  轴方向的长度
Sx_board=Ly_board*Lz_board ;                          %  x方向卫星主体的面积    
Sy_board=Lx_board*Lz_board ;                          %  y方向卫星主体的面积    
Sz_board=Lx_board*Ly_board;                           %  z方向卫星主体的面积    

rho_panel= 1;                       %  帆板的反射系数
Crs_panel= 0.8;                    %  帆板的镜射系数
Crd_panel= 1-Crs_panel;    %  帆板的漫反射系数
Crs_board= 0.8;                    %  星体各面的镜反射系数
Crd_board= 1-Crs_board;    %  星体各面的漫反射系数

orientation1=[1 0 0]';              %  +x方向卫星板单位方向向量
orientation2=[-1 0 0]';             %  -x方向卫星板单位方向向量
orientation3=[0 1 0]';              %  +y方向卫星板单位方向向量
orientation4=[0 -1 0]';             %  -y方向卫星板单位方向向量
orientation5=[0 0 1]';              %  +z方向卫星板单位方向向量
orientation6=[0 0 -1]';             %  -z方向卫星板单位方向向量

rour0=7.34e-13*0+1.346e-12;               %参考面r=r0上的大气密度
hr0=63.44e3;                   %参考面上的密度标高 hr=hr0+mur*(r-r0)/2;
r0=500e3;                         %参考面的轨道高度           
mur=0.05576;                  %系数
% Crou_q=rour0*exp(-(H-r0)/(hr0+mur*(H-r0)/2));     % rou
% rou = 2.6859e-15;% 1200km大气密度
Cd = 2.2;% 大气阻尼系数
% AttenuationRate = 0.7;%单位m/D,高年2.5，低年0.2

% 信关站经纬高/地面站经纬高
global Long_XGZ Lati_XGZ High_XGZ Long_DMZ Lati_DMZ High_DMZ MinAgl_DMZ;
% Long_XGZ = 2.144808;Lati_XGZ = -1.20331;High_XGZ = 0;% 默认值
% Long_XGZ = -1.51718;Lati_XGZ = 0.27806;High_XGZ = 524.39;  %20min
% Long_XGZ = -1.56109;Lati_XGZ = -1.22932;High_XGZ = 524.39;   %10min新轨道
% Long_XGZ = -1.5977;Lati_XGZ = 0.552032;High_XGZ = 524.39;  %细则中用
% Long_DMZ = -1.5977;Lati_DMZ = 0.552032;High_DMZ = 524.39;  %细则中用
%02星数学仿真报告用，适用于1175kmGW轨道
Long_DMZ = 90/180*pi;Lati_DMZ = 30/180*pi;High_DMZ = 524.39; %经纬高
Long_XGZ = 90/180*pi;Lati_XGZ = 30/180*pi;High_XGZ = 524.39;
    
MinAgl_DMZ = 0.261799;%15deg
global T0_QV1 Tf_QV1 T0_QV2 Tf_QV2 T0_QV Tm_QV Tf_QV QVChoice KaChoice Aop Wop Agl4Ctrl_QV;
T0_QV1=0;Tf_QV1=0;T0_QV2=0;Tf_QV2=0;T0_QV=0;Tm_QV=0;Tf_QV=0;QVChoice=0;KaChoice=1;Aop=[0;0;0];Wop=[0;0;0];Agl4Ctrl_QV=[0;0;0;0];
global T0_Sat Tf_Sat;    %LC
T0_Sat=0;Tf_Sat=0;
% jwB=0.108;        % 动量轮
% TB=0.1;    % ？？？？
global FwTmax FwHmax jw;
jw=0.0064;         % 反作用飞轮转动惯量
FwTmax0 = 0.05;    % 反作用飞轮最大力矩0.1Nm
FwTmax = FwTmax0;
FwHmax = 4;
% T_rwB=0.075;   % 偏置动量轮力矩限幅
MaxRWn= FwHmax/jw;%飞轮最大转速，rad/s
% MaxRWnB=6000*pi/30;%偏置动量轮最大转速6000rpm
Fw_Mode = 2;   %1--转矩模式，2--转速模式
Fw_tao = 0.3;  %飞轮响应时间

w_center=[-20;0;-250]/30*pi*1;

% 磁卸载系数
global K_MagUnload Mag_max;
K_MagUnload=2*[10;10;10];%100磁棒用8  70用5  25用2 
Mag_max=25;  % 磁棒的最大磁矩Am^2，阈值
global Matrix_I0 Matrix_I Hw0;
% Matrix_I0 = [1 0 0 sqrt(1/3);...
%             0 1 0 sqrt(1/3);...
%             0 0 1 sqrt(1/3)];
% 飞轮金字塔布局，四棱锥中垂线沿+x轴
% Matrix_I0 = [-sqrt(1/2) -sqrt(1/2) -sqrt(1/2) -sqrt(1/2);...
%             -1/2     -1/2     1/2     1/2;...
%             1/2     -1/2    -1/2     1/2];
Matrix_I0 = [sin(50/180*pi)*cos(45/180*pi) -sin(50/180*pi)*cos(45/180*pi) -sin(50/180*pi)*cos(45/180*pi) sin(50/180*pi)*cos(45/180*pi);...
          sin(50/180*pi)*sin(45/180*pi) sin(50/180*pi)*sin(45/180*pi) -sin(50/180*pi)*sin(45/180*pi) -sin(50/180*pi)*sin(45/180*pi);...
          cos(50/180*pi) cos(50/180*pi) cos(50/180*pi) cos(50/180*pi)];
Matrix_I=Matrix_I0;
% Hw0=1000*[1;1;1;-sqrt(3)]*pi/30*jw;% 飞轮组合的平衡转速Matrix_D*Matrix_I*[1000;0;0;0]-[1000;0;0;0]
Hw0=1000*[1;-1;1;-1]*pi/30*jw;
H0 = 0*[2;-2;-2;0];
sss = Matrix_I'/(Matrix_I*Matrix_I');
FwFrictionOn = 0;

% 算法1三轴PIT控制指令(对应惯量20)
global T_PID T_Fw H_Fw MagCmd T_Mag;
T_PID=[0;0;0];T_Fw=[0;0;0;0];H_Fw=[0;0;0;0];MagCmd=[0;0;0];T_Mag=[0;0;0];

global K_MagDamp;K_MagDamp = 8e3;
global tao_Mag;tao_Mag = 1*0.5;    % 用于计算磁场强度微分的滤波值
% global Kp_Mag Kd_Mag;
% Kp_Mag=[0.032;0.025;0.016];   % lamda = 0.02;ksi =0.8
% Kd_Mag=[2.54;1.98;1.27];
global Kp_Mag Ki_Mag Kt_Mag;
Kp_Mag=[0.00095;0.00074;0.00048];   % lamda = 0.02;ksi =0.8 磁控PIT系数
Ki_Mag=[0.0000064;0.0000049;0.0000032];
Kt_Mag=[0.006;0.006;0.006];
global Kp_Mag1 Kd_Mag1;% 磁控能量法PD系数
Kp_Mag1 = [79.47;61.86;39.58]*0.02^2;
Kd_Mag1 = 2*[79.47;61.86;39.58]*0.02*0.8;

% QV控制参数
global Kp_QV Ki_QV Ki_Sat_max_QV W_max_QV QVStartTime1 QVDeltaStartTime QVSendToModeChange;
Kp_QV = 1;Ki_QV = .4;Ki_Sat_max_QV = 1;W_max_QV = 1.5;QVStartTime1=33;QVDeltaStartTime=5;QVSendToModeChange=0;

% 太阳光压力矩
% Crou=0.2;   % 反射系数  条件较好
% Cmu=0.8;    % 散射系数
Crou=1;   % 反射系数  条件恶劣
Cmu=0.2;    % 散射系数
S0=S_panel;   % 帆板面积
%　气动力矩
% rour0=7.34e-13*0+1.346e-12;               %参考面r=r0上的大气密度 500km 最高1.346e-12
% hr0=63.44e3;                   %参考面上的密度标高 hr=hr0+mur*(r-r0)/2;
% r0=500e3;                         %参考面的轨道高度           
% mur=0.05576;                  %系数
% Crou_q=rour0*exp(-(H-r0)/(hr0+mur*(H-r0)/2));     % rou
% Cd=2.2;
% V0=sqrt(mu/a);

%% 大气模型参数
HPcoeff
% % Time_step = 60;
% upper_limit =     2000; % Upper height limit [km]
% lower_limit =      100; % Lower height limit [km]
ra_lag      = 0.523599; % Right ascension lag [rad]
n_prm       =        3; % Harris-Priester parameter 
                        % 2(6) low(high) inclination

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
end

%计算剩磁力矩
deadM=2;% 星体剩磁Am2
deadzeroM=[deadM;-deadM;deadM];%星体坐标系下

%磁强计
Abm_MmA = [0 -1 0;1 0 0;0 0 1];  % 安装矩阵
Abm_MmB = [0 -1 0;1 0 0;0 0 1];  % 安装矩阵

Noise_Mm = 7e-5*Kg_Noise_Mm;  % 70uGs(3sigma)
global Mag_sys dBb MagCtrl_First Mag_TD dBb_TD;% 磁场微分
Mag_sys=[0;0;0];dBb=[0;0;0];MagCtrl_First=1;Mag_TD=[0;0;0];dBb_TD=[0;0;0];

% 地球引力势的低阶谐系数 
J2=1.08263*10^(-3);
J3=-2.5321531*10^(-6);
J4=-1.6109876*10^(-6);

%日月引力
Rsun=1.49598*10^11;                             %太阳地球距离（天文单位长度）  
miusun=1.3271244*10^20;                         %太阳引力常数

amoon=384747.981*10^3;                          %月球长半轴
emoon=0.054879905;                              %月球偏心率
imoon=2*asin(0.044751305);                      %月球白道与黄道夹角
miumoon=0.4902802627*10^13;                     %月球引力常数

%星敏感器参数  20190411更新，安装矩阵参见正样极性报告，噪声参见正样设计报告（清华星敏）
global Qbs_StAFix Qbs_StBFix;
Qbs_StAFix=quatmultiply(angle2quat(pi,0,0,'XYZ'),angle2quat(-40/180*pi,20/180*pi,0,'XYZ'));
Qbs_StBFix=quatmultiply(angle2quat(pi,0,0,'XYZ'),angle2quat(40/180*pi,20/180*pi,0,'XYZ'));
Noise_StA=[3;3;30]*Kg_Noise_St;%星敏2a测量误差值，白噪声3σ，单位角秒
Noise_StB=[3;3;30]*Kg_Noise_St;%星敏2b测量误差值，白噪声3σ，单位角秒
FixErr_StA=[15;15;15]*0*Kg_FixErr_St;%星敏2a安装及热变形误差，常值，单位角秒
FixErr_StB=([15;15;15]*0+[10800;0;0])*Kg_FixErr_St;%星敏2b安装及热变形误差，常值，单位角秒
% gpsrx=84.44 ;  gpsry=90.97 ;  gpsrz=115.74 ;%GPS定轨三轴位移误差，白噪声1σ，单位m
% gpsvx=0.05589 ;  gpsvy=0.05541 ;  gpsvz=0.05 ;%GPS定轨三轴速度误差，白噪声1σ，单位m/s
stepOMG=3.7e-3 * Kg_Noise_Orbit;  stepi=6e-4  * Kg_Noise_Orbit;  stepu=0.015  * Kg_Noise_Orbit;%轨道递推六根数误差，常值，单位deg
global tao_St;  % 星敏伪速率滤波时间常数
tao_St = 0;
global St_Ok Agl_St AglRate_St StAttCal_First Qib_StA Qib_StB AssMagAttCal_First
St_Ok=0;Agl_St=[0;0;0];AglRate_St=[0;0;0];StAttCal_First=1;Qib_StA=[1;0;0;0];Qib_StB=[1;0;0;0];AssMagAttCal_First=1;


% 陀螺参数
global Abg_GyroA Abg_GyroB;
Abg_GyroA = [0 0 1;0 1 0;-1 0 0];    % 安装矩阵，陀螺到本体
Abg_GyroB = angle2dcm(-pi*3/4,0,-58/180*pi,'ZYX');
% global Abg_GyroC;
% Abg_GyroC = angle2dcm(pi/4,pi/4,0,'XYZ')*[0 -1 0;-1 0 0;0 0 -1];%
Ngc1=[0.01;-0.02;-0.03]*Kg_Drift_Gyro*1+0*[0.001;-0.001;-0.001];%陀螺常值漂移，单位deg/s
Ngc2=[-0.02;0.01;-0.03]*Kg_Drift_Gyro;
% Ngc3=[0.02;0.01;-0.01]*Kg_Drift_Gyro+0*[0.01;-0.01;0.01];
% Ngwx=0.002;Ngwy=0.002;Ngwz=0.002;%陀螺白噪声3σ，单位deg/h
NgwxA=0.005/60*sqrt(1/T_ctrl)*3*Kg_Noise_Gyro;NgwyA=NgwxA;NgwzA=NgwxA;% 光纤陀螺白噪声3σ，单位deg/s   对应0.005deg/sqrt(h)
NgwxB=0.02/60*sqrt(1/T_ctrl)*3*Kg_Noise_Gyro;NgwyB=NgwxB;NgwzB=NgwxB;% MEMS陀螺白噪声3σ，单位deg/s   对应0.02deg/sqrt(h)

%kalman滤波增益系数
kalp=0.004;%四元数偏差增益系数
kald=-0.00002;%陀螺漂移增益系数

global Gyro_Ok Agl_Gyro AglRate_Gyro Qnb_Gyro;
Gyro_Ok=0;Agl_Gyro=[0;0;0];AglRate_Gyro=[0;0;0];Qnb_Gyro=[0;0;0];
global Drift_GyroA Drift_GyroB Drift_GyroC Delta_Drift_GyroA Delta_Drift_GyroB Delta_Drift_GyroC Kalman_GyroA Kalman_GyroB;
Drift_GyroA=[0;0;0];Drift_GyroB=[0;0;0];Drift_GyroC=[0;0;0];Delta_Drift_GyroA=[0;0;0];Delta_Drift_GyroB=[0;0;0];Delta_Drift_GyroC=[0;0;0];
Kalman_GyroA = W_bi0; Kalman_GyroB=W_bi0;
% Kalman_GyroA = [0;0;0]; Kalman_GyroB=[0;0;0];
% 太敏参数
kS_Ass=0.104;   % 太阳常数*电池片面积，具体数值不祥，仅知最大电流104mA   20190709由104mA改为100mA
R_Ass = 2.5; % 采样电阻2.5欧姆
global AbAssA AbAssB ApAssC ApAssD n_Ass;
n1_Ass = [1/sqrt(2);0;-1/sqrt(2)];   % 太敏系下4个电池片的法线方向
n2_Ass = [0;1/sqrt(2);-1/sqrt(2)];
n3_Ass = [-1/sqrt(2);0;-1/sqrt(2)];
n4_Ass = [0;-1/sqrt(2);-1/sqrt(2)];
n_Ass = [n1_Ass n2_Ass n3_Ass n4_Ass];
AbAssA = angle2dcm(-pi/2,0,pi/2,'ZYX');   %-Y体装板太敏 太敏系到本体系的转换矩阵
AbAssB = angle2dcm(-pi/2,0,pi/2,'ZYX');   %-Y体装板太敏 
ApAssC = eye(3);                          %帆板太敏
ApAssD = eye(3);
% global n_AssA_b n_AssB_b;
% n_AssA_b = AbAssA*[n1_Ass n2_Ass n3_Ass n4_Ass];   % 本体系下太敏电池片法线方向
% n_AssB_b = AbAssB*[n1_Ass n2_Ass n3_Ass n4_Ass];
Noise_Ass = Kg_Noise_Ass * 0.005;  %  太敏电池片噪声(5mV)3sigma
global tao_Ass;
tao_Ass =1;    % 太阳角滤波时间常数
global Ass_Ok Agl_Ass AglRate_Ass AssAttCal_First;
Ass_Ok=0;Agl_Ass=[0;0;0];AglRate_Ass=[0;0;0];AssAttCal_First=1;

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

%无陀螺动力学递推
Kex_NS=0.122;%角速度增益
Key_NS=0.101;
Kez_NS=0.162;
Kwx_NS=0.177;%姿态角增益
Kwy_NS=0.162;
Kwz_NS=0.2;
kg_NS=0;%无敏感器时为0；

global RunTimer;RunTimer = 1;

global DesNodeTimeArea DesNodeTime_First_Tc;
DesNodeTimeArea=2;DesNodeTime_First_Tc=0;%发射时间降交点地方时9：55~10:05，位于区间2

% global Time0 Att_q0 Hw_Re0 Tset;  %  路径规划用参数
% Time0 = 0;Att_q0=[1;0;0;0];Hw_Re0=[0;0;0;0];Tset=0;

%% 全局变量释义
% SADA_GL_State -- SADA归零状态：0-无，1-归零中，2-归零完成，3-归零故障
% SADA_ZL_State -- SADA增量状态(4*1,+Y_A/B,-Y_A/B)：0-无，1-增量中，2-增量完成
% Allow_SADACopy_Tc -- SADA联合控制指令：1-联合，0独立
% SADA_Priority -- SADA优先级（联合控制时使用）：0-以+Y为参考，1-以-Y为参考
% SADA_ACtrlFlag_Tc -- SADA A轴控制算法强选标志：0-滞环,1-PI
% Drift_GyroA_Tc --- 陀螺A零位注数值（表头）
% Drift_GyroB_Tc --- 陀螺B零位注数值（表头）
% GyroDriftSet ---% 陀螺零位设置，当取1时，将实时零位估计结果放到Drift_GyroA_Tc、Drift_GyroB_Tc中，并将零位估计结果清零
% ImagingMode --- 成像模式：1-推扫成像，2-立体成像
% Band_1 --- 条带：【侧摆角、开始时间、结束时间】
toc