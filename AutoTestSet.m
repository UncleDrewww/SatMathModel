function AutoTestSet(T_sys)
Choice = 0;
% 0-test
% 1-成像模式
% 2-凝视
% 3-俯仰渐进


global Mode SubMode Allow_StUse Allow_KalmanCal_Tc Allow_KalmanUse_Tc Allow_StareCtrl_Tc Allow_SADAWTCtrl_Tc Allow_FYJJCtrl_Tc SunDirFace_Tgt;
global Gyro_Ok_Set tao_Ass tao_St Allow_KalmanUse Allow_KalmanCal SunDirFace UnlockSpread0;
global Long_XGZ Lati_XGZ High_XGZ Long_DMZ Lati_DMZ High_DMZ;
global FwCtrlForce indexForce;
global ImagingMode;
global T_ctrl;
global Band_1 Band_2 Band_3;
global index;

persistent First;
if isempty(First)
    First = 1;
end
% disp(1)
if Choice == 0
    if First == 1
        First = 3;
        Mode = 3;
        SubMode = 1;
        Allow_StUse = 1;
        Allow_SADAWTCtrl_Tc = 0*[1;1];
%         FwCtrlForce = [13;13;13];%强选轮控算法，默认[9；9]
        FwCtrlForce = [9;9;9];%强选轮控算法，默认[9；9]
        SunDirFace = 4;
        SunDirFace_Tgt = 4;  % 对地：0：负X；其他：负Z   对日：0：负X    4  负Z
        Allow_KalmanUse_Tc=0;
        Allow_KalmanCal_Tc=0;
%         Allow_FYJJCtrl_Tc=1;
%         Gyro_Ok_Set = [0;0];
        indexForce=[9;9];%强选坐标系，默认[9；9]
    end
%     if (T_sys>=1000)
%         index = [0;0];
%     elseif (T_sys>=1500)
%         index = [0;2];
%     elseif (T_sys>=2000)
%         index = [0;1];
%     else 
%         index = [0;1];
%     end
elseif Choice == 1
%% 成像模式仿真
    % Year=2019;Month=10;Day=25;Hour=04;Min=14;Sec=21;%
    % a=7546e3;e=0.001217;i=86.504/180*pi;omg=39.622/180*pi;OMG=2.348/180*pi;f=(-40*(Sunshine0==0)+(Sunshine0==1)*198.778)/(180/pi);%GW用1175Km轨道
%     Year=2020;Month=2;Day=26;Hour=04;Min=0;Sec=0;%%%%UTC 2020-02-26 04:00:00
%     a=7553369.655000004;e=0.002700004503512;i=1.509674896390055;omg=1.579540411752145;OMG=1.745329251994330;f=4.712406433677209-omg;%标称轨道对应顺根(轨控测试用)
% 
%     Att0=1*[80;50;-80]*pi/180;% 轮控  xyz%[0.01;-0.01;0.01]°/s
%     W_bi0=0*[0;-w0;0]+0*[2;-2;2]*pi/180;
% 
%     Ngc1=[0.01;-0.02;-0.03];%陀螺常值漂移，单位deg/s
%     Ngc2=[-0.02;0.01;-0.03];
    if First == 1
        Mode = 3;
        SubMode = 1;
        Allow_StUse = 1;
        Allow_SADAWTCtrl_Tc = 0*[1;1];
        FwCtrlForce = [11;11;11];
%         FwCtrlForce = [9;9;9];
        UnlockSpread0=0*[1;1];
        Allow_KalmanUse_Tc=0;
        Allow_KalmanCal_Tc=0;
        First = 0;
        ImagingMode = 1;
        Band_1 = [0;300;350];Band_2=[30;450;500];Band_3=[30;1600;1700];
%         Band_1 = [20;130;5000];Band_2=[0;0;0];Band_3=[0;0;0];
    end
%     if (T_sys<1001) && (T_sys>=1000)
%         Allow_KalmanCal_Tc = 1;
%         Allow_KalmanUse_Tc = 1;
%     end
elseif Choice == 2
    %% 凝视
    if First == 1
        Mode = 3;
        SubMode = 1;
        Allow_StUse = 1;
        Allow_KalmanCal_Tc = 1;
        Allow_KalmanUse_Tc = 1;
        Allow_SADAWTCtrl_Tc = 0*[1;1];
        UnlockSpread0=1*[1;1];
        First = 0;
    end
%     Long_DMZ = -103.053/180*pi;Lati_DMZ = 8.70968/180*pi;High_DMZ = 524.39;%02星数学仿真报告用，适用于1175kmGW轨道
%     Long_XGZ = -103.053/180*pi;Lati_XGZ = 8.70968/180*pi;High_XGZ = 524.39;
%     Long_DMZ = -122/180*pi;Lati_DMZ = 19.7/180*pi;High_DMZ = 524.39;%适用于轨控标称轨道，对应2000s星下点
%     Long_XGZ = -122/180*pi;Lati_XGZ = 19.7/180*pi;High_XGZ = 524.39;
    Long_DMZ = 95/180*pi;Lati_DMZ = 60/180*pi;High_DMZ = 45.59;%
    Long_XGZ = 95/180*pi;Lati_XGZ = 60/180*pi;High_XGZ = 45.59;
elseif Choice == 3
    %% 俯仰渐进
    if First == 1
        Mode = 3;
        SubMode = 1;
        Allow_StUse = 1;
        Allow_KalmanCal_Tc = 1;
        Allow_KalmanUse_Tc = 1;
        Allow_SADAWTCtrl_Tc = [1;1];
        Allow_FYJJCtrl_Tc = 1;
        SADA_Spread = [1;1];
        First = 0;
    end
elseif Choice == 4
    
end