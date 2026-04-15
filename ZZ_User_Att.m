function [AlgthmChoiceFlag,Index,RotateAglRateTgt, NominalCSYS ,Td, I] = ...
    ZZ_User_Att(SatParam,OrbitElement,EnvirParam,SadaParam,CamParam,MmRef,StRef,GyroRef,StGyroAttRef,AssRef,AttRef,Att4Ctrl,Hw3,t_ctrl,RunCount)

[AlgthmChoiceFlag,Index,RotateAglRateTgt] = ZZ_ModeDeterAndInit(MmRef,StRef,GyroRef,StGyroAttRef,AssRef,OrbitElement,Att4Ctrl,t_ctrl);
NominalCSYS = ZZ_Nominal_CSYS(OrbitElement,EnvirParam,CamParam,AttRef,Index,t_ctrl,RunCount);

[I, Cent, StructParam, Rsa_p1, Rsa_p2, dRsa_p1, dRsa_p2,dI, Wc_p1, Ac_p1, Wc_p2, Ac_p2] = SatParamCal(SatParam,SadaParam);

[Fs_all, Ts_all, Fa_all, Ta_all, Tg, To] = TdCal(StructParam,I,Cent,EnvirParam,OrbitElement,AttRef,Hw3);
T_Sada1 = Rsa_p1 * Ac_p1 + Rsa_p2 * Ac_p2;
T_Sada2 = dRsa_p1 * Wc_p1 + dRsa_p2 * Wc_p2;
T_Sada3 = dI * AttRef.Wbi;
T_Sada4 = 0*xw(AttRef.Wbi) * (I * AttRef.Wbi);   % 陀螺力矩用To
T_Sada5 = xw(AttRef.Wbi) * (Rsa_p1 * Wc_p1 + Rsa_p2 * Wc_p2);
T_Sada = T_Sada1 + T_Sada2 + T_Sada3 + T_Sada4 + T_Sada5;% SADA受到的力矩
Td = struct('Ts',Ts_all,'Ta',Ta_all,'Tg',Tg,'Tm',[0;0;0],'To',To,'Tp',-T_Sada);
end

%% 模式确定
function [AlgthmChoiceFlag,Index_out,RotateAglRateTgt] = ZZ_ModeDeterAndInit(MmRef,StRef,GyroRef,StGyroAttRef,AssRef,OrbitElement,Att4Ctrl,t_ctrl)
global LcMark Tc_StUse_Enable Tc_SadaCtrl_Enable Tc_StareCtrl_Enable Tc_ZeroDplCtrl_Enable Tc_PLJCtrl_Enable ...
    Tc_ZeroDpl_IncidAgl Tc_PLJ_DB_Enable Tc_PLJ_TgtAgl Tc_Aligned_and_Constrained SatShape SadaParam SadaModeParam Spread0;
persistent ModeCnt SubModeCnt SteadyCnt Mode1Cnt1 Mode1Cnt2 Mode1Cnt3 Mode1Cnt4 Mode1Cnt5 Mode2Cnt1 Mode2Cnt2 Mode2Cnt3 Mode2Cnt4 Mode3Cnt1 Mode3Cnt2 ...
    SadaSpreadCnt AttRefFailCnt AssRefOkCnt StRefOkCnt SafeModeStepFlag;
[ModeCnt, SubModeCnt,SteadyCnt,Mode1Cnt1,Mode1Cnt2,Mode1Cnt3,Mode1Cnt4,Mode1Cnt5,Mode2Cnt1,Mode2Cnt2,Mode2Cnt3,Mode2Cnt4,Mode3Cnt1,Mode3Cnt2,SadaSpreadCnt,AttRefFailCnt,AssRefOkCnt,StRefOkCnt] = ...
    PersistentInit(ModeCnt,SubModeCnt,SteadyCnt,Mode1Cnt1,Mode1Cnt2,Mode1Cnt3,Mode1Cnt4,Mode1Cnt5,Mode2Cnt1,Mode2Cnt2,Mode2Cnt3,Mode2Cnt4,Mode3Cnt1,Mode3Cnt2,SadaSpreadCnt,AttRefFailCnt,AssRefOkCnt,StRefOkCnt);
persistent Mode5Cnt1 Mode5Cnt2 Mode5Cnt3 Mode5Cnt4 SafeModeStepNum;
[Mode5Cnt1,Mode5Cnt2,Mode5Cnt3,Mode5Cnt4,SafeModeStepNum] = PersistentInit(Mode5Cnt1,Mode5Cnt2,Mode5Cnt3,Mode5Cnt4,SafeModeStepNum);
persistent Index_last Index Tc_StareCtrl_Enable_last Tc_ZeroDplCtrl_Enable_last Tc_ZeroDpl_IncidAgl_last ...
    Tc_PLJCtrl_Enable_last Tc_PLJ_DB_Enable_last Tc_PLJ_TgtAgl_last;
if isempty(Index_last)
    Index_last = [0;0];
    Tc_StareCtrl_Enable_last = 0;
    Tc_ZeroDplCtrl_Enable_last = 0;
    Tc_ZeroDpl_IncidAgl_last = 0;
    Tc_PLJCtrl_Enable_last = 0;
    Tc_PLJ_DB_Enable_last = 0;
    Tc_PLJ_TgtAgl_last = [0;0;0];
end
if isempty(Index)
    Index = [0;0];
end
if isempty(SafeModeStepFlag)
    SafeModeStepFlag = 1;
end
% CntFor30s = 30*2;
DegP02 = 0.02/180*pi;
DegP1 = 0.1/180*pi;
DegP5 = 0.5/180*pi;
Deg1 = 1/180*pi;
Deg2 = 2/180*pi;
Deg5 = 5/180*pi;
Deg7 = 7/180*pi;
Deg20 = 20/180*pi;
% Deg10 = 10/180*pi;
% Deg20 = 20/180*pi;
dGs = 0.002*1e-4;   %0.002Gs,转为T

if LcMark.Mode_last ~= LcMark.Mode
    ModeCnt = 0;SubModeCnt = 0;
    LcMark.SubMode = 1;
    SadaModeParam.IncrementTime = [1;1;1;1];
end
if LcMark.SubMode~=LcMark.SubMode_last
    SubModeCnt = 0;
end
if LcMark.Mode_last ~= LcMark.Mode || LcMark.SubMode~=LcMark.SubMode_last
    Mode1Cnt1 = 0;Mode2Cnt1 = 0;Mode2Cnt2 = 0;Mode3Cnt1=0;Mode1Cnt2=0; Mode1Cnt3=0; Mode1Cnt4=0;
    Mode1Cnt5=0; Mode2Cnt3=0; Mode2Cnt4=0;Mode3Cnt2=0; Mode5Cnt1=0; Mode5Cnt2=0; Mode5Cnt3=0; Mode5Cnt4=0;
    SadaModeParam.Mode1Cnt1 = 0;SadaModeParam.Mode1Cnt2 = 0;SadaModeParam.Mode2Cnt1=0;SadaModeParam.Mode2Cnt2=0;
    SadaModeParam.ModeLevelA=1; SadaModeParam.ModeLevelB=1;SadaModeParam.ModeLevelC=1;SadaModeParam.ModeLevelD=1;
    SadaModeParam.Sada1WorkEnd = 0;SadaModeParam.Sada2WorkEnd = 0;
end

ModeCnt = Cnt(ModeCnt,1);        % 模式计时，单位：拍
SubModeCnt = Cnt(SubModeCnt,1);  % 阶段计时，单位：拍

SteadyCnt = Cnt(SteadyCnt,Att4Ctrl.Ok==1 && abs(Att4Ctrl.Agl(1))<Deg5 && abs(Att4Ctrl.Agl(2))<Deg5 ...
    && abs(Att4Ctrl.Agl(3))<Deg5 && abs(Att4Ctrl.Wbn(1))<DegP1 && abs(Att4Ctrl.Wbn(2))<DegP1 && abs(Att4Ctrl.Wbn(3))<DegP1);
AssRefOkCnt = Cnt(AssRefOkCnt,AssRef.Ok==1);
StRefOkCnt = Cnt(StRefOkCnt,StRef.Ok==1 && Tc_StUse_Enable==1);
SadaSpreadCnt = Cnt(SadaSpreadCnt,SadaParam.Spread(1) == 1);
AttRefFailCnt = Cnt(AttRefFailCnt,Att4Ctrl.Ok == 0);  %% LC 姿态基准还是控制基准更好？

LcMark.Mode_last = LcMark.Mode;
LcMark.SubMode_last = LcMark.SubMode;

if LcMark.Mode == 0
    if ModeCnt >= 30 * 2
        %         LcMark.Mode = 1;
        %         LcMark.SubMode = 1;
    end
elseif LcMark.Mode == 1 %%阻尼模式
    if LcMark.SubMode == 1
        Mode1Cnt1 = Cnt(Mode1Cnt1,Att4Ctrl.Ok==1 && abs(Att4Ctrl.Wbn(1))<Deg1 && abs(Att4Ctrl.Wbn(2))<Deg1 && abs(Att4Ctrl.Wbn(3))<Deg1);
        Mode1Cnt4 = Cnt(Mode1Cnt4,Att4Ctrl.Ok==0 && MmRef.Ok==1 && abs(MmRef.dBb_TimeSlice(1))<dGs && abs(MmRef.dBb_TimeSlice(2))<dGs && abs(MmRef.dBb_TimeSlice(3))<dGs);
        Mode1Cnt5 = Cnt(Mode1Cnt5,Att4Ctrl.Ok==0 && StRef.Ok==1 && abs(StRef.Wbi(1))<DegP5 && abs(StRef.Wbi(2))<DegP5 && abs(StRef.Wbi(3))<DegP5);
        if Mode1Cnt1 >= 5/t_ctrl           %控制基准有效，连续5s满足三轴控制用角度<1°/s，转入阶段二
            LcMark.Mode = 1;
            LcMark.SubMode = 2;
        elseif Mode1Cnt4 >= 100/t_ctrl     %控制基准有效，连续100s满足三轴控制用角度<1°/s，转入阶段二
            LcMark.Mode = 1;
            LcMark.SubMode = 3;
        elseif Mode1Cnt5 >= 100/t_ctrl     %控制基准有效，连续100s满足星敏有效且三轴星敏单机姿态角速度<0.5°/s，转入阶段三
            LcMark.Mode = 1;
            LcMark.SubMode = 3;
        elseif SubModeCnt >= 4000/t_ctrl    %阶段1计时满 4000s 转入阶段二
            LcMark.Mode = 1;
            LcMark.SubMode = 2;
        end
    elseif LcMark.SubMode == 2
        Mode1Cnt2 = Cnt(Mode1Cnt2,Att4Ctrl.Ok==1 && abs(Att4Ctrl.Wbn(1))<DegP5 && abs(Att4Ctrl.Wbn(2))<DegP5 && abs(Att4Ctrl.Wbn(3))<DegP5);
        if Mode1Cnt2 >= 5/t_ctrl           %控制基准有效，连续5s满足三轴控制用角度<0.5°/s，转入阶段三
            LcMark.Mode = 1;
            LcMark.SubMode = 3;
        elseif SubModeCnt >= 100/t_ctrl     %阶段1计时满 100s 转入阶段三
            LcMark.Mode = 1;
            LcMark.SubMode = 3;
        end
    elseif LcMark.SubMode == 3
        Mode1Cnt3 = Cnt(Mode1Cnt3,(SadaParam.Sada1.HLState(2)==1 || SadaParam.Sada1.HLState(4)==1) && (SadaParam.Sada2.HLState(2)==1 || SadaParam.Sada2.HLState(4)==1));
        if Mode1Cnt3 >= 40/t_ctrl           %连续40s两翼SADA B轴霍尔触发，转入-Z对日
            SadaParam.Spread = [1;1];       %星上展开标志
            Spread0(1:2) = [1;1;];          %动力学展开标志
            LcMark.Mode = 2;
            LcMark.SubMode = 1;%对日工况
        elseif SubModeCnt >= 600/t_ctrl      %计时满600s
            if (SadaParam.Sada1.HLState(2)==0 && SadaParam.Sada1.HLState(4)==0) && (SadaParam.Sada2.HLState(2)==0 || SadaParam.Sada2.HLState(4)==0)   %两翼均未解锁
                LcMark.Mode = 2;
                LcMark.SubMode = 1;%对日工况
            else
                if (SadaParam.Sada1.HLState(2)==1 || SadaParam.Sada1.HLState(4)==1)
                    SadaParam.Spread(1) = 1;
                end
                if (SadaParam.Sada2.HLState(2)==1 || SadaParam.Sada2.HLState(4)==1)
                    SadaParam.Spread(2) = 1;
                end
                LcMark.Mode = 2;
                LcMark.SubMode = 1;%对日工况
            end
        end
    end
elseif LcMark.Mode == 2  %% 对日模式
    if LcMark.SubMode == 1  %% 对日一阶段：状态处理
        Mode2Cnt1 = Cnt(Mode2Cnt1,(SadaParam.Sada1.Ok(2)==1)&&(abs(SadaParam.Sada1.Agl4Ctrl(2))<Deg2));
        if (SadaModeParam.Sada1WorkEnd && SadaModeParam.Sada2WorkEnd)       %+YSADA流程结束、-YSADA流程结束、控制用角速度<0.1，转阶段二
            if SteadyCnt >= 5/t_ctrl
                LcMark.SubMode = 2;
            end
        elseif SubModeCnt >= 1500/t_ctrl      %阶段计时满1500s，转阶段二
            LcMark.SubMode = 2;
        end
    elseif LcMark.SubMode == 2  %% 对日二阶段：太阳搜索
        if StRefOkCnt > 5/t_ctrl || AssRefOkCnt > 5/t_ctrl           %星敏或太敏有效，转阶段三
            LcMark.SubMode = 3;
        end
        if (SubModeCnt >= 3500/t_ctrl)
            LcMark.Mode = 5;
            LcMark.SubMode = 1;
        end
    elseif LcMark.SubMode == 3  %% 对日三阶段：太阳捕获
        if SteadyCnt >= 5/t_ctrl
            LcMark.SubMode = 4;
        end
        if (SubModeCnt >= 2000/t_ctrl)
            LcMark.Mode = 5;
            LcMark.SubMode = 1;
        end
    elseif LcMark.SubMode == 4  %% 对日四阶段：太阳指向
        Mode2Cnt2 = Cnt(Mode2Cnt2,Att4Ctrl.Ok==1 && (abs(Att4Ctrl.Agl(1))>Deg7 || abs(Att4Ctrl.Agl(2))>Deg7 || abs(Att4Ctrl.Agl(3))>Deg7));
        Mode2Cnt3 = Cnt(Mode2Cnt3,StRef.Ok==0 && StGyroAttRef.Ok==0 && OrbitElement.Sunshine==1 && AssRef.Ok==0);
        Mode2Cnt4 = Cnt(Mode2Cnt4,StRef.Ok==0 && StGyroAttRef.Ok==0 && AssRef.Ok==0);
        if isequal(Index_last,Index) == 0    %标称系切换，转入阶段一
            LcMark.SubMode = 1;
        elseif Mode2Cnt3 >= 200/t_ctrl       %连续200s星敏基准无效，星敏+陀螺基准无效，轨道有效，卫星处于光照区且太敏基准无效，转入阶段二
            LcMark.SubMode = 2;
        elseif Mode2Cnt2 >= 5/t_ctrl         %连续5s控制用姿态有效，任意控制用姿态角大于7°，转入阶段3
            LcMark.SubMode = 3;
        elseif Mode2Cnt4 >= 2400/t_ctrl      %连续40min星敏基准无效，星敏+陀螺基准无效，太敏基准无效，转入阶段2
            LcMark.SubMode = 2;
        end
    end
elseif LcMark.Mode == 3
    if LcMark.SubMode == 1  %% 对地一阶段：地球捕获
        if SteadyCnt >= 5/t_ctrl
            LcMark.SubMode = 2;
        end
        if SubModeCnt >= 2000/t_ctrl       %阶段计时满2000s，转入安全模式
            LcMark.Mode = 5;
            LcMark.SubMode = 1;
        end
    else
        Mode3Cnt1 = Cnt(Mode3Cnt1,Att4Ctrl.Ok==1 && (abs(Att4Ctrl.Agl(1))>Deg20 || abs(Att4Ctrl.Agl(2))>Deg20 || abs(Att4Ctrl.Agl(3))>Deg20));
        Mode3Cnt2 = Cnt(Mode3Cnt2,StRef.Ok==0);
        if Mode3Cnt1 >= 5*2
            LcMark.Mode = 5;
            LcMark.SubMode = 1;
        end
        if isequal(Index_last,Index) == 0
            LcMark.SubMode = 1;
        end
        if Tc_ZeroDpl_IncidAgl ~= Tc_ZeroDpl_IncidAgl_last || Tc_PLJ_DB_Enable ~= Tc_PLJ_DB_Enable_last || isequal(Tc_PLJ_TgtAgl,Tc_PLJ_TgtAgl_last)==0
            LcMark.SubMode = 1;
        end
        if Mode3Cnt2 >= 2*3600/t_ctrl     %连续2h单星敏基准无效，转入安全模式
            LcMark.Mode = 5;
            LcMark.SubMode = 1;
        end
    end
elseif LcMark.Mode == 5    %安全模式
    if LcMark.SubMode == 1  %% 安全模式第一阶段：帆板设置
        if (SadaModeParam.Sada1WorkEnd && SadaModeParam.Sada2WorkEnd)       %+YSADA流程结束、-YSADA流程结束、控制用角速度<0.1，转阶段二
            if SteadyCnt >= 5/t_ctrl
                LcMark.SubMode = 2;
            end
            if AttRefFailCnt >= 5/t_ctrl      %连续5s控制用基准无效，转阶段五
                LcMark.SubMode = 5;
            end
        elseif SubModeCnt >= 1500/t_ctrl      %阶段计时满1500s，转阶段二
            LcMark.SubMode = 2;
        end
    elseif LcMark.SubMode == 2
        if (GyroRef.Ok==0)
            LcMark.SubMode = 5;
        end
        if AssRefOkCnt >= 5/t_ctrl
            LcMark.SubMode = 3;
        end
        if SafeModeStepFlag == 1
            Mode5Cnt1 = Cnt(Mode5Cnt1,1);
            if Mode5Cnt1 >= 1000/t_ctrl
                SafeModeStepFlag = 2;
                Mode5Cnt1 = 0;
            end
        elseif SafeModeStepFlag == 2
            if SteadyCnt >= 5/t_ctrl
                SafeModeStepFlag = 3;
            end
        elseif SafeModeStepFlag == 3
            Mode5Cnt1 = Cnt(Mode5Cnt1,1);
            if Mode5Cnt1 >= 1000/t_ctrl
                SafeModeStepFlag = 4;
                Mode5Cnt1 = 0;
            end
        elseif SafeModeStepFlag == 4
            if SteadyCnt >= 5/t_ctrl
                SafeModeStepFlag = 5;
            end
        elseif SafeModeStepFlag == 5
            Mode5Cnt1 = Cnt(Mode5Cnt1,1);
            if Mode5Cnt1 >= 1000/t_ctrl
                SafeModeStepFlag = 6;
                Mode5Cnt1 = 0;
            end
        elseif SafeModeStepFlag == 6
            if SteadyCnt >= 5/t_ctrl
                if SafeModeStepNum < 2
                    SafeModeStepNum = SafeModeStepNum+1;
                    SafeModeStepFlag = 1;
                else
                    SafeModeStepFlag = 7;
                end
            end
        else
        end
    elseif LcMark.SubMode == 3
        if SteadyCnt >= 5/t_ctrl
            LcMark.SubMode = 4;
        elseif AttRefFailCnt >= 5/t_ctrl
            LcMark.SubMode = 5;
        end
        if (SubModeCnt >= 2000/t_ctrl)
            LcMark.SubMode = 2;
        end
    elseif LcMark.SubMode == 4
        Mode5Cnt2 = Cnt(Mode5Cnt2,Att4Ctrl.Ok==1 && (abs(Att4Ctrl.Agl(1))>Deg7 || abs(Att4Ctrl.Agl(2))>Deg7 || abs(Att4Ctrl.Agl(3))>Deg7));
        Mode5Cnt3 = Cnt(Mode5Cnt3,OrbitElement.Sunshine==1 && AssRef.Ok==0);
        Mode5Cnt4 = Cnt(Mode5Cnt4,AssRef.Ok==0);
        if isequal(Index_last,Index) == 0    %标称系切换，转入阶段一
            LcMark.SubMode = 1;
        elseif Mode2Cnt3 >= 200/t_ctrl       %连续200s轨道有效，卫星处于光照区且太敏基准无效，转入阶段二
            LcMark.SubMode = 2;
        elseif Mode2Cnt2 >= 5/t_ctrl         %连续5s控制用姿态有效，任意控制用姿态角大于7°，转入阶段3
            LcMark.SubMode = 3;
        elseif Mode2Cnt4 >= 2400/t_ctrl      %连续40min太敏基准无效，转入阶段2
            LcMark.SubMode = 2;
        end
    end
end
Index_last = Index;
Tc_StareCtrl_Enable_last = Tc_StareCtrl_Enable;
Tc_ZeroDplCtrl_Enable_last = Tc_ZeroDplCtrl_Enable;
Tc_ZeroDpl_IncidAgl_last = Tc_ZeroDpl_IncidAgl;
Tc_PLJCtrl_Enable_last = Tc_PLJCtrl_Enable;
Tc_PLJ_DB_Enable_last = Tc_PLJ_DB_Enable;
Tc_PLJ_TgtAgl_last = Tc_PLJ_TgtAgl;
%% Mode  Initalize
if LcMark.Mode == 0
    AttRefFlag = 5555;        % 强选单陀螺
    MagRefFlag = 0011;        % 强选磁强计
    Att4CtrlAlgthm = [0;0;0]; % 无
    FwCtrlAlgthm = [0 0;0 0;0 0]; % 不控
    MagCtrlAlgthm = 0;        % 不控
    Index = [0;0];
    RotateAglRateTgt = [0;0;0];
elseif LcMark.Mode == 1
    if LcMark.SubMode == 1
        AttRefFlag = 5555;        % 强选单陀螺
        MagRefFlag = 0011;        % 强选磁强计
        Att4CtrlAlgthm = [3;3;3]; % 角速度控制
        FwCtrlAlgthm = [0 0;0 0;0 0];   % 不控
        if GyroRef.Ok == 1
            MagCtrlAlgthm = 1;    % 角速度负反馈
        else
            MagCtrlAlgthm = 2;    % Bdot
        end
        Index = [0;0];
        RotateAglRateTgt = [0;0;0];
    elseif LcMark.SubMode == 2
        AttRefFlag = 5555;        % 强选单陀螺
        MagRefFlag = 0011;        % 强选磁强计
        Att4CtrlAlgthm = [3;3;3]; % 角速度控制
        FwCtrlAlgthm = [5 1;5 1;5 1];  % DT
        if GyroRef.Ok == 1
            MagCtrlAlgthm = 1;    % 角速度负反馈
        else
            MagCtrlAlgthm = 2;    % Bdot
        end
        Index = [0;0];
        RotateAglRateTgt = [0;0;0];
    else
        AttRefFlag = 5555;        % 强选单陀螺
        MagRefFlag = 0011;        % 强选磁强计
        Att4CtrlAlgthm = [0;0;0]; % 不控
        FwCtrlAlgthm = [0 0;0 0;0 0];  % 不控
        MagCtrlAlgthm = 0;
        Index = [0;0];
        RotateAglRateTgt = [0;0;0];
    end
elseif LcMark.Mode == 2
    if Tc_Aligned_and_Constrained == 0
        Index = [1;4]; % -Z对日
    else
        Index = [5;1]; %约束对日
    end
    
    if  LcMark.SubMode == 1
        AttRefFlag = 5555;         % 强选单陀螺
        MagRefFlag = 0011;         % 强选磁强计
        Att4CtrlAlgthm = [3;3;3];  % 角速度控制
        FwCtrlAlgthm = [5 1;5 1;5 1];  % DT
        MagCtrlAlgthm = 3;         % 磁卸载
        RotateAglRateTgt = [0;0;0]/180*pi;
    elseif LcMark.SubMode == 2
        AttRefFlag = 5555;         % 强选单陀螺
        MagRefFlag = 0011;         % 强选磁强计
        Att4CtrlAlgthm = [3;3;3];  % 角速度控制
        FwCtrlAlgthm = [5 1;5 1;5 1];  % DT
        MagCtrlAlgthm = 3;         % 磁卸载
        RotateAglRateTgt = [0.5;0.5;0]/180*pi;
    elseif LcMark.SubMode == 3
        AttRefFlag = 2200;        % 准则二自主
        MagRefFlag = 1100;        % 优选磁强计
        if Tc_Aligned_and_Constrained == 0
            Att4CtrlAlgthm = [2;2;3]; % 矢量角控制
            FwCtrlAlgthm = [4 1;4 1;5 1]; % 4-PIDT
            RotateAglRateTgt = [0;0;0.2]/180*pi;
        else
            Att4CtrlAlgthm = [1;1;1]; % 四元数控制
            FwCtrlAlgthm = [4 1;4 1;4 1]; % 4-PIDT
            RotateAglRateTgt = [0;0;0]/180*pi;
        end
        MagCtrlAlgthm = [3;3;3];  % 磁卸载
    elseif LcMark.SubMode == 4
        AttRefFlag = 2200;        % 准则二自主
        MagRefFlag = 1100;        % 优选磁强计
        if Tc_Aligned_and_Constrained == 0
            Att4CtrlAlgthm = [2;2;3]; % 矢量角控制
            FwCtrlAlgthm = [4 1;4 1;5 1]; % 4-PIDT
            RotateAglRateTgt = [0;0;0.2]/180*pi;
        else
            Att4CtrlAlgthm = [1;1;1]; % 四元数控制
            FwCtrlAlgthm = [4 1;4 1;4 1]; % 4-PIDT
            RotateAglRateTgt = [0;0;0]/180*pi;
        end
        MagCtrlAlgthm = [3;3;3];  % 磁卸载
    end
    
elseif LcMark.Mode == 3
    AttRefFlag = 3300;         % 准则三自主
    MagRefFlag = 1100;         % 优选磁强计
    Att4CtrlAlgthm = [1;1;1];  % 四元数控制
%     FwCtrlAlgthm = [2 1;2 1;2 1];  % PIDT 4  PID 2
    FwCtrlAlgthm = [1 1;1 1;1 1];
%     if Index(1) == 6
%         FwCtrlAlgthm = [1 1;1 1;1 1];
%     end
    MagCtrlAlgthm = 3;         % 磁卸载
    if Tc_StareCtrl_Enable == 1
        Index = [2;1];
    elseif Tc_ZeroDplCtrl_Enable == 1
        Index = [3;1];
    elseif Tc_PLJCtrl_Enable == 1
        Index = [4;1];
    else
        Index = [0;1];  % 对地【0；0；0】
    end
    RotateAglRateTgt = [0;0;0];
elseif LcMark.Mode == 5
    Index = [1;4]; % -Z对日
    if  LcMark.SubMode == 1
        AttRefFlag = 5555;         % 强选单陀螺
        MagRefFlag = 0011;         % 强选磁强计
        Att4CtrlAlgthm = [3;3;3];  % 角速度控制
        FwCtrlAlgthm = [5 1;5 1;5 1];  % DT
        MagCtrlAlgthm = 3;         % 磁卸载
        RotateAglRateTgt = [0;0;0]/180*pi;
    elseif LcMark.SubMode == 2
        AttRefFlag = 5555;         % 强选单陀螺
        MagRefFlag = 0011;         % 强选磁强计
        Att4CtrlAlgthm = [3;3;3];  % 角速度控制
        FwCtrlAlgthm = [5 1;5 1;5 1];  % DT
        MagCtrlAlgthm = 3;         % 磁卸载
        if SafeModeStepFlag == 1
            RotateAglRateTgt = [0.5;0;0]/180*pi;
        elseif SafeModeStepFlag == 2
            RotateAglRateTgt = [0;0;0]/180*pi;
        elseif SafeModeStepFlag == 3
            RotateAglRateTgt = [0;0.5;0]/180*pi;
        elseif SafeModeStepFlag == 4
            RotateAglRateTgt = [0;0;0]/180*pi;
        elseif SafeModeStepFlag == 5
            RotateAglRateTgt = [0;0;0.5]/180*pi;
        elseif SafeModeStepFlag == 6
            RotateAglRateTgt = [0;0;0]/180*pi;
        else
            RotateAglRateTgt = [0;0;0]/180*pi;
        end
    elseif LcMark.SubMode == 3
        AttRefFlag = 1100;        % 准则一自主
        MagRefFlag = 1100;        % 优选磁强计
        Att4CtrlAlgthm = [2;2;3]; % 矢量角控制
        FwCtrlAlgthm = [4 1;4 1;5 1]; % 4-PIDT
        MagCtrlAlgthm = [3;3;3];  % 磁卸载
        RotateAglRateTgt = [0;0;0.2]/180*pi;
    elseif LcMark.SubMode == 4
        AttRefFlag = 1100;        % 准则一自主
        MagRefFlag = 1100;        % 优选磁强计
        Att4CtrlAlgthm = [2;2;3]; % 矢量角控制
        FwCtrlAlgthm = [4 1;4 1;5 1]; % PIDT
        MagCtrlAlgthm = 3;        % 磁卸载
        RotateAglRateTgt = [0;0;0.2]/180*pi;
    end
end

global Tc_MagCtrlAlgthm;
if Tc_MagCtrlAlgthm ~= 9 % 磁控算法强选
    MagCtrlAlgthm = Tc_MagCtrlAlgthm;
end

AlgthmChoiceFlag = struct('AttRefFlag',AttRefFlag,'MagRefFlag',MagRefFlag,'Att4CtrlAlgthm',Att4CtrlAlgthm,...
    'FwCtrlAlgthm',FwCtrlAlgthm,'MagCtrlAlgthm',MagCtrlAlgthm);

global Tc_Index;
if Tc_Index(1) ~= 9
    Index = Tc_Index;
end
Index_out = Index;
end

%% 标称系
function NominalCSYS = ZZ_Nominal_CSYS(OrbitElement,EnvirParam,CamParam,AttRef,Index,t_ctrl,RunCount)
persistent Cnt_DPL Cnt_PLJ Qin_last Qon_last Wni_n_last Wno_n First Cnt_Al_Co Path;
if isempty(Cnt_DPL)
    Cnt_DPL = 0;
    Cnt_PLJ = 0;
    Cnt_Al_Co = 0;
    Qin_last = [1;0;0;0];
    Qon_last = [1;0;0;0];
    Wni_n_last = [0;0;0];
    Wno_n = [0;0;0];
    First = 0;
end
global Tc_StareCtrl_LLA Tc_ZeroDpl_IncidAgl;
global Tc_PLJ_DB_Enable Tc_PLJ_TgtAgl Tc_Index;
if Index(1) ==  0                % 轨道系
    %     if Index(20 ) <= 6
    if Index(2) == 0
        Roll_Nominal = 0;
        Pitch_Nominal = 0;
        Yaw_Nominal = pi/2;
    elseif Index(2) == 1
        Roll_Nominal = 0;
        Pitch_Nominal = 0;
        Yaw_Nominal = 0;
    elseif Index(2) == 2
        Roll_Nominal = 0;
        Pitch_Nominal = 0;
        Yaw_Nominal = -pi/2;
    elseif Index(2) == 3
        Roll_Nominal = -pi/2;
        Pitch_Nominal = 0;
        Yaw_Nominal = 0;
    elseif Index(2) == 4
        Roll_Nominal = pi/2;
        Pitch_Nominal = 0;
        Yaw_Nominal = 0;
    elseif Index(2) == 5
        Roll_Nominal = -pi/2;
        Pitch_Nominal = 0;
        Yaw_Nominal = pi;
    elseif Index(2) == 6
        Roll_Nominal = pi/2;
        Pitch_Nominal = 0;
        Yaw_Nominal = pi;
    elseif Index(2) == 7
        Roll_Nominal = 0/180*pi;
        Pitch_Nominal = 60/180*pi;
        Yaw_Nominal = 0/180*pi;
    elseif Index(2) == 8
        Roll_Nominal = 0/180*pi;
        Pitch_Nominal = -60/180*pi;
        Yaw_Nominal = 0/180*pi;
    else
        Roll_Nominal = 30/180*pi;
        Pitch_Nominal = 00/180*pi;
        Yaw_Nominal = 0/180*pi;
    end
    % 计算轨道系到标称系的转移矩阵Ano及四元数Qon
    Ano=angle2dcm(Yaw_Nominal,Pitch_Nominal,Roll_Nominal,'ZYX');
    Qon=angle2quat(Yaw_Nominal,Pitch_Nominal,Roll_Nominal,'ZYX')';
    
    
    if Index(2) == 100
        Qon = [cosd(5),sind(5)/sqrt(3)*[1,1,1]]';
%         Qon = [cosd(5),sind(5)*[1,0,0]]';
        Ano = quat2dcm(Qon');
    end
    %     else
    %         % 对齐轴+约束轴
    %         Axis_Aligned = [1;0;0];%本体轴
    %         Axis_Constrained = [0;1;0];
    %         RefVector_Aligned = [1;0;0];% 轨道系矢量
    %         RefVector_Constrained = [0;0;1];
    %
    %     end
    
    
    % 计算惯性系到标称系的四元数Qin
    Qin = quatmultiply(OrbitElement.Qio',Qon')';
    
    % 标称姿态角速度Nominal_AngleRate_Nominal_2_J2000(wni_n)
    Wni_n = Ano*[0;-OrbitElement.w0;0];
    Ani_n = [0;0;0];
    Qiz0 = OrbitElement.Qio;
    Wzi_z0 = [0;-OrbitElement.w0;0];
    Qiz = Qin;
    Wzi_z = Wni_n;
elseif Index(1) == 1                    % 惯性系
    % 根据index确定惯性系系到标称系的三轴欧拉角[Roll_Nominal;Pitch_Nominal;Yaw_Nominal]
    % [1  6]  Book模式下正对日_帆板轴指北（-Z对日，帆板轴指向惯性系Z轴）
    % [1  7]  Book模式下正对日_帆板轴垂直于北（-Z对日，帆板轴垂直惯性系Z轴）
    % [1  8]  Book模式下侧对日（-Z对日，帆板轴指向惯性系Z轴后旋转35°）
    % [1  9]  Shark模式下正对日_帆板轴指北（-X对日，帆板轴指向惯性系Z轴）
    % [1  10] Shark模式下正对日_帆板轴垂直于北（-X对日，帆板轴垂直于惯性系Z轴）
    % [1  11] Shark模式下侧对日（-X对日，帆板轴指向惯性系Z轴后旋转35°）
    if Index(2) == 0  %-X对日，Z指惯性北？
        Roll_Nominal = 0;
        Pitch_Nominal = 0;
        Yaw_Nominal = 0;
    elseif Index(2) == 1 % +X对日
        Roll_Nominal = 0;
        Pitch_Nominal = 0;
        Yaw_Nominal = pi;
    elseif Index(2) == 2 % -Y对日
        Roll_Nominal = 0;
        Pitch_Nominal = 0;
        Yaw_Nominal = -pi/2;
    elseif Index(2) == 3 % +Y对日
        Roll_Nominal = 0;
        Pitch_Nominal = 0;
        Yaw_Nominal = pi/2;
    elseif Index(2) == 4 % -Z对日
        Roll_Nominal = 0;
        Pitch_Nominal = pi/2; % +Z对日
        Yaw_Nominal = 0;
    elseif Index(2) == 5
        Roll_Nominal = 0;
        Pitch_Nominal = -pi/2;
        Yaw_Nominal = 0;
    elseif Index(2) == 6 % -X对日(SharkFin)，帆板轴向惯性系Z轴后绕Z轴转-35°（按321转序则先z35，再x180）
        Roll_Nominal = pi;
        Pitch_Nominal = 0;
        Yaw_Nominal = 0+35/180*pi;
    elseif Index(2) == 7 % -z对日(Book)，+X朝向惯性系Z轴后绕X轴转35°
        Roll_Nominal = 0+35/180*pi;
        Pitch_Nominal = -pi/2;
        Yaw_Nominal = pi;
    elseif Index(2) == 8 % -z对日(Book)，+X朝惯性系Z轴
        Roll_Nominal = 0;
        Pitch_Nominal = -pi/2;
        Yaw_Nominal = pi;
    elseif Index(2) == 9 % -z对日(Book)，+X垂直惯性系Z轴
        Roll_Nominal = pi/2;
        Pitch_Nominal = 0;
        Yaw_Nominal = pi/2;
    elseif Index(2) == 10 % -X对日(SharkFin)，帆板轴向惯性系Z轴
        Roll_Nominal = pi;
        Pitch_Nominal = 0;
        Yaw_Nominal = 0;
    elseif Index(2) == 11 % -X对日(SharkFin)，帆板轴垂直于惯性系Z轴
        Roll_Nominal = pi/2;
        Pitch_Nominal = 0;
        Yaw_Nominal = 0;
    else
        Roll_Nominal = 10/180*pi;
        Pitch_Nominal = -5/180*pi;
        Yaw_Nominal = 5/180*pi;
    end
    % 计算惯性系到标称系的转移矩阵Ani及四元数Qin
    Qin = quatmultiply(angle2quat(pi+EnvirParam.Asc_Sun,EnvirParam.Dec_Sun,0,'ZYX'),angle2quat(Yaw_Nominal,Pitch_Nominal,Roll_Nominal,'ZYX'))';
    Qon = quatmultiply(quatinv(OrbitElement.Qio'),Qin')';
    
    Wni_n = [0;0;0];
    Ani_n = [0;0;0];
    Qiz0 = angle2quat(pi+EnvirParam.Asc_Sun,EnvirParam.Dec_Sun,0,'ZYX')';
    Wzi_z0 = [0;0;0];
    Qiz = Qin;
    Wzi_z = [0;0;0];
elseif Index(1) == 2   %  凝视算法
    Long_DMZ = Tc_StareCtrl_LLA(1);% deg
    Lati_DMZ = Tc_StareCtrl_LLA(2);% deg
    Alti_DMZ = Tc_StareCtrl_LLA(3);% m
    A_GI = quat2dcm(OrbitElement.Qif');
    Rib_I = OrbitElement.RV_i(1:3);
    dRib_I = OrbitElement.RV_i(4:6);
    [Rbn_I,yo_I,Rin_I] = GndStaVecCal(A_GI,OrbitElement,Long_DMZ/180*pi,Lati_DMZ/180*pi,Alti_DMZ);
    We_I  = [0,0,7.292116e-5]'; %地球自转角速度在惯性系中的分量列阵
    mu=3.9860044e14;
    Woi_o = [0;-OrbitElement.w0;0];
    Woi_i = quat2dcm(OrbitElement.Qio')'*Woi_o;
    %凝视坐标系的定义
    zN=Rbn_I/norm(Rbn_I);
    pN = cross(yo_I,zN);
    xN = pN/norm(pN);
    yN = cross(zN,xN);
    
    %矢量一阶导数
    dRin_I = cross(We_I,Rin_I);  %Rin_I未求
    dRbn_I = dRin_I - dRib_I;
    dzN = f0(zN,Rbn_I,dRbn_I);
    dyo_I = cross(Woi_i,yo_I);
    dpN = cross(dyo_I,zN)+cross(yo_I,dzN);
    dxN = f0(xN,pN,dpN);
    dyN = cross(dzN,xN)+cross(zN,dxN);
    
    %矢量二阶导数
    ddRib_I = -mu/norm(Rib_I)^3*Rib_I;    %万有引力求解卫星加速度信息，没考虑摄动
    ddRin_I = cross(We_I,dRin_I);
    ddzI = f1(zN,dzN,Rbn_I,dRbn_I,ddRin_I-ddRib_I);
    ddpI = cross(cross(Woi_i,dyo_I),zN)+2*cross(dyo_I,dzN)+cross(yo_I,ddzI);
    ddxI = f1(xN,dxN,pN,dpN,ddpI);
    ddyI = cross(ddzI,xN)+2*cross(dzN,dxN)+cross(zN,ddxI);
    
    %姿态四元数求解
    ATi = [xN yN zN]';
    Qin = dcm2quat(ATi)';
    Qon = quatmultiply(quatinv(OrbitElement.Qio'),Qin')';
    %姿态角速度求解
    Wni_n = ([zN'*dyN;xN'*dzN;yN'*dxN]);
    
    %姿态角加速度求解
    Ani_n = ([dzN'*dyN;dxN'*dzN;dyN'*dxN])+([zN'*ddyI; xN'*ddzI; yN'*ddxI]);
    Qiz0 = OrbitElement.Qio;
    Wzi_z0 = [0;-OrbitElement.w0;0];
    Qiz = Qiz0;
    Wzi_z = Wzi_z0;
elseif Index(1) == 3 % 零多普勒
    if Cnt_DPL == 0 || Cnt_DPL ~= RunCount
        First = 1;
        Second = 0;
        Cnt_DPL = RunCount;
    elseif First == 1
        Second = 1;
        First = 0;
    else
        Second = 0;
        First = 0;
    end
    omegaearth=7.2921e-5;
    N = OrbitElement.w0/omegaearth;
    hangji = asin(OrbitElement.e*sin(OrbitElement.f)/sqrt(1+OrbitElement.e^2+2*OrbitElement.e*cos(OrbitElement.f)));
    psi = -atan(cos(OrbitElement.omg+OrbitElement.f)*sin(OrbitElement.i)/(N*cos(hangji)-cos(OrbitElement.i)));
    % psai = -atan(cos(u_orbit)*sin(i_orbit)/(N-cos(i_orbit)));
    theta = asin(N*sin(hangji)/sqrt(N^2-2*N*cos(hangji)*cos(OrbitElement.i)+cos(OrbitElement.i)^2+sin(OrbitElement.i)^2*cos(OrbitElement.omg+OrbitElement.f)^2));
    %theta = asin(N*e_orbit*sin(f_orbit)/sqrt(1+e_orbit^2+2*e_orbit*cos(f_orbit))/(N-cos(i_orbit)));    %俯仰
    Qon = angle2quat(psi,theta,Tc_ZeroDpl_IncidAgl/180*pi,'ZYX')';   %%% 20240723 原来用的下一行的123转序，应该是321
    %     Qon = angle2quat(Tc_ZeroDpl_IncidAgl/180*pi,theta,psi,'XYZ')';% wrong
    % 计算惯性系到标称系的四元数Qin
    Qin = quatmultiply(OrbitElement.Qio',Qon')';
    
    % 标称姿态角速度Nominal_AngleRate_Nominal_2_J2000(wni_n)
    if First == 1
        Wni_n = quat2dcm(Qon')*[0;0;0];
        Ani_n = [0;0;0];
    else
        Wni_n = QuatW(Qin,Qin_last,t_ctrl);
        if Second == 1
            Ani_n = [0;0;0];
        else
            Ani_n = (Wni_n - Wni_n_last)/t_ctrl;
        end
    end
    
    Qiz0 = OrbitElement.Qio;
    Wzi_z0 = [0;-OrbitElement.w0;0];
    Qiz = Qiz0;
    Wzi_z = Wzi_z0;
    Cnt_DPL = Cnt_DPL + 1;
elseif Index(1) == 4 % 偏流角
    if Cnt_PLJ == 0 || Cnt_PLJ ~= RunCount
        First = 1;
        Second = 0;
        Cnt_PLJ = RunCount;
        Wno_n=[0;0;0];
    elseif First == 1
        Second = 1;
        First = 0;
    else
        Second = 0;
        First = 0;
    end
    we=7.292115855305e-5; % 地球自转速度，rad/s
    ae=6378.137e3;% 地球长半轴，m
    be=6356.752e3; % 地球短半轴，m
    w0=OrbitElement.w0; %轨道角速度，rad/s
    
    % 相机参数
    f_Camera = CamParam.CamA.f;     % 焦距，m
    d_Camera = CamParam.CamA.d;     % 像元尺寸4.6um
    lizhou =  CamParam.CamA.lizhou; % 离轴角，rad
    Rsc_b = [0;0;0];        % 星本体质心到相机(物镜)距离 Cam_r_L_L-Cent，可以忽略
    Mcb = CamParam.CamA.Aba';       % 本体系到相机的安装矩阵，
    
    AI2G=quat2dcm(OrbitElement.Qif');
    AG2I=AI2G';
    Agl_Tgt = Tc_PLJ_TgtAgl/180*pi;%侧摆目标角，123转序
    if Tc_PLJ_DB_Enable == 1%定标开关，定标时仅偏航90°+偏流角
        Agl_Tgt = [0;0;0];
    end
    Ao2bT=angle2dcm(Agl_Tgt(1),Agl_Tgt(2),0,'XYZ');  %轨道系到本体系转换矩阵; 20240723原Ao2bT计算中多了一个转置，删除。
    Ab2o=Ao2bT';
    if First == 1
        Wno_n=[0;0;0];
    end
    dAo2b=-[0 -Wno_n(3) Wno_n(2);Wno_n(3) 0 -Wno_n(1);-Wno_n(2) Wno_n(1) 0]*Ao2bT;   %转换矩阵的导数
    Ai2o=quat2dcm(OrbitElement.Qio');
    Ao2i=Ai2o';
    rs=OrbitElement.a*(1-OrbitElement.e^2)/(1+OrbitElement.e*cos(OrbitElement.f));%地心距，m
    Res_o=[0 0 -rs]'; % 地球到星体矢量
    dRes_o=[0 0 0 ]';%按照圆轨道处理
    dAi2o=[0 0 w0;0 0 0;-w0 0 0]*Ai2o;
    
    A=AI2G*Ao2i*Ab2o/Mcb*[tan(lizhou) 0 1]';
    B=AI2G*Ao2i*Res_o;
    A1=A(1);A2=A(2);A3=A(3);
    B1=B(1);B2=B(2);B3=B(3);
    a=A1^2/ae^2+A2^2/ae^2+A3^2/be^2;
    b=2*A1*B1/ae^2+2*A2*B2/ae^2+2*A3*B3/be^2;
    c=B1^2/ae^2+B2^2/ae^2+B3^2/be^2-1;
    if (b^2-4*a*c)>0 && a~=0
        Zc=(-b-sqrt(b^2-4*a*c))/(2*a);%即文中的x
        
        Ret_G=A*Zc+B;
        Ret_I=AG2I*Ret_G;
        dRet_I=AG2I*[0 -we 0;we 0 0;0 0 0]*Ret_G;
        
        %计算像移速度和坐标。若仅计算偏流角，则下列可以不用。
        %    p1=f_Camera*tan(-lizhou);
        %    p2=0;
        %    g12=[ -(p1)/f_Camera*Zc -(p2)/f_Camera*Zc ];%像点坐标
        Rst_o=Ai2o*Ret_I-Res_o;
        Rst_b=Ao2bT*Rst_o;
        Rct_c=Mcb*(Rst_b-Rsc_b);
        H=(Rct_c(3));
        %--若仅计算偏流角，则下式中的-f/H可不要-----
        dRpi_p=-f_Camera/H*Mcb*(dAo2b*(Ai2o*Ret_I-Res_o)+Ao2bT*(dAi2o*Ret_I+Ai2o*dRet_I-dRes_o));
        beta=atan2(dRpi_p(2),dRpi_p(1));%偏流角，deg
        Vp=[dRpi_p(1);dRpi_p(2);0];%像移速度
    else
        beta = 0;
        Vp=[0;0;0];
    end
    % 计算目标姿态
    if Tc_PLJ_DB_Enable == 1
        Qon = angle2quat(Agl_Tgt(1),Agl_Tgt(2),beta+pi/2,'XYZ')';
    else
        Qon = angle2quat(Agl_Tgt(1),Agl_Tgt(2),beta,'XYZ')';
    end
    
    % 临时变量处理
    if First==1
        Wno_n=[0;0;0];
    else
        if 1==1 % 20240723当安装矩阵非单位阵时，会引起误差逐渐累积发散，误差累积速度与侧摆角有关。因此选择直接赋0
            Wno_n=[0;0;0];%精确求解时需要考虑偏流角引起的偏航角速度。
            % Wno计算Qon,再通过Qon计算Wno，形成了循环，当安装矩阵非单位阵时，会引起误差逐渐累积发散，误差累积速度与侧摆角有关。
        else
            Wno_n = QuatW(Qon,Qon_last,t_ctrl);
        end
    end
    
    % 计算相机积分时间
    Norm_Vp = norm(Vp);
    if Norm_Vp == 0
        InteTime = 0;
    else
        Qob_Tgt0 = angle2quat(Agl_Tgt(1),Agl_Tgt(2),0,'XYZ')';
        QbT0b = quatmultiply(quatinv(Qob_Tgt0'),AttRef.Qob')';
        if Tc_PLJ_DB_Enable == 1
            V_fly = Vp'*(quat2dcm(QbT0b')'*[0;-1;0]);
        else
            V_fly = Vp'*(quat2dcm(QbT0b')'*[1;0;0]);%
        end
        InteTime = floor(d_Camera/V_fly*1e8+0.5);%相机积分时间，单位0.01us
    end
    
    % 计算惯性系到标称系的四元数Qin
    Qin = quatmultiply(OrbitElement.Qio',Qon')';
    
    % 标称姿态角速度Nominal_AngleRate_Nominal_2_J2000(wni_n)
    if First == 1
        Wni_n = quat2dcm(Qon')*[0;0;0];
        Ani_n = [0;0;0];
    else
        Wni_n = QuatW(Qin,Qin_last,t_ctrl);
        if Second == 1
            Ani_n = [0;0;0];
        else
            Ani_n = (Wni_n - Wni_n_last)/t_ctrl;
        end
    end
    
    Qiz0 = OrbitElement.Qio;
    Wzi_z0 = [0;-OrbitElement.w0;0];
    Qiz = Qiz0;
    Wzi_z = Wzi_z0;
    Cnt_PLJ = Cnt_PLJ + 1;
elseif Index(1) == 5 %Aligned and Constrained
    if Cnt_Al_Co == 0 || Cnt_Al_Co ~= RunCount
        First = 1;
        Second = 0;
        Cnt_Al_Co = RunCount;
        Wno_n=[0;0;0];
    elseif First == 1
        Second = 1;
        First = 0;
    else
        Second = 0;
        First = 0;
    end
    
    %     AlignedVector = [1;0;0];  % 本体系对齐轴
    %     ConstrainedVector = [0;0;-1]; % 本体系约束轴
    %     AlignedReferenceVector = [1;0;0];%对齐轴参考矢量（轨道系）
    %     ConstrainedReferenceVector = quat2dcm(OrbitElement.Qio')*EnvirParam.R_Sun;% 约束轴参考矢量（轨道系）
    AlignedVector = [0;-1;0];  % 本体系对齐轴
    ConstrainedVector = [0;0;1]; % 本体系约束轴
    AlignedReferenceVector = quat2dcm(OrbitElement.Qio')*EnvirParam.R_Sun;%对齐轴参考矢量（轨道系）
    ConstrainedReferenceVector =[0;0;1]; % 约束轴参考矢量（轨道系）
    Qon = AlignConstrainCalQuat(AlignedReferenceVector,AlignedVector,ConstrainedReferenceVector,ConstrainedVector);
    Qin = quatmultiply(OrbitElement.Qio',Qon')';
    % 标称姿态角速度Nominal_AngleRate_Nominal_2_J2000(wni_n)
    if First == 1
        Wni_n = quat2dcm(Qon')*[0;0;0];
        Ani_n = [0;0;0];
    else
        Wni_n = QuatW(Qin,Qin_last,t_ctrl);
        if Second == 1
            Ani_n = [0;0;0];
        else
            Ani_n = (Wni_n - Wni_n_last)/t_ctrl;
        end
    end
    Qiz0 = OrbitElement.Qio;
    Wzi_z0 = [0;-OrbitElement.w0;0];
    Qiz = Qiz0;
    Wzi_z = Wzi_z0;
    Cnt_Al_Co = Cnt_Al_Co + 1;
elseif Index(1) == 6
    a0max_w = 0.08;
%     a0max_w = 0.12;
    wmax_w = 2;
    if isempty(Path)
        T0 = OrbitElement.Time;
        qob = AttRef.Qob';
        qt = [cosd(5),sind(5)/sqrt(3)*[1,1,1]]';
%         qt = [cosd(5),sind(5)*[1,0,0]]';
        Path = generatePath(T0,qob,qt,a0max_w,wmax_w);
    end
    Ts = OrbitElement.Time;
    T0 = Path.T0;
    tr = Path.tr;
    ts = Path.ts;
    q0 = Path.q0;
    e = Path.e;

    if Ts < T0+tr
        k = (Ts-T0-0.5*tr)/tr*pi;
        wt = a0max_w * tr * (0.5*sin(k)+0.5);
        thetat = a0max_w*tr*0.5*(Ts-T0-cos(k)*tr/pi);
    elseif Ts < T0+tr+ts
        wt = a0max_w*tr;
        thetat = a0max_w*tr^2/2+a0max_w*tr*(Ts-t0-tr);
    elseif Ts < T0+2*tr+ts
        k = (T0 + 2*tr + ts - Ts -0.5*tr)/tr*pi;
        wt = a0max_w*tr * (0.5*sin(k)+0.5);
        thetat = a0max_w*tr*0.5*(Ts-T0+ts+cos(k)*tr/pi);
    else
        wt = 0;
        thetat = a0max_w*tr*(tr+ts);   
        Tc_Index = [0;100];
        clear Path
    end
%     Ts = Ts + 1;
    if Ts < T0+tr
        k = (Ts-T0-0.5*tr)/tr*pi;
        at = a0max_w * pi *0.5*cos(k);
    elseif Ts < T0+tr+ts
        at = 0;
    elseif Ts < T0+2*tr+ts
        k = (T0 + 2*tr + ts - Ts -0.5*tr)/tr*pi;
        at = -a0max_w*pi*0.5*cos(k);
    else
        at = 0;
    end
 
    theta = thetat*pi/360;
    Qon = quatmultiply(q0,[cos(theta),sin(theta)*e;]);
    Qin = quatmultiply(OrbitElement.Qio',Qon)';
    Wni_n = wt*e'/norm(e)/180*pi + quat2dcm(Qon)*[0;-OrbitElement.w0;0];
    Ani_n = at*e'/norm(e)/180*pi;
    Qiz0 = OrbitElement.Qio;
    Wzi_z0 = [0;-OrbitElement.w0;0];
    Qiz = Qiz0;
%     Wzi_z = Wzi_z0;
    Wzi_z = [at;wt;thetat];
else
    error('NominalCYSYDeterError!');
end
NominalCSYS = struct('Qin',Qin,'Wni_n',Wni_n,'Ani_n',Ani_n,'Qiz0',Qiz0,'Wzi_z0',Wzi_z0,'Qiz',Qiz,'Wzi_z',Wzi_z);
Qin_last = Qin;
Qon_last = Qon;
Wni_n_last = Wni_n;
end

function [I, Cent, StructParam, Rsa_p1, Rsa_p2, dRsa_p1, dRsa_p2,dI, Wc_p1, Ac_p1, Wc_p2, Ac_p2] = SatParamCal(SatParam,SadaParam)
Agl_Sada = [SadaParam.Sada1.Agl;SadaParam.Sada2.Agl];
% W_Sada = [SadaParam.Sada1.Wc;SadaParam.Sada2.Wc];
Aba1 = SadaParam.Sada1.Aba0*angle2dcm(Agl_Sada(1),0,0,SadaParam.Sada1.RotOdr)';
Aba2 = SadaParam.Sada2.Aba0*angle2dcm(Agl_Sada(3),0,0,SadaParam.Sada2.RotOdr)';
Aa1p1 = angle2dcm(0,Agl_Sada(2),0,SadaParam.Sada1.RotOdr)';
Aa2p2 = angle2dcm(0,Agl_Sada(4),0,SadaParam.Sada2.RotOdr)';
Abp1 = SadaParam.Sada1.Aba0*angle2dcm(Agl_Sada(1),Agl_Sada(2),0,SadaParam.Sada1.RotOdr)';
Abp2 = SadaParam.Sada2.Aba0*angle2dcm(Agl_Sada(3),Agl_Sada(4),0,SadaParam.Sada2.RotOdr)';

Wc_a1 = zeros(3,1);
Ac_a1 = zeros(3,1);
Wc_a2 = zeros(3,1);
Ac_a2 = zeros(3,1);
W_Sada1 = SadaParam.Sada1.Wc;
A_Sada1 = SadaParam.Sada1.Ac;
for i = 1:2
    switch SadaParam.Sada1.RotOdr(i)
        case 'X'
            Wc_a1(1) = W_Sada1(i);
            Ac_a1(1) = A_Sada1(i);
        case 'x'
            Wc_a1(1) = W_Sada1(i);
            Ac_a1(1) = A_Sada1(i);
        case 'Y'
            Wc_a1(2) = W_Sada1(i);
            Ac_a1(2) = A_Sada1(i);
        case 'y'
            Wc_a1(2) = W_Sada1(i);
            Ac_a1(2) = A_Sada1(i);
        case 'Z'
            Wc_a1(3) = W_Sada1(i);
            Ac_a1(3) = A_Sada1(i);
        case 'z'
            Wc_a1(3) = W_Sada1(i);
            Ac_a1(3) = A_Sada1(i);
    end
end
W_Sada2 = SadaParam.Sada2.Wc;
A_Sada2 = SadaParam.Sada2.Ac;
for i = 1:2
    switch SadaParam.Sada2.RotOdr(i)
        case 'X'
            Wc_a2(1) = W_Sada2(i);
            Ac_a2(1) = A_Sada2(i);
        case 'x'
            Wc_a2(1) = W_Sada2(i);
            Ac_a2(1) = A_Sada2(i);
        case 'Y'
            Wc_a2(2) = W_Sada2(i);
            Ac_a2(2) = A_Sada2(i);
        case 'y'
            Wc_a2(2) = W_Sada2(i);
            Ac_a2(2) = A_Sada2(i);
        case 'Z'
            Wc_a2(3) = W_Sada2(i);
            Ac_a2(3) = A_Sada2(i);
        case 'z'
            Wc_a2(3) = W_Sada2(i);
            Ac_a2(3) = A_Sada2(i);
    end
end

Wc_p1 = Aa1p1'*Wc_a1;
Ac_p1 = Aa1p1'*Ac_a1;
Wc_p2 = Aa2p2'*Wc_a2;
Ac_p2 = Aa2p2'*Ac_a2;

if SadaParam.Spread(1) == 1 || SadaParam.Spread(2) == 1  % 0未展开1已展开
    mL = SatParam.Body.m;
    r_L_Lc_L = SatParam.Body.rc;
    I_L_Lc_L = SatParam.Body.Ic;
    AbL = SatParam.Body.Aba;
    r_L_L_L = SatParam.Body.FixPoint;
else
    mL = SatParam.Body_Fold.m;
    r_L_Lc_L = SatParam.Body_Fold.rc;
    I_L_Lc_L = SatParam.Body_Fold.Ic;
    AbL = SatParam.Body_Fold.Aba;
    r_L_L_L = SatParam.Body_Fold.FixPoint;
end
if SadaParam.Spread(1) == 1
    ma1 = SatParam.A1.m;
    r_a1_ac1_a1 = SatParam.A1.rc;
    I_a1_ac1_a1 = SatParam.A1.Ic;
    %     Aba1 = Aba1;
    r_L_a1_L = SatParam.A1.FixPoint;
    mp1 = SatParam.P1.m;
    r_p1_pc1_p1 = SatParam.P1.rc;
    I_p1_pc1_p1 = SatParam.P1.Ic;
    %     Abp1 = Abp1;
    r_a1_p1_a1 = SatParam.P1.FixPoint;
    r_L_p1_L = r_L_a1_L + Aba1 * r_a1_p1_a1;
else
    ma1 = SatParam.A1_Fold.m;
    r_a1_ac1_a1 = SatParam.A1_Fold.rc;
    I_a1_ac1_a1 = SatParam.A1_Fold.Ic;
    Aba1 = SadaParam.Sada1.Aba0;
    r_L_a1_L = SatParam.A1_Fold.FixPoint;
    mp1 = SatParam.P1_Fold.m;
    r_p1_pc1_p1 = SatParam.P1_Fold.rc;
    I_p1_pc1_p1 = SatParam.P1_Fold.Ic;
    Abp1 = Aba1;
    r_a1_p1_a1 = SatParam.P1_Fold.FixPoint;
    r_L_p1_L = r_L_a1_L + Aba1 * r_a1_p1_a1;
end
if SadaParam.Spread(2) == 1
    ma2 = SatParam.A2.m;
    r_a2_ac2_a2 = SatParam.A2.rc;
    I_a2_ac2_a2 = SatParam.A2.Ic;
    %     Aba2 = Aba2;
    r_L_a2_L = SatParam.A2.FixPoint;
    
    mp2 = SatParam.P2.m;
    r_p2_pc2_p2 = SatParam.P2.rc;
    I_p2_pc2_p2 = SatParam.P2.Ic;
    %     Abp2 = Abp2;
    r_a2_p2_a2 = SatParam.P2.FixPoint;
    r_L_p2_L = r_L_a2_L + Aba2 * r_a2_p2_a2;
else
    ma2 = SatParam.A2_Fold.m;
    r_a2_ac2_a2 = SatParam.A2_Fold.rc;
    I_a2_ac2_a2 = SatParam.A2_Fold.Ic;
    Aba2 = SadaParam.Sada2.Aba0;
    r_L_a2_L = SatParam.A2_Fold.FixPoint;
    
    mp2 = SatParam.P2_Fold.m;
    r_p2_pc2_p2 = SatParam.P2_Fold.rc;
    I_p2_pc2_p2 = SatParam.P2_Fold.Ic;
    Abp2 = Aba2;
    r_a2_p2_a2 = SatParam.P2_Fold.FixPoint;
    r_L_p2_L = r_L_a2_L + Aba2 * r_a2_p2_a2;
end

[mh,r_L_hc_L,I_h_hc_L,Abh,r_L_h_L] = MassPropertyOfCombination...
    (mL,r_L_Lc_L,I_L_Lc_L,AbL,r_L_L_L,ma1,r_a1_ac1_a1,I_a1_ac1_a1,Aba1,r_L_a1_L);
[mh,r_L_hc_L,I_h_hc_L,Abh,r_L_h_L] = MassPropertyOfCombination...
    (mh,r_L_hc_L,I_h_hc_L,Abh,r_L_h_L,ma2,r_a2_ac2_a2,I_a2_ac2_a2,Aba2,r_L_a2_L);
[mh,r_L_hc_L,I_h_hc_L,Abh,r_L_h_L] = MassPropertyOfCombination...
    (mh,r_L_hc_L,I_h_hc_L,Abh,r_L_h_L,mp1,r_p1_pc1_p1,I_p1_pc1_p1,Abp1,r_L_p1_L);
[~,r_L_hc_L,I_h_hc_L,~,~] = MassPropertyOfCombination...
    (mh,r_L_hc_L,I_h_hc_L,Abh,r_L_h_L,mp2,r_p2_pc2_p2,I_p2_pc2_p2,Abp2,r_L_p2_L);
I = I_h_hc_L;
Cent = r_L_hc_L;

% 计算布局系下各个面的形心、法线
FigCent_Px = SatParam.StructPx.FigCent;
FigCent_Nx = SatParam.StructNx.FigCent;
FigCent_Py = SatParam.StructPy.FigCent;
FigCent_Ny = SatParam.StructNy.FigCent;
FigCent_Pz = SatParam.StructPz.FigCent;
FigCent_Nz = SatParam.StructNz.FigCent;

FigCent_Pp1 = SatParam.A1.FixPoint+SatParam.A1.Aba*SatParam.P1.FixPoint+Abp1*SatParam.StructPp1.FigCent;  %r_L_a1_b+Aba10*r_a1_p1_a1+Abp*CenterOfFigPa
FigCent_Np1 = SatParam.A1.FixPoint+SatParam.A1.Aba*SatParam.P1.FixPoint+Abp1*SatParam.StructNp1.FigCent;
FigCent_Pp2 = SatParam.A2.FixPoint+SatParam.A2.Aba*SatParam.P2.FixPoint+Abp2*SatParam.StructPp2.FigCent;
FigCent_Np2 = SatParam.A2.FixPoint+SatParam.A2.Aba*SatParam.P2.FixPoint+Abp2*SatParam.StructNp2.FigCent;
n_Px = SatParam.StructPx.n;
n_Nx = SatParam.StructNx.n;
n_Py = SatParam.StructPy.n;
n_Ny = SatParam.StructNy.n;
n_Pz = SatParam.StructPz.n;
n_Nz = SatParam.StructNz.n;
n_Pp1 = Abp1 * SatParam.StructPp1.n;
n_Np1 = Abp1 * SatParam.StructNp1.n;
n_Pp2 = Abp2 * SatParam.StructPp2.n;
n_Np2 = Abp2 * SatParam.StructNp2.n;
StructPx = struct('n',n_Px,'S',SatParam.StructPx.S,'FigCent',FigCent_Px);
StructNx = struct('n',n_Nx,'S',SatParam.StructNx.S,'FigCent',FigCent_Nx);
StructPy = struct('n',n_Py,'S',SatParam.StructPy.S,'FigCent',FigCent_Py);
StructNy = struct('n',n_Ny,'S',SatParam.StructNy.S,'FigCent',FigCent_Ny);
StructPz = struct('n',n_Pz,'S',SatParam.StructPz.S,'FigCent',FigCent_Pz);
StructNz = struct('n',n_Nz,'S',SatParam.StructNz.S,'FigCent',FigCent_Nz);
StructPp1 = struct('n',n_Pp1,'S',SatParam.StructPp1.S,'FigCent',FigCent_Pp1);
StructNp1 = struct('n',n_Np1,'S',SatParam.StructNp1.S,'FigCent',FigCent_Np1);
StructPp2 = struct('n',n_Pp2,'S',SatParam.StructPp2.S,'FigCent',FigCent_Pp2);
StructNp2 = struct('n',n_Np2,'S',SatParam.StructNp2.S,'FigCent',FigCent_Np2);
StructParam = struct('Px',StructPx,'Nx',StructNx,'Py',StructPy,'Ny',StructNy,'Pz',StructPz,'Nz',StructNz,...
    'Pp1',StructPp1,'Np1',StructNp1,'Pp2',StructPp2,'Np2',StructNp2);

% 计算帆板转动耦合系数
if SadaParam.Spread(1) == 1  % 0未展开1已展开
    Ip1_p1_p1 = I_p1_pc1_p1 + mp1*(r_p1_pc1_p1'*r_p1_pc1_p1*eye(3)-r_p1_pc1_p1*r_p1_pc1_p1');
    r_b_p1_b = r_L_a1_L + Aba1 * r_a1_p1_a1 - Cent;
    Rsa_p1 = xw(r_b_p1_b)*Abp1*mp1*xw(r_p1_pc1_p1)'+Abp1*Ip1_p1_p1;
    dRsa_p1 = xw(r_b_p1_b)*Abp1*xw(Wc_p1)*mp1*xw(r_p1_pc1_p1)'+Abp1*xw(Wc_p1)*Ip1_p1_p1;
    dI1 =  xw(r_b_p1_b)*mp1*Abp1*xw(Wc_p1)*xw(r_p1_pc1_p1)'*Abp1' - ...
        xw(r_b_p1_b)*mp1*Abp1*xw(r_p1_pc1_p1)'*xw(Wc_p1)*Abp1' + ...
        mp1*Abp1*xw(Wc_p1)*xw(r_p1_pc1_p1)'*Abp1'*xw(r_b_p1_b)' - ...
        mp1*Abp1*xw(r_p1_pc1_p1)'*xw(Wc_p1)*Abp1'*xw(r_b_p1_b)' + ...
        Abp1*xw(Wc_p1)*Ip1_p1_p1*Abp1' - ...
        Abp1*Ip1_p1_p1*xw(Wc_p1)*Abp1';
else
    Rsa_p1 = zeros(3);
    dRsa_p1 = zeros(3);
    dI1 = zeros(3);
end
if SadaParam.Spread(2)==1  % 0未展开1已展开
    Ip2_p2_p2 = I_p2_pc2_p2 + mp2*(r_p2_pc2_p2'*r_p2_pc2_p2*eye(3)-r_p2_pc2_p2*r_p2_pc2_p2');
    r_b_p2_b = r_L_a2_L + Aba2 * r_a2_p2_a2 - Cent;
    Rsa_p2 = xw(r_b_p2_b)*Abp2*mp2*xw(r_p2_pc2_p2)'+Abp2*Ip2_p2_p2;
    dRsa_p2 = xw(r_b_p2_b)*Abp2*xw(Wc_p2)*mp2*xw(r_p2_pc2_p2)'+Abp2*xw(Wc_p2)*Ip2_p2_p2;
    dI2 =  xw(r_b_p2_b)*mp2*Abp2*xw(Wc_p2)*xw(r_p2_pc2_p2)'*Abp2' - ...
        xw(r_b_p2_b)*mp2*Abp2*xw(r_p2_pc2_p2)'*xw(Wc_p2)*Abp2' + ...
        mp2*Abp2*xw(Wc_p2)*xw(r_p2_pc2_p2)'*Abp2'*xw(r_b_p2_b)' - ...
        mp2*Abp2*xw(r_p2_pc2_p2)'*xw(Wc_p2)*Abp2'*xw(r_b_p2_b)' + ...
        Abp2*xw(Wc_p2)*Ip2_p2_p2*Abp2' - ...
        Abp2*Ip2_p2_p2*xw(Wc_p2)*Abp2';
    
else
    Rsa_p2 = zeros(3);
    dRsa_p2 = zeros(3);
    dI2 = zeros(3);
end
dI = dI1+dI2;
end

%% 质量特性组合
function [mh,r_L_hc_L,I_h_hc_L,Abh,r_L_h_L] = MassPropertyOfCombination...
    (m1,r_p1_pc1_p1,I_p1_pc1_p1,Abp1,r_L_p1_L,m2,r_p2_pc2_p2,I_p2_pc2_p2,Abp2,r_L_p2_L)
% 附件p1和p2组合成组合体h
Abh = eye(3);
r_L_h_L = [0;0;0];
mh = m1+m2;
r_L_pc1_L = r_L_p1_L + Abp1*r_p1_pc1_p1;
r_L_pc2_L = r_L_p2_L + Abp2*r_p2_pc2_p2;
r_L_hc_L = (m1*r_L_pc1_L+m2*r_L_pc2_L)/(m1+m2);


r_pc1_hc_L = r_L_pc1_L - r_L_hc_L;
r_pc2_hc_L = r_L_pc2_L - r_L_hc_L;

I_h_hc_L = Abp1 * I_p1_pc1_p1 * Abp1' + m1 * (r_pc1_hc_L'*r_pc1_hc_L*eye(3)-r_pc1_hc_L*r_pc1_hc_L')...
    +Abp2 * I_p2_pc2_p2 * Abp2' + m2 * (r_pc2_hc_L'*r_pc2_hc_L*eye(3)-r_pc2_hc_L*r_pc2_hc_L');
end

function [Fs_all, Ts_all, Fa_all, Ta_all,Tg,To] = TdCal(StructParam,I,Cent,EnvirParam,OrbitElement,AttRef,Hw3)
global Tc_TdCal_Enable;%力矩前馈开关:1光压/2气动/3重力梯度/4磁控/5轨道陀螺/6 SADA驱动

if Tc_TdCal_Enable(3) == 1% 重力梯度
    Abo = quat2dcm(AttRef.Qob');
    Tg = 3*OrbitElement.w0^2*cross(Abo*[0;0;1],I*Abo*[0;0;1]);
else
    Tg = [0;0;0];
end
if Tc_TdCal_Enable(5) == 1% 轨道陀螺
    To = -cross(AttRef.Wbi,I*AttRef.Wbi+Hw3);
else
    To= [0;0;0];
end
Fs = zeros(3,10);
Ts = zeros(3,10);
if Tc_TdCal_Enable(1) == 1%光压
    gama_Sun = EnvirParam.gama_Sun;
    Sb = quat2dcm(AttRef.Qib')*EnvirParam.R_Sun;
    [Fs(:,1), Ts(:,1)] = Ts_Cal(Sb,StructParam.Px.S,StructParam.Px.n,StructParam.Px.FigCent,Cent,gama_Sun);
    [Fs(:,2), Ts(:,2)] = Ts_Cal(Sb,StructParam.Nx.S,StructParam.Nx.n,StructParam.Nx.FigCent,Cent,gama_Sun);
    [Fs(:,3), Ts(:,3)] = Ts_Cal(Sb,StructParam.Py.S,StructParam.Py.n,StructParam.Py.FigCent,Cent,gama_Sun);
    [Fs(:,4), Ts(:,4)] = Ts_Cal(Sb,StructParam.Ny.S,StructParam.Ny.n,StructParam.Ny.FigCent,Cent,gama_Sun);
    [Fs(:,5), Ts(:,5)] = Ts_Cal(Sb,StructParam.Pz.S,StructParam.Pz.n,StructParam.Pz.FigCent,Cent,gama_Sun);
    [Fs(:,6), Ts(:,6)] = Ts_Cal(Sb,StructParam.Nz.S,StructParam.Nz.n,StructParam.Nz.FigCent,Cent,gama_Sun);
    [Fs(:,7), Ts(:,7)] = Ts_Cal(Sb,StructParam.Pp1.S,StructParam.Pp1.n,StructParam.Pp1.FigCent,Cent,gama_Sun);
    [Fs(:,8), Ts(:,8)] = Ts_Cal(Sb,StructParam.Np1.S,StructParam.Np1.n,StructParam.Np1.FigCent,Cent,gama_Sun);
    [Fs(:,9), Ts(:,9)] = Ts_Cal(Sb,StructParam.Pp2.S,StructParam.Pp2.n,StructParam.Pp2.FigCent,Cent,gama_Sun);
    [Fs(:,10), Ts(:,10)] = Ts_Cal(Sb,StructParam.Np2.S,StructParam.Np2.n,StructParam.Np2.FigCent,Cent,gama_Sun);
end
Fs_all = sum(Fs,2);
Ts_all = sum(Ts,2);
Fa = zeros(3,10);
Ta = zeros(3,10);
if Tc_TdCal_Enable(2) == 1
    AirDensity = EnvirParam.AirDensity;
    V84 = OrbitElement.RV_f(4:6);
    Qfb = quatmultiply(quatinv(OrbitElement.Qif'),AttRef.Qib')';
    V84_b = quat2dcm(Qfb')*V84;
    [Fa(:,1), Ta(:,1)] = Ta_Cal(V84_b,AirDensity,StructParam.Px.S,StructParam.Px.n,StructParam.Px.FigCent,Cent);
    [Fa(:,2), Ta(:,2)] = Ta_Cal(V84_b,AirDensity,StructParam.Nx.S,StructParam.Nx.n,StructParam.Nx.FigCent,Cent);
    [Fa(:,3), Ta(:,3)] = Ta_Cal(V84_b,AirDensity,StructParam.Py.S,StructParam.Py.n,StructParam.Py.FigCent,Cent);
    [Fa(:,4), Ta(:,4)] = Ta_Cal(V84_b,AirDensity,StructParam.Ny.S,StructParam.Ny.n,StructParam.Ny.FigCent,Cent);
    [Fa(:,5), Ta(:,5)] = Ta_Cal(V84_b,AirDensity,StructParam.Pz.S,StructParam.Pz.n,StructParam.Pz.FigCent,Cent);
    [Fa(:,6), Ta(:,6)] = Ta_Cal(V84_b,AirDensity,StructParam.Nz.S,StructParam.Nz.n,StructParam.Nz.FigCent,Cent);
    [Fa(:,7), Ta(:,7)] = Ta_Cal(V84_b,AirDensity,StructParam.Pp1.S,StructParam.Pp1.n,StructParam.Pp1.FigCent,Cent);
    [Fa(:,8), Ta(:,8)] = Ta_Cal(V84_b,AirDensity,StructParam.Np1.S,StructParam.Np1.n,StructParam.Np1.FigCent,Cent);
    [Fa(:,9), Ta(:,9)] = Ta_Cal(V84_b,AirDensity,StructParam.Pp2.S,StructParam.Pp2.n,StructParam.Pp2.FigCent,Cent);
    [Fa(:,10), Ta(:,10)] = Ta_Cal(V84_b,AirDensity,StructParam.Np2.S,StructParam.Np2.n,StructParam.Np2.FigCent,Cent);
end
Fa_all = sum(Fa,2);
Ta_all = sum(Ta,2);
end

%% 光压力矩计算子函数
function [Fs, Ts] = Ts_Cal(Sb,S,nb,r,Cent,gama_Sun)
rou = 1;
mu = 0.2;
Ps = 1395/3e8;
if Sb'*nb <= 0
    Fs = [0;0;0];
    Ts = [0;0;0];
else
    theta = acos(Sb'*nb);
    CrossTmp = cross(cross(Sb,nb),nb);
    tao = CrossTmp/norm(CrossTmp);
    n = -nb;
    Fn = n*(Ps*S*cos(theta)*(2/3*rou*mu+(1+rou-rou*mu)*cos(theta)));
    Ft = tao*(Ps*S*cos(theta)*(1-rou+rou*mu)*sin(theta));
    Fs = gama_Sun*(Fn+Ft);
    R = r-Cent;
    Ts = cross(R,Fs);
end
end

%% 气动力矩计算子函数
function [Fa, Ta] = Ta_Cal(V84_b,rou_air,S,nb,r,Cent)
Cd = 2.2;

if nb'*V84_b <= 0
    Fa = [0;0;0];
    Ta = [0;0;0];
else
    Fa = -0.5*Cd*rou_air*V84_b*S*(nb'*V84_b);
    R = r - Cent;
    Ta = cross(R,Fa);
end
end
%%
function A_i284=DcmECI2ECEF(u)
%#codegen
we = 7.2921158e-5;
Tjc = u(1);
% r_j2000 = u(2:4);
% v_j2000 = u(5:7);
zetaA=(2306.2181*Tjc+0.30188*Tjc^2)/3600/180*pi;
zA=(2306.2181*Tjc+1.09468*Tjc^2)/3600/180*pi;
thetaA=(2004.3109*Tjc-0.42665*Tjc^2)/3600/180*pi;
RP=angle2dcm(-zetaA,thetaA,-zA,'ZYZ');

Omg_M=mod(125.044555556-1934.1361850*Tjc,360)/180*pi;
eps=0.409036061380284;
deltaRA=-17.2*sin(Omg_M)/3600/180*pi;
deltaeps=9.202*cos(Omg_M)/3600/180*pi;
RN=[cos(deltaRA*sin(eps)) 0 -sin(deltaRA*sin(eps));...
    -sin(deltaeps)*sin(deltaRA*sin(eps)) cos(deltaeps) -sin(deltaeps)*cos(deltaRA*sin(eps));...
    cos(deltaeps)*sin(deltaRA*sin(eps)) sin(deltaeps) cos(deltaeps)*cos(deltaRA*sin(eps))];

Theta=(mod(280.46061837+360.98564736629*Tjc*36525,360))/180*pi;
RS=[cos(Theta) sin(Theta) 0;-sin(Theta) cos(Theta) 0;0 0 1];
dRS = we*[-sin(Theta) cos(Theta) 0;-cos(Theta) -sin(Theta) 0;0 0 0];
RM=eye(3);
A_i284 = RM*RS*RN*RP;
% r_84 = A_i284 * r_j2000;
% v_84 = A_i284 * (v_j2000 - RP'*RN'*dRS'*RM'*r_84);
%
% y = [r_84;v_84];
end
%%
function [R_XGZ2Sat_i,yo_I,R_XGZ_i] = GndStaVecCal(A_GI,OrbitElement,Longitude,Latitude,Height)
% 本函数计算信关站相对于卫星的位置
% 信关站经纬高
% 轨道六根数转化为惯性系位置速度
a = OrbitElement.a;
e = OrbitElement.e;
i = OrbitElement.i;
OMG = OrbitElement.OMG;
omg = OrbitElement.omg;
E = OrbitElement.E;
f = OrbitElement.f;
mu=3.9860044e14;
sp=[cos(omg)*cos(OMG)-sin(omg)*sin(OMG)*cos(i);cos(omg)*sin(OMG)+sin(omg)*cos(OMG)*cos(i);sin(omg)*sin(i)];
sq=[-sin(omg)*cos(OMG)-cos(omg)*sin(OMG)*cos(i);-sin(omg)*sin(OMG)+cos(omg)*cos(OMG)*cos(i);cos(omg)*sin(i)];
R_Sat_i=a*(cos(E)-e)*sp+a*sqrt(1-e^2)*sin(E)*sq;
V_Sat_i=sqrt(mu/a/(1-e^2))*(-sin(f)*sp+(e+cos(f))*sq);
% disp(R_Sat_i');
% 信关站位置转化为地固系矢量
N=6378137*(1-2*0.00335281*(1-0.00335281/2)*sin(Latitude)^2)^(-1/2);
R_XGZ_84=[(N+Height)*cos(Latitude)*cos(Longitude);(N+Height)*cos(Latitude)*sin(Longitude);(N*(1-0.00335281)^2+Height)*sin(Latitude)];

% 计算信关站在惯性系下的位置

R_XGZ_i = A_GI' * R_XGZ_84;
% disp([1 R_XGZ_i']);
% 计算信关站在轨道系下的位置
Ai2o=angle2dcm(-pi/2,0,0,'XYZ')*angle2dcm(OMG,i,omg+f+pi/2,'ZXZ');
% Aoi_row3 = -R_Sat_i'/norm(R_Sat_i);
% Aoi_row2 = -cross(R_Sat_i,V_Sat_i)'/norm(cross(R_Sat_i,V_Sat_i));
% Aoi_row1 = cross(Aoi_row2',Aoi_row3')';
% Aio = [Aoi_row1;Aoi_row2;Aoi_row3];
R_XGZ2Sat_i=(R_XGZ_i-R_Sat_i);
R_XGZ2Sat_o=Ai2o*(R_XGZ_i-R_Sat_i);
% R_XGZ2Sat_o=R_XGZ2Sat_o/norm(R_XGZ2Sat_o);
R_stalat_o = Ai2o*A_GI' * (cross([0;0;1],R_XGZ_84));   %%%  LC 需要的是北东地系下的Y轴在o系下的投影 （巧用84系的信关站位置和84系的Z轴解算北东地系下的Y轴在84系下的表示）
R_stalat_o = R_stalat_o/norm(R_stalat_o);
yo_I = Ai2o'* [0;1;0];
end
%% 四元数求伪速率
function W = QuatW(Q,Q_last,t_ctrl)
Q = quatnormalize(Q');
Q_last = quatnormalize(Q_last');
dQ = quatmultiply(quatinv(Q_last),Q);
dQ = sign(dQ(1))*dQ;
Norm1 = norm(dQ(2:4));
if Norm1 >= 1e-6
    W = 2*acos(dQ(1))*dQ(2:4)'/Norm1/t_ctrl;
else
    W = [0;0;0];
end
end
%%
function y = f0(a,b,db)    % 单位向量一阶导数求解
y = (eye(3)-a*a')*db/norm(b);
end
%%
function y = f1(a,da,b,db,ddb)  % 单位向量二阶导数求解
y = -(da*a'+a*da')*db/norm(b)+(eye(3)-a*a')*(ddb/norm(b)-db*b'*db/norm(b)^3);
end
%%
function varargout = PersistentInit(varargin)
varargout = cell(nargin,1);
for i = 1:nargin
    if isempty(varargin{i})
        varargout{i}=0;% 或者varargout(i)={0};
    else
        varargout{i} = varargin{i};%或者varargout(i) = varargin(i);
    end
end
end

%%
function Timer = Cnt(Timer,V)
if V > 0.5
    Timer = Timer + 1;
else
    Timer = 0;
end
end
%% 根据对齐轴和约束轴获取坐标系转换四元数
function Q = AlignConstrainCalQuat(AlignedReferenceVector,AlignedVector,ConstrainedReferenceVector,ConstrainedVector)
% 根据对齐轴和约束轴获取坐标系转换四元数，对齐轴与约束轴应在同一坐标系下

% 根据对齐轴矢量计算旋转四元数Qob_0
Qob0 = Vec2Quat(AlignedVector,AlignedReferenceVector);%轨道系到本体系

% 计算b0系下的约束轴矢量
ReferenceVector2_b0 = quat2dcm(Qob0')*ConstrainedReferenceVector;

% 计算b0系下约束轴矢量和本体系下的约束轴矢量的二面角
Agl = DhdAglCal(ConstrainedVector,ReferenceVector2_b0,AlignedVector);

% 根据转轴和转角创建四元数
Qb0b = [cos(Agl/2);sin(Agl/2)*AlignedVector];

Qob = quatmultiply(Qob0',Qb0b')';

Q = Qob;

end

%% 计算二面角
function y = DhdAglCal(FromVec,ToVec,AboutVec)
% 计算二面角（带方向）
% FromVec：源平面的某一向量
% ToVec：目标平面的某一向量
% AboutVec：平面交线
V1 = cross(AboutVec,FromVec);
V2 = cross(AboutVec,ToVec);
Agl = acos(V1'*V2/norm(V1)/norm(V2));
V3 = cross(V1,V2);
AxisAgl = acos(AboutVec'*V3/norm(AboutVec)/norm(V3));
if (AxisAgl > pi/2)
    y = -Agl;
else
    y = Agl;
end
end

%% 根据两个三轴向量获取旋转四元数
function y = Vec2Quat(V1,V2)
%通过两个三轴向量获取旋转四元数
V1 = V1/norm(V1);
V2 = V2/norm(V2);
theta = acos(V1'*V2);
if (abs(theta-pi)<1e-12)
    if((abs(V1(1)-V1(2))<1e-12) && (abs(V1(2)-V1(3))<1e-12))
        V1(1) = -1;
    else
        V1 = [V1(2);V1(3);V1(1)];
    end
end
V3 = cross(V1,V2);
if norm(V3) < 1e-6
    Q = [1;0;0;0];
else
    Q  = [cos(theta/2);sin(theta/2)*V3/norm(V3)];
end
y = Q;

end

function y = xw(r)
y = [0 -r(3) r(2);r(3) 0 -r(1);-r(2) r(1) 0];
end

function Path = generatePath(T0,qob,qt,a0max_w,wmax_w)
dq = quatmultiply(quatinv(qob),qt');
theta = 2*acosd(dq(1));
if theta<1e-6
    theta = 0;
    e = [1,0,0];
else
    e = dq(2:4)/sind(theta/2);
end
if abs(theta) > wmax_w^2/a0max_w
    tr = abs(wmax_w/a0max_w);
    ts = abs(theta - a0max_w*tr^2)/wmax_w;
else
    tr = sqrt(abs(theta)/a0max_w);
    ts = 0;
end
Path.T0 = T0;
Path.tr = tr;
Path.ts = ts;
Path.q0 = qob;
Path.e = e;
end