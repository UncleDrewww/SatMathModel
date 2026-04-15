function [SADA_Mode, SadaCtrlMethod, Agl_Sada_Tgt] = ZZ_User_Sada(EnvirParam,NominalCSYS,SadaParam,Agl_Sada_Ctrl,OrbitElement,t_ctrl)

R_Sun = EnvirParam.R_Sun/norm(EnvirParam.R_Sun);
S_z = quat2dcm(NominalCSYS.Qiz')*R_Sun;%零标称系
S_a1_0 = SadaParam.Sada1.Aba0'*S_z;
S_a2_0 = SadaParam.Sada2.Aba0'*S_z;
Agl_Sada_Tgt = SADA_Nominal_CSYS(S_a1_0,S_a2_0,SadaParam,EnvirParam); % 各项目需根据太阳翼安装位置及SADA转序修改，rad

[SADA_Mode, SadaCtrlMethod] = SADA_ModeDeterAndInit(Agl_Sada_Tgt,Agl_Sada_Ctrl,SadaParam,OrbitElement,t_ctrl);% 0不控、1待机、2停转、3闭环+角度、4开环+角速度、5增量+增量角、6归零+初始角？
end

function Agl_Sada_Tgt = SADA_Nominal_CSYS(S_a1_0,S_a2_0,SadaParam,EnvirParam)
Agl_Sada_Tgt = [0;0;0;0;0;0];
Q1 = Vet2Quat([0;0;1], S_a1_0);
Q2 = Vet2Quat([0;0;1], S_a2_0);
[Agl_Sada_Tgt(1), Agl_Sada_Tgt(2)] = quat2angle(Q1',SadaParam.Sada1.RotOdr);
[Agl_Sada_Tgt(3), Agl_Sada_Tgt(4)] = quat2angle(Q2',SadaParam.Sada2.RotOdr);

% Agl_Sada_Tgt(1) = atan2(S_a1_0(2),-S_a1_0(3)); % 先X后Y的结果
% Agl_Sada_Tgt(2) = atan2(-S_a1_0(1),sqrt(S_a1_0(2)^2+S_a1_0(3)^2));
% Agl_Sada_Tgt(3) = atan2(S_a2_0(2),-S_a2_0(3));
% Agl_Sada_Tgt(4) = atan2(-S_a2_0(1),sqrt(S_a2_0(2)^2+S_a2_0(3)^2));

% Agl_Lmt = [150;45;150;45;0;0]/180*pi;     % SADA目标角度限幅，若有软限位，则该值比软限位略小。
% Agl_Sada_Tgt = max(min(Agl_Sada_Tgt,Agl_Lmt),-Agl_Lmt);
end

function [SADA_Mode_out,SadaCtrlMethod] = SADA_ModeDeterAndInit(Agl_Sada_Tgt,Agl_Sada_Ctrl,SadaParam,OrbitElement,t_ctrl)
% disp(1)
% CtrlMode Fre Dir Step
% 0不控、1待机、2停转、3闭环+角度、4开环+角速度、5增量+增量角、6归零+方向？
% Fre 1Hz  0.018deg/s
% Dir 0+  1-
SADA_HL_OK = [1;1;1;1];%霍尔状态
Orbit_Ok = 1;%轨道状态


global LcMark Tc_SadaCtrl_Cmd Tc_SadaCtrl_Enable Tc_SadaCtrlMethod SadaCtrlCoeff SadaModeParam;
persistent IncrementAgl SADA_Mode;
if isempty(SADA_Mode)
    SADA_Mode = zeros(6,2);
end
if isempty(IncrementAgl)
    IncrementAgl = [0;0;0;0];
end

W_Increment = SadaCtrlCoeff.w_max_ZL/180*pi;%角度增量时对应的角速度

SADA_Mode_last = SADA_Mode;

if LcMark.Mode == 1
    % 阶段1/2不发指令，阶段3帆板展开--待定
    if LcMark.SubMode == 3
        SadaModeParam.Mode1Cnt1 = Cnt(SadaModeParam.Mode1Cnt1,SadaParam.Sada1.HLState(4) == 1);
        SadaModeParam.Mode1Cnt2 = Cnt(SadaModeParam.Mode1Cnt2,SadaParam.Sada2.HLState(4) == 1);
        if SadaModeParam.ModeLevelB == 1
            SADA_Mode(2,:) = [5 190/180*pi];%增量模式
            if SadaModeParam.Mode1Cnt1 > 10 / t_ctrl
                SadaModeParam.ModeLevelB = 2;
            end
        elseif SadaModeParam.ModeLevelB == 2
            SadaModeParam.ModeLevelB = 3; %可以设置A轴的运动
        elseif SadaModeParam.ModeLevelB == 3
            SADA_Mode(2,:) = [1 0]; %待机模式
        end
        if SadaModeParam.ModeLevelD == 1
            SADA_Mode(4,:) = [5 -190/180*pi];%增量模式
            if SadaModeParam.Mode1Cnt2 > 10 / t_ctrl
                SadaModeParam.ModeLevelD = 2;
            end
        elseif SadaModeParam.ModeLevelD == 2
            SadaModeParam.ModeLevelD = 3;%%可以设置A轴的运动
        elseif SadaModeParam.ModeLevelD == 3
            SADA_Mode(4,:) = [1 0];%待机模式
        end
    else
        SADA_Mode(1,:) = [1 0];%待机
        SADA_Mode(2,:) = [1 0];
        SADA_Mode(3,:) = [1 0];
        SADA_Mode(4,:) = [1 0];
    end
elseif (LcMark.Mode == 2)||(LcMark.Mode == 5)
    SADA_Mode(2,:) = [1 0]; %B轴待机模式
    SADA_Mode(4,:) = [1 0]; %B轴待机模式
    if LcMark.SubMode == 1     
        SadaModeParam.Mode2Cnt1 = Cnt(SadaModeParam.Mode2Cnt1,SadaParam.Sada1.HLState(1) == 1);        
        if Tc_SadaCtrl_Enable(1) == 1
            if SadaModeParam.ModeLevelA == 1
                SADA_Mode(1 ,:) = [6 -1*sign(SadaParam.Sada1.Agl(1))]; %归零模式
                if SadaModeParam.Mode2Cnt1 > 20/t_ctrl
                    SadaModeParam.ModeLevelA = 2;
                end
            elseif SadaModeParam.ModeLevelA == 2  %增量阶段
                SADA_Mode(1,:) = [5 pi];%增量模式
                if SadaModeParam.IncrementTime(1) == 0
                    SadaModeParam.ModeLevelA = 3;
                end
            elseif SadaModeParam.ModeLevelA == 3
                SADA_Mode(1,:) = [1 0];%待机模式
                SadaModeParam.Sada1WorkEnd = 1;
            end
        end
        SadaModeParam.Mode2Cnt2 = Cnt(SadaModeParam.Mode2Cnt2,SadaParam.Sada1.HLState(3) == 1);
        if Tc_SadaCtrl_Enable(3) == 1
            if SadaModeParam.ModeLevelC == 1
                SADA_Mode(3 ,:) = [6 -1*sign(SadaParam.Sada2.Agl(1))]; %归零模式
                if SadaModeParam.Mode2Cnt2 > 20/t_ctrl
                    SadaModeParam.ModeLevelC = 2;
                end
            elseif SadaModeParam.ModeLevelC == 2  %增量阶段
                SADA_Mode(3,:) = [5 -pi];%增量模式
                if SadaModeParam.IncrementTime(3) == 0
                    SadaModeParam.ModeLevelC = 3;
                end
            elseif SadaModeParam.ModeLevelC == 3
                SADA_Mode(3,:) = [1 0];%待机模式
                SadaModeParam.Sada2WorkEnd = 1;
            end
        end        
    else
        SADA_Mode(1,:) = [1 0]; %A轴待机模式
        SADA_Mode(3,:) = [1 0]; %A轴待机模式
    end        
end
for i = 1:4
    if LcMark.Mode == 1
        
    elseif(LcMark.Mode == 2)||(LcMark.Mode == 5)

    elseif LcMark.Mode == 3
        if Orbit_Ok == 1
            SADA_Mode(i,:) = [3 Agl_Sada_Tgt(i)];   %AB轴闭环模式
        end    
%         if abs(Agl_Sada_Ctrl(i*2,1)) > 5/180*pi
%             if SADA_Mode(i*2,1) ~= 5
%                 IncrementAgl(i*2,1) = Agl_Sada_Ctrl(i*2,1);
%             end
%             SADA_Mode(i*2,:) = [5 IncrementAgl(i*2,1)];%增量模式
%         end
    else
        SADA_Mode(i,:) = [0 0];%不控模式
    end

    if Tc_SadaCtrl_Cmd(i,1) ~= 99  % 地面遥控指令：模式(默认99星上自主)+参数
        if Tc_SadaCtrl_Cmd(i,1)==3 || Tc_SadaCtrl_Cmd(i,1)==4 || Tc_SadaCtrl_Cmd(i,1)==5
            SADA_Mode(i,1) = Tc_SadaCtrl_Cmd(i,1);
            SADA_Mode(i,2) = Tc_SadaCtrl_Cmd(i,2)/180*pi;
        else
            SADA_Mode(i,:) = Tc_SadaCtrl_Cmd(i,:);
        end
    end
    if Tc_SadaCtrl_Enable(i,:) == 0 % SADA控制允许
        SADA_Mode(i,:) = [0 0];%不控模式
    end
end
for i = 1:4
    if SADA_Mode(i,1) == 5 % 增量模式
        if SADA_Mode_last(i,1) ~= 5 || SADA_Mode_last(i,2) ~= SADA_Mode(i,2)
            SadaModeParam.IncrementTime(i,1) = abs(SADA_Mode(i,2))/W_Increment(i);
        end
        if SadaModeParam.IncrementTime(i,1) > 0 % 增量进行中
            SadaModeParam.IncrementTime(i,1) = SadaModeParam.IncrementTime(i,1) - t_ctrl;
        else
            SadaModeParam.IncrementTime(i,1) = 0;
            SADA_Mode(i,:) = [1 0]; % 待机模式
        end
    end
end
for i = 5:6
% 展开电机
    SADA_Mode(i,:) = [1 0]; % 待机模式
end
SADA_Mode_out = SADA_Mode;

% ModeInitialization
SadaCtrlCoeff.ZH_W0(1) = abs(OrbitElement.w0)*180/pi;%滞环中心转速
SadaCtrlCoeff.ZH_W0(3) = -abs(OrbitElement.w0)*180/pi;
SadaCtrlMethod = Tc_SadaCtrlMethod;  %0-PI,1-滞环
% for i = 1:2:3
%     if LcMark.Mode == 3
%         if abs(Agl_Sada_Tgt(i))<150/180*pi
%             SadaCtrlMethod(i) = Tc_SadaCtrlMethod(i);
%         else
%             SadaCtrlMethod(i) = 0;
%         end
%     end
% end

end
%%
function Quat = Vet2Quat(Vet1,Vet2)
if norm(Vet1)<1e-6 || norm(Vet2)<1e-6
    Quat = [1;0;0;0];
else
    Vet1 = Vet1/norm(Vet1);
    Vet2 = Vet2/norm(Vet2);
    Agl = acos(max(min(Vet1'*Vet2,1),-1));
    if abs(Agl) < 1e-6 % Vet1/Vet2平行
        Quat = [1;0;0;0];
    else
        Axis = cross(Vet1, Vet2);
        Axis = Axis / norm(Axis);
        Quat = [cos(Agl/2);sin(Agl/2)*Axis];
    end
end
end

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

function Timer = Cnt(Timer,V)
if V > 0.5
    Timer = Timer + 1;
else
    Timer = 0;
end
end