function [QVEquipmentCmd, QVParam, Agl_QV_Tgt,Agl_QV_Ctrl] = ZZ_QV(OrbitElement,RunCount,t_ctrl,QVParam)
global Tc_QVCtrl_Enable Tc_QVWork_LLA Tc_QVWork_Time LcMark QVCtrlCoeff AttRef;

Agl_QV = [QVParam.QV1.Agl;QVParam.QV2.Agl]; % 【QVx Qvy】rad

%用户自定义函数，QV模式确定
[QV_Mode,Agl_QV_Tgt] = ZZ_User_QV(LcMark,OrbitElement,Tc_QVWork_LLA,QVParam,Tc_QVCtrl_Enable,Tc_QVWork_Time,QVCtrlCoeff,RunCount,t_ctrl,AttRef);

% Att4Ctrl
[Agl_QV_Ctrl, W_QV_Ctrl] = QV_Att4Ctrl(Agl_QV_Tgt,Agl_QV,QVParam,QVCtrlCoeff,t_ctrl);
QVParam.QV1.Agl4Ctrl=Agl_QV_Ctrl(1:2);
QVParam.QV2.Agl4Ctrl=Agl_QV_Ctrl(3:4);
%AttCtrl
[QVEquipmentCmd, Wc, Ac] = QV_AttCtrl(QV_Mode,Agl_QV_Ctrl,W_QV_Ctrl,Agl_QV,QVCtrlCoeff,t_ctrl);

QVParam.QV1.Wc = Wc(1:2);% QV系的角速度a,p  rad/s
QVParam.QV1.Ac = Ac(1:2);
QVParam.QV2.Wc = Wc(3:4);
QVParam.QV2.Ac = Ac(3:4);

end

%% QV模式确定
function [QV_Mode,Agl_QV_Tgt] = ZZ_User_QV(LcMark,OrbitElement,Tc_QVWork_LLA,QVParam,Tc_QVCtrl_Enable,Tc_QVWork_Time,QVCtrlCoeff,RunCount,t_ctrl,AttRef)
QV_Mode = zeros(4,2);
Agl_QV_Tgt = [0;0;0;0];
if (Tc_QVCtrl_Enable==1 && (LcMark.Mode == 3) && (LcMark.SubMode == 2))
  %% 对地模式第二阶段，QV处于工作区间内，计算模式、目标角度
    if Tc_QVWork_Time(1) < Tc_QVWork_Time(2)
        if (RunCount*t_ctrl >= Tc_QVWork_Time(1) && RunCount*t_ctrl <= Tc_QVWork_Time(2))
            [Agl_QV_Tgt(1),Agl_QV_Tgt(2)] = Agl_QV_Tgt_Cal(OrbitElement,AttRef,QVParam.QV1,Tc_QVWork_LLA);% QV1目标角度计算
            [Agl_QV_Tgt(3),Agl_QV_Tgt(4)] = Agl_QV_Tgt_Cal(OrbitElement,AttRef,QVParam.QV2,Tc_QVWork_LLA);% QV2目标角度计算
            for i = 1:4
                QV_Mode(i,1) = 3;              %闭环
                QV_Mode(i,2) = Agl_QV_Tgt(i);
            end
        elseif RunCount*t_ctrl > Tc_QVWork_Time(2)
             for i = 1:4
                QV_Mode(i,1) = 3;              %任务结束：闭环0°
                QV_Mode(i,2) = 0;
            end                       
        end
    end
end
Agl_QV_Tgt = Limit(Agl_QV_Tgt,QVCtrlCoeff.WorkArea(:,1),QVCtrlCoeff.WorkArea(:,2));  %目标角度限幅
end

function [Agl_QV_Ctrl, W_QV_Ctrl] = QV_Att4Ctrl(Agl_QV_Tgt,Agl_QV_Cur,QVParam,QVCtrlCoeff,t_ctrl)
persistent W_QV_Tgt_last Agl_QV_Tgt_last;
if isempty(W_QV_Tgt_last)
    W_QV_Tgt_last = [0;0;0;0];
end
if isempty(Agl_QV_Tgt_last)
    Agl_QV_Tgt_last = [0;0;0;0];
end

RotateType = QVCtrlCoeff.RotType;
LLmt = QVCtrlCoeff.WorkArea(:,1)/180*pi;  % 工作区左边界
RLmt = QVCtrlCoeff.WorkArea(:,2)/180*pi;  % 工作区右边界

Agl_Nogo = mod(LLmt-RLmt,2*pi);%禁区宽度
Mid_NoGo = Limit_pi(Agl_Nogo/2+RLmt);% 禁区中线

R_Cur = [cos(Agl_QV_Cur');sin(Agl_QV_Cur');[0 0 0 0]];
R_Tgt = [cos(Agl_QV_Tgt');sin(Agl_QV_Tgt');[0 0 0 0]];
R_Mid = [cos(Mid_NoGo');sin(Mid_NoGo');[0 0 0 0]];

Agl_QV_Ctrl = Agl_QV_Tgt - Agl_QV_Cur; % rad
Agl_QV_Ctrl = Limit_pi(Agl_QV_Ctrl);

for i = 1:4
    if strcmp(RotateType(i),'Around') % 绕行
        % 根据Agl_Sada_Ctrl符号可判断最短路径是想正转还是负转
        if Agl_QV_Ctrl(i) >0 %正转
            if RotZAgl(R_Cur(:,i),R_Mid(:,i))<RotZAgl(R_Cur(:,i),R_Tgt(:,i)) % C往T去经过M
                Agl_QV_Ctrl(i) = (abs(Agl_QV_Ctrl(i))-2*pi)*sign(Agl_QV_Ctrl(i));
            end
        else
            if RotZAgl(R_Cur(:,i),R_Mid(:,i))>RotZAgl(R_Cur(:,i),R_Tgt(:,i)) % C往T去经过M
                Agl_QV_Ctrl(i) = (abs(Agl_QV_Ctrl(i))-2*pi)*sign(Agl_QV_Ctrl(i));
            end
        end
    else % Stop不绕行

    end
end

W_QV_Tgt = (Agl_QV_Tgt-Agl_QV_Tgt_last)/t_ctrl;
W_QV_Tgt = W_QV_Tgt .* (abs(W_QV_Tgt)<QVCtrlCoeff.w_max) + W_QV_Tgt_last.* (abs(W_QV_Tgt)>QVCtrlCoeff.w_max);
W_QV_Ctrl = W_QV_Tgt -[QVParam.QV1.Wc;QVParam.QV2.Wc];

% % 系统app中用下面的方式计算W_Sada_Ctrl
% Delta_Agl_Sada_Ctrl = Agl_Sada_Ctrl-Agl_Sada_Ctrl_last;
% W_Sada_Ctrl = Delta_Agl_Sada_Ctrl/t_ctrl;
% W_Sada_Ctrl = W_Sada_Ctrl .* (abs(W_Sada_Ctrl)<SadaCtrlCoeff.w_max);

W_QV_Tgt_last = W_QV_Tgt;
Agl_QV_Tgt_last = Agl_QV_Tgt;
end

function [QVEquipmentCmd_out, Wc, Ac] = QV_AttCtrl(QVSysMode,Agl4Ctrl,W4Ctrl,Agl_QV_Cur,QVCtrlCoeff,t_ctrl)
% QVSysMode      :0不控、1待机、2停转、3闭环+目标角、4开环+角速度、5增量+增量角、6归零+初始角？
% QVEquipmentCmd :1待机、2停转、3开环+角速度、4归零+方向
% 用于判断是否在禁区
RotateType = QVCtrlCoeff.RotType;
LLmt = QVCtrlCoeff.WorkArea(:,1)/180*pi;  % 工作区左边界
RLmt = QVCtrlCoeff.WorkArea(:,2)/180*pi;  % 工作区右边界
Agl_Nogo = mod(LLmt-RLmt,2*pi);%禁区宽度
R_RLmt = [cos(RLmt');sin(RLmt');[0 0 0 0]];
R_Cur = [cos(Agl_QV_Cur');sin(Agl_QV_Cur');[0 0 0 0]];

QVEquipmentCmd = zeros(4,2);
Wc = zeros(4,1);
Ac = zeros(4,1);

for i = 1:4 % 4个电机
    if QVSysMode(i,1) == 0                          % 系统不控
        QVEquipmentCmd(i,:) = [1 0];                                       % 驱动器待机
    elseif QVSysMode(i,1) == 1                      % 系统待机
        QVEquipmentCmd(i,:) = [1 0];                                       % 驱动器待机
    elseif QVSysMode(i,1) == 2                      % 系统停转保持
        QVEquipmentCmd(i,:) = [2 0];                                       % 驱动器保持
    elseif QVSysMode(i,1) == 3                      % 系统闭环模式
        [Wc(i,1), Ac(i,1)] = PI_new(Agl4Ctrl,W4Ctrl,i,QVCtrlCoeff,t_ctrl); % 单位：rad/s、rad/s^2
        if strcmp(RotateType(i),'No') % 无禁区
        else
            % 判断当前是否已经进禁区且在深入    
            if RotZAgl(R_RLmt(:,i),R_Cur(:,i))<Agl_Nogo(i)/2 % 在负半边禁区且正向深入
                if Agl4Ctrl(i) > 0
                    Wc(i,1) = 0;
                    Ac(i,1) = 0;
                end
            elseif RotZAgl(R_RLmt(:,i),R_Cur(:,i))<Agl_Nogo(i) % 在正半边禁区且负向深入
                if Agl4Ctrl(i) < 0
                    Wc(i,1) = 0;
                    Ac(i,1) = 0;
                end
            else
            end
        end
        QVEquipmentCmd(i,:) = [3 Wc(i,1)];                                 % 驱动器开环
    elseif QVSysMode(i,1) == 4                      % 系统开环模式
        QVEquipmentCmd(i,:) = [3 QVSysMode(i,2)];                        % 驱动器开环
    elseif QVSysMode(i,1) == 5                      % 系统增量模式
        QVEquipmentCmd(i,:) = [3 sign(QVSysMode(i,2))*QVCtrlCoeff.w_max_ZL(i)/180*pi];   % 驱动器开环
    elseif QVSysMode(i,1) == 6                      % 系统归零模式
        QVEquipmentCmd(i,:) = [4 sign(QVSysMode(i,2))];                  % 驱动器归零
    end
end
QVEquipmentCmd_out = [QVEquipmentCmd(1,:)';QVEquipmentCmd(2,:)';QVEquipmentCmd(3,:)';QVEquipmentCmd(4,:)'];
end

%% 计算QV目标角度
function [Agl_X,Agl_Y] = Agl_QV_Tgt_Cal(OrbitElement,AttRef,QVParam,Tc_QVWork_LLA)
A_GI = quat2dcm(OrbitElement.Qif');
R_XGZ2Sat_o = GndStaVecCal(A_GI,OrbitElement,Tc_QVWork_LLA(1)/180*pi,Tc_QVWork_LLA(2)/180*pi,Tc_QVWork_LLA(3)/180*pi);
Abo = quat2dcm(AttRef.Qob');
R_XGZ2Sat_QV=QVParam.Aba0'*Abo*R_XGZ2Sat_o;
Q1 = Vet2Quat([0;0;1],R_XGZ2Sat_QV);
[Agl_X,Agl_Y]=quat2angle(Q1',QVParam.RotOdr);
% 计算信关站在本体系下的角度
% [Agl_X,Agl_Y]=atan2(-R_XGZ2Sat_QV(2),R_XGZ2Sat_QV(3));
% Agl_Y=atan2(R_XGZ2Sat_QV(1),sqrt(R_XGZ2Sat_QV(2)^2+R_XGZ2Sat_QV(3)^2));
end

function R_XGZ2Sat_o = GndStaVecCal(A_GI,OrbitElement,Longitude,Latitude,Height)
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
sp=[cos(omg)*cos(OMG)-sin(omg)*sin(OMG)*cos(i);cos(omg)*sin(OMG)+sin(omg)*cos(OMG)*cos(i);sin(omg)*sin(i)];
sq=[-sin(omg)*cos(OMG)-cos(omg)*sin(OMG)*cos(i);-sin(omg)*sin(OMG)+cos(omg)*cos(OMG)*cos(i);cos(omg)*sin(i)];
R_Sat_i=a*(cos(E)-e)*sp+a*sqrt(1-e^2)*sin(E)*sq;
% disp(R_Sat_i');
% 信关站位置转化为地固系矢量
N=6378137*(1-2*0.00335281*(1-0.00335281/2)*sin(Latitude)^2)^(-1/2);
R_XGZ_84=[(N+Height)*cos(Latitude)*cos(Longitude);(N+Height)*cos(Latitude)*sin(Longitude);(N*(1-0.00335281)^2+Height)*sin(Latitude)];
% 计算信关站在惯性系下的位置
R_XGZ_i = A_GI' * R_XGZ_84;
% 计算信关站在轨道系下的位置
Ai2o=angle2dcm(-pi/2,0,0,'XYZ')*angle2dcm(OMG,i,omg+f+pi/2,'ZXZ');
R_XGZ2Sat_o=Ai2o*(R_XGZ_i-R_Sat_i);
end

function [W_Sada_out, A_Sada_out,Agl_c_lim] = PI_new(Agl4Ctrl, W4Ctrl, Axis, SadaCtrlCoeff, t_Ctrl)
% 带角加速度限幅的PI
persistent W_Sada Ui;
if isempty(W_Sada)
    W_Sada = [0;0;0;0;0;0];
    Ui = [0;0;0;0;0;0];
end
Kp = SadaCtrlCoeff.Kp;
Ki = SadaCtrlCoeff.Ki;
Kd = SadaCtrlCoeff.Kd;
Ui_max = SadaCtrlCoeff.Ui_max/180*pi;
w_max = SadaCtrlCoeff.w_max/180*pi;
a_max = SadaCtrlCoeff.a_max/180*pi;
kw = 0.8;

if sum(Kp==0)==0
    W_lim = min(w_max(Axis),kw*sqrt(2*a_max(Axis)*abs(Agl4Ctrl(Axis))));
    Theta_lim = Kd(Axis)/Kp(Axis)*W_lim;
    Agl_c_lim = max(-Theta_lim,min(Agl4Ctrl(Axis),Theta_lim));
    Ui(Axis) = Ui(Axis) + Ki(Axis)*Agl_c_lim*t_Ctrl;
    Ui(Axis) = max(-Ui_max(Axis),min(Ui(Axis),Ui_max(Axis)));
    A_Sada = Kp(Axis)*Agl_c_lim + Kd(Axis)*W4Ctrl(Axis) + Ui(Axis);
    if a_max(Axis) ~= 0 
        A_Sada = max(-a_max(Axis),min(A_Sada,a_max(Axis)));
    end

    W_Sada(Axis) = W_Sada(Axis) + A_Sada * t_Ctrl;
    W_Sada(Axis) = max(-w_max(Axis),min(W_Sada(Axis),w_max(Axis)));

    W_Sada_out = W_Sada(Axis);
    A_Sada_out = A_Sada;
else
    W_Sada_out = 0;
    A_Sada_out = 0;
end
end

function y = Limit(u,u_min,u_max)
for i = 1:length(u)
    if u(i)<u_min(i)
        u(i) = u_min(i);
    elseif u(i)>u_max(i)
        u(i) = u_max(i);
    end
    y = u;
end
end

function y = Limit_pi(u)
y = mod(u,2*pi);
y = y - (y>=pi)*2*pi;
end
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

function Agl = RotZAgl(r1,r2)
% 计算XOY平面内向量r1绕+Z轴正转到r2的转角[0~2pi]
r1 = r1/norm(r1);
r2 = r2/norm(r2);
Cross1 = cross(r1,r2);
if Cross1(3)>=0
    Agl = acos(max(min(r1'*r2,1),-1));
else
    Agl = 2*pi-acos(max(min(r1'*r2,1),-1));
end
end

function Timer = Cnt(Timer,V)
if V > 0.5
    Timer = Timer + 1;
else
    Timer = 0;
end
end