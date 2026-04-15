function [SadaEquipmentCmd, SadaParam,Agl_Sada_Tgt] = ZZ_Sada(EnvirParam,NominalCSYS,SadaParam,OrbitElement,RunCount,t_ctrl)
global Tc_SadaCtrl_Enable LcMark SadaCtrlCoeff;
persistent  Agl_Sada_Ctrl;
if isempty(Agl_Sada_Ctrl)
    Agl_Sada_Ctrl = [0;0;0;0];
end

% AttDeter
Agl_Sada = [SadaParam.Sada1.Agl;SadaParam.Sada2.Agl;SadaParam.Sada3.Agl]; % 【a p】rad

% 用户自定义函数，包含 NominalCSYS 和 ModeDeterAndInit
[SADA_Mode, SadaCtrlMethod, Agl_Sada_Tgt] = ZZ_User_Sada(EnvirParam,NominalCSYS,SadaParam,Agl_Sada_Ctrl,OrbitElement,t_ctrl);
% SADA_Mode SysApp的模式：0不控、1待机、2停转、3闭环+角度、4开环+角速度、5增量+增量角、6归零+初始角？

% Att4Ctrl
[Agl_Sada_Ctrl, W_Sada_Ctrl] = SADA_Att4Ctrl(Agl_Sada_Tgt,Agl_Sada,SadaParam,SadaCtrlCoeff,t_ctrl);
SadaParam.Sada1.Agl4Ctrl=Agl_Sada_Ctrl(1:2);
SadaParam.Sada2.Agl4Ctrl=Agl_Sada_Ctrl(3:4);
% AttCtrl
[SadaEquipmentCmd, Wc, Ac] = SADA_AttCtrl(SADA_Mode,Agl_Sada_Ctrl,W_Sada_Ctrl,Agl_Sada,SadaParam,SadaCtrlMethod,SadaCtrlCoeff,RunCount,t_ctrl);

SadaParam.Sada1.Wc = Wc(1:2);% SADA系的角速度a,p  rad/s
SadaParam.Sada1.Ac = Ac(1:2);
SadaParam.Sada2.Wc = Wc(3:4);
SadaParam.Sada2.Ac = Ac(3:4);
SadaParam.Sada3.Wc = Wc(5:6);
SadaParam.Sada3.Ac = Ac(5:6);

end


function [Agl_Sada_Ctrl, W_Sada_Ctrl] = SADA_Att4Ctrl(Agl_Sada_Tgt,Agl_Sada_Cur,SadaParam,SadaCtrlCoeff,t_ctrl)
persistent W_Sada_Tgt_last Agl_Sada_Tgt_last;
if isempty(W_Sada_Tgt_last)
    W_Sada_Tgt_last = [0;0;0;0;0;0];
end
if isempty(Agl_Sada_Tgt_last)
    Agl_Sada_Tgt_last = [0;0;0;0;0;0];
end

RotateType = SadaCtrlCoeff.RotType;
LLmt = SadaCtrlCoeff.WorkArea(:,1)/180*pi;  % 工作区左边界
RLmt = SadaCtrlCoeff.WorkArea(:,2)/180*pi;  % 工作区右边界

Agl_Nogo = mod(LLmt-RLmt,2*pi);%禁区宽度
Mid_NoGo = Limit_pi(Agl_Nogo/2+RLmt);% 禁区中线

R_Cur = [cos(Agl_Sada_Cur');sin(Agl_Sada_Cur');[0 0 0 0 0 0]];
R_Tgt = [cos(Agl_Sada_Tgt');sin(Agl_Sada_Tgt');[0 0 0 0 0 0]];
R_Mid = [cos(Mid_NoGo');sin(Mid_NoGo');[0 0 0 0 0 0]];

Agl_Sada_Ctrl = Agl_Sada_Tgt - Agl_Sada_Cur; % rad
Agl_Sada_Ctrl = Limit_pi(Agl_Sada_Ctrl);

for i = 1:6
    if strcmp(RotateType(i),'Around') % 绕行
        % 根据Agl_Sada_Ctrl符号可判断最短路径是想正转还是负转
        if Agl_Sada_Ctrl(i) >0 %正转
            if RotZAgl(R_Cur(:,i),R_Mid(:,i))<RotZAgl(R_Cur(:,i),R_Tgt(:,i)) % C往T去经过M
                Agl_Sada_Ctrl(i) = (abs(Agl_Sada_Ctrl(i))-2*pi)*sign(Agl_Sada_Ctrl(i));
            end
        else
            if RotZAgl(R_Cur(:,i),R_Mid(:,i))>RotZAgl(R_Cur(:,i),R_Tgt(:,i)) % C往T去经过M
                Agl_Sada_Ctrl(i) = (abs(Agl_Sada_Ctrl(i))-2*pi)*sign(Agl_Sada_Ctrl(i));
            end
        end
    else % Stop不绕行

    end
end

W_Sada_Tgt = (Agl_Sada_Tgt-Agl_Sada_Tgt_last)/t_ctrl;
W_Sada_Tgt = W_Sada_Tgt .* (abs(W_Sada_Tgt)<SadaCtrlCoeff.w_max) + W_Sada_Tgt_last.* (abs(W_Sada_Tgt)>SadaCtrlCoeff.w_max);
W_Sada_Ctrl = W_Sada_Tgt -[SadaParam.Sada1.Wc;SadaParam.Sada2.Wc;SadaParam.Sada3.Wc];

% % 系统app中用下面的方式计算W_Sada_Ctrl
% Delta_Agl_Sada_Ctrl = Agl_Sada_Ctrl-Agl_Sada_Ctrl_last;
% W_Sada_Ctrl = Delta_Agl_Sada_Ctrl/t_ctrl;
% W_Sada_Ctrl = W_Sada_Ctrl .* (abs(W_Sada_Ctrl)<SadaCtrlCoeff.w_max);

W_Sada_Tgt_last = W_Sada_Tgt;
Agl_Sada_Tgt_last = Agl_Sada_Tgt;
end

function [SadaEquipmentCmd_out, Wc, Ac] = SADA_AttCtrl(SadaSysMode,Agl4Ctrl,W4Ctrl,Agl_Sada_Cur,SadaParam,SadaCtrlMethod,SadaCtrlCoeff,RunCount,t_ctrl)
% SadaSysMode      :0不控、1待机、2停转、3闭环+目标角、4开环+角速度、5增量+增量角、6归零+初始角？
% SadaEquipmentCmd :1待机、2停转、3开环+角速度、4归零+方向
% 用于判断是否在禁区
RotateType = SadaCtrlCoeff.RotType;
LLmt = SadaCtrlCoeff.WorkArea(:,1)/180*pi;  % 工作区左边界
RLmt = SadaCtrlCoeff.WorkArea(:,2)/180*pi;  % 工作区右边界
Agl_Nogo = mod(LLmt-RLmt,2*pi);%禁区宽度
R_RLmt = [cos(RLmt');sin(RLmt');[0 0 0 0 0 0]];
R_Cur = [cos(Agl_Sada_Cur');sin(Agl_Sada_Cur');[0 0 0 0 0 0]];

SadaEquipmentCmd = zeros(6,2);
Wc = zeros(6,1);
Ac = zeros(6,1);

for i = 1:6 % 4个电机
    if SadaSysMode(i,1) == 0                          % 系统不控
        SadaEquipmentCmd(i,:) = [1 0];                                       % 驱动器待机
    elseif SadaSysMode(i,1) == 1                      % 系统待机
        SadaEquipmentCmd(i,:) = [1 0];                                       % 驱动器待机
    elseif SadaSysMode(i,1) == 2                      % 系统停转保持
        SadaEquipmentCmd(i,:) = [2 0];                                       % 驱动器保持
    elseif SadaSysMode(i,1) == 3                      % 系统闭环模式
        if SadaCtrlMethod(i,1) == 0 % 0-PI,1-滞环
            [Wc(i,1), Ac(i,1)] = PI_new(Agl4Ctrl,W4Ctrl,i,SadaCtrlCoeff,t_ctrl); % 单位：rad/s、rad/s^2
        else
            [Wc(i,1), Ac(i,1)] = SADA_ZH_Controller(Agl4Ctrl,i,SadaCtrlCoeff);
        end
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
        SadaEquipmentCmd(i,:) = [3 Wc(i,1)];                                 % 驱动器开环
    elseif SadaSysMode(i,1) == 4                      % 系统开环模式
        SadaEquipmentCmd(i,:) = [3 SadaSysMode(i,2)];                        % 驱动器开环
    elseif SadaSysMode(i,1) == 5                      % 系统增量模式
        SadaEquipmentCmd(i,:) = [3 sign(SadaSysMode(i,2))*SadaCtrlCoeff.w_max_ZL(i)/180*pi];   % 驱动器开环
    elseif SadaSysMode(i,1) == 6                      % 系统归零模式
        SadaEquipmentCmd(i,:) = [4 sign(SadaSysMode(i,2))];                  % 驱动器归零
    end
end
SadaEquipmentCmd_out = [SadaEquipmentCmd(1,:)';SadaEquipmentCmd(2,:)';SadaEquipmentCmd(3,:)';SadaEquipmentCmd(4,:)';SadaEquipmentCmd(5,:)';SadaEquipmentCmd(6,:)'];
end
%%
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
    W_Sada_last = W_Sada(Axis);
    W_Sada(Axis) = W_Sada(Axis) + A_Sada * t_Ctrl;
    W_Sada(Axis) = max(-w_max(Axis),min(W_Sada(Axis),w_max(Axis)));

    W_Sada_out = W_Sada(Axis);
    A_Sada_out = (W_Sada(Axis) - W_Sada_last)/t_Ctrl;
else
    W_Sada_out = 0;
    A_Sada_out = 0;
end
end
%%
function [W_Sada_out, A_Sada_out] = PI(Agl4Ctrl,W4Ctrl,Axis,SadaCtrlCoeff,t_ctrl)
% 输出：rad/s
Kp = SadaCtrlCoeff.Kp;
Ki = SadaCtrlCoeff.Ki;
Kd = SadaCtrlCoeff.Kd;
Ui_max = SadaCtrlCoeff.Ui_max/180*pi;
w_max = SadaCtrlCoeff.w_max/180*pi;
a_max = SadaCtrlCoeff.a_max/180*pi;
persistent Ui W_Sada A_Sada;
if isempty(Ui)
    Ui = [0;0;0;0];
end
if isempty(W_Sada)
    W_Sada = [0;0;0;0];
end
if isempty(A_Sada)
    A_Sada = [0;0;0;0];
end
W_last = W_Sada;
Ui(Axis) = Ui(Axis) + Ki(Axis) * Agl4Ctrl(Axis) * t_ctrl;
Ui(Axis) = Limit_max1(Ui(Axis),Ui_max(Axis));
W(Axis) = Kp(Axis)*Agl4Ctrl(Axis)+Ui(Axis)+Kd(Axis)*W4Ctrl(Axis);
W(Axis) = Limit_max1(W(Axis),w_max(Axis));

Dw = W(Axis) - W_last(Axis);
Dw = Limit_max1(Dw,a_max(Axis)*t_ctrl);
W_Sada(Axis) = Dw + W_last(Axis);
A_Sada(Axis) = Dw/t_ctrl;
W_Sada_out = W_Sada(Axis);
A_Sada_out = A_Sada(Axis);
% A_Sada = [0;0;0;0];
end

function [Wc, Ac] = SADA_ZH_Controller(Agl4Ctrl,Axis,SadaCtrlCoeff)
persistent y_last;
if isempty(y_last)
    y_last = [0;0;0;0;0;0];
end
w_center = SadaCtrlCoeff.ZH_W0(Axis)/180*pi;
ZH_dW = SadaCtrlCoeff.ZH_dW(Axis)/180*pi; % 0.002°/s
ZH_Wmax = SadaCtrlCoeff.ZH_Wmax(Axis)/180*pi;
agl1 = SadaCtrlCoeff.ZH_L2(Axis)/180*pi;% 内边界
agl2 = SadaCtrlCoeff.ZH_L1(Axis)/180*pi;% 外边界

if Agl4Ctrl(Axis) > agl2
    Wc = ZH_Wmax;
elseif Agl4Ctrl(Axis)<=agl2 && Agl4Ctrl(Axis)>agl1
    Wc = w_center+ZH_dW;
elseif Agl4Ctrl(Axis)<=-agl1 && Agl4Ctrl(Axis)>-agl2
    Wc = w_center-ZH_dW;
elseif Agl4Ctrl(Axis)<=-agl2
    Wc = -ZH_Wmax;
else
    Wc = y_last(Axis);
end
Ac = 0;
y_last(Axis) = Wc;
end

function A = Rx(alpha)
A = [1 0 0;0 cos(alpha) sin(alpha);0 -sin(alpha) cos(alpha)];
end

function y = Limit_max1(u,Ymax)
Flag = abs(u)>Ymax;%超出阈值的量
y = (1-Flag).*u+sign(u).*Flag*Ymax;
end

function y = Limit_pi(u)
y = mod(u,2*pi);
y = y - (y>=pi)*2*pi;
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