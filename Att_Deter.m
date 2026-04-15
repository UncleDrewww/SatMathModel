function Att_Deter(Qib_St,wbi_Gyro,Mag_Mm,Qin,Qnnp,wni_n)

global  tao_St;

% 星敏姿态解算
StAttCal(Qin,Qib_St,Qnnp,tao_St);
% 陀螺姿态解算
GyroAttCal(wbi_Gyro,Qnnp,wni_n);

% 星敏+陀螺基准
StGyroRef(Qnnp);

% 姿态基准选择
AttRef;
% 磁场基准选择
MagRef(Mag_Mm);

%% 星敏姿态计算
function StAttCal(Qin,Qib_St,Qnnp,tao_St)
global T_ctrl St_Ok Agl_St AglRate_St StAttCal_First Qnb_St;
persistent Delta_Qnb_St_lb;
if isempty(Delta_Qnb_St_lb)
    Delta_Qnb_St_lb = [1;0;0;0];
end
if St_Ok == 1
    if StAttCal_First == 1
        Qnb_St = quatmultiply(quatinv(Qin'),Qib_St')';
        Qnb_St_last = Qnb_St;
    else
        Qnpb_St_last = Qnb_St;
        Qnb_St_last = quatmultiply(Qnnp',Qnpb_St_last')';
        Qnb_St = quatmultiply(quatinv(Qin'),Qib_St')';
    end

    Agl_St = [0;0;0];
    [Agl_St(3) Agl_St(2) Agl_St(1)] = quat2angle(Qnb_St','ZYX');
    
    Delta_Qnb = quatmultiply(quatinv(Qnb_St_last'),Qnb_St')';
    
    if StAttCal_First == 1
        Delta_Qnb_St_lb_last = Delta_Qnb;
    else
        Delta_Qnb_St_lb_last = Delta_Qnb_St_lb;
    end
    StAttCal_First = 0;
    
    if Delta_Qnb_St_lb_last(1)<0
        Delta_Qnb_St_lb_last = -Delta_Qnb_St_lb_last;
    end
    Delta_Qnb_St_lb = Filter(Delta_Qnb_St_lb_last,Delta_Qnb,tao_St,T_ctrl);
    Delta_Qnb_St_lb = quatnormalize(Delta_Qnb_St_lb')';
    if Delta_Qnb_St_lb(1)<0
        Delta_Qnb_St_lb = -Delta_Qnb_St_lb;
    end

    if norm(Delta_Qnb_St_lb(2:4,1)) <= 10^-6
        AglRate_St = [0;0;0];
    else
        AglRate_St = 2*acos(Delta_Qnb_St_lb(1,1))/T_ctrl*Delta_Qnb_St_lb(2:4,1)/norm(Delta_Qnb_St_lb(2:4,1));
        for i = 1:3
            if abs(AglRate_St(i,1))>2/180*pi
                AglRate_St(i,1) = sign(AglRate_St(i,1))*2/180*pi;
            end
        end
    end
else
    StAttCal_First = 1;
    Agl_St = [0;0;0];
    AglRate_St = [0;0;0];
end

%% 陀螺姿态计算
function GyroAttCal(wbi_Gyro,Qnnp,wni_n)
global T_ctrl Allow_AttRefToGyro Gyro_Ok Agl_Gyro AglRate_Gyro Qnb_Gyro Agl_Deter;
% Qnb_Gyro_last = Qnb_Gyro;
if Gyro_Ok == 1
    Qnpb_AttRef_last = angle2quat(Agl_Deter(3),Agl_Deter(2),Agl_Deter(1),'ZYX')';
    Qnb_AttRef_last = quatmultiply(Qnnp',Qnpb_AttRef_last')';
    AglRate_Gyro = wbi_Gyro - quat2dcm(Qnb_AttRef_last')*wni_n;
    if Allow_AttRefToGyro == 1
        Qnpb_Gyro_last = Qnpb_AttRef_last;
    else
        Qnpb_Gyro_last = Qnb_Gyro;
    end
    Qnb_Gyro_last = quatmultiply(Qnnp',Qnpb_Gyro_last')';
    Qnb_Gyro = zeros(4,1);
    Qnb_Gyro(1) = Qnb_Gyro_last(1) -T_ctrl/2 * (AglRate_Gyro(1)*Qnb_Gyro_last(2) + AglRate_Gyro(2)*Qnb_Gyro_last(3) + AglRate_Gyro(3)*Qnb_Gyro_last(4));
    Qnb_Gyro(2) = Qnb_Gyro_last(2) +T_ctrl/2 * (AglRate_Gyro(1)*Qnb_Gyro_last(1) + AglRate_Gyro(3)*Qnb_Gyro_last(3) - AglRate_Gyro(2)*Qnb_Gyro_last(4));
    Qnb_Gyro(3) = Qnb_Gyro_last(3) +T_ctrl/2 * (AglRate_Gyro(2)*Qnb_Gyro_last(1) - AglRate_Gyro(3)*Qnb_Gyro_last(2) + AglRate_Gyro(1)*Qnb_Gyro_last(4));
    Qnb_Gyro(4) = Qnb_Gyro_last(4) +T_ctrl/2 * (AglRate_Gyro(3)*Qnb_Gyro_last(1) + AglRate_Gyro(2)*Qnb_Gyro_last(2) - AglRate_Gyro(1)*Qnb_Gyro_last(3));
    Qnb_Gyro = quatnormalize(Qnb_Gyro')';
    Agl_Gyro = zeros(3,1);
    [Agl_Gyro(3) Agl_Gyro(2) Agl_Gyro(1)] = quat2angle(Qnb_Gyro','ZYX');
%     disp([wbi_Gyro' Qnnp' wni_n' Qnb_Gyro_last' Qnb_Gyro' Agl_Gyro']);
end


%% 磁基准
function MagRef(Mag_Mm)
global MagRefFlag Mm_Ok MagRefChoice;
global MagRef_Ok Mag_sys;
if MagRefFlag == 0011  % 强选磁强计
    if Mm_Ok == 1
        MagRef_Ok = 1;
        Mag_sys = Mag_Mm;
        MagRefChoice = 1;
    else
        MagRef_Ok = 0;
        Mag_sys = [0;0;0];
        MagRefChoice = 0;
    end
else
    MagRef_Ok = 0;
    Mag_sys = [0;0;0];
    MagRefChoice = 0;
end

%% 姿态基准
function AttRef
global AttRefFlag StGyroRef_Ok Agl_StGyroRef AglRate_StGyroRef;
global AttRef_Ok AttRefChoice Agl_Deter W_Deter;
if AttRefFlag == 5511      % 强选星敏+陀螺基准
    if StGyroRef_Ok == 1
        AttRef_Ok = 1;
        AttRefChoice = 1;
    else
        AttRef_Ok = 0;
        AttRefChoice = 0;
    end

else
    AttRef_Ok = 0;
    AttRefChoice = 0;
end

if AttRefChoice == 1
    Agl_Deter = Agl_StGyroRef;
    W_Deter = AglRate_StGyroRef;
else
    Agl_Deter = [0;0;0];
    W_Deter = [0;0;0];
end
% disp(Agl_Deter');

%% 星敏+陀螺基准
function StGyroRef(Qnnp)
global T_ctrl Allow_StUse St_Ok Agl_St Gyro_Ok AglRate_Gyro Agl_StGyroRef AglRate_StGyroRef StGyroRef_Ok;
persistent StGyroRefTimer1;
if isempty(StGyroRefTimer1)
    StGyroRefTimer1 = 0;
end
if (Allow_StUse == 1)&&(Gyro_Ok == 1)            % 星敏允许接入闭环且陀螺有效
    if St_Ok == 1
        StGyroRef_Ok = 1;
        Agl_StGyroRef = Agl_St;
        AglRate_StGyroRef = AglRate_Gyro;
        StGyroRefTimer1 = 0;
    elseif StGyroRefTimer1 <= 600*2              % 星敏有故障，陀螺无故障,10min内基准为陀螺四元数积分       
        Qnpb_StGyroRef_last = angle2quat(Agl_StGyroRef(3),Agl_StGyroRef(2),Agl_StGyroRef(1),'ZYX');
        Qnb_StGyroRef_last = quatmultiply(Qnnp',Qnpb_StGyroRef_last);
        Qnb_StGyroRef = zeros(4,1);
        Qnb_StGyroRef(1) = Qnb_StGyroRef_last(1) -T_ctrl/2 * (AglRate_Gyro(1)*Qnb_StGyroRef_last(2) + AglRate_Gyro(2)*Qnb_StGyroRef_last(3) + AglRate_Gyro(3)*Qnb_StGyroRef_last(4));
        Qnb_StGyroRef(2) = Qnb_StGyroRef_last(2) +T_ctrl/2 * (AglRate_Gyro(1)*Qnb_StGyroRef_last(1) + AglRate_Gyro(3)*Qnb_StGyroRef_last(3) - AglRate_Gyro(2)*Qnb_StGyroRef_last(4));
        Qnb_StGyroRef(3) = Qnb_StGyroRef_last(3) +T_ctrl/2 * (AglRate_Gyro(2)*Qnb_StGyroRef_last(1) - AglRate_Gyro(3)*Qnb_StGyroRef_last(2) + AglRate_Gyro(1)*Qnb_StGyroRef_last(4));
        Qnb_StGyroRef(4) = Qnb_StGyroRef_last(4) +T_ctrl/2 * (AglRate_Gyro(3)*Qnb_StGyroRef_last(1) + AglRate_Gyro(2)*Qnb_StGyroRef_last(2) - AglRate_Gyro(1)*Qnb_StGyroRef_last(3));
        Qnb_StGyroRef = quatnormalize(Qnb_StGyroRef')';
        Agl_StGyroRef = zeros(3,1);
        [Agl_StGyroRef(3) Agl_StGyroRef(2) Agl_StGyroRef(1)] = quat2angle(Qnb_StGyroRef','ZYX');

        AglRate_StGyroRef = AglRate_Gyro;
        StGyroRef_Ok = 1;
        StGyroRefTimer1 = StGyroRefTimer1 + 1;
    else
        Agl_StGyroRef = [0;0;0];
        AglRate_StGyroRef = [0;0;0];
        StGyroRef_Ok = 0;
    end
else                                             % 星敏故障超过10min，或陀螺故障，赋0
    Agl_StGyroRef = [0;0;0];
    AglRate_StGyroRef = [0;0;0];
    StGyroRef_Ok = 0;
    StGyroRefTimer1 = 0;
end

%% 滤波子程序
function y = Filter(y_last,u,tao,T_ctrl)
y = (T_ctrl*u+tao*y_last)/(T_ctrl+tao);
