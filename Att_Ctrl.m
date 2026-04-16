function Att_Ctrl(FwSpd,T_sys,Qon)
% %#codegen
global AttRef_Ok Agl_Deter W_Deter FwCtrlFlag W_Target Att4CtrlRefFlag MagRef_Ok MagCtrlFlag Mag_sys;
global T_ctrl Matrix_I FwTmax FwHmax FwMode FwMode_last Mag_max jw Mode;
global Agl4Ctrl W4Ctrl AA_bTo T_PID T_Fw T_Fwz H_Fw MagCmd T_Mag CtrlParam Hw0;
global RapidManeuver;
global w_Tgt theta_Tgt a_f_Tgt;
persistent  H_Ilast Path;
if isempty(H_Ilast)
    H_Ilast = [0;0;0;0];
end
if isempty(AA_bTo)
    AA_bTo = [0;0;0];
end
% Angle_p=[00;70;0;00;-70;0]/180*pi;
[I, MainI, XieI, CtrlParam] = CtrlParamCal;
% disp([MainI XieI])
% 控制用姿态角计算
if AttRef_Ok == 1
    Qnb = angle2quat(Agl_Deter(3),Agl_Deter(2),Agl_Deter(1),'ZYX')';
    for i =1:3
        if Att4CtrlRefFlag(i) == 1    %四元数控制
            Agl4Ctrl(i) = 2*sign(Qnb(1))*Qnb(i+1);
            W4Ctrl(i) = W_Deter(i);
        elseif Att4CtrlRefFlag(i) == 3   %角速度控制
            Agl4Ctrl(i) = 0;
            W4Ctrl(i) = W_Deter(i)-W_Target(i);
        else
            Agl4Ctrl(i) = 0;
            W4Ctrl(i) = 0;
        end
    end
else
    Agl4Ctrl = [0;0;0]; 
    W4Ctrl =[0;0;0];
end
T_Mag = MagCtrlAlgorithm(MagRef_Ok,MagCtrlFlag,Agl4Ctrl,W4Ctrl,FwSpd/30*pi*jw);
if Mode == 0

else
    if sum(FwMode) >= 3
        AglRate_lim = 1/180*pi;
    else
        AglRate_lim = 0.05/180*pi;
    end
    T_PID_last = T_PID;
    if (FwCtrlFlag(1) == 11)&&(FwCtrlFlag(2) == 11) && (FwCtrlFlag(3) == 11)%快速机动算法
        a0max_w = 0.03;
        wmax_w = 0.5;
        if isempty(Path)
            T0 = T_sys;
            qob = Qnb;
            qt = [cosd(5),sind(5)/sqrt(3)*[1,1,1]]';
            Path = generatePath(T0,qob',qt,a0max_w,wmax_w);
        end
        Ts = T_sys;
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
            thetat = a0max_w*tr^2/2+a0max_w*tr*(Ts-T0-tr);
        elseif Ts < T0+2*tr+ts
            k = (T0 + 2*tr + ts - Ts -0.5*tr)/tr*pi;
            wt = a0max_w*tr * (0.5*sin(k)+0.5);
            thetat = a0max_w*tr*0.5*(Ts-T0+ts+cos(k)*tr/pi);
        else
            wt = 0;
            thetat = a0max_w*tr*(tr+ts);   
        end
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
        QobT = quatmultiply(q0,[cos(theta),sin(theta)*e;]);
        WbTo_bT = wt*e'/norm(e)/180*pi;
        AbTo_bT = at*e'/norm(e)/180*pi;
        Qnb = angle2quat(Agl_Deter(3),Agl_Deter(2),Agl_Deter(1),'ZYX')';
        QbTb = quatmultiply(quatinv(QobT),quatmultiply(Qon',Qnb'))';
        Agl4Ctrl = 2*sign(QbTb(1))*QbTb(2:4);
        W4Ctrl = W_Deter - quat2dcm(QbTb')*WbTo_bT;
        PDParam = 1;
        T_PID(1) = FwCtrlAlgorithm(3,Agl4Ctrl,W4Ctrl,T_PID_last(1),AglRate_lim,1,PDParam);
        T_PID(2) = FwCtrlAlgorithm(3,Agl4Ctrl,W4Ctrl,T_PID_last(2),AglRate_lim,2,PDParam);
        T_PID(3) = FwCtrlAlgorithm(3,Agl4Ctrl,W4Ctrl,T_PID_last(3),AglRate_lim,3,PDParam);
        [~, MainI, ~, ~] = CtrlParamCal;
        T_PID1 = T_PID - MainI.*quat2dcm(QbTb')*AbTo_bT;
    else
        PDParam = 1;
        T_PID(1) = FwCtrlAlgorithm(FwCtrlFlag(1),Agl4Ctrl,W4Ctrl,T_PID_last(1),AglRate_lim,1,PDParam);
        T_PID(2) = FwCtrlAlgorithm(FwCtrlFlag(2),Agl4Ctrl,W4Ctrl,T_PID_last(2),AglRate_lim,2,PDParam);
        T_PID(3) = FwCtrlAlgorithm(FwCtrlFlag(3),Agl4Ctrl,W4Ctrl,T_PID_last(3),AglRate_lim,3,PDParam);
        T_PID1 = T_PID-cross((W_Deter+[0.0635;0;0]/180*pi),Matrix_I*H_Fw)*0;
    end
    if (FwCtrlFlag(1)==0)&&(FwCtrlFlag(2)==0)&&(FwCtrlFlag(3)==0)
        T_Fw = [0;0;0;0];
        H_Fw = [0;0;0;0];
    else
        if (FwMode(1) ~= FwMode_last(1))||(FwMode(2) ~= FwMode_last(2))||(FwMode(3) ~= FwMode_last(3))||(FwMode(4) ~= FwMode_last(4))%||((Mode_last == 8)&&(SubMode_last==2))
            H_Fw_last = FwSpd/30*pi*jw;
            [~,col] = find(FwMode==0);
            H_Fw_last(col) = 0;
        else
%             H_Fw_last = H_Fw;
            H_Fw_last = FwSpd/30*pi*jw;
        end
        if (sum(FwMode)==4)%||(sum(FwMode)==3)
            Matrix_D = Matrix_I'/(Matrix_I*Matrix_I');
            T_Fw = Matrix_D * T_PID1;
            T_Fwc = limit4(T_Fw,0.8*FwTmax);
            DeltaH_Fwc = T_ctrl * T_Fwc;
            h_Fwz = 0.1*(Hw0.*FwMode-H_Fw_last);        %0.5为比例系数，正常为1
            DeltaH_Fwz1 = (eye(4)-Matrix_D*Matrix_I) * h_Fwz;
            DeltaH_Fwz =  limit4(DeltaH_Fwz1,0.2*FwTmax*T_ctrl);
            T_Fwz = DeltaH_Fwz/T_ctrl;
            T_Fw = T_Fwc + T_Fwz;
            T_Fw = limit4(T_Fw,1*FwTmax);
            DeltaH_Fw = DeltaH_Fwc +1*DeltaH_Fwz;
            H_Fw = H_Fw_last + DeltaH_Fw;
            H_Fw =  limit4(H_Fw,FwHmax);
        elseif sum(FwMode)==3
            Matrix_D = Matrix_I'/(Matrix_I*Matrix_I');
            T_Fw = Matrix_D * T_PID1;
            T_Fw = limit4(T_Fw,FwTmax);
            DeltaH_Fw = T_ctrl * T_Fw;
            H_Fw = H_Fw_last + DeltaH_Fw;
            H_Fw =  limit4(H_Fw,FwHmax);
        elseif sum(FwMode)==2
            Nonezero = find(FwMode)';
            T_Mag_Vector = cross(Matrix_I(:,Nonezero(1)),Matrix_I(:,Nonezero(2)));
            Matrix_Inew = [Matrix_I T_Mag_Vector];
            Matrix_Dnew = Matrix_Inew'/(Matrix_Inew*Matrix_Inew');
%             disp(Matrix_Dnew*[0.025367;-0.025659;0])
            T5 = Matrix_Dnew * T_PID1;
            Norm = norm(Mag_sys);
            if Norm ~= 0
                MagCmd1 = -cross(Mag_sys,T5(5,1)*T_Mag_Vector) / Norm^2*1e4;
            else
                MagCmd1 = [0;0;0];
            end
            MagCmd1 = limit2(MagCmd1,Mag_max);  % 最大值限幅
            MagCmd = MagCmd + MagCmd1;
            MagCmd = limit2(MagCmd,Mag_max);
            T_PID_tmp = T_PID1 + cross(MagCmd1,Mag_sys*1e-4);
            T5_tmp = Matrix_Dnew * T_PID_tmp;
            T_Fw = T5_tmp(1:4,1);
            T_Fw = limit4(T_Fw,FwTmax);
            DeltaH_Fw = T_ctrl * T_Fw;
            H_Fw = H_Fw_last + DeltaH_Fw;
            H_Fw =  limit4(H_Fw,FwHmax);
        else
            MagCmd = [0;0;0];
            T_Fw = [0;0;0;0];
            H_Fw = [0;0;0;0];
        end
    end
end

%% 磁控算法选择
function T_Mag = MagCtrlAlgorithm(MagRef_Ok,MagCtrlFlag,Agl4Ctrl,W4Ctrl,Hw)

persistent Mag10_First MagCtrlTimer Mag_sys_last;
if isempty(Mag10_First)
    Mag10_First = 1;
end
if isempty(MagCtrlTimer)
    MagCtrlTimer = 0;
end
if isempty(Mag_sys_last)
    Mag_sys_last=[0;0;0];
end
global K_MagUnload K_MagDamp tao_Mag T_ctrl Matrix_I0 Mag_max Mag_sys  MagCtrl_First dBb_TD FwMode Matrix_I MagCmd;
global dBb Hw0;
if MagRef_Ok == 1
    if MagCtrl_First == 1
        Mag_sys_last = Mag_sys;
    end
    MagCtrl_First = 0;
    dBb_last = dBb;
    dBb = ((Mag_sys - Mag_sys_last) + tao_Mag * dBb_last)/(T_ctrl+tao_Mag);
    T_Mag = [0;0;0];
    if MagCtrlFlag == 0
        MagCmd = [0;0;0];
    elseif MagCtrlFlag == 1   % 角速度负反馈法
        MagCmd = -cross(Mag_sys,K_MagDamp*W4Ctrl);
    elseif MagCtrlFlag == 2   % Bdot控制算法
        MagCmd_tmp = -K_MagDamp * dBb;
        if MagCtrlTimer >= 10
            dBb_TD = dBb;
            MagCmd = MagCmd_tmp;
            MagCtrlTimer = 1;
        else
            MagCtrlTimer = MagCtrlTimer + 1;
        end
    elseif MagCtrlFlag == 3   % 磁卸载算法
        H = Matrix_I0 * Hw;
        Norm1 = norm(Mag_sys);
        if Norm1 ~= 0
            MagCmd = -K_MagUnload .* cross(Mag_sys,(H-Matrix_I*(Hw0.*FwMode))) / Norm1^2;
        else
            MagCmd = [0;0;0];
        end

    elseif MagCtrlFlag ==4    % 三轴磁控稳定算法
        Vomg = -W4Ctrl'*(vx(Mag_sys))'*vx(Mag_sys)*(K_MagDamp*W4Ctrl);
        Vksi = -W4Ctrl'*(vx(Mag_sys))'*vx(Mag_sys)*(K_MagDamp/1000*(Agl4Ctrl));
        if abs(Vomg)<Vksi
            k = abs(Vomg)/2/Vksi;
        else
            k = 1;
        end
        MagCmd = -K_MagDamp*cross(Mag_sys,W4Ctrl)-1*k*K_MagDamp/1000*cross(Mag_sys,(Agl4Ctrl));
    else
        error('MagCtrlAlgorithmError!');
    end
    MagCmd = limit1(MagCmd,Mag_max);
    Mag_sys_last = Mag_sys;
else
    MagCtrl_First = 1;
    MagCmd = [0;0;0];
    T_Mag = [0;0;0];
    dBb = [0;0;0];
end

%% 轮控算法(单轴)
function T = FwCtrlAlgorithm(FwCtrlFlag,Agl4Ctrl,W4Ctrl,T_last,AglRate_lim,Axis,PDParam)
global T_ctrl CtrlParam;
Kp1 = CtrlParam(:,1);
Kd1 = CtrlParam(:,2);
Kt1 = CtrlParam(:,3);
Kd2 = CtrlParam(:,4);
Kt2 = CtrlParam(:,5);
if PDParam == 2
    Kp3 = CtrlParam(:,8);
    Kd3 = CtrlParam(:,9);
    Ki3 = Kp3/100*0;
else
    Kp3 = CtrlParam(:,6);
    Kd3 = CtrlParam(:,7);
    Ki3 = Kp3/100*0;
end
persistent Iner_last Iner_last1;
if isempty(Iner_last)
    Iner_last = [0;0;0];
end
if isempty(Iner_last1)
    Iner_last1 = [0;0;0];
end

if FwCtrlFlag == 0        % 不控模式
    T = 0;
elseif FwCtrlFlag == 1    % PIDT
    InerMax=0.1/57.3*Kd1;
    Agl_limit = AglRate_lim* Kd1(1)/Kp1(1);
    Agl4Ctrl_Lim = limit3(Agl4Ctrl,Agl_limit);
%     Iner(Axis) = Iner_last1(Axis) + 1*Kp1(Axis)/100*Agl4Ctrl_Lim(Axis)/T_ctrl;
    Iner(Axis) = Iner_last1(Axis) + 1*Kp1(Axis)/100*Agl4Ctrl_Lim(Axis)*T_ctrl;
    if Iner(Axis)> InerMax(Axis)
        Iner(Axis) = InerMax(Axis);
    elseif Iner(Axis) < -InerMax(Axis)
        Iner(Axis) = -InerMax(Axis);
    end
    T = (Kd1(Axis) * W4Ctrl(Axis) * T_ctrl + Kp1(Axis) * T_ctrl * Agl4Ctrl_Lim(Axis) +Iner(Axis) * T_ctrl + T_last)/(1+ T_ctrl * Kt1(Axis));
elseif FwCtrlFlag == 2    % DT
    T = (T_ctrl * Kd2(Axis) * W4Ctrl(Axis) + Kt2(Axis) * T_last)/(T_ctrl+Kt2(Axis));    
elseif FwCtrlFlag == 3    % PD
    Angle_limit = AglRate_lim*Kd3(1)/Kp3(1);
    Agl4Ctrl_Lim = limit3(Agl4Ctrl,Angle_limit);
    T = Kp3(Axis)*Agl4Ctrl_Lim(Axis) + Kd3(Axis)*W4Ctrl(Axis);
elseif FwCtrlFlag == 4    % PID
    InerMax=[0.1;0.1;0.1]/57.3.*Kd3;
    Angle_limit = min(AglRate_lim,0.8*sqrt(2*0.05/280*max(abs(Agl4Ctrl))))*Kd3(1)/Kp3(1);%  LC
    Agl4Ctrl_Lim = limit3(Agl4Ctrl,Angle_limit);
%     disp([Agl4Ctrl_Lim']); 
    Iner(Axis) = Iner_last(Axis) + Ki3(Axis)*Agl4Ctrl_Lim(Axis)*T_ctrl;
    if Iner(Axis)> InerMax(Axis)
        Iner(Axis) = InerMax(Axis);
    elseif Iner(Axis) < -InerMax(Axis)
        Iner(Axis) = -InerMax(Axis);
    end
    T = Iner(Axis) + Kp3(Axis)*Agl4Ctrl_Lim(Axis) + Kd3(Axis)*W4Ctrl(Axis);
    Iner_last(Axis) = Iner(Axis);
%     disp(Iner(Axis));
else
    error('FwCtrlAlgorithmError');
end


function [I, MainI, XieI, CtrlParam] = CtrlParamCal
Ibxx=101;Ibyy=109;Ibzz=108;
Ibxy=-3.03;Ibxz= 11.24;Ibyz=15.02;
I=[Ibxx Ibxy Ibxz;Ibxy Ibyy Ibyz;Ibxz Ibyz Ibzz]; % 星体相对其自身质心的转动惯量
MainI=[I(1,1);I(2,2);I(3,3)];
XieI=[I(1,2);I(1,3);I(2,3)];
lamda = 0.3;ksi=0.8;%sqrt(2)/2;
Kd1 = 3*MainI*lamda^2;     %  算法1：PIDT参数
Kp1 = MainI*lamda^3;
Kt1 = 3*lamda*[1;1;1];
Kd2 = MainI*lamda/2/ksi;   %  算法2：DT参数
% disp(1)
Kt2 = 1/2/lamda/ksi*[1;1;1];
lamda_1=0.3;lamda_2=0.3;
Kp3_1 = MainI.*lamda_1.^2;       %  算法3：PD参数，第一组参数1带宽0.2
Kd3_1 = 2*MainI.*lamda_1*ksi;
% MainI = [64.37 39.21 76.00];
Kp3_2 = MainI.*lamda_2.^2;       %  算法3：PD参数，第二组参数2带宽0.25
Kd3_2 = 2*MainI.*lamda_2*ksi;
% disp([Kp3_2';Kd3_2'])
CtrlParam=[Kp1 Kd1 Kt1 Kd2 Kt2 Kp3_1 Kd3_1 Kp3_2 Kd3_2];
% disp(CtrlParam)
%% 反对称阵
function y = vx(x)
y=[0 -x(3) x(2);...
    x(3) 0 -x(1);...
    -x(2) x(1) 0];

function Z = limit1(X,Y)
% 三轴最大值限幅
Z = zeros(3,1);
for i =1:3
    if X(i,1) >= Y
        Z(i,1) = Y;
    elseif X(i,1)<= -Y
        Z(i,1) = -Y;
    else
        Z(i,1) = X(i,1);
    end
end

function Z = limit2(X,Y)
% 三维等比例限幅（限幅值不同）
if abs(X(1))<=Y(1) && abs(X(2))<=Y(2) && abs(X(3))<=Y(3)
    Z=X;
else
    if Y(1)*Y(2)*Y(3)==0
        Z = [0;0;0];
    else
        X1 = max(abs(X(1))/Y(1),abs(X(2))/Y(2)); 
        X2 =max(X1,abs(X(3))/Y(3));
        Z = X/X2;
    end
end

function Z = limit3(X,Y)
% 三维等比例限幅
if abs(X(1))<=Y && abs(X(2))<=Y&& abs(X(3))<=Y
    Z=X;
else
    X1 = max(abs(X(1)),abs(X(2)));
    X2 =max(X1,abs(X(3)));
    Z = X/X2*Y;
end

function Z = limit4(X,Y)
% 四维等比例限幅
if abs(X(1))<=Y && abs(X(2))<=Y && abs(X(3))<=Y && abs(X(4))<=Y
    Z=X;
else
    X1 = max(abs(X(1)),abs(X(2)));
    X2 =max(X1,abs(X(3)));
    X3 =max(X2,abs(X(4)));
    Z = X/X3*Y;
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