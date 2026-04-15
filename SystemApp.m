function y = SystemApp(u)
Qib_StA = u(1:4);
Qib_StB = u(5:8);
wgi_GyroA = u(9:11);
Qio = u(12:15);
Mag_MmA = u(16:18);
Mag_MmB = u(19:21);
T_sys = u(22);
FwSpd = u(23:26);
w0 = u(27);

% disp(1)
global RunTimer Mode SubMode;
RunTimer = RunTimer + 1;
% 单机修正与重构  EquipmentSelection
% disp(1)
[Qib_St,wbi_Gyro,Mag_Mm] = EquipmentSelection(Qib_StA,Qib_StB,wgi_GyroA,Mag_MmA,Mag_MmB);

% 模式确定及状态设置  ModeDeter
% disp(FbLockOk')
ModeDeter();

% 标称系计算  Nominal_CSYS_Deter
[Qin,Qnnp,wni_n,Qon] = Nominal_CSYS_Deter(Qio,w0);

% 姿态确定 Att_Deter
Att_Deter(Qib_St,wbi_Gyro,Mag_Mm,Qin,Qnnp,wni_n);

% disp(Agl_SADA)
% 姿态控制Att_Ctrl
Att_Ctrl(FwSpd,T_sys,Qon);


global index AttRefChoice Agl_Deter Agl4Ctrl W_Deter W4Ctrl T_Fw H_Fw MagCmd dBb_TD MagCtrlFlag w_Tgt theta_Tgt AA_bTo;
if Mode == 1 && MagCtrlFlag == 2
    W4Ctrl_Tm = dBb_TD/180*pi;
else
    W4Ctrl_Tm = W4Ctrl;
end
y = [Mode;SubMode;index;Agl_Deter;W_Deter;Agl4Ctrl;W4Ctrl_Tm;T_Fw;H_Fw;MagCmd; w_Tgt; theta_Tgt; AA_bTo];