close all;clc;
%slCharacterEncoding('ISO-8859-1');
Tstart = 0000;
Tend= 3000;
% set(0,'defaultAxesFontName','Monospaced')

% figure;
% plot(DataT,DataAglP(:,1),DataT,DataAglP(:,2),DataT,DataAglP(:,3),DataT,DataAglP(:,4),'linewidth',2);grid;
% xlabel('时间 (s)');ylabel('帆板转角 (\circ)');title('帆板转角');
% legend('\alpha_+_Y','\beta_+_Y','\alpha_-_Y','\beta_-_Y');axis([Tstart Tend -20 360]);
% figure;
% subplot(2,1,1);plot(DataT,DataAgl4Ctrl_P(:,1),DataT,DataAgl4Ctrl_P(:,2),'linewidth',2);grid;
% ylabel('+Y SADA');legend('\Delta\alpha','\Delta\beta');title('帆板角度控制误差(\circ)');axis([Tstart Tend -inf inf]);
% subplot(2,1,2);plot(DataT,DataAgl4Ctrl_P(:,3),DataT,DataAgl4Ctrl_P(:,4),'linewidth',2);grid;
% xlabel('时间 (s)');ylabel('-Y SADA');
% legend('\Delta\alpha','\Delta\beta');axis([Tstart Tend -inf inf]);
% figure;
% subplot(2,1,1);plot(DataT,DataAssCur(:,9),DataT,DataAssCur(:,10),DataT,DataAssCur(:,11),DataT,DataAssCur(:,11),'linewidth',2);grid;
% ylabel('太敏C');legend('1','2','3','4');title('太敏C/D电池片电流（A）');axis([Tstart Tend -inf inf]);
% subplot(2,1,2);plot(DataT,DataAssCur(:,13),DataT,DataAssCur(:,14),DataT,DataAssCur(:,15),DataT,DataAssCur(:,16),'linewidth',2);grid;
% xlabel('时间 (s)');ylabel('太敏D');
% legend('1','2','3','4');axis([Tstart Tend -inf inf]);

% figure;
% plot(DataT,DataWbi_GyroAB(:,1),DataT,DataWbi_GyroAB(:,2),DataT,DataWbi_GyroAB(:,3),'linewidth',2);grid;%,DataT,20*DataFlag(:,5)-10
% figure;
% plot(DataT,DataWbi_GyroAB(:,4),DataT,DataWbi_GyroAB(:,5),DataT,DataWbi_GyroAB(:,6),'linewidth',2);grid;%,DataT,20*DataFlag(:,5)-10
% figure;
% plot(DataT,DatadB(:,1),DataT,DatadB(:,2),DataT,DatadB(:,3),'linewidth',2);grid;
% xlabel('时间 (s)');ylabel('磁强计微分滤波值 (Gs/s)');title('磁强计微分滤波值');
% legend('dB_x','dB_y','dB_z');axis([0 Tend -0.08 0.08]);

figure;
plot(DataT,DataAtt4Ctrl(:,1),DataT,DataAtt4Ctrl(:,2),DataT,DataAtt4Ctrl(:,3),'linewidth',2);grid;%,DataT,20*DataFlag(:,5)-10
xlabel('时间 (s)');ylabel('控制用姿态角 (\circ)');title('控制用姿态角');
legend('\phi_c','\theta_c','\psi_c');axis([Tstart Tend -inf inf]);%axis([0 Tend -1 1]);%,'光照标志'
figure;
plot(DataT,DataAtt4Ctrl(:,4),DataT,DataAtt4Ctrl(:,5),DataT,DataAtt4Ctrl(:,6),'linewidth',2);grid;
xlabel('时间 (s)');ylabel('控制用姿态角速度 (\circ/s)');title('控制用姿态角速度');
legend('w_c_x','w_c_y','w_c_z');axis([Tstart Tend -inf inf]);%axis([Tstart Tend -0.04 0.04]);%
figure;
plot(DataT,DataFwSpd(:,1),DataT,DataFwSpd(:,2),DataT,DataFwSpd(:,3),DataT,DataFwSpd(:,4),'linewidth',2);grid;
xlabel('Time (s)');ylabel('飞轮转速(rpm)');title('飞轮转速');
legend('2a','2b','2c','2d');axis([Tstart Tend -inf inf]);
figure;
plot(DataT,DataT_Fw(:,1),DataT,DataT_Fw(:,2),DataT,DataT_Fw(:,3),DataT,DataT_Fw(:,4),'linewidth',2);grid;
xlabel('Time (s)');ylabel('飞轮转矩(rpm)');title('飞轮转矩');
legend('2a','2b','2c','2d');axis([Tstart Tend -inf inf]);
% % 
figure;
plot(DataT,DataM(:,1),DataT,DataM(:,2),DataT,DataM(:,3),'linewidth',2);grid;
xlabel('时间 (s)');ylabel('指令磁矩 (Am^2)');title('指令磁矩');
legend('M_x','M_y','M_z');axis([Tstart Tend -28 28]);

% figure;
% subplot(2,1,1);plot(DataT,DataSADA(:,1),DataT,DataSADA(:,3),DataT,DataSADA(:,5),DataT,DataSADA(:,7),'linewidth',2);grid;
% ylabel('帆板转角(\circ)');title('帆板转角及零位状态');legend('\alpha_+_Y','\beta_+_Y','\alpha_-_Y','\beta_-_Y');%axis([Tstart Tend -28 28]);
% subplot(2,1,2);plot(DataT,DataSADA(:,2),DataT,DataSADA(:,4),DataT,DataSADA(:,6),DataT,DataSADA(:,8),'linewidth',2);grid;
% xlabel('时间 (s)');ylabel('帆板零位');
% legend('A轴零位_+_Y','B轴零位_+_Y','A轴零位_-_Y','B轴零位_-_Y');axis([Tstart Tend -1 2]);
% figure;
% plot(DataT,DataAtt_Deter(:,1),DataT,DataAtt_Deter(:,2),DataT,DataAtt_Deter(:,3),'linewidth',2);grid;
% xlabel('时间 (s)');ylabel('姿态基准角度 (\circ)');title('姿态基准角度');
% legend('\phi_c','\theta_c','\psi_c');axis([Tstart Tend -120 120]);%axis([0 Tend -1 1]);%
% % % 
% figure;
% plot(DataT,DataAtt_Deter(:,4),DataT,DataAtt_Deter(:,5),DataT,DataAtt_Deter(:,6),'linewidth',2);grid;
% xlabel('时间 (s)');ylabel('姿态基准角速度 (\circ/s)');title('姿态基准角速度');
% legend('w_c_x','w_c_y','w_c_z');axis([Tstart Tend -.8 .8]);%axis([0 Tend -1 1]);%
figure;
plot(DataT,DataAtt_Dyn(:,1),DataT,DataAtt_Dyn(:,2),DataT,DataAtt_Dyn(:,3),'linewidth',2);grid;%DataT,40*DataFlag(:,5)-20,
xlabel('时间 (s)');ylabel('动力学姿态角(轨道系) (\circ)');title('动力学姿态角(轨道系)');
legend('\phi','\theta','\psi');axis([Tstart Tend -inf inf]);%axis([0 Tend -200 200]);%,'光照标志'
figure;
plot(DataT,DataAtt_Dyn(:,7),DataT,DataAtt_Dyn(:,8),DataT,DataAtt_Dyn(:,9),'linewidth',2);grid;
xlabel('时间 (s)');ylabel('动力学姿态角速度 (\circ/s)');title('动力学姿态角速度');
legend('w_x','w_y','w_z');axis([Tstart Tend -inf inf]);%axis([0 Tend -0.01 0.01]);%
% figure;
% plot(DataT,DataAtt_Dyn(:,4),DataT,DataAtt_Dyn(:,5),DataT,DataAtt_Dyn(:,6),'linewidth',2);grid;
% xlabel('时间 (s)');ylabel('动力学姿态角 (标称系)(\circ)');title('动力学姿态角(标称系)');
% legend('\phi','\theta','\psi');axis([Tstart Tend -200 200]);%axis([0 Tend -200 200]);%
% % 
% figure;
% plot(DataT,DataAtt_Dyn(:,10),DataT,DataAtt_Dyn(:,11),DataT,DataAtt_Dyn(:,12),'linewidth',2);grid;
% xlabel('时间 (s)');ylabel('动力学惯性角速度 (\circ/s)');title('动力学惯性角速度');
% legend('w_x','w_y','w_z');axis([Tstart Tend -0.2 0.3]);
% 
% figure;
% plot(DataT,DataH(:,1),DataT,DataH(:,2),DataT,DataH(:,3),'linewidth',2);grid;
% xlabel('时间 (s)');ylabel('三轴飞轮角动量 (Nms)');title('三轴飞轮角动量');
% legend('H_X','H_y','H_z');axis([0 Tend -0.6 0.6]);
% 
% figure;
% subplot(3,1,1);plot(DataT,DataFlag(:,1),DataT,DataFlag(:,2),DataT,DataFlag(:,5),'linewidth',2);grid;
% ylabel('模式状态字');title('标志字');
% legend('主模式标志','子模式标志','光照标志');axis([Tstart Tend -1 4]);
% subplot(3,1,2);plot(DataT,DataFlag(:,3),DataT,DataFlag(:,4),'linewidth',2);grid;
% ylabel('标称系');
% legend('主标志','子标志');axis([Tstart Tend -1 5]);
% subplot(3,1,3);plot(DataT,DataFlag(:,6),'linewidth',2);grid;
% xlabel('时间(s)');ylabel('姿态基准');axis([Tstart Tend -1 6]);
%
% figure;
% subplot(3,1,1);plot(DataT,DataFlag(:,1),DataT,DataFlag(:,2),DataT,DataSafeModeStepFlag(:,1),'linewidth',2);grid;
% ylabel('模式状态字');title('标志字');
% legend('主模式标志','子模式标志','阶段标志');axis([Tstart Tend -1 9]);
% subplot(3,1,2);plot(DataT,DataFlag(:,3),DataT,DataFlag(:,4),'linewidth',2);grid;
% ylabel('标称系');
% legend('主标志','子标志');axis([Tstart Tend -1 5]);
% subplot(3,1,3);plot(DataT,DataFlag(:,6),DataT,DataFlag(:,5),'linewidth',2);grid;
% xlabel('时间(s)');ylabel('姿态基准');axis([Tstart Tend -1 6]);
% legend('姿态基准标志','光照标志');
% 
% figure;
% plot(DataT,DataFbLock(:,1),DataT,DataFbLock(:,2),'linewidth',2);grid;
% xlabel('时间 (s)');title('SADA解锁/锁定标志');axis([Tstart Tend -1 3])
% figure;
% plot(DataT,DataSunb(:,1),DataT,DataSunb(:,2),DataT,DataSunb(:,3),DataT,-0.1+0.5*DataFlag(:,5),'linewidth',2);grid;
% xlabel('时间 (s)');ylabel('SunVector');title('星体系下的太阳矢量');
% legend('r_x','r_y','r_z','光照标志');axis([Tstart Tend -1.2 1.2]);


% figure;
% plot(DataT,DataTw3(:,1),DataT,DataTw3(:,2),DataT,DataTw3(:,3),'linewidth',2);grid;
% xlabel('时间 (s)');ylabel('三轴控制力矩');title('三轴控制力矩');
% legend('T_d_x','T_d_y','T_d_z');axis([Tstart Tend -0.3 0.3]);
% % figure;
% plot(DataT,DataFwH3(:,1),DataT,DataFwH3(:,2),DataT,DataFwH3(:,3),'linewidth',2);grid;
% xlabel('Time (s)');ylabel('飞轮角动量(Nms)');title('飞轮角动量');
% legend('x','y','z');axis([Tstart Tend -10 10]);
% 
% figure;
% plot(DataT,DataM(:,1),DataT,DataM(:,2),DataT,DataM(:,3),'linewidth',2);grid;
% xlabel('时间 (s)');ylabel('指令磁矩 (Am^2)');title('指令磁矩');
% legend('M_x','M_y','M_z');axis([Tstart Tend -20 20]);

% figure;
% plot(DataT,DataAglP(:,1),DataT,DataAglP(:,2),'linewidth',2);grid;
% xlabel('时间 (s)');ylabel('帆板转角 (\circ)');title('帆板转角');
% legend('\alpha','\beta');axis([Tstart Tend -180 180]);
% % figure;
% plot(DataT,Dataq(:,1),DataT,Dataq(:,2),DataT,Dataq(:,3),DataT,Dataq(:,4),DataT,Dataq(:,5),DataT,Dataq(:,6),'linewidth',2);grid;
% xlabel('时间 (s)');ylabel('帆板模态坐标)');title('帆板模态坐标');
% % legend('1st','2nd','3rd','4th','5th','6th');%axis([Tstart Tend -28 28]);
% figure;
% plot(DataT,DataTd(:,1),DataT,DataTd(:,2),DataT,DataTd(:,3),'linewidth',2);grid;
% xlabel('时间 (s)');ylabel('干扰力矩');title('总干扰力矩');
% legend('T_d_x','T_d_y','T_d_z');axis([Tstart Tend -inf inf]);
% figure;
% subplot(2,1,1);plot(DataT,DataTall(:,1),DataT,DataTall(:,2),DataT,DataTall(:,3),'linewidth',2);grid;ylabel('重力梯度干扰力矩');title('干扰力矩');legend('T_m_x','T_m_y','T_m_z');axis([Tstart Tend -inf inf]);
% subplot(2,1,2);plot(DataT,DataTall(:,4),DataT,DataTall(:,5),DataT,DataTall(:,6),'linewidth',2);grid;ylabel('光压干扰力矩');xlabel('时间 (s)');legend('T_s_x','T_s_y','T_s_z');axis([Tstart Tend -inf inf]);
% figure;
% subplot(2,1,1);plot(DataT,DataTall(:,7),DataT,DataTall(:,8),DataT,DataTall(:,9),'linewidth',2);grid;ylabel('气动干扰力矩');title('干扰力矩');legend('T_a_x','T_a_y','T_a_z');axis([Tstart Tend -inf inf]);
% subplot(2,1,2);plot(DataT,DataTall(:,10),DataT,DataTall(:,11),DataT,DataTall(:,12),'linewidth',2);grid;ylabel('剩磁干扰力矩');xlabel('时间 (s)');legend('T_g_x','T_g_y','T_g_z');axis([Tstart Tend -inf inf]);
% figure;
% plot(DataT,DataTWCross(:,1),DataT,DataTWCross(:,2),DataT,DataTWCross(:,3),'linewidth',2);grid;title('陀螺力矩');
% legend('X','Y','Z');axis([Tstart Tend -inf inf]);xlabel('时间 (s)');
% figure;
% plot(Tstart:0.01:Tend,DataTops(:,1));grid;title('TOPS扫描镜驱动干扰力矩');
% axis([Tstart Tend -inf inf]);xlabel('时间 (s)');ylabel('干扰力矩（Nm）')
% figure;
% plot(DataT,DataTd_I(:,1),DataT,DataTd_I(:,2),DataT,DataTd_I(:,3),'linewidth',2);grid;title('干扰力矩累计（惯性系）');
% legend('X','Y','Z');axis([Tstart Tend -inf inf]);xlabel('时间 (s)');
% figure;
% plot(DataT,DataTd_O(:,1),DataT,DataTd_O(:,2),DataT,DataTd_O(:,3),'linewidth',2);grid;title('干扰力矩累计（本体系）');
% legend('X','Y','Z');axis([Tstart Tend -inf inf]);xlabel('时间 (s)');
% 
% figure;
% plot(DataT,DataStareErr,'linewidth',2);grid;
% xlabel('时间 (s)');ylabel('凝视误差 (\circ)');title('凝视误差');
% axis([Tstart Tend -inf inf]);

% 
% figure;
% plot(DataT,DatadB(:,7),DataT,DatadB(:,8),DataT,DatadB(:,9),'linewidth',2);grid;
% xlabel('时间 (s)');ylabel('磁强计微分滤波值 (Gs/s)');title('磁强计微分滤波值');
% legend('Bdot_x','Bdot_y','Bdot_z');axis([0 Tend -0.015 0.015]);

%% kalman filter

% figure;
% plot(DataT,DataScopeqib(:,2),DataT,DataScopeqib(:,3),DataT,DataScopeqib(:,4),DataT,DataScopeqib(:,5),'linewidth',2);grid;
% xlabel('时间 (s)');ylabel('星敏四元数qib');title('星敏四元数qib');
% legend('q0','q1','q2','q3');axis([0 Tend -1 1]);
% 
% figure;
% plot(DataT,DataAgl(:,1),DataT,DataAgl(:,2),DataT,DataAgl(:,3),'linewidth',2);grid;
% xlabel('时间 (s)');ylabel('姿态角 (\circ)');title('姿态角');
% legend('\phi','\theta','\psi');%axis([0 Tend -150 250]);
% 
% figure;
% plot(DataT,DataSunb(:,1),DataT,DataSunb(:,2),DataT,DataSunb(:,3),DataT,DataFlag(:,5),'linewidth',2);grid;
% xlabel('时间 (s)');ylabel('太阳矢量');title('星体系下的太阳矢量');
% legend('r_x','r_y','r_z','光照标志');axis([0 Tend -inf inf]);
% 
% figure;
% plot(DataT,Datawbi(:,1)*180/pi,DataT,Datawbi(:,2)*180/pi,DataT,Datawbi(:,3)*180/pi,'linewidth',2);grid;
% xlabel('时间 (s)');ylabel('惯性角速度 (\circ/s)');title('惯性角速度');
% legend('w_x','w_y','w_z');axis([0 Tend -0.6 0.5]);
% figure;
% plot(DataT,DataOMGErr,'linewidth',2);grid;
% xlabel('时间 (s)');ylabel('升交点赤经误差(°)');title('轨道递推误差');
% 
% figure;
% plot(DataT,DataIErr,'linewidth',2);grid;
% xlabel('时间 (s)');ylabel('倾角误差(°)');title('轨道递推误差');
% 
% figure;
% plot(DataT,DataUErr,'linewidth',2);grid;
% xlabel('时间 (s)');ylabel('纬度幅角误差(°)');title('轨道递推误差');
% DataAtt_DynErr = DataAtt_Dyn - DataAtt_DynOrbitErr;
% figure;
% plot(DataT,DataAtt_DynErr(:,1),DataT,DataAtt_DynErr(:,2),DataT,DataAtt_DynErr(:,3),'linewidth',2);grid;%DataT,40*DataFlag(:,5)-20,
% xlabel('时间 (s)');ylabel('定轨误差带来的定姿误差 (\circ)');title('定轨误差带来的定姿误差 (\circ)');
% legend('\phi','\theta','\psi');axis([Tstart Tend -inf inf]);%axis([0 Tend -200 200]);%,'光照标志'
% figure;
% plot(DataT,DataAtt_DynErr(:,7),DataT,DataAtt_DynErr(:,8),DataT,DataAtt_DynErr(:,9),'linewidth',2);grid;
% xlabel('时间 (s)');ylabel('定轨误差带来的定姿误差 (\circ/s)');title('定轨误差带来的定姿误差 (\circ/s)');
% legend('w_x','w_y','w_z');axis([Tstart Tend -inf inf]);%axis([0 Tend -0.01 0.01]);%
% figure;
% plot(DataT,DataScopebgyro(:,2),DataT,DataScopebgyro(:,3),DataT,DataScopebgyro(:,4),'linewidth',2);grid;
% xlabel('时间 (s)');ylabel('陀螺漂移(\circ/s)');title('陀螺漂移');
% legend('b_x','b_y','b_z');%axis([0 Tend -0.01 0.03]);
% figure;
% % plot(DataT,DataWbi_GyroAB(:,1),DataT,DataWbi_GyroAB(:,2)+0.0011*180/pi,DataT,DataWbi_GyroAB(:,3),'linewidth',2);grid;
% plot(DataT,DataAtt4Ctrl(:,4),DataT,DataAtt4Ctrl(:,5),DataT,DataAtt4Ctrl(:,6),'linewidth',2);grid;
% 
% xlabel('时间 (s)');ylabel('陀螺测量噪声(\circ/s)');title('滤波后陀螺测量噪声');
% legend('N_x','N_y','N_z');axis([0 Tend -.0015 .0015]);
% figure;
% plot(DataT,DataStarSenserReal(:,1),DataT,DataStarSenserReal(:,2),DataT,DataStarSenserReal(:,3),'linewidth',2);grid;
% xlabel('时间 (s)');ylabel('星敏原始姿态角 (\circ)');title('星敏原始姿态角');
% legend('\phi','\theta','\psi');axis([0 Tend -0.5 0.5]);