%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%% 标称系计算   %%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [Qin Qnnp wni_n Qon] = Nominal_CSYS_Deter(Qio,w0)
% clear;clc;
% index = [1;0];index_last=[0;0];Qin_last=[1;0;0;0;];Qio=[1;0;0;0];Sun_Right_Asc=10/180*pi;Sun_Dec=20/180*pi;w0=0.00106;
global index;
persistent Qin_last Qon_last;

if index(1) == 0                  % 轨道系
    if index(2) == 0
        Roll_Nominal = 80/180*pi;
        Pitch_Nominal = 8.5/180*pi;
        Yaw_Nominal = 0;
    elseif index(2) == 1
        Roll_Nominal = 0;
        Pitch_Nominal = 0;
        Yaw_Nominal = 0;
    elseif index(2) == 2
        Roll_Nominal = 0;
        Pitch_Nominal = 0;
        Yaw_Nominal = 3/4*pi+3.2/180*pi;
    elseif index(2) == 3
        Roll_Nominal =0;
        Pitch_Nominal = 0;
        Yaw_Nominal = 3/4*pi+3.2/180*pi;
    elseif index(2) == 4
        Roll_Nominal = pi/2;
        Pitch_Nominal = 0;
        Yaw_Nominal = 0;
    elseif index(2) == 5
        Roll_Nominal = -pi/2;
        Pitch_Nominal = 0;
        Yaw_Nominal = pi;
    elseif index(2) == 6
        Roll_Nominal = pi/2;
        Pitch_Nominal = 0;
        Yaw_Nominal = pi;
    elseif index(2) == 7
        Roll_Nominal = 0;
        Pitch_Nominal = 0;
        Yaw_Nominal = pi;
    else
        Roll_Nominal = 0;
        Pitch_Nominal = 0;
        Yaw_Nominal = 0;
    end
    % 计算轨道系到标称系的转移矩阵Ano及四元数Qon
    Ano=angle2dcm(Yaw_Nominal,Pitch_Nominal,Roll_Nominal,'ZYX');
    Qon=angle2quat(Yaw_Nominal,Pitch_Nominal,Roll_Nominal,'ZYX')';
    % 计算惯性系到标称系的四元数Qin
    Qin = quatmultiply(Qio',Qon')';
    % 计算惯性系到标称系的转移矩阵Ani
    %     Ani = quat2dcm(Qin);
    % 计算标称系变化矩阵An'n和标称系变化四元数Qnn'（n为当前标称系，n'为上一拍标称系）
    if isempty(Qon_last)
        Qon_last = Qon;
    end
    Qnnp = quatmultiply(quatinv(Qon'),Qon_last')';
    
    % 标称姿态角速度Nominal_AngleRate_Nominal_2_J2000(wni_n)
    wni_n = Ano*[0;-w0;0];
else
    error('NominalCYSYDeterError!');
end
Qin_last = Qin;
Qon_last = Qon;
% Ani_line = [Ani(:,1);Ani(:,2);Ani(:,3)];
% Anpn_line = [Anpn(:,1);Anpn(:,2);Anpn(:,3)];
% Qin = Qin';
% Qnnp = Qnnp';
% Qon = Qon';