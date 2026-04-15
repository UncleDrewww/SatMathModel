function [Qib_St,wbi_Gyro,Mag_Mm] = EquipmentSelection(Qib_StA_F,Qib_StB_F,wgi0_GyroA,Mag_MmA,Mag_MmB)
% 星敏修正与重构
Qib_St=StRefChoice(Qib_StA_F,Qib_StB_F);
% 陀螺修正与重构
wbi_Gyro=GyroRefChoice(wgi0_GyroA);
% 飞轮诊断与重构
FwRefChoice;

% 磁强计诊断与重构
[Mag_Mm] = MmRefChoice(Mag_MmA,Mag_MmB);

%% 飞轮诊断与重构
function FwRefChoice
global FwMode FwMode_last FwModeForce Matrix_I;
FwMode_last = FwMode;
if sum(FwModeForce)<=4
    FwMode = FwModeForce;
end
for i =1:4
    if FwMode(i) == 0
        Matrix_I(:,i) = zeros(3,1);
    end
end

%% 磁基准修正与重构
function Mag_Mm = MmRefChoice(Mag_MmA,Mag_MmB)
global MmPriority Mm_Ok;
MmA_Ok = 1;
MmB_Ok = 1;
if MmPriority == 1
    if MmA_Ok == 1
        Mm_Ok = 1;
        Mag_Mm = Mag_MmA;
    elseif MmB_Ok == 1
        Mm_Ok = 1;
        Mag_Mm = Mag_MmB;
    else
        Mm_Ok = 0;
        Mag_Mm = [0;0;0];
    end
else
    if MmB_Ok == 1
        Mm_Ok = 1;
        Mag_Mm = Mag_MmB;
    elseif MmA_Ok == 1
        Mm_Ok = 1;
        Mag_Mm = Mag_MmA;
    else
        Mm_Ok = 0;
        Mag_Mm = [0;0;0];
    end
end

%% 星敏修正与重构
function Qib_St=StRefChoice(Qib_StA_F,Qib_StB_F)
% StarRefFlag星敏基准选择标志：1-星敏A;2-星敏B;3-双星敏;0-无星敏
global  St_Ok StPriorityFlag Allow_DoubleSt Qib_DoubleSt;
global Qbs_StAFix Qbs_StBFix Qib_StA Qib_StB;
persistent DeltaQAB_lb;
if isempty(DeltaQAB_lb)
    DeltaQAB_lb=[1;0;0;0];
end
Qib_StA = Qib_StA_F;
Qib_StB = Qib_StB_F;
% 双星敏定姿
OpticAxisA_SA = [0;0;1];
OpticAxisB_SB = [0;0;1];
A_SAb = quat2dcm(Qbs_StAFix);
A_SBb = quat2dcm(Qbs_StBFix);
OpticAxisA_b = A_SAb'*OpticAxisA_SA;
OpticAxisA_i = quat2dcm(quatinv(Qib_StA')) * OpticAxisA_b;
OpticAxisB_b = A_SBb'*OpticAxisB_SB;
OpticAxisB_i = quat2dcm(quatinv(Qib_StB')) * OpticAxisB_b;
Matrix_b1 = OpticAxisA_b;
Matrix_b2 = cross(Matrix_b1,OpticAxisB_b)/norm(cross(Matrix_b1,OpticAxisB_b));
Matrix_b3 = cross(Matrix_b1,Matrix_b2)/norm(cross(Matrix_b1,Matrix_b2));
Matrix_b = [Matrix_b1 Matrix_b2 Matrix_b3];
Matrix_i1 = OpticAxisA_i;
Matrix_i2 = cross(Matrix_i1,OpticAxisB_i)/norm(cross(Matrix_i1,OpticAxisB_i));
Matrix_i3 = cross(Matrix_i1,Matrix_i2)/norm(cross(Matrix_i1,Matrix_i2));
Matrix_i = [Matrix_i1 Matrix_i2 Matrix_i3];
Abi = Matrix_b*Matrix_i';
Qib_DoubleSt = (dcm2quat(Abi))';
if Allow_DoubleSt == 1
    St_Ok = 1;
    Qib_St = Qib_DoubleSt;
elseif StPriorityFlag == 1 % 星敏A优先级高
    St_Ok = 1;
    Qib_St = Qib_StA;
else
    St_Ok = 1;
    Qib_St = Qib_StB;
end

%% 陀螺修正与重构
function W_Gyro=GyroRefChoice(Wgi0_GyroA)
global  Gyro_Ok;
global Abg_GyroA  Drift_GyroA_Tc ;
Wgi_GyroA = Wgi0_GyroA - Drift_GyroA_Tc;
Wgi_GyroA_Aft = Wgi_GyroA;
W_Gyro = Abg_GyroA * Wgi_GyroA_Aft;
Gyro_Ok = 1;



