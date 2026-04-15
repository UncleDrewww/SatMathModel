function [Att4Ctrl, FwSpd, FwT, Tw3, Mt_M,M3,NominalCSYS,Td] = ...
    ZZ_Att(UTC_1970,EnvirParam,OrbitElement,StParam,GyroParam,AssParam,MmParam,SadaParam,CamParam,Td_Dyn,t_ctrl,RunCount)
global SatParam AttRef MagRef StRef GyroRef AssRef MmRef MagCtrlCoeff AssMmRef StGyroAttRef;
persistent Att4Ctrl_last Hw3_last;
if isempty(Att4Ctrl_last)
    Att4Ctrl_last = struct('Ok',0,'Qnb',[1;0;0;0],'Agl',[0;0;0],'Wbn',[0;0;0]);
    Hw3_last = [0;0;0];% 飞轮组合的三轴角动量
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%% 用户自定义函数，包含 ModeDeterAndInit/NominalCSYS/SatParamCal/TdCal %%%%%%%%%%%%%%%%%%%%%%
[AlgthmChoiceFlag,Index,RotateAglRateTgt, NominalCSYS, Td, I] = ...
    ZZ_User_Att(SatParam,OrbitElement,EnvirParam,SadaParam,CamParam,MmRef,StRef,GyroRef,StGyroAttRef,AssRef,AttRef,Att4Ctrl_last,Hw3_last,t_ctrl,RunCount);
%     feval(FuncName,{SatParam,OrbitElement,EnvirParam,SadaParam,StRef,AssRef,AttRef,Att4Ctrl_last});

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

[MagRef, MmRef] = MagDeter(AlgthmChoiceFlag,OrbitElement,AttRef,MagRef,StRef,MmRef,MmParam,t_ctrl,RunCount);

[AttRef,StRef] = AttDeter(AlgthmChoiceFlag,StParam,GyroParam,AssParam,AttRef,StRef,NominalCSYS,OrbitElement,t_ctrl,EnvirParam,MmRef,MagRef);

Att4Ctrl = ZZ_Att4Ctrl(AlgthmChoiceFlag, AttRef, NominalCSYS, RotateAglRateTgt,EnvirParam.R_Sun);

[FwSpd, FwT, Hw3, Tw3, Mt_M, M3] = ZZ_AttCtrl(Att4Ctrl,NominalCSYS,MagRef,MmRef,AlgthmChoiceFlag,SadaParam,I,MagCtrlCoeff,Td,t_ctrl,RunCount);

Att4Ctrl_last = Att4Ctrl;
Hw3_last = Hw3;
end


%% 姿态确定
function [AttRef,StRef] = AttDeter(AlgthmChoiceFlag,StParam,GyroParam,AssParam,AttRef,StRef,NominalCSYS,OrbitElement,t_ctrl,EnvirParam,MmRef,MagRef)
global GyroRef AssRef AssMmRef StAttRef AssAttRef GyroAttRef AssMmAttRef StGyroAttRef AssGyroAttRef AssMmGyroAttRef;
%星敏姿态解算
StRef = StAttCal(StRef,StParam,t_ctrl,EnvirParam.R_Sun_long,OrbitElement.RV_i);
%陀螺姿态解算
GyroRef = GyroAttCal(GyroRef,GyroParam,AttRef,t_ctrl);
%太敏姿态解算
AssRef = AssAttCal(AssRef,AssParam,t_ctrl,EnvirParam.R_Sun);
%太敏磁强计联合姿态解算
AssMmRef = AssMmAttCal(AssMmRef,MmRef,MagRef,EnvirParam.R_Sun,AssRef,OrbitElement,t_ctrl);
%单星敏基准
StAttRef = StAttRefCal(StRef);
%单太敏基准
AssAttRef = AssAttRefCal(AssRef);
%单陀螺基准
GyroAttRef = GyroAttRefCal(GyroRef);
%双矢量基准
AssMmAttRef = AssMmAttRefCal(AssMmRef);
%星敏陀螺基准
StGyroAttRef = StGyroAttRefCal(StGyroAttRef,StAttRef,GyroRef,t_ctrl);
%太敏陀螺基准
AssGyroAttRef = AssGyroAttRefCal(AssGyroAttRef,AssAttRef,GyroRef,t_ctrl);
%双矢量陀螺基准
AssMmGyroAttRef = AssMmGyroAttRefCal(AssMmGyroAttRef,AssMmAttRef,GyroRef,t_ctrl);
%基准选择
AttRef = AttRefCal(AttRef,AlgthmChoiceFlag,StAttRef,AssAttRef,GyroAttRef,AssMmAttRef,StGyroAttRef,AssGyroAttRef,AssMmGyroAttRef,NominalCSYS,OrbitElement);

end

function StRef = StAttCal(StRef,StParam,t_ctrl,R_Sun,RV_i)
global Tc_StChoiceIndex;
persistent Qib_last DeltaQib_last StAttCalFirstCheck;
if isempty(Qib_last)
    Qib_last = [1;0;0;0];
end
if isempty(DeltaQib_last)
    DeltaQib_last = [1;0;0;0];
end
if isempty(StAttCalFirstCheck)
    StAttCalFirstCheck = 1;
end
StParam = StAbrFix(StParam,R_Sun,RV_i,t_ctrl); %光行差补偿算法
St_Axis = [0;0;1];% 星敏坐标系下光轴矢量
Qis_StA = StParam.StA.Qis;
Qsb_StAFix = StParam.StA.Qsb;
StParam.StA.Qib = quatmultiply(Qis_StA',Qsb_StAFix')';
Qis_StB = StParam.StB.Qis;
Qsb_StBFix = StParam.StB.Qsb;
StParam.StB.Qib = quatmultiply(Qis_StB',Qsb_StBFix')';
Qis_StC = StParam.StC.Qis;
Qsb_StCFix = StParam.StC.Qsb;
StParam.StC.Qib = quatmultiply(Qis_StC',Qsb_StCFix')';
[StParam.StAB.Qib,StParam.StAB.Ok] = DbVecAttDeter(quat2dcm(Qsb_StAFix')*St_Axis,quat2dcm(Qis_StA')'*St_Axis,quat2dcm(Qsb_StBFix')*St_Axis,quat2dcm(Qis_StB')'*St_Axis,StParam.StA.Ok,StParam.StB.Ok);
[StParam.StAC.Qib,StParam.StAC.Ok] = DbVecAttDeter(quat2dcm(Qsb_StAFix')*St_Axis,quat2dcm(Qis_StA')'*St_Axis,quat2dcm(Qsb_StCFix')*St_Axis,quat2dcm(Qis_StC')'*St_Axis,StParam.StA.Ok,StParam.StC.Ok);
[StParam.StBC.Qib,StParam.StBC.Ok] = DbVecAttDeter(quat2dcm(Qsb_StBFix')*St_Axis,quat2dcm(Qis_StB')'*St_Axis,quat2dcm(Qsb_StCFix')*St_Axis,quat2dcm(Qis_StC')'*St_Axis,StParam.StB.Ok,StParam.StC.Ok);
StRef.Qib = [1;0;0;0];
StRef.Ok = 0;
StRef.Wbi = [0;0;0];
StRef.Choice = '0';
for Index = 1:length(Tc_StChoiceIndex)
    St_Index = eval(['StParam.' Tc_StChoiceIndex{Index}]);
    if St_Index.Ok == 1
        StRef.Qib = St_Index.Qib;
        StRef.Ok = 1;
        StRef.Choice = Tc_StChoiceIndex{Index};
        break;
    end
end
if StRef.Ok == 1
    DeltaQib = [1;0;0;0];
    if StAttCalFirstCheck == 1
        DeltaQib_last = DeltaQib;
    else
        DeltaQib = quatmultiply(quatinv(Qib_last'),(StRef.Qib)')';
        DeltaQib = QuatRectify(DeltaQib);
        DeltaQib = DeltaQib/norm(DeltaQib);
    end
    DeltaQib_Filter = Filter(DeltaQib_last,DeltaQib,StRef.tao,t_ctrl);
    DeltaQib_Filter = DeltaQib_Filter/norm(DeltaQib_Filter);
    DeltaQib_Filter = QuatRectify(DeltaQib_Filter);
    if norm(DeltaQib_Filter(2:4)) <= 1e-6
        StRef.Wbi = [0;0;0];
    else
        StRef.Wbi = 2*acos(DeltaQib_Filter(1))/t_ctrl*DeltaQib_Filter(2:4)/norm(DeltaQib_Filter(2:4));
    end
    Qib_last = StRef.Qib;
    DeltaQib_last = DeltaQib;
    StAttCalFirstCheck = 0;
else
    StAttCalFirstCheck = 1;
    StRef.Wbi = [0;0;0];
end

end

function GyroRef = GyroAttCal(GyroRef,GyroParam,AttRef,t_ctrl) 
global Tc_GyroChoiceIndex;
Wgi_GyroA = GyroParam.GyroA.Wgi;
GyroA_Fix_Abg = GyroParam.GyroA.Abg;
GyroParam.GyroA.Wbi = GyroA_Fix_Abg*Wgi_GyroA;
Wgi_GyroB = GyroParam.GyroB.Wgi;
GyroB_Fix_Abg = GyroParam.GyroB.Abg;
GyroParam.GyroB.Wbi = GyroB_Fix_Abg*Wgi_GyroB;
GyroRef.Wbi = [0;0;0];
GyroRef.Ok = 0;
GyroRef.Choice = '0';
GyroRef.Qib = [1;0;0;0];
for Index = 1:length(Tc_GyroChoiceIndex)
    Gyro_Index = eval(['GyroParam.' Tc_GyroChoiceIndex{Index}]);
    if Gyro_Index.Ok == 1
        GyroRef.Wbi = Gyro_Index.Wbi;
        GyroRef.Ok = 1;
        GyroRef.Choice = Tc_GyroChoiceIndex{Index};
        break;
    end
end
if GyroRef.Ok == 1
    Qib_last = AttRef.Qib;
    GyroRef.Qib(1) = Qib_last(1) -t_ctrl/2 * (GyroRef.Wbi(1)*Qib_last(2) + GyroRef.Wbi(2)*Qib_last(3) + GyroRef.Wbi(3)*Qib_last(4));
    GyroRef.Qib(2) = Qib_last(2) +t_ctrl/2 * (GyroRef.Wbi(1)*Qib_last(1) + GyroRef.Wbi(3)*Qib_last(3) - GyroRef.Wbi(2)*Qib_last(4));
    GyroRef.Qib(3) = Qib_last(3) +t_ctrl/2 * (GyroRef.Wbi(2)*Qib_last(1) - GyroRef.Wbi(3)*Qib_last(2) + GyroRef.Wbi(1)*Qib_last(4));
    GyroRef.Qib(4) = Qib_last(4) +t_ctrl/2 * (GyroRef.Wbi(3)*Qib_last(1) + GyroRef.Wbi(2)*Qib_last(2) - GyroRef.Wbi(1)*Qib_last(3));
    
end
end

function AssRef = AssAttCal(AssRef,AssParam,t_ctrl,Sj2000)
global Tc_AssChoiceIndex;
persistent Qib_last DeltaQib_last AssAttCalFirstCheck;
if isempty(Qib_last)
    Qib_last = [1;0;0;0];
end
if isempty(DeltaQib_last)
    DeltaQib_last = [1;0;0;0];
end
if isempty(AssAttCalFirstCheck)
    AssAttCalFirstCheck = 1;
end
AssMinVolt = 0.3;
M_temp = zeros(3,3);
V_temp = zeros(3,1);
AssRef.Ok = 0;
AssRef.Sb = [0;0;0];
for Index = 1:length(Tc_AssChoiceIndex)
    Ass_Index = eval(['AssParam.' Tc_AssChoiceIndex{Index}]);
    M = eye(4);
    M(:,Ass_Index.AssVolt<AssMinVolt) = 0;
    M_temp = M_temp+Ass_Index.Abs*Ass_Index.Ass_n*M*(Ass_Index.Abs*Ass_Index.Ass_n*M)';
    V_temp = V_temp+Ass_Index.Abs*Ass_Index.Ass_n*M*Ass_Index.AssVolt;
end
if rank(M_temp) >= 3
    AssRef.Ok = 1;
    AssRef.Sb = M_temp\V_temp;
    AssRef.Sb = AssRef.Sb/norm(AssRef.Sb);
end
if AssRef.Ok == 1
    AssRef.Qib = Vec2Quat(AssRef.Sb,Sj2000);
    DeltaQib = [1;0;0;0];
    if AssAttCalFirstCheck == 1
        DeltaQib_last = DeltaQib;
    else
        DeltaQib = quatmultiply(quatinv(Qib_last'),(AssRef.Qib)')';
        DeltaQib = QuatRectify(DeltaQib);
        DeltaQib = DeltaQib/norm(DeltaQib);
    end
    DeltaQib_Filter = Filter(DeltaQib_last,DeltaQib,AssRef.tao,t_ctrl);
    DeltaQib_Filter = DeltaQib_Filter/norm(DeltaQib_Filter);
    DeltaQib_Filter = QuatRectify(DeltaQib_Filter);
    if norm(DeltaQib_Filter(2:4)) <= 1e-6
        AssRef.Wbi = [0;0;0];
    else
        AssRef.Wbi = 2*acos(DeltaQib_Filter(1))/t_ctrl*DeltaQib_Filter(2:4)/norm(DeltaQib_Filter(2:4));
    end
    Qib_last = AssRef.Qib;
    DeltaQib_last = DeltaQib;
    AssAttCalFirstCheck = 0;
else
    AssAttCalFirstCheck = 1;
    AssRef.Qib = [1;0;0;0];
    AssRef.Wbi = [0;0;0];
end
   
end

function AssMmRef = AssMmAttCal(AssMmRef,MmRef,MagRef,R_Sun,AssRef,OrbitElement,t_ctrl)
persistent Qib_last DeltaQib_last AssMmAttCalFirstCheck;
if isempty(Qib_last)
    Qib_last = [1;0;0;0];
end
if isempty(DeltaQib_last)
    DeltaQib_last = [1;0;0;0];
end
if isempty(AssMmAttCalFirstCheck)
    AssMmAttCalFirstCheck = 1;
end
B_b = MmRef.Bb;
% 轨道系磁场应该用磁场公式算
B_o = MagRef.Bo_MagEqu;
S_b = AssRef.Sb;
S_o = quat2dcm(OrbitElement.Qio')*R_Sun;
[AssMmRef.Qob,AssMmRef.Ok] = DbVecAttDeter(B_b,B_o,S_b,S_o,MmRef.Ok,AssRef.Ok);
if AssMmRef.Ok == 1
    AssMmRef.Qib = quatmultiply((OrbitElement.Qio)',(AssMmRef.Qob)')';
    DeltaQib = [1;0;0;0];
    if AssMmAttCalFirstCheck == 1
        DeltaQib_last = DeltaQib;
    else
        DeltaQib = quatmultiply(quatinv(Qib_last'),(AssMmRef.Qib)')';
        DeltaQib = QuatRectify(DeltaQib);
        DeltaQib = DeltaQib/norm(DeltaQib);
    end
    DeltaQib_Filter = Filter(DeltaQib_last,DeltaQib,AssMmRef.tao,t_ctrl);
    DeltaQib_Filter = DeltaQib_Filter/norm(DeltaQib_Filter);
    DeltaQib_Filter = QuatRectify(DeltaQib_Filter);
    if norm(DeltaQib_Filter(2:4)) <= 1e-6
        AssMmRef.Wbi = [0;0;0];
    else
        AssMmRef.Wbi = 2*acos(DeltaQib_Filter(1))/t_ctrl*DeltaQib_Filter(2:4)/norm(DeltaQib_Filter(2:4));
    end
    Qib_last = AssMmRef.Qib;
    DeltaQib_last = DeltaQib;
    AssMmAttCalFirstCheck = 0;
else
    AssMmAttCalFirstCheck = 1;
    AssMmRef.Qib = [1;0;0;0];
    AssMmRef.Wbi = [0;0;0];
end


end

function [Qba,Qba_Ok] = DbVecAttDeter(Vec1_a,Vec1_b,Vec2_a,Vec2_b,Vec1_Ok,Vec2_Ok)
% 双矢量定姿，没加共线判断
if Vec1_Ok == 1 && Vec2_Ok == 1
    Matrix_a1 = Vec1_a/norm(Vec1_a);
    Matrix_a2 = cross(Matrix_a1,Vec2_a)/norm(cross(Matrix_a1,Vec2_a));
    Matrix_a3 = cross(Matrix_a1,Matrix_a2)/norm(cross(Matrix_a1,Matrix_a2));
    Matrix_a = [Matrix_a1 Matrix_a2 Matrix_a3];
    Matrix_b1 = Vec1_b/norm(Vec1_b);
    Matrix_b2 = cross(Matrix_b1,Vec2_b)/norm(cross(Matrix_b1,Vec2_b));
    Matrix_b3 = cross(Matrix_b1,Matrix_b2)/norm(cross(Matrix_b1,Matrix_b2));
    Matrix_b = [Matrix_b1 Matrix_b2 Matrix_b3];
    Aab = Matrix_a*Matrix_b';
    Qba = dcm2quat(Aab)';
    Qba_Ok = 1;
else
    Qba = [1;0;0;0];
    Qba_Ok = 0;
end
end

function StAttRef = StAttRefCal(StRef)
global Tc_StUse_Enable;
if Tc_StUse_Enable == 0 || StRef.Ok == 0
    StAttRef.Ok = 0;
    StAttRef.Qib = [1;0;0;0];
    StAttRef.Wbi = [0;0;0];
else
    StAttRef.Ok = 1;
    StAttRef.Qib = StRef.Qib;
    StAttRef.Wbi = StRef.Wbi;    
end
end

function StParam = StAbrFix(StParam,R_Sun,RV_i,t_ctrl)
global Tc_StAbr_Enable;
persistent R_Sun_last;
if isempty(R_Sun_last)
    R_Sun_last = R_Sun;
end
C = 3e08;% 光速
if Tc_StAbr_Enable == 1
    Vearth = -(R_Sun - R_Sun_last)/t_ctrl;%地球公转角速度
    Vs = RV_i(4:6)+Vearth;
    Qvs = [0 Vs(1) Vs(2) Vs(3)];
    Qvs_A = quatmultiply(quatmultiply(quatinv(StParam.StA.Qis'),Qvs),StParam.StA.Qis')';
    AlfaX_A = Qvs_A(2)/C;
    AlfaY_A = Qvs_A(3)/C;
    DeltaQ_A = [sqrt(1-(AlfaX_A/2)^2-(AlfaY_A/2)^2); -AlfaY_A/2; AlfaX_A/2; 0];
    StParam.StA.Qis = quatmultiply(StParam.StA.Qis',DeltaQ_A')';
    Qvs_B = quatmultiply(quatmultiply(quatinv(StParam.StB.Qis'),Qvs),StParam.StB.Qis')';
    AlfaX_B = Qvs_B(2)/C;
    AlfaY_B = Qvs_B(3)/C;
    DeltaQ_B = [sqrt(1-(AlfaX_B/2)^2-(AlfaY_B/2)^2); -AlfaY_B/2; AlfaX_B/2; 0];
    StParam.StB.Qis = quatmultiply(StParam.StB.Qis',DeltaQ_B')';  
    Qvs_C = quatmultiply(quatmultiply(quatinv(StParam.StC.Qis'),Qvs),StParam.StC.Qis')';
    AlfaX_C = Qvs_C(2)/C;
    AlfaY_C = Qvs_C(3)/C;
    DeltaQ_C = [sqrt(1-(AlfaX_C/2)^2-(AlfaY_C/2)^2); -AlfaY_C/2; AlfaX_C/2; 0];
    StParam.StC.Qis = quatmultiply(StParam.StC.Qis',DeltaQ_C')';
    Qvs_D = quatmultiply(quatmultiply(quatinv(StParam.StD.Qis'),Qvs),StParam.StD.Qis')';
    AlfaX_D = Qvs_D(2)/C;
    AlfaY_D = Qvs_D(3)/C;
    DeltaQ_D = [sqrt(1-(AlfaX_D/2)^2-(AlfaY_D/2)^2); -AlfaY_D/2; AlfaX_D/2; 0];
    StParam.StD.Qis = quatmultiply(StParam.StD.Qis',DeltaQ_D')';
end
R_Sun_last = R_Sun;    
end

function AssAttRef = AssAttRefCal(AssRef)
if AssRef.Ok == 0
    AssAttRef.Ok = 0;
    AssAttRef.Qib = [1;0;0;0];
    AssAttRef.Wbi = [0;0;0];
else
    AssAttRef.Ok = 1;
    AssAttRef.Qib = AssRef.Qib;
    AssAttRef.Wbi = AssRef.Wbi;    
end
end

function GyroAttRef = GyroAttRefCal(GyroRef)
if GyroRef.Ok == 0
    GyroAttRef.Ok = 0;
    GyroAttRef.Qib = [1;0;0;0];
    GyroAttRef.Wbi = [0;0;0];
else
    GyroAttRef.Ok = 1;
    GyroAttRef.Qib = GyroRef.Qib;
    GyroAttRef.Wbi = GyroRef.Wbi;    
end
end

function AssMmAttRef = AssMmAttRefCal(AssMmRef)
if AssMmRef.Ok == 0
    AssMmAttRef.Ok = 0;
    AssMmAttRef.Qib = [1;0;0;0];
    AssMmAttRef.Wbi = [0;0;0];
else
    AssMmAttRef.Ok = 1;
    AssMmAttRef.Qib = AssMmRef.Qib;
    AssMmAttRef.Wbi = AssMmRef.Wbi;    
end
end

function StGyroAttRef = StGyroAttRefCal(StGyroAttRef,StAttRef,GyroRef,t_ctrl)
global Tc_StUse_Enable;
persistent StGyroRefTimer1;
if isempty(StGyroRefTimer1)
    StGyroRefTimer1 = 100000;
end
if Tc_StUse_Enable == 1 && GyroRef.Ok == 1
    if StAttRef.Ok == 1
        StGyroAttRef.Ok = 1;
        StGyroAttRef.Qib = StAttRef.Qib;
        StGyroAttRef.Wbi = GyroRef.Wbi;
        StGyroRefTimer1 = 0;
    elseif StGyroRefTimer1 <= 1800/t_ctrl       % 星敏有故障，陀螺无故障,30min内基准为陀螺四元数积分
        StGyroAttRef.Ok = 1;
        Qib_last = StGyroAttRef.Qib;
        StGyroAttRef.Qib(1) = Qib_last(1) -t_ctrl/2 * (GyroRef.Wbi(1)*Qib_last(2) + GyroRef.Wbi(2)*Qib_last(3) + GyroRef.Wbi(3)*Qib_last(4));
        StGyroAttRef.Qib(2) = Qib_last(2) +t_ctrl/2 * (GyroRef.Wbi(1)*Qib_last(1) + GyroRef.Wbi(3)*Qib_last(3) - GyroRef.Wbi(2)*Qib_last(4));
        StGyroAttRef.Qib(3) = Qib_last(3) +t_ctrl/2 * (GyroRef.Wbi(2)*Qib_last(1) - GyroRef.Wbi(3)*Qib_last(2) + GyroRef.Wbi(1)*Qib_last(4));
        StGyroAttRef.Qib(4) = Qib_last(4) +t_ctrl/2 * (GyroRef.Wbi(3)*Qib_last(1) + GyroRef.Wbi(2)*Qib_last(2) - GyroRef.Wbi(1)*Qib_last(3));
        StGyroAttRef.Qib = quatnormalize(StGyroAttRef.Qib')';
        StGyroAttRef.Wbi = GyroRef.Wbi;
        StGyroRefTimer1 = StGyroRefTimer1+1;
    else
        StGyroAttRef.Ok = 0;
        StGyroAttRef.Qib = [1;0;0;0];
        StGyroAttRef.Wbi = [0;0;0];
        StGyroRefTimer1 = 100000;
    end
else
    StGyroAttRef.Ok = 0;
    StGyroAttRef.Qib = [1;0;0;0];
    StGyroAttRef.Wbi = [0;0;0];
    StGyroRefTimer1 = 100000;
end
end

function AssGyroAttRef = AssGyroAttRefCal(AssGyroAttRef,AssAttRef,GyroRef,t_ctrl)
persistent AssGyroRefTimer1;
if isempty(AssGyroRefTimer1)
    AssGyroRefTimer1 = 100000;
end
if GyroRef.Ok == 1
    if AssAttRef.Ok == 1
        AssGyroAttRef.Ok = 1;
        AssGyroAttRef.Qib = AssAttRef.Qib;
        AssGyroAttRef.Wbi = GyroRef.Wbi;
        AssGyroRefTimer1 = 0;
    elseif AssGyroRefTimer1 <= 1800/t_ctrl       % 星敏有故障，陀螺无故障,30min内基准为陀螺四元数积分
        AssGyroAttRef.Ok = 1;
        Qib_last = AssGyroAttRef.Qib;
        AssGyroAttRef.Qib(1) = Qib_last(1) -t_ctrl/2 * (GyroRef.Wbi(1)*Qib_last(2) + GyroRef.Wbi(2)*Qib_last(3) + GyroRef.Wbi(3)*Qib_last(4));
        AssGyroAttRef.Qib(2) = Qib_last(2) +t_ctrl/2 * (GyroRef.Wbi(1)*Qib_last(1) + GyroRef.Wbi(3)*Qib_last(3) - GyroRef.Wbi(2)*Qib_last(4));
        AssGyroAttRef.Qib(3) = Qib_last(3) +t_ctrl/2 * (GyroRef.Wbi(2)*Qib_last(1) - GyroRef.Wbi(3)*Qib_last(2) + GyroRef.Wbi(1)*Qib_last(4));
        AssGyroAttRef.Qib(4) = Qib_last(4) +t_ctrl/2 * (GyroRef.Wbi(3)*Qib_last(1) + GyroRef.Wbi(2)*Qib_last(2) - GyroRef.Wbi(1)*Qib_last(3));
        AssGyroAttRef.Qib = quatnormalize(AssGyroAttRef.Qib')';
        AssGyroAttRef.Wbi = GyroRef.Wbi;
        AssGyroRefTimer1 = AssGyroRefTimer1+1;
    else
        AssGyroAttRef.Ok = 0;
        AssGyroAttRef.Qib = [1;0;0;0];
        AssGyroAttRef.Wbi = [0;0;0];
        AssGyroRefTimer1 = 100000;
    end
else
    AssGyroAttRef.Ok = 0;
    AssGyroAttRef.Qib = [1;0;0;0];
    AssGyroAttRef.Wbi = [0;0;0];
    AssGyroRefTimer1 = 100000;
end

end

function AssMmGyroAttRef = AssMmGyroAttRefCal(AssMmGyroAttRef,AssMmAttRef,GyroRef,t_ctrl)
persistent AssMmGyroRefTimer1;
if isempty(AssMmGyroRefTimer1)
    AssMmGyroRefTimer1 = 100000;
end
if GyroRef.Ok == 1
    if AssMmAttRef.Ok == 1
        AssMmGyroAttRef.Ok = 1;
        AssMmGyroAttRef.Qib = AssMmAttRef.Qib;
        AssMmGyroAttRef.Wbi = GyroRef.Wbi;
        AssMmGyroRefTimer1 = 0;
    elseif AssMmGyroRefTimer1 <= 1800/t_ctrl       % 星敏有故障，陀螺无故障,30min内基准为陀螺四元数积分
        AssMmGyroAttRef.Ok = 1;
        Qib_last = AssMmGyroAttRef.Qib;
        AssMmGyroAttRef.Qib(1) = Qib_last(1) -t_ctrl/2 * (GyroRef.Wbi(1)*Qib_last(2) + GyroRef.Wbi(2)*Qib_last(3) + GyroRef.Wbi(3)*Qib_last(4));
        AssMmGyroAttRef.Qib(2) = Qib_last(2) +t_ctrl/2 * (GyroRef.Wbi(1)*Qib_last(1) + GyroRef.Wbi(3)*Qib_last(3) - GyroRef.Wbi(2)*Qib_last(4));
        AssMmGyroAttRef.Qib(3) = Qib_last(3) +t_ctrl/2 * (GyroRef.Wbi(2)*Qib_last(1) - GyroRef.Wbi(3)*Qib_last(2) + GyroRef.Wbi(1)*Qib_last(4));
        AssMmGyroAttRef.Qib(4) = Qib_last(4) +t_ctrl/2 * (GyroRef.Wbi(3)*Qib_last(1) + GyroRef.Wbi(2)*Qib_last(2) - GyroRef.Wbi(1)*Qib_last(3));        
        AssMmGyroAttRef.Wbi = GyroRef.Wbi;
        AssMmGyroRefTimer1 = AssMmGyroRefTimer1+1;
    else
        AssMmGyroAttRef.Ok = 0;
        AssMmGyroAttRef.Qib = [1;0;0;0];
        AssMmGyroAttRef.Wbi = [0;0;0];
        AssMmGyroRefTimer1 = 100000;
    end
else
    AssMmGyroAttRef.Ok = 0;
    AssMmGyroAttRef.Qib = [1;0;0;0];
    AssMmGyroAttRef.Wbi = [0;0;0];
    AssMmGyroRefTimer1 = 100000;
end
end

function AttRef = AttRefCal(AttRef,AlgthmChoiceFlag,StAttRef,AssAttRef,GyroAttRef,AssMmAttRef,StGyroAttRef,AssGyroAttRef,AssMmGyroAttRef,NominalCSYS,OrbitElement)
global Tc_AttDeterChoiceIndex1 Tc_AttDeterChoiceIndex2 Tc_AttDeterChoiceIndex3;
Qin = NominalCSYS.Qin;
Qio = OrbitElement.Qio;
Qiz0 = NominalCSYS.Qiz0;
Wzi_z0 = NominalCSYS.Wzi_z0;
p = struct('Ok',0,'Qib',[1;0;0;0],'Wbi',[0;0;0]);
if AlgthmChoiceFlag.AttRefFlag == 1100  %准则一
    for i = 1: length(Tc_AttDeterChoiceIndex1)
        switch(Tc_AttDeterChoiceIndex1{i})
            case 'StGyro'
                p = StGyroAttRef;
            case 'St'
                p = StAttRef;
            case 'AssGyro'
                p = AssGyroAttRef;
            case 'Ass'
                p = AssAttRef;
            case 'Gyro'
                p = GyroAttRef;
            case 'AssMmGyro'
                p = AssMmGyroAttRef;
            case 'AssMm'
                p = AssMmAttRef;
            otherwise                
        end
        if p.Ok == 1
            AttRef.Qib = p.Qib;
            AttRef.Wbi = p.Wbi;
            AttRef.Ok = 1;
            AttRef.Choice = Tc_AttDeterChoiceIndex1{i};
            break;
        end
    end
elseif AlgthmChoiceFlag.AttRefFlag == 2200 %准则二
    for i = 1:length(Tc_AttDeterChoiceIndex2)
        switch(Tc_AttDeterChoiceIndex2{i})
            case 'StGyro'
                p = StGyroAttRef;
            case 'St'
                p = StAttRef;
            case 'AssGyro'
                p = AssGyroAttRef;
            case 'Ass'
                p = AssAttRef;
            case 'Gyro'
                p = GyroAttRef;
            case 'AssMmGyro'
                p = AssMmGyroAttRef;
            case 'AssMm'
                p = AssMmAttRef;
            otherwise                
        end
        if p.Ok == 1
            AttRef.Qib = p.Qib;
            AttRef.Wbi = p.Wbi;
            AttRef.Ok = 1;
            AttRef.Choice = Tc_AttDeterChoiceIndex2{i};
            break;
        end
    end    
elseif  AlgthmChoiceFlag.AttRefFlag == 3300 %准则三
    for i = 1:length(Tc_AttDeterChoiceIndex3)
        switch(Tc_AttDeterChoiceIndex3{i})
            case 'StGyro'
                p = StGyroAttRef;
            case 'St'
                p = StAttRef;
            case 'AssGyro'
                p = AssGyroAttRef;
            case 'Ass'
                p = AssAttRef;
            case 'Gyro'
                p = GyroAttRef;
            case 'AssMmGyro'
                p = AssMmGyroAttRef;
            case 'AssMm'
                p = AssMmAttRef;
            otherwise                
        end
        if p.Ok == 1
            AttRef.Qib = p.Qib;
            AttRef.Wbi = p.Wbi;
            AttRef.Ok = 1;
            AttRef.Choice = Tc_AttDeterChoiceIndex3{i};
            break;
        end
    end   
elseif  AlgthmChoiceFlag.AttRefFlag == 5511  %强选星敏陀螺基准
    AttRef.Ok = StGyroAttRef.Ok;
    AttRef.Qib = StGyroAttRef.Qib;
    AttRef.Wbi = StGyroAttRef.Wbi;
    AttRef.Choice = 'StGyro';
elseif AlgthmChoiceFlag.AttRefFlag == 5522  %强选单星敏基准
    AttRef.Ok = StAttRef.Ok;
    AttRef.Qib = StAttRef.Qib;
    AttRef.Wbi = StAttRef.Wbi;
    AttRef.Choice = 'St';
elseif AlgthmChoiceFlag.AttRefFlag == 5533  %强选太敏陀螺基准
    AttRef.Ok = AssGyroAttRef.Ok;
    AttRef.Qib = AssGyroAttRef.Qib;
    AttRef.Wbi = AssGyroAttRef.Wbi;
    AttRef.Choice = 'AssGyro';    
elseif AlgthmChoiceFlag.AttRefFlag == 5544  %强选单太敏基准
    AttRef.Ok = AssAttRef.Ok;
    AttRef.Qib = AssAttRef.Qib;
    AttRef.Wbi = AssAttRef.Wbi;
    AttRef.Choice = 'Ass';       
elseif AlgthmChoiceFlag.AttRefFlag == 5555  %强选单陀螺基准
    AttRef.Ok = GyroAttRef.Ok;
    AttRef.Qib = GyroAttRef.Qib;
    AttRef.Wbi = GyroAttRef.Wbi;
    AttRef.Choice = 'Gyro';
elseif AlgthmChoiceFlag.AttRefFlag == 5566  %强选双矢量陀螺基准
    AttRef.Ok = AssMmGyroAttRef.Ok;
    AttRef.Qib = AssMmGyroAttRef.Qib;
    AttRef.Wbi = AssMmGyroAttRef.Wbi;
    AttRef.Choice = 'AssMmGyro';    
elseif AlgthmChoiceFlag.AttRefFlag == 5577  %强选双矢量基准
    AttRef.Ok = AssMmAttRef.Ok;
    AttRef.Qib = AssMmAttRef.Qib;
    AttRef.Wbi = AssMmAttRef.Wbi;
    AttRef.Choice = 'AssMm';  
else
    AttRef.Ok = 0;
    AttRef.Qib = [1;0;0;0];
    AttRef.Wbi = [0;0;0];
    AttRef.Choice = '0';      
end
AttRef.Qob = quatmultiply(quatinv(Qio'),AttRef.Qib')';
AttRef.Qnb = quatmultiply(quatinv(Qin'),AttRef.Qib')';
AttRef.Qz0b = quatmultiply(quatinv(Qiz0'),AttRef.Qib')';
AttRef.Wbz0_b = AttRef.Wbi - quat2dcm(AttRef.Qz0b')*Wzi_z0;
end

%% 磁场确定
% function [MagRef, MmRef] = MagDeter(MagRef,MmParam,t_ctrl,RunCount)
% global Tc_MagDump_TimeshareCnt Tc_Bdot_TimeshareCnt;
% Bb_last = MagRef.Bb;
% Bb_Timeshare_last = MagRef.Bb_Timeshare;
% dBb_Timeshare_last = MagRef.dBb_Timeshare;
% 
% MagRef.Ok = 1;
% MagRef.Bb = MmParam.MmA.Abm*MmParam.MmA.Bm*1e-4;
% MagRef.dBb = (MagRef.Bb - Bb_last)/t_ctrl;
% % 计算分时磁场
% if sum(Tc_MagDump_TimeshareCnt) == 0
%     MagRef.Bb_Timeshare = MagRef.Bb;
% else
%     cnt1 = mod(RunCount,sum(Tc_MagDump_TimeshareCnt));
%     if cnt1 == 1
%         MagRef.Bb_Timeshare = MagRef.Bb;
%     else
%         MagRef.Bb_Timeshare = Bb_Timeshare_last;
%     end
% end
% % 计算分时磁场微分
% if Tc_Bdot_TimeshareCnt == 0
%     MagRef.dBb_Timeshare = MagRef.dBb;
% else
%     cnt2 = mod(RunCount,Tc_Bdot_TimeshareCnt);
%     if cnt2 == 0
%         MagRef.dBb_Timeshare = MagRef.dBb;
%     else
%         MagRef.dBb_Timeshare = dBb_Timeshare_last;
%     end
% end
% % 磁场基准选择(磁场优选)
% MagRef.Bb_Ref = MagRef.Bb_Timeshare;
% end
function [MagRef, MmRef] = MagDeter(AlgthmChoiceFlag,OrbitElement,AttRef,MagRef,StRef,MmRef,MmParam,t_ctrl,RunCount)
global Tc_MagDump_TimeSliceCnt Tc_Bdot_TimeSliceCnt Tc_MagEquUse_Enable;
MagRefFlag = AlgthmChoiceFlag.MagRefFlag;
% 磁强计基准
Mm_Bb_last = MmRef.Bb;
MmRef.Ok = 1;
MmRef.Bb = MmParam.MmA.Abm*MmParam.MmA.Bm*1e-4;%输入GS，转为T
MmRef.dBb = (MmRef.Bb - Mm_Bb_last)/t_ctrl;

MmRef_Bb_Filter_last = MmRef.Bb_Filter;
MmRef.Bb_Filter = Filter(MmRef_Bb_Filter_last,MmRef.Bb,MmRef.tao,t_ctrl);
MmRef.dBb_Filter = (MmRef.Bb_Filter - MmRef_Bb_Filter_last)/t_ctrl;

Bb_TimeSlice_last = MmRef.Bb_TimeSlice;
dBb_TimeSlice_last = MmRef.dBb_TimeSlice;

% 计算分时磁场
if sum(Tc_MagDump_TimeSliceCnt) == 0 % 磁卸载分时
    MmRef.Bb_TimeSlice = MmRef.Bb;
else
    cnt1 = mod(RunCount,sum(Tc_MagDump_TimeSliceCnt));
    if cnt1 == 1
        MmRef.Bb_TimeSlice = MmRef.Bb;
    else
        MmRef.Bb_TimeSlice = Bb_TimeSlice_last;
    end
end
% 计算分时磁场微分
if Tc_Bdot_TimeSliceCnt == 0 % Bdot分时
    MmRef.dBb_TimeSlice = MmRef.dBb_Filter;
else
    cnt2 = mod(RunCount,Tc_Bdot_TimeSliceCnt);
    if cnt2 == 0
        MmRef.dBb_TimeSlice = MmRef.dBb_Filter;
    else
        MmRef.dBb_TimeSlice = dBb_TimeSlice_last;
    end
end

% 磁场基准选择(磁场优选)
MagRef.Bo_MagEqu = MagCal(OrbitElement.RV_f)*1e-4;   %T,磁场公式计算的轨道系磁场
MagRef.Bb_MagEqu = quat2dcm(AttRef.Qob')*MagRef.Bo_MagEqu;
if MagRefFlag == 1100     % 优选选磁强计
    if MmRef.Ok == 1
        MagRef.Ok = 1;
        MagRef.Bb = MmRef.Bb_TimeSlice;
        MagRef.Choice = 1;
    elseif (OrbitElement.Ok == 1)&&(StRef.Ok == 1)&&(Tc_MagEquUse_Enable==1)
        MagRef.Ok = 1;
        MagRef.Bb = MagRef.Bb_MagEqu;
        MagRef.Choice = 2;
    else
        MagRef.Ok = 0;
        MagRef.Bb = [0;0;0];
        MagRef.Choice = 0;
    end
elseif MagRefFlag == 2200     % 优选磁场公式
    if (OrbitElement.Ok == 1)&&(StRef.Ok == 1)&&(Tc_MagEquUse_Enable==1)
        MagRef.Ok = 1;
        MagRef.Bb = MagRef.Bb_MagEqu;
        MagRef.Choice = 2;
    elseif MmRef.Ok == 1
        MagRef.Ok = 1;
        MagRef.Bb = MmRef.Bb_TimeSlice;
        MagRef.Choice = 1;
    else
        MagRef.Ok = 0;
        MagRef.Bb = [0;0;0];
        MagRef.Choice = 0;
    end
elseif MagRefFlag == 0011  % 强选磁强计
    if MmRef.Ok == 1
        MagRef.Ok = 1;
        MagRef.Bb = MmRef.Bb_TimeSlice;
        MagRef.Choice = 1;
    else
        MagRef.Ok = 0;
        MagRef.Bb = [0;0;0];
        MagRef.Choice = 0;
    end
elseif MagRefFlag == 0022  % 强选磁场公式
    if (OrbitElement.Ok == 1)&&(StRef.Ok == 1)&&(Tc_MagEquUse_Enable==1)
        MagRef.Ok = 1;
        MagRef.Bb = MagRef.Bb_MagEqu;
        MagRef.Choice = 2;
    else
        MagRef.Ok = 0;
        MagRef.Bb = [0;0;0];
        MagRef.Choice = 0;
    end
else
    MagRef.Ok = 0;
    MagRef.Bb = [0;0;0];
    MagRef.Choice = 0;
end


end

%% 
function Att4Ctrl = ZZ_Att4Ctrl(AlgthmChoiceFlag,AttRef,NominalCSYS,RotateAglRateTgt,Rsun)
Qin = NominalCSYS.Qin;
Qiz0 = NominalCSYS.Qiz0;
R_T = quat2dcm(quatmultiply(quatinv(Qiz0'),Qin'))*[-1;0;0];  %本体系下的对日轴
Wni_n = NominalCSYS.Wni_n;
Att4CtrlAlgthm = AlgthmChoiceFlag.Att4CtrlAlgthm;
Att4Ctrl = struct('Ok',0,'Qnb',[1;0;0;0],'Agl',[0;0;0],'Wbn',[0;0;0]);

Qib = AttRef.Qib;
Rb = quat2dcm(Qib')*Rsun;    %本体系下的太阳矢量
if norm(cross(Rb,R_T)) >= 1e-6
    Agl_Vector = cross(Rb,R_T)/norm(cross(Rb,R_T))*acos(Rb'*R_T);
else
    Agl_Vector = [0;0;0];
end
Wbi_b = AttRef.Wbi;
Att4Ctrl.Qnb = quatmultiply(quatinv(Qin'),Qib')';
Att4Ctrl.Wbn = Wbi_b-quat2dcm(Att4Ctrl.Qnb')*Wni_n;
Qnb = Att4Ctrl.Qnb; 

Att4Ctrl.Ok = AttRef.Ok;
if AttRef.Ok == 1
    for i = 1:3
        if Att4CtrlAlgthm(i) == 1  % 四元数角度
            Att4Ctrl.Agl(i) = 2*sign(Qnb(1))*Qnb(i+1);
            Att4Ctrl.Wbn(i) = Att4Ctrl.Wbn(i);
        elseif Att4CtrlAlgthm(i) == 2  % 矢量角  LC 没写还
            Att4Ctrl.Agl(i) = Agl_Vector(i);
            Att4Ctrl.Wbn(i) = Att4Ctrl.Wbn(i);
        elseif Att4CtrlAlgthm(i) == 3  % 角速度方式
            Att4Ctrl.Agl(i) = 0;
            Att4Ctrl.Wbn(i) = Att4Ctrl.Wbn(i)-RotateAglRateTgt(i);
        else
            Att4Ctrl.Agl(i) = 0;
            Att4Ctrl.Wbn(i) = 0;
        end
    end
else
    Att4Ctrl.Agl = [0;0;0];
    Att4Ctrl.Wbn = [0;0;0];
end
end

%%
function [FwSpd, Tw, Hw3, Tw3, Mt_M, M3] = ZZ_AttCtrl...
    (Att4Ctrl,NominalCSYS,MagRef,MmRef,AlgthmChoiceFlag,SadaParam,I,MagCtrlCoeff,Td,t_ctrl,RunCount)
% Agl4Ctrl = [10;20;30]/180*pi;
% W4Ctrl = [0.2;0.3;0.4]/180*pi;
Agl4Ctrl = Att4Ctrl.Agl;
W4Ctrl = Att4Ctrl.Wbn;
FwCtrlAlgthm = AlgthmChoiceFlag.FwCtrlAlgthm;
MagCtrlAlgthm = AlgthmChoiceFlag.MagCtrlAlgthm;
% FwCtrlAlgthm轮控算法选择标志1~5对应PD/PID/PDT/PIDT/DT
% SatShape   卫星构型标志，0-ShakFin;1-OpenBook

persistent Mt_M_last;
if isempty(Mt_M_last)
    Mt_M_last = [0;0;0;0;0;0];
end
% 惯量计算

I3 = [I(1,1);I(2,2);I(3,3)];

% 计算控制参数
% PIDT_Coeff = struct('lamda',[0;0;0],'xi',[0;0;0],'ki',[0;0;0],'w_ui_max',[0;0;0]);
PIDT_Param = struct('Kp',[0;0;0],'Ki',[0;0;0],'Kd',[0;0;0],'Kt',[0;0;0],'Ui_max',[0;0;0]);

global FwParam MtParam FwCtrlCoeff NotchFiterCoeff Tc_MtUse_Enable Tc_NotchFilter_Enable Tc_AglAccFdFwd_Enable ...
    Tc_TdFdFwd_Enable ;%力矩前馈开关:光压/气动/重力梯度/磁控/陀螺/SADA驱动;
% PD_Param PID_Param PDT_Param PIDT_Param DT_Patam
for i = 1:3
%     CoeffIdx = ['Coeff' num2str(i)];
    switch FwCtrlAlgthm(i,1)
        case 1
            PIDT_Coeff = eval(['FwCtrlCoeff.PDCoeff.Coeff' num2str(FwCtrlAlgthm(i,2))]);
        case 2
            PIDT_Coeff = eval(['FwCtrlCoeff.PIDCoeff.Coeff' num2str(FwCtrlAlgthm(i,2))]);
        case 3
            PIDT_Coeff = eval(['FwCtrlCoeff.PDTCoeff.Coeff' num2str(FwCtrlAlgthm(i,2))]);
        case 4
            PIDT_Coeff = eval(['FwCtrlCoeff.PIDTCoeff.Coeff' num2str(FwCtrlAlgthm(i,2))]);
        case 5
            PIDT_Coeff = eval(['FwCtrlCoeff.DTCoeff.Coeff' num2str(FwCtrlAlgthm(i,2))]);
    end
    switch FwCtrlAlgthm(i,1)
        case {1,2}
            PIDT_Param.Kp(i) = I3(i)*PIDT_Coeff.lamda(i)^2;
            PIDT_Param.Ki(i) = PIDT_Coeff.ki(i)*PIDT_Param.Kp(i);
            PIDT_Param.Kd(i) = 2*I3(i)*PIDT_Coeff.lamda(i)*PIDT_Coeff.xi(i);
            PIDT_Param.Kt(i) = 0;
            PIDT_Param.Ui_max(i) = pi/180*PIDT_Coeff.w_ui_max(i)*PIDT_Param.Kd(i);
        case {3,4}
            PIDT_Param.Kp(i) = I3(i)*PIDT_Coeff.lamda(i)^2/3;
            PIDT_Param.Ki(i) = PIDT_Coeff.ki(i)*PIDT_Param.Kp(i);
            PIDT_Param.Kd(i) = I3(i)*PIDT_Coeff.lamda(i);
            PIDT_Param.Kt(i) = 1/3/PIDT_Coeff.lamda(i);
            PIDT_Param.Ui_max(i) = pi/180*PIDT_Coeff.w_ui_max(i)*PIDT_Param.Kd(i);
        case 5
            PIDT_Param.Kp(i) = 0;
            PIDT_Param.Ki(i) = 0;
            PIDT_Param.Kd(i) = I3(i)*PIDT_Coeff.lamda(i)/PIDT_Coeff.xi(i)/2;
%             PIDT_Param.Kt(i) = 1;
            PIDT_Param.Kt(i) = 1/2/PIDT_Coeff.lamda(i)/PIDT_Coeff.xi(i);
%             PIDT_Param.Kd(i) = I3(i)*PIDT_Coeff.lamda(i)/PIDT_Coeff.xi(i);
%             PIDT_Param.Kt(i) = 1/4/PIDT_Coeff.lamda(i)/PIDT_Coeff.xi(i);
%             PIDT_Param.Kd(i) = 2*I3(i)*PIDT_Coeff.lamda(i)*PIDT_Coeff.xi(i);
%             PIDT_Param.Kt(i) = 3;
%             PIDT_Param.Kd(i) = I3(i)*PIDT_Coeff.lamda(i);
%             PIDT_Param.Kt(i) = 1/3/PIDT_Coeff.lamda(i);
            PIDT_Param.Ui_max(i) = 0;
    end
end

if FwCtrlAlgthm(1,1)==0&&FwCtrlAlgthm(2,1)==0&&FwCtrlAlgthm(3,1)==0  %%%  LC ???x轴Kp参数是0时怎么处理
    Tw3 = [0;0;0];
    Tw = [0;0;0;0;0;0];
    Hw = [0;0;0;0;0;0];
else
    Agl_lim = [0;0;0];
    Agl4Ctrl_lim = [0;0;0];
    for i = 1:3
        if PIDT_Param.Kp(i) == 0
            Agl_lim(i) = 0;
        else
            Agl_lim(i) = PIDT_Coeff.w_max(i)/180*pi*PIDT_Param.Kd(i)/PIDT_Param.Kp(i);
        end
        Agl4Ctrl_lim(i) = Limit_sameratio(Agl4Ctrl(i),Agl_lim(i));
    end
    Tw3 = [0;0;0];
    Tw3(1) = Torque_PIDT(PIDT_Param,t_ctrl,Agl4Ctrl_lim,W4Ctrl,1);
    Tw3(2) = Torque_PIDT(PIDT_Param,t_ctrl,Agl4Ctrl_lim,W4Ctrl,2);
    Tw3(3) = Torque_PIDT(PIDT_Param,t_ctrl,Agl4Ctrl_lim,W4Ctrl,3);
    
    Tw3 = Tw3 + Tc_TdFdFwd_Enable(1)*Td.Ts+Tc_TdFdFwd_Enable(2)*Td.Ta+Tc_TdFdFwd_Enable(3)*Td.Tg+...
        Tc_TdFdFwd_Enable(4)*cross(MtParam.Abt * Mt_M_last,MagRef.Bb)+...
        Tc_TdFdFwd_Enable(5)*Td.To+Tc_TdFdFwd_Enable(6)*Td.Tp;
    Ani_b = quat2dcm(Att4Ctrl.Qnb') * NominalCSYS.Ani_n;
    Tw3 = Tw3 - Tc_AglAccFdFwd_Enable * I * Ani_b; % 标称系的角加速度前馈

    if Tc_NotchFilter_Enable == 1
        Tw3 = NotchFilter(Tw3,NotchFiterCoeff,t_ctrl);        
    end
    [Tw, Hw] = FwSpdAllocation(FwParam, FwCtrlCoeff, Tw3, t_ctrl);
end



% Tw = FwParam.Abf'/(FwParam.Abf*FwParam.Abf')*Tw3;
% Tw = Limit_sameratio(Tw,FwParam.Tmax);
% Hw = Hw_last + Tw*t_ctrl;
% Hw = Limit_sameratio(Hw,FwParam.Hmax);

FwSpd = Hw/FwParam.Jw*30/pi;

Hw3 = FwParam.Abf * Hw;
H3 = Hw3 + I * Att4Ctrl.Wbn; %%% LC 自旋的时候

M3 = MagCtrlAlgorithm(MagRef,MmRef,MagCtrlAlgthm,Att4Ctrl,H3,MagCtrlCoeff,RunCount);
if sum(Tc_MtUse_Enable) > 0  %强选
    Mt_M = (MtParam.AAA * M3).*Tc_MtUse_Enable;
else
    Mt_M = MtParam.AAA * M3;
end
Mt_M(1) = Limit_max1(Mt_M(1),MtParam.M(1));
Mt_M(2) = Limit_max1(Mt_M(2),MtParam.M(2));
Mt_M(3) = Limit_max1(Mt_M(3),MtParam.M(3));
Mt_M(4) = Limit_max1(Mt_M(4),MtParam.M(4));
Mt_M(5) = Limit_max1(Mt_M(4),MtParam.M(5));
Mt_M(6) = Limit_max1(Mt_M(4),MtParam.M(6));
Mt_M_last = Mt_M;
end

%%
function [Tw, Hw] = FwSpdAllocation(FwParam, FwCtrlCoeff, Tw3, t_ctrl)
global Tc_FwUse_Enable;
persistent Hw_last Fw_Choice_last;
if isempty(Hw_last)
    Hw_last = [0;0;0;0;0;0];
    Fw_Choice_last = [0;0;0;0;0;0];
end

%飞轮重构Priority:1-正常；2-性能下降
Fw_Abf = zeros(3,6);
Fw_Choice = [0;0;0;0;0;0];
if sum(Tc_FwUse_Enable) > 0  % 强选
    Fw_Choice = Tc_FwUse_Enable;
    for i = 1:6
        if Tc_FwUse_Enable(i) == 1
            Fw_Abf(:,i) = FwParam.Abf(:,i);
        end
    end
else
    for i = 1:6
        if FwParam.Ok(i) == 1 && FwParam.Priority(i) == 1 % 性能正常
            Fw_Choice(i) = 1;
            Fw_Abf(:,i) = FwParam.Abf(:,i);
        end
    end
    if rank(Fw_Abf) < 3
        for i = 1:6
            if FwParam.Ok(i) == 1 && FwParam.Priority(i) == 2 %性能下降
                Fw_Choice(i) = 1;
                Fw_Abf(:,i) = FwParam.Abf(:,i);
                if rank(Fw_Abf) == 3
                    break;
                end
            end
        end
    end
end
if rank(Fw_Abf) < 3
    Fw_Choice = [0;0;0;0;0;0];
    Fw_Abf = zeros(3,6);
end

%平衡转速计算
%置首个可用飞轮的静态转速为StcSpd，然后将这个动量按所有可用飞轮来分配，找到每个轮子的静态动量
%然后将可用飞轮的静态动量同比例扩大到StcSpd
if (sum(Fw_Choice) == 0)
    StcSpdEnable = 0;
    Hw0 = [0;0;0;0;0;0];
else
    ID = find(Fw_Choice,1);% 第一个在用的飞轮编号
    Ht = [0;0;0;0;0;0];
    Ht(ID) =FwCtrlCoeff.StaticSpd /30*pi*FwParam.Jw;
    Hw0 = Fw_Abf'/(Fw_Abf*Fw_Abf')*(Fw_Abf*Ht)-Ht;
    if (abs(Hw0(ID)) > 1e-6)
        StcSpdEnable = 1;
        Hw0 = Hw0*(FwParam.Jw*FwCtrlCoeff.StaticSpd/30*pi/Hw0(ID));
    else
        StcSpdEnable = 0;
        Hw0 = [0;0;0;0;0;0];
    end
end

% 分配
% 首拍处理
if FwParam.Mode == 1%力矩模式
    Hw_last = FwParam.FwSpd/30*pi*FwParam.Jw;
end
if Fw_Choice ~= Fw_Choice_last
    Hw_last = FwParam.FwSpd/30*pi*FwParam.Jw.*FwParam.Ok;
end

if (sum(Fw_Choice) ~= 0)
    Tw = Fw_Abf'/(Fw_Abf*Fw_Abf')*Tw3;
    Tw = Limit_sameratio(Tw,FwParam.Tmax);
    DeltaHw =  Tw*t_ctrl;
    if (StcSpdEnable)
        %动态分配：力矩用满则不分配平衡转速
        if (min(FwParam.Tmax) - max(abs(Tw))) > 1e-6
            DeltaHwStc = 0.01*(Hw0 - Hw_last);  %防止力矩太大
            DeltaHwStc = (eye(6)-Fw_Abf'/(Fw_Abf*Fw_Abf')*Fw_Abf)*DeltaHwStc;   %why
            TwStc = DeltaHwStc/t_ctrl;
            Tw = Tw+TwStc;
            DeltaHw = DeltaHw+DeltaHwStc;
        end
    end
else
    DeltaHw = [0;0;0;0;0;0];
end
Hw = Hw_last + DeltaHw;
Hw = Limit_sameratio(Hw,FwParam.Hmax);
Hw = Hw.*Fw_Choice;
Hw_last = Hw;
Fw_Choice_last = Fw_Choice;
end

%% 
function T = Torque_PIDT(FwCtrlParam,t_ctrl,Agl4Ctrl_lim,W4Ctrl,Axis)
persistent Tc Ui;
if isempty(Tc)
    Tc = [0;0;0];
end
if isempty(Ui)
    Ui = [0;0;0];
end
%     Tc = [0;0;0];
Ui(Axis) = Ui(Axis) + FwCtrlParam.Ki(Axis)*Agl4Ctrl_lim(Axis)*t_ctrl;
Ui(Axis) = Limit_max1(Ui(Axis),FwCtrlParam.Ui_max(Axis));
Tc(Axis) = ((FwCtrlParam.Kp(Axis)*Agl4Ctrl_lim(Axis)+FwCtrlParam.Kd(Axis)*W4Ctrl(Axis)...
    +Ui(Axis))*t_ctrl+FwCtrlParam.Kt(Axis)*Tc(Axis))/(t_ctrl+FwCtrlParam.Kt(Axis));
T = Tc(Axis);
end

function T_out = NotchFilter(T_in,NotchFiterCoeff,t_ctrl) 
persistent T_in1 T_in2 T_out1 T_out2;
if isempty(T_in1)
    T_in1 = T_in;
end
if isempty(T_in2)
    T_in2 = T_in;
end
if isempty(T_out1)
    T_out1 = T_in;
end
if isempty(T_out2)
    T_out2 = T_in;
end
lamda = NotchFiterCoeff.lamda;   % 陷波滤波器的带宽
h = NotchFiterCoeff.h;           % 陷波滤波器的深度
f = NotchFiterCoeff.f;           % 太阳翼模态频率

tw2 = (t_ctrl*2*pi*f)^2;
a2 = 4-2*h*lamda*t_ctrl+tw2;
a1 = -8+2*tw2;
a0 = 4+2*h*lamda*t_ctrl+tw2;
b2 = 4-2*lamda*t_ctrl+tw2;
b1 = -8+2*tw2;
b0 = 4+2*lamda*t_ctrl+tw2;
T_out = (-b2*T_out2-b1*T_out1+a2*T_in2+a1*T_in1+a0*T_in)/b0;
T_out2 = T_out1;
T_out1 = T_out;
T_in2 = T_in1;
T_in1 = T_in;
end

%% 磁控算法选择
function MagCmd = MagCtrlAlgorithm(MagRef,MmRef,MagCtrlAlgthm,Att4Ctrl,H3,MagCtrlCoeff,RunCount)
global Tc_MagDump_TimeSliceCnt;

if MagRef.Ok == 1
    if MagCtrlAlgthm(1) == 0
        MagCmd = [0;0;0];
    elseif MagCtrlAlgthm(1) == 1   % 角速度负反馈法
        if MmRef.Ok == 1
            MagCmd = MagCtrlCoeff.WFdbk*cross(MmRef.Bb,Att4Ctrl.Wbn);
        else
            MagCmd = [0;0;0];
        end
%         if (mod(RunTimer,20)<2.5) && (mod(RunTimer,20)>0.5)  %  分时控制方案20190520
%             MagCmd = [0;0;0];
%             Mag10_First = 1;
%         else
%             if Mag10_First == 1
%                 Mag_TD = Mag_sys;
%                 Mag10_First =0;
%             end
%             MagCmd = -cross(Mag_TD,K_MagDamp*W4Ctrl);
%         end
%         Norm1 = norm(Mag_sys);    %  变系数角速度负反馈法
%         if Norm1 ~= 0
%             MagCmd = -1000 .* cross(Mag_sys,W4Ctrl) / Norm1^2;
%         else
%             MagCmd = [0;0;0];
%         end
    elseif MagCtrlAlgthm(1) == 2   % Bdot控制算法
        if MmRef.Ok == 1
            MagCmd = MagCtrlCoeff.Bdot * MmRef.dBb_TimeSlice;
        else
            MagCmd = [0;0;0];
        end
    elseif MagCtrlAlgthm == 3   % 磁卸载算法
        if MagRef.Ok == 1
            Norm1 = norm(MagRef.Bb);
            if Norm1 ~= 0
                if sum(Tc_MagDump_TimeSliceCnt) == 0
                    MagCmd = MagCtrlCoeff.MagDump .* cross(MagRef.Bb,H3) / Norm1^2;
                else
                    cnt = mod(RunCount,sum(Tc_MagDump_TimeSliceCnt));
                    if cnt < Tc_MagDump_TimeSliceCnt(1)
                        MagCmd = MagCtrlCoeff.MagDump .* cross(MagRef.Bb,H3) / Norm1^2;
                    else
                        MagCmd = [0;0;0];
                    end
                end
            else
                MagCmd = [0;0;0];
            end
        else
            MagCmd = [0;0;0];
        end
    elseif MagCtrlAlgthm ==4    % 三轴磁控稳定算法
        Vomg = -Att4Ctrl.Wbn'*(vx(MagRef.Bb))'*vx(MagRef.Bb)*(K_MagDamp*Att4Ctrl.Wbn);
        Vksi = -Att4Ctrl.Wbn'*(vx(MagRef.Bb))'*vx(MagRef.Bb)*(K_MagDamp/1000*(Att4Ctrl.Agl));
        if abs(Vomg)<Vksi
            k = abs(Vomg)/2/Vksi;
        else
            k = 1;
        end
        MagCmd = -MagCtrlCoeff.MagDump*cross(MagRef.Bb,Att4Ctrl.Wbn)-1*k*MagCtrlCoeff.MagDump/1000*cross(MagRef.Bb,(Att4Ctrl.Agl));
    else
        error('MagCtrlAlgorithmError!');
    end
else
    MagCmd = [0;0;0];
end

end

%% 轨道系磁场计算
function Mag_VVLH = MagCal(RV_f)
[~, ~, ~, L_Geocentric, B_Geocentric,~] = RV2LBH(RV_f);
Long = L_Geocentric;% 算磁场用地心经纬度
theta=pi/2-B_Geocentric;
x=RV_f(1);
y=RV_f(2);
z=RV_f(3);
vx=RV_f(4);
vy=RV_f(5);
vz=RV_f(6);
r=norm([x,y,z]);

EARTH_RADIUS_M=6371230;
re_r=EARTH_RADIUS_M/r;
re_r3=re_r*re_r*re_r;
re_r4=re_r3*re_r;
re_r5=re_r4*re_r;
sin_theta=sin(theta);
cos_theta=cos(theta);
sin_2theta=sin(2.0*theta);
cos_2theta=cos(2.0*theta);
sin_lambdal=sin(Long);
cos_lambdal=cos(Long);
sin_2lambdal=sin(2.0*Long);
sin_3lambdal=sin(3.0*Long);
cos_2lambdal=cos(2.0*Long);
cos_3lambdal=cos(3.0*Long);
sin2_theta=sin_theta*sin_theta;
sin3_theta=sin2_theta*sin_theta;
cos2_theta=cos_theta*cos_theta;
cos3_theta=cos2_theta*cos_theta;
G10=-29404.8;G11=-1450.9;H11=4652.5;
G20=-2499.6;G21=2982.0;H21=-2991.6;G22=1677.0;H22=-734.6;
G30=1363.2;G31=-2381.2;H31=-82.1;G32=1236.2;H32=241.9;
G33=525.7;H33=-543.4;
SQRT3=sqrt(3);
HALFSQRT3=sqrt(3)/2;
SQRT6D4=sqrt(6)/4;
HALFSQRT15=sqrt(15)/2;
SQRT10M3D4=sqrt(10)*3/4;
SQRT10D4=sqrt(10)/4;
re3_f64=(G10*(-sin_theta)+(G11*cos_lambdal+H11*sin_lambdal)*cos_theta);
re4_f64=(G20*(-3.0*sin_theta*cos_theta)+...
(G21*cos_lambdal+H21*sin_lambdal)*(SQRT3*cos_2theta)+...
(G22*cos_2lambdal+H22*sin_2lambdal)*(HALFSQRT3*sin_2theta));
re5_f64=(G30*(1.5*sin_theta*(1.0-5.0*cos2_theta))+...
(G31*cos_lambdal+H31*sin_lambdal)*(SQRT6D4*cos_theta*(4.0-15.0*sin2_theta))+...
(G32*cos_2lambdal+H32*sin_2lambdal)*(HALFSQRT15*sin_theta*(3.0*cos2_theta-1.0))+...
(G33*cos_3lambdal+H33*sin_3lambdal)*(SQRT10M3D4*sin2_theta*cos_theta));
bn=re_r3*re3_f64+re_r4*re4_f64+re_r5*re5_f64;
re3_f64=((G11*sin_lambdal-H11*cos_lambdal)*sin_theta);
re4_f64=((G21*sin_lambdal-H21*cos_lambdal)*(HALFSQRT3*sin_2theta)+...
2.0*(G22*sin_2lambdal-H22*cos_2lambdal)*(HALFSQRT3*sin2_theta));
re5_f64=((G31*sin_lambdal-H31*cos_lambdal)*(SQRT6D4*sin_theta*(5.0*cos2_theta-1.0))+...
2.0*(G32*sin_2lambdal-H32*cos_2lambdal)*(HALFSQRT15*sin2_theta*cos_theta)+...
3.0*(G33*sin_3lambdal-H33*cos_3lambdal)*(SQRT10D4*sin3_theta));

Mag_VVLH = [0;0;0];
if sin_theta~=0.0
    be=1.0/sin_theta*(re_r3*re3_f64+re_r4*re4_f64+re_r5*re5_f64);
    re3_f64=(G10*cos_theta+(G11*cos_lambdal+H11*sin_lambdal)*sin_theta);
    re4_f64=(G20*(1.5*cos2_theta-0.5)+...
    (G21*cos_lambdal+H21*sin_lambdal)*(HALFSQRT3*sin_2theta)+...
    (G22*cos_2lambdal+H22*sin_2lambdal)*(HALFSQRT3*sin2_theta));
    re5_f64=(G30*(2.5*cos3_theta-1.5*cos_theta)+...
    (G31*cos_lambdal+H31*sin_lambdal)*(SQRT6D4*sin_theta*(5.0*cos2_theta-1.0))+...
    (G32*cos_2lambdal+H32*sin_2lambdal)*(HALFSQRT15*sin2_theta*cos_theta)+...
    (G33*cos_3lambdal+H33*sin_3lambdal)*(SQRT10D4*sin3_theta));
    bg=-2.0*re_r3*re3_f64-3.0*re_r4*re4_f64-4.0*re_r5*re5_f64;
    
    
    vn=(-z*(x*vx+y*vy)+(x*x+y*y)*vz)/r/sqrt(x*x+y*y);
    ve=(-y*vx+x*vy)/sqrt(x*x+y*y);
    vh=norm([vn,ve]);
    vn=vn/vh;
    ve=ve/vh;
    aon_m33=[vn ve 0;-ve vn 0;0 0 1];
    Mag_NEG=[bn;be;bg]*1.0e-5; % Gs
    Mag_VVLH=aon_m33*Mag_NEG;
end


end

%% 84系下位置矢量计算地理经纬高、地心经纬高
function [L_Geodetic, B_Geodetic, H_Geodetic, L_Geocentric, B_Geocentric,H_Geocentric] = RV2LBH(RV_f)

X = RV_f(1);
Y = RV_f(2);
Z = RV_f(3);
a = 6378137;
b = 6356752;
e2 = (a^2-b^2)/a^2;
ep2 = (a^2-b^2)/b^2;
theta = atan(Z*a/sqrt(X^2+Y^2)/b);

L_Geodetic = atan2(Y,X);
B_Geodetic = atan((Z+ep2*b*(sin(theta))^3)/(sqrt(X^2+Y^2)-e2*a*(cos(theta))^3));

N = a/sqrt(1-e2*sin(B_Geodetic)^2);
H_Geodetic = sqrt(X^2+Y^2)/cos(B_Geodetic)-N;

L_Geocentric = L_Geodetic;
B_Geocentric = atan(Z/sqrt(X^2+Y^2));
H_Geocentric = sqrt(X^2+Y^2+Z^2);

end

function y = Filter(y_last,u,tao,T_ctrl)
y = (T_ctrl*u+tao*y_last)/(T_ctrl+tao);
end

function y = QuatRectify(Q)  %四元数标部取正
    if Q(1) < 0 
        Q(1) = -Q(1);
        Q(2) = -Q(2);
        Q(3) = -Q(3);
        Q(4) = -Q(4);
    end
    y = Q;
end

function y = Vec2Quat(V1,V2)  %通过两个三轴向量获取旋转四元数
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
Q  = [cos(theta/2);sin(theta/2)*V3/norm(V3)];
y = Q;

end


function y = Limit_max1(u,Ymax)
Flag = abs(u)>Ymax;%超出阈值的量
y = (1-Flag).*u+sign(u).*Flag*Ymax;
end

function y = Limit_sameratio(u,Ymax)
if max(abs(u))<=Ymax
    y=u;
else
    y = u/max(abs(u))*Ymax;
end
end

function y = xw(r)
y = [0 -r(3) r(2);r(3) 0 -r(1);-r(2) r(1) 0];
end