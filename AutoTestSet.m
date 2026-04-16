function AutoTestSet(T_sys)


global Mode SubMode Allow_StUse  ;
global FwCtrlForce indexForce;

persistent First;
if isempty(First)
    First = 1;
end
% disp(1)
if First == 1
    First = 3;
    Mode = 3;
    SubMode = 1;
    Allow_StUse = 1;
    FwCtrlForce = [9;9;9];%强选轮控算法，默认[9；9]
    indexForce=[9;9];%强选坐标系，默认[9；9]
else
    if T_sys>=1000
        FwCtrlForce = [11;11;11];   %快速机动
    end
end

    
