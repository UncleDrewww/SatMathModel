function AutoTestSet(T_sys)
Choice = 0;
% 0-test
% 1-成像模式
% 2-凝视
% 3-俯仰渐进


global Mode SubMode Allow_StUse  ;
global FwCtrlForce indexForce;
global ImagingMode;
global Band_1 Band_2 Band_3;
global index;

persistent First;
if isempty(First)
    First = 1;
end
% disp(1)
if Choice == 0
    if First == 1
        First = 3;
        Mode = 3;
        SubMode = 1;
        Allow_StUse = 1;
%         FwCtrlForce = [13;13;13];%强选轮控算法，默认[9；9]
        FwCtrlForce = [9;9;9];%强选轮控算法，默认[9；9]
        indexForce=[9;9];%强选坐标系，默认[9；9]
    end
elseif Choice == 1
%% 成像模式仿真
    if First == 1
        Mode = 3;
        SubMode = 1;
        Allow_StUse = 1;
        FwCtrlForce = [11;11;11];
        First = 0;
        ImagingMode = 1;
        Band_1 = [0;300;350];Band_2=[30;450;500];Band_3=[30;1600;1700];
%         Band_1 = [20;130;5000];Band_2=[0;0;0];Band_3=[0;0;0];
    end
    if (T_sys<1001) && (T_sys>=1000)
        
    end
    
end