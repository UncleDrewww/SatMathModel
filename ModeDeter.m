function ModeDeter()
global Mode SubMode index Mode_last SubMode_last index_last  SunDirFace_last SunDirFace;
global AttRef_Ok Agl4Ctrl W4Ctrl  ;
global AttRefFlag MagRefFlag FwCtrlFlag W_Target Att4CtrlRefFlag MagCtrlFlag;
global Allow_SADAWTCtrl_Tc ;


DegP5 = 0.5/180*pi;
% Deg1 = 1/180*pi;

Deg5 = 5/180*pi;

persistent ModeDeter3Timer1 ModeDeter3Timer2 ModeDeter3Timer3;
[ModeDeter3Timer1, ModeDeter3Timer2, ModeDeter3Timer3] = PersistentInit(ModeDeter3Timer1,ModeDeter3Timer2,ModeDeter3Timer3);

global ModeChange SubModeChange;
Mode_last = Mode;SubMode_last = SubMode; % 确认注数改index与last赋值的先后关系--LC
if Mode == 3    % 地球捕获及指向模式
    if (ModeChange == 1) || (SubModeChange == 1)
        ModeDeter3Timer1 = 0;ModeDeter3Timer2 = 0;ModeDeter3Timer3 = 0;
    end
    if SubMode == 1        % 阶段1--地球捕获
        ModeDeter3Timer1 = Cnt(ModeDeter3Timer1,(AttRef_Ok==1)&&(abs(Agl4Ctrl(1))<Deg5)&&(abs(Agl4Ctrl(2))<Deg5)&&(abs(Agl4Ctrl(3))<Deg5)...
            &&(abs(W4Ctrl(1))<DegP5)&&(abs(W4Ctrl(2))<DegP5)&&(abs(W4Ctrl(3))<DegP5));
        if (ModeDeter3Timer1 >= 5*2)
            SubMode = 2;
        end
    elseif SubMode == 2        % 阶段2--地球指向
        if (index(1)~=index_last(1))||(index(2)~=index_last(2))
            SubMode = 1;   % 转阶段1
        end        
    else
        error('ModeDeter4Error');
    end
elseif Mode == 0
    
else
    error('ModeDeterError');
end
if Mode ~= Mode_last
    ModeChange = 1;
else
    ModeChange = 0;
end
if SubMode ~= SubMode_last
    SubModeChange = 1;
else
    SubModeChange = 0;
end

%% 模式初始化设置
SunDirFace_last = SunDirFace;
global indexForce MagRefForce AttRefForce Att4CtrlRefForce Allow_KalmanCal Allow_KalmanUse;
global AttRefChoice DesNodeTimeArea;
global  FwCtrlForce W_TargetForce MagCtrlForce;

if Mode == 0
    Allow_SADAWTCtrl_Tc = [0;0];
    index = [1;SunDirFace];         % 惯性系012345：-X+X-Y+Y-Z+Z对日
    AttRefFlag = 5555;        % 强选单陀螺
    MagRefFlag = 0011;        % 强选磁强计
    MagCtrlFlag = 0;   % 1-角速度负反馈;2-Bdot
    FwCtrlFlag = [0;0;0];
    W_Target = [0;0;0];
    Att4CtrlRefFlag = [3;3;3];
    Allow_KalmanCal = 0;
    Allow_KalmanUse = 0;
elseif Mode == 3
    AttRefFlag = 5511;         % 强选星敏+陀螺
    MagRefFlag = 0011;         % 强选磁强计
    FwCtrlFlag = [4;4;4];
    Att4CtrlRefFlag = [1;1;1];
    W_Target = [0;0;0];
    MagCtrlFlag = 3;           % 磁卸载
else
    error('ModeInitialError');
end
% 强选基准
if (indexForce(1) ~= 9)&&(indexForce(2) ~= 9)
    index = indexForce;
end
if AttRefForce ~= 9
    AttRefFlag = AttRefForce;
end
if MagRefForce ~= 9
    MagRefFlag = MagRefForce;
end
if Att4CtrlRefForce ~= 9
    Att4CtrlRefFlag = [Att4CtrlRefForce;Att4CtrlRefForce;Att4CtrlRefForce];
end

% 强选控制算法
if (FwCtrlForce(1) ~= 9) && (FwCtrlForce(2) ~= 9) && (FwCtrlForce(3) ~= 9)
    FwCtrlFlag = FwCtrlForce;
    W_Target = W_TargetForce;
end
if MagCtrlForce ~= 9
    MagCtrlFlag = MagCtrlForce;
end

function Timer = Cnt(Timer,V)
if V > 0.5
    Timer = Timer + 1;
else
    Timer = 0;
end

function varargout = PersistentInit(varargin)
varargout = cell(nargin,1);
for i = 1:nargin
    if isempty(varargin{i})
        varargout{i}=0;% 或者varargout(i)={0};
    else
        varargout{i} = varargin{i};%或者varargout(i) = varargin(i);
    end
end