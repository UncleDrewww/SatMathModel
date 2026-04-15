% 创建图形界面
fig = uifigure('Position', [100, 100, 400, 250], 'Name', '数学仿真遥控');

% 创建标签
lblSelect = uilabel(fig, 'Position', [50, 180, 120, 30], 'Text', '选择变量', 'FontSize', 12);

% 创建下拉框
dropDown = uidropdown(fig, 'Position', [180, 180, 150, 30], 'Items', {'控制模式','标称系工况','飞轮强选标识','磁棒强选标识','SADA控制开关'...
'星敏选择顺序','陀螺选择顺序','姿态基准强选','电机闭环控制方式','磁卸载分时系数'}, 'FontSize', 12);

% 创建标签
lblValue = uilabel(fig, 'Position', [50, 120, 120, 30], 'Text', '输入', 'FontSize', 12);

% 创建文本框
txtValue = uieditfield(fig, 'text', 'Position', [180, 120, 150, 30], 'FontSize', 12);

% 创建发送按钮
btnSend = uibutton(fig, 'Position', [150, 50, 100, 40], 'Text', '发送', 'FontSize', 12);

% 按钮回调函数
btnSend.ButtonPushedFcn = @(src, event) updateVariable(dropDown, txtValue);

function updateVariable(dropDown, txtValue)
    % 获取选择的变量名和输入的值
    selectedVar = dropDown.Value;

%     % 判断输入值是否为有效数字
%     if isnan(newValue)
%         msgbox('请输入有效的数字！', '错误', 'error');
%         return;
%     end
    
    % 根据下拉框选择修改对应的变量
    global LcMark Tc_Index Tc_FwUse_Enable Tc_MtUse_Enable Tc_SadaCtrl_Enable Tc_StChoiceIndex Tc_GyroChoiceIndex;
    global Tc_AttDeterChoiceIndex1 Tc_AttDeterChoiceIndex2 Tc_AttDeterChoiceIndex3 Tc_SadaCtrlMethod Tc_MagDump_TimeSliceCnt;
    switch selectedVar
        case '控制模式'
            LcMark.Mode = str2num(txtValue.Value);
        case '标称系工况'
            Tc_Index = str2num(txtValue.Value);
        case '飞轮强选标识'
            Tc_FwUse_Enable = str2num(txtValue.Value);
        case '磁棒强选标识'
            Tc_MtUse_Enable = str2num(txtValue.Value);
        case 'SADA控制开关'
            Tc_SadaCtrl_Enable = str2num(txtValue.Value);
        case '星敏选择顺序'
            Tc_StChoiceIndex = strsplit(txtValue.Value,',');
        case '陀螺选择顺序'
            Tc_GyroChoiceIndex = strsplit(txtValue.Value,',');
        case '姿态基准强选'
            Tc_AttDeterChoiceIndex1 = strsplit(txtValue.Value,',');
            Tc_AttDeterChoiceIndex2 = strsplit(txtValue.Value,',');
            Tc_AttDeterChoiceIndex3 = strsplit(txtValue.Value,',');
        case '电机闭环控制方式'
            Tc_SadaCtrlMethod = str2num(txtValue.Value);
        case '磁卸载分时系数'
            Tc_MagDump_TimeSliceCnt = str2num(txtValue.Value);
        otherwise
            disp('选择了无效的变量');
    end
    
end