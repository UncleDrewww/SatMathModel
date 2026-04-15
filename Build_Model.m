clear all
%% 自动生成dll脚本文件，只需运行该文件即可实现转换
global SatClass    % 卫星类别，用来区分批产型号的不同状态，通过SatNum执行对应初始化文件。
sat_num = 1;         % 一次编译的卫星个数,例如宏图1+3，则是4颗星
if exist('slprj', 'dir') ~= 0   %编译前清空存在的临时文件slprj，有时slprj会导致编译报错
    rmdir('slprj', 's')  
end
%%
for buildcnt = 1 : sat_num
    SatClass = buildcnt;
    Initial
    slbuild('GS_model');
    slbuild('ProjectModel');
    filename = strcat('dll_file', num2str(buildcnt));
    if exist(filename, 'dir') ~= 0
        rmdir(filename, 's')  % 删除所有dll_file文件夹
    end
    mkdir (filename)
    
    cd (filename)
    movefile ../GS_model_niVeriStand_rtw/*.dll
    movefile ../ProjectModel_niVeriStand_rtw/*.dll
    cd ../
    
    delete *.autosave
    rmdir GS_model_niVeriStand_rtw 's'
    rmdir ProjectModel_niVeriStand_rtw 's'
    rmdir slprj 's'
end
clear all;
close_system('GS_model.slx',0);   %关闭内存中的模型，不保存
close_system('ProjectModel.slx',0);   %关闭内存中的模型，不保存
delete('ProjectModel.slxc');
delete('GS_model.slxc');
disp('build success')

