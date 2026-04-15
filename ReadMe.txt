待办：
1) GNSS采集频率没做
2) 磁棒上升时间只有一个值
3) 画图加自定义时间范围

20251029
1) 前馈用重力梯度力矩符号修正
2）明确表格中所有惯量积的定义方法

20250905
1)  QV目标角度计算用姿态基准的Qob，不用动力学的Qob；
2)  增加磁场公式计算，双矢量定姿的轨道系磁场改用磁场公式计算的结果；

20250625 修复SADA前馈bug
SADA前馈力矩缺项；
前馈用角加速度计算未考虑限幅后的角速度；

20250617
1) SADA前馈之前只有单翼，已补充为双翼，但是前馈效果不好，还需要排查

20250514 Init/SystemApp/project
1) 实现F107系数插值
2) 数学仿真模型电机初始化序列中角加速度限幅给大值

20250513 ProjectModel/User_Att/SatelliteParamters
1) 惯量计算方式修改
   收拢态：含全部收拢组件的固定惯量，附件收拢态惯量赋0，也支持拼接计算的方式。
   展开态/单翼展开：星体惯量+附件惯量拼接。

20250331 Satellite Parameters
1) 调整卫星编号的位置。
2) 增加单机数量给C动力学适配。
3) 移除磁棒分配矩阵，在软件参数中填写，修改磁棒分配矩阵命名。

20250321 封装tag1.0
1）Initial去掉NI端口相关处理
2）ProjectModel：输入端口重新排序，方便读取输入Bus。
3）BuildModel：编译完成后，关闭内存中的模型，不保存，删除slxc缓存文件。

20250314 ZZ_DataPlot
1) ZZ_Dataplot提供单图模式和子图模式
2) Initial中通过函数的形式创建SatProperty Bus，摒弃bus editor创建。
3) FlexibleDynamics函数还需要手动设置NumMode1、NumMode2参数，还没解决...

20250311 Initial读取卫星设计参数表格，使输入受控。
1) 由于修改了挠性的阶数，因此projectmodel中的向量维数以及satparam中的bus需要修改。
2) 卫星参数结构体中，删除不必要的本体安装矩阵和安装点，对应UserAtt中计算前馈用也修改。
3）Initial中，规范了一些参数的名称，对应GSmodel中调用的参数名称也修改。

20250307 增加QV控制（V0.2）
1) QV仿真用法：地面站经纬度、工作时间、控制开关。
2) ZZ_QV：目标角度计算、控制角度计算、目标角速度计算、跟踪完归零。
3) 遥测：QV目标角度、指令角速度模式。
4) ProjectModel：修正SADA、QV初始化指令部分bug。移除PI控制部分到系统APP。
5) SADAB轴控制方式改为闭环，增量没有角加速度限幅。

20250224  SADA电机个数扩展为6个
1）ZZ_User_Sada：目标角度、指令模式扩展为6维，展开电机默认为待机模式。
2）ZZ_Sada：所有计算用向量维数扩展。
3）ZZ_SystemApp：加入读取展开电机角度、3322初始化电机角度、电机驱动指令、遥测等对应维数扩展、SadaParam结构体加入展开电机（Sada3）。
4）Initial：初始化文件Sada相关参数初始化为6×1向量。
5）ProjectModel：驱动器循环个数改为6个，遥测加入展开电机相关（真实角度、指令模式、角速度等），展开电机角度没有引入姿态动力学模型中。

20241218
1) Initial：处理NI端口时的open操作改为load_system，不需要重新打开模型，因此去掉AutoNIPortSet变量；
            T_Ctrl：编译模型时，运行频率为0.02s（50Hz），因此数学仿真时如果为4Hz控制，编译时会报错，配置成两组。
            自动处理ProjectModel时删除处理NI模块的代码
2）ProjectModel：删除所有NI端口

20241213
1) Initial：SADA控制参数遥控默认PI控制；
   分时参数默认关；
   B轴加速度限幅从100到0.02；
2）SystemAPP：删除全局变量SadaWorkEnd，增加全局变量SadaModeParam；
   增加全局变量Tc_PLJCtrl_Enable初始化；
3）ProjectModel：优化遥测布局；驱动器不设角加速度限幅。
4）ZZ_ATS:新增
5）ZZ_Att:修正敏感器无效时，Wbi赋值错误；
   飞轮无效时，指令角动量赋0；
6）ZZ_AutoTestSet:删除多余代码；
7）UserAtt：Bdot阈值单位改成T；
   SADA模式相关计时、阶段字和模式相关计时在一个地方清，否则会导致清不掉。
   阻尼完成后置动力学展开标志位展开；
8）UserSada：SADA相关计数、阶段字不作为局部变量，作为全局变量在UserAtt中管理
   SADA控制算法取遥控值。

20241209
1) 增加飞轮、磁棒强选功能Tc_MtUse_Enable（Att、System）
2）修改遥测顺序（模式、控制用姿态提前）、增加30个预留遥测（System、Project）
3) 默认分时拍数改成无（Initial）

20241203
1) 修正磁基准选择时，选分时后磁场。
2) 增加磁棒上升时间功能，磁强计基准遥测修改。

20241119 Xia
1)解算SADA目标角速度时，不再考虑帆板展开以及允许控制标志。
2)加入编译脚本，后续编译通过编译脚本编译。
3)初始化文件修改clear all，代替为更加精准的clearvars

20241114 Xia
1)电机驱动器部分，接收霍尔无效指令从12/14改为11/13。

20241113 Xia
1)增加遥控变量：Tc_Aligned_and_Constrained
2)User_Att增加约束对齐姿态，可配置对齐轴、约束轴，在对日中根据遥控变量选择控制算法。
3)修正干扰力矩计算公式：帆板形心计算时缺少ap向量

20241021 Xia
1)ProjectModel中的Motor模块里面位置门限的功能勘误，正向==负向时不启用，错判成了正向==正向
2)Initial文件中加一个IsOpenModel的开关，免得每次仿真都打开重新打开模型麻烦
3)Initial文件中，明确SADA霍尔触发步数SADA_Agl_HL1应当是RotUint系的步数，即应当是单机方按照他们的定义给过来的步数，与数管初始化序列设置的值相同

20241014 Xia
GS_model/Tc/SADA2 极性转换去掉

20240929 Xia
1) GNSS简单模块，动力学机箱时间对齐验证：1天半差了1s
2) 模型自动处理开关：数学仿真的时候可以不处理


20240924 Xia
1) 光行差补偿（Att）
2）姿态基准相关遥测（Show、ProjectModel）
3）增加惯性系太阳矢量（非单位向量）（System）

20240923 Xia
1）姿态基准（七种）（Att、Initial）
2）遥控设置姿态基准选择规则（三种）
3）姿态基准强选功能（Att）
2）增加双矢量定姿（轨道系磁场没用磁场公式）

20240919 Xia
1) 增加DT控制，DT控制增加控制角度限幅除0判断（Att）
2）增加单太敏解算太阳矢量、单太敏解算姿态四元数、角速度（Att）
3）增加太敏相关配置的初始化（System）

20240913 Xia
1）初始化文件增加gitHash值读取、卫星代号（Initial）
2）使用简化版GNSS递推模型（Gsmodel）

20240912 Xia
1) 增加陀螺姿态确定

20240910 Xia
1) 星敏、陀螺、磁强计、飞轮单机数据传到系统APP里：扩充至全部数据(SystemAPP、ProjectModel)
2）数学仿真，遥控初值：星敏噪声系数置成1、遮挡计算开关置成改（GSmodel）
3）增加星敏选择顺序Tc_StChoiceIndex（Initial)，要做成遥控可以切换
4) 增加双矢量计算结果StAC、StBC、StAB结构体用于星敏基准选择（System）
5）增加星敏定姿算法StAttCal：根据选择顺序选基准（ZZ_Att）
6）增加双星敏定姿算法（ZZ_Att）

20240817  LC
1）Initial文件，星敏C/D安装矩阵改为为行向量，并增加备注1*4

20240816  LC
1）仿真函数：ZZ_User_Att中SADA耦合系数Rsa计算勘误，+Y翼的计算多判了-Y翼的展开状态，删除。
2）仿真函数：ZZ_User_Att中干扰力矩Tg计算勘误。

20240807 LC
1）仿真函数：MagCtrlCoeff中磁卸载变量名MagDamp更正为MagDump

20240805 LC
1) 仿真函数：凝视目标点的经纬高设置合并到Tc_StareCtrl_LLA中，相应更改NSYS中使用经纬高的地方。
2）把轮控、磁控、分时等可能改的参数从SystemApp转移到Initial文件中
3) Init文件中增加FwMode用于选择力矩模式或转速模式
4）ProjectModel里面驱动模型中，动力学设置霍尔常高/常低的位置往前移，否则这个霍尔故障在归零模式下不起作用。

20240801 LC
1)仿真函数：ModeDeter给出的FwCtrlCoeff由之前的仅选择PD/PID/PDT/PIDT/DT算法,改为选择算法+选择第几组参数，用于比如诺亚展开前用一组高带宽参数，展开后用一组低带宽参数。
2)仿真函审：FwCtrlCoeff中增加最大机动角速度，这样以后就方便以后在用户函数中改动

20240730 LC
1）仿真函数：EnvirParam里面增加Beta角，并传给ZZ_Sada函数
2）仿真函数：ZZ_User_Att中重力梯度力矩计算勘误

20240729 LC
凝视算法勘误：四元数Qif使用时少了一个列向量转行向量的转置。

20240722 完成初版
ZZ_函数中把用户自定义部分独立出来，形成ZZ_User_Att、ZZ_User_Sada两部分
ZZ_User_Att中包含ModeDeterAndInit、NominalCSYS(包含对地、对日、凝视、零多普勒、偏流角)、SatParamCal、TdCal
ZZ_User_Sada中包含NominalCSYS、ModeDeterAndInit
