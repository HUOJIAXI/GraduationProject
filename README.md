# GraduationProject 项目日志 
## 经过测试，对于13*13环境下，16个机器人的环境，对于设计的三种方法（动态算法、传统IP模型，改进优化单行线法则）对于相同的起点终点分别进行了测试，测试结果如下：

## 1. 测试时间：
动态算法：236秒 

传统IP模型：1954秒  

改进优化单行线法则：48秒

## 2. 测试效果：
动态算法：存在死锁，并且会出现无法解决的冲突，只能够解决一部分的冲突，总路径不具备最优性，并且对环境的适应性不高。总路径长度：287

传统IP模型：能够解决所有的冲突，并且能够达到该环境下的最优总路径，但是求解时间太长。最优路径：132

改进优化单行线法则：能够解决所有的冲突，求解时间很短，是三种算法中最短的。但是无法达到该环境下的最优解。最优路径：171

## 3. 结论：
动态算法的求解时间可以接受，但是算法的稳定性不高，并且求解所得路径太长，由于死锁的存在，总路径不具备最优性。

传统IP模型的稳定性很高，对环境有极高的适应性，并且能够得到最优的路径，在相同的环境下，路径长度是三个算法中最短的，但是求解时间过长，性价比比较低。

改进优化单行线法则的求解速度很快，并且稳定性很高，适应性比较高，但是只能保证在单行线法则的要求下的最优总路径长度。

## 4. 可视化测试：

1. 改进优化单行线法则：

![images](https://github.com/HUOJIAXI/GraduationProject/blob/master/For_result/simulation_fortest_heu.gif)

2. 传统IP模型：

![images](https://github.com/HUOJIAXI/GraduationProject/blob/master/For_result/simulation_fortest.gif)

3. 动态算法：

![images](https://github.com/HUOJIAXI/GraduationProject/blob/master/For_result/simulation_fortest_dyn.gif)


## 得益于启发式初始解算法，在13*17规模下的最大求解机器人个数已经达到36个，并且仍然可以继续提高

![images](https://github.com/HUOJIAXI/GraduationProject/blob/master/ForPre/36_heu.jpg)

绕行方向

![images](https://github.com/HUOJIAXI/GraduationProject/blob/master/ForPre/dir_36.jpg)

## 作为对比组，我们按照传统的IP模型对多机器人环境进行了建模，对与障碍物之间的冲突、机器人之间的直接冲突以及机器人之间的交叉冲突提高了约束的解决方案，通过该传统方法得到的解是没有冲突的在该环境下的最优解。求解时间比单行线优化模型会长很多，对于9机器人下的9*13的拓扑环境，需要600多秒的时间求解。

![images](https://github.com/HUOJIAXI/GraduationProject/blob/master/res_tradition/tradition_9.jpg)

## 而对于单行道优化模型下的13*17的环境，传统方法的求解时间会达到6000多秒以上。而通过对比结果，我们可以发现，在相同的起点终点下，单行线优化模型的最短路径总长度为133，传统方法的最优路径为109，单行线优化模型的最短路径会比传统方法略差，但是求解时间仅需要7秒，效率比传统方法会高很多。

![images](https://github.com/HUOJIAXI/GraduationProject/blob/master/res_tradition/simulation_tradition_way.gif)

## 对传统IP模型的目标函数进行了改进，将直线距离改为折现距离（abs(y1-y2)+abs(x1-x2)），求解时间会缩短，但是求解时间仍然较长，在13*17的环境下九机器人求解，需要381秒，而对于相同的环境，优化单行道模型仅需要7秒。对于最优路径，传统方法求解所得为92，而优化单行线法则下为115。相较而言，优化单行道法则的性价比更高

![images](https://github.com/HUOJIAXI/GraduationProject/blob/master/res_tradition/simulation_fortest_trad_ops.gif)

## 连续性路径演示：(机器人最终进入货架)
## 1. 单行道方法（25机器人，13*17仓库环境）

![images](https://github.com/HUOJIAXI/GraduationProject/blob/master/ForPre/ONE_WAY/simulation_fortest25_ops.gif)

![images](https://github.com/HUOJIAXI/GraduationProject/blob/master/ForPre/ONE_WAY/simulation_fortest25_ind.jpg)

## 2. 动态启发式分割方法（16机器人，13*13仓库环境）

![images](https://github.com/HUOJIAXI/GraduationProject/blob/master/ForPre/simulation_dyn_pre.gif)

# 进入测试阶段：
## 优化测试1: 控制环境大小，增加机器人数量，对每个机器人个数求解10次测试集中随机选取的起点终点对，作出误差条，yalmip求解器在运行前需要清空存储变量，降低求解时间，在调用清空方法之后，线性度更好

对每组测试集进行十次测试后可以得到更加平衡的解：

![images](https://github.com/HUOJIAXI/GraduationProject/blob/master/test_keep_map_size/heu_start_15_10.jpg)

## 优化测试2：控制机器人个数（为了同时确保提高测试的普遍性和可行性，将机器人个数定为10），将拓扑图不断扩大，从9X9扩展到19X19，并且每个环境规模从从测试集中取10组进行测试，得出每个环境规模的平均求解时间，作出误差图如下。

![images](https://github.com/HUOJIAXI/GraduationProject/blob/master/ForPre/keep_rob_1919.jpg)

## 优化测试3:控制机器人的个数以及环境大小，随机对机器人群中的某一台机器人的起点或终点进行十次扰动，在启发式初始解启用的情况下，运行时间会出现一定程度的浮动，但是扰动后模型的求解时间会比扰动前的求解时间短：

![images](https://github.com/HUOJIAXI/GraduationProject/blob/master/ForPre/small_disturb_10_11_11.jpg)

![images](https://github.com/HUOJIAXI/GraduationProject/blob/master/ForPre/small_disturb_10_9_9.jpg)


## 测试项目3: 随机对机器人群中的一个进行扰动，比如起点或终点偏移一个单位，通过由扰动前所得到的初始解，可以降低求解的难度，降低求解时间，因此系统对小扰动的容忍度很高，时间浮动在5s以内。

![images](https://github.com/HUOJIAXI/GraduationProject/blob/master/res_test/disturb_test1.jpg)

![images](https://github.com/HUOJIAXI/GraduationProject/blob/master/res_test/disturb_test2.jpg)

## 测试项目4：系统可能会由于一台机器人故障而减少一台机器人，测试结果求解时间会大大降低。

![images](https://github.com/HUOJIAXI/GraduationProject/blob/master/res_test/reduce_one_rob.jpg)


# Version 8.1 削减式动态算法最终版(具有在货架下躲避冲突的功能)
# Version final 最终效果：

![images](https://github.com/HUOJIAXI/GraduationProject/blob/master/results/simulation_16ROB_Versionfinal.gif)

# 当前进行：单行线法则版本 V2 3.0
## 仿真平台：Intel I7 RAM 16G - Matlab for mac
## 到目前为止的效果：  

![images](https://github.com/HUOJIAXI/GraduationProject/blob/master/Result_one_way_V3/simulation_16ROB_test.gif)

![images](https://github.com/HUOJIAXI/GraduationProject/blob/master/Result_one_way_V3/simulation_8ROB_V3.gif)

## Version: 1.0 02/02/2020
Author: HUO JIAXI. 

已完成单机器人的IP模型构建，通过调用基于CPLEX的Yalmip线性求解器成功求解最短路径，但是由于CPLEXcommunity版变量个数限制，环境规模被限制在5*5

## Version: 2.0 03/02/2020
Author: HUO JIAXI

调用Gurobi求解器，成功增大模型规模，目前暂时定为12*12，由于gurobi不限制变量个数，有增大模型规模的可能性，但是会造成比较长的延时。  

模型被成功求解，得到单个最短路径  

耗时24.957595秒.  

为多机器人系统搭建环境  

引入按次序执行路径规划的多机器人IP模型，求解成功，并且不会发生冲突，但是空间时间效率低，需要引入同步模型。
耗时为单个机器人运行时间的叠加，并且路径被单个机器人直接占用，未被释放，效率低，需改进。

![images](https://github.com/HUOJIAXI/GraduationProject/blob/master/results/Map12*12.jpg)
![images](https://github.com/HUOJIAXI/GraduationProject/blob/master/results/Result12*12.jpg)  

1-AS ⬆️  

![images](https://github.com/HUOJIAXI/GraduationProject/blob/master/results/3-MAS_12*12.jpg)  

3-MAS ⬆️
求解MAS模型耗时 79.981009s

## 对于优化MAS路径规划的一点想法：  
动态规划，暂时先不考虑启发式算法。
1. 先对每个机器人执行不考虑其他机器人存在的单机器人路径规划，将各机器人的最短路径存储在PATH.mat中，作为优先路径选择集合。
2. 正式开始启动系统，每个机器人按照优先路径行动，（for循环）每次行动存储实际所在位置在temp.mat中（暂定），并判断下一步节点是否与其他机器人优先路径集合中的下一节点重合（以及head-on-head collision需要另外考虑），或是和已经达到目的地的机器人的实际节点重合，若重合，或是停止等待，或是重新调用一次IP_solver求解器，求出新的无撞撞的路径选择，并对所有的机器人执行，直到所有的机器人达到目的节点，得到一个最终的路径cell集合pathcellfinal.matrix。

## Version: 2.1 04/02/2020  
Author: HUO JIAXI  

尝试将系统动态化  

根据昨天的想法，我尝试将想法实例化，目前代码运行正常，但是耗时长久。  

需要初始化路径，即将所有个体机器人在不考虑其他机器人存在的情况下计算出可行的最优路径，作为首选集合存储。而之后则需要考虑到其他机器人的存在，所以我同步运行所有的机器人，每一个时间节点都会判断一次冲突情况，若有冲突，则重新规划路径，而对于重新规划路径而言，则需要更新环境，需要将所有机器人的下一节点作为障碍物，使得发生冲突的机器人不会在更新过后的路径中的下一节点发生冲突。  

最后的temp.matrix矩阵包含了所有机器人的最终目的地。  

之后的任务则是优化现有的模型，想办法引入新的启发式算法，减小耗时。  

![images](https://github.com/HUOJIAXI/GraduationProject/blob/master/results/MAS_syn_base.jpg)

现有模型耗时 80.381949s  

## Version 3.0 05/02/2019  
Author: HUO JIAXI  

3.0版本将求解MAS模型所得到的优化路径动态化，并且作为mp4视频文件作为运行结果存储。  

在调试代码的过程中，发现如果仍然将已经达到目的地的机器人作为障碍物放置在环境中，会造成很大的资源浪费（造成一整条线路无法通行），因此考虑一种情况，到达目的地的机器人会进行捡货等操作，会离开现有环境，因此可以将与已经达到终点的机器人发生碰撞的情况忽略（但是在程序运行结果中需要指出，如下图）。  

在这样的条件下，可以避免一些非必要的堵塞情况。  

为了提高结果的视觉效果，将系统初始的机器人状态定为黄色圆点，达到终点的机器人状态定为绿色圆点，而运行中的机器人定为红色圆点，这样可以提高结果的可读性。  

不足：耗时仍然较长，并且避碰算法仍需要改进，在调试过程中发现head-on-head冲突没有被避免，只能避免对角处的冲突，需要进一步调试代码。  

6台机器人运行耗时：224.504326s    

![images](https://github.com/HUOJIAXI/GraduationProject/blob/master/results/MAS_COLI_BASE.png)

## Version 4.0 06/02/2020  
Author: HUO JIAXI  

4.0版本已经实现多机器人系统的对角冲突以及非交叉冲突，需要进一步调试交叉冲突的躲避方法。  

6台机器人运行耗时：359.223416s  

![images](https://github.com/HUOJIAXI/GraduationProject/blob/master/results/version4.0.png)

## Version 4.1 07/02/2020
Author: HUO JIAXI

4.1版本实现了非交叉冲突的优化设计，并且调试成功了交叉冲突的躲避方法。但是对于终点在某一时刻被包围的情况还是没有得到非常好的优化，容易造成求解器错误，但是以目前的环境密度，4.1版本能够实现7机器人环境的优化路径规划。  

不足：需要优化搜寻方法，效率太低，特别是对于终点被包围的情况，求解器很难求解，需要设置备用终点，备用终点的选取也需要优化。

7台机器人运行耗时：481.073032s  

![images](https://github.com/HUOJIAXI/GraduationProject/blob/master/results/version4.1results.png)

## Version 4.1 08/02/2019
Author: HUO JIAXI  

![images](https://github.com/HUOJIAXI/GraduationProject/blob/master/results/时间分配结果4.1.png)

经过分析，由于原始路径的规划需要对每个机器人进行全图搜寻，导致在原始路径的规划中耗费了非常多的时间，另外对于解决冲突的算法，也对出现冲突的机器人进行全图搜寻（4.0版本需要对每对冲突机器人的每个个体进行全图搜寻，所导致的资源浪费更加严重，4.1版本对其进行优化，只需选定一个机器人进行重新规划即可，冲突即可得到解除）。全图搜寻的效率比较低，调用Gurobi耗费大量的时间，所以对于启发式算法，可以考虑对全图的分割。不进行原始路径的规划，而是对于每个时刻进行逐个阶段规划以及冲突解除，可以根据机器人个体的起始点和终点设定对于每个机器人个体不同的全图分割方式，以缩短模型的状态空间。

## Version 4.2 09/02/2020
Author: HUO JIAXI  

4.2版本将备用终点的启用集成在function文件中，能够在一定程度上降低时间时延，根据仿真结果，能够降低50s左右的延时。  

另外4.2版本实现了对终点被包围但是其附近存在0点的情况下进行备用终点启用的方法。通过不断对备用终点的判断，得到可行的备用终点。但是发现了新的问题：  

现有的IP模型在求解路径失败的情况下无法判断路径被堵塞的情况，导致后续的冲突接触存在困难。在调用路径修改程序时无法准确启用备用终点或是备用起点，起点被包围的情况无法和终点被包围的情况分开。  

7机器人环境运行耗时：432.608s

![images](https://github.com/HUOJIAXI/GraduationProject/blob/master/results/Version4.2.png)

## Version 4.2 10/02/2020  
Author: HUO JIAXI  

考虑到现行版本很高的时延，并且障碍解除不稳定。对于现行版本的优化，考虑先优化避障方法。而较高的时延很大一部分在于发生冲突时求解Modify_IP函数的IP模型所需要的时间很长。  

由于冲突的产生是在两个机器人即将碰撞的时刻发生，因此对于原始最佳路径的修改不需要扩展到整个环境中，可以在原始最佳路径上进行修改。  

考虑一个基本的启发式方法，在规划避障修改路线时不将原最佳路径的终点作为Modify_IP函数的终点，而是将环境图中取出包含机器人实际位置的一正方形部分，将该部分与原最佳路径的靠近原始终点的交点作为Modify_IP函数的终点。原因是修改之后的路径并不会与原最佳路径偏移太多。  

此时求解Modify_IP函数的IP模型所对应的IP模型会大大减小，只需要求解对应的一部分环境。考虑到环境中的障碍物为2x1大小，因此考虑求解修改路径的IP模型环境为以机器人实际位置为中心的7x7的正方形环境。可行性仍有待验证，将可行性分析作为4.3版本进行探究。

## Version 5.0 11/02/2020
Author: HUO JIAXI  

针对昨天提出的想法，将4.2版本进行了优化，尝试将发生冲突的路径矩阵的一部分替换，而非替换所有的部分，还需要进行测试。  

而对于原始路径规划，由于不考虑机器人之间的碰撞，并且是有规律的障碍物，因此原始最优路径不会会在在包含起点终点的一个正方形区域中（可以更小），起点或是终点是该正方形区域的一个顶点，且起点和终点都在正方形区域的边缘。在这种情况下对于动态的规划机器人的原始路径可以避免很大的资源浪费，缩减对于机器人个体的IP模型的状态空间。  

对于相同的环境和机器人，改进后的MAS路径规划IP模型求解总耗时：272.666104s，比4.2版本的432.608s缩短了160秒，节约了三分钟的运行时间，由于还没有对冲突解除算法进行优化，因此仍然有优化的空间。  

![images](https://github.com/HUOJIAXI/GraduationProject/blob/master/results/result_7MAS_op.png)

## Version 5.1 12/02/2020
Author: HUO JIAXI  

5.1版本对冲突解除算法进行了一定程度上的优化，并且经过测试，暂时没有发现问题，模型能够成功被求解，但是根据现在的环境密度，该启发式算法还没有很大的用武之地，在靠近环境边缘发生的冲突并没有启动该启发式算法，因此之后还可以对边缘发生的冲突的启发式解决方法进行一定的优化，在环境中部发生的冲突能够调用该启发式算法，并且能够求解。  

5.1版本进一步验证了对于原始最优路径启发式算法的可行性，并且将环境密度提高，容纳八台机器人。并且优化了补充规划，由于某些机器人在特殊情况下，会在临近终点时发生冲突，因此其在其他所有机器人到达终点后仍然停留在备用终点位置上，由于这汇总情况不常见，因此对于补充算法没有调用冲突解决算法，出现碰撞的可能性不高，为了节约时间，也引入了启发式算法，成功求解模型。  

由于扩大了环境密度，因此发生的冲突变多，求解冲突的情况也会增多，因此求解时间会变长，但是仍然优于未优化前的模型求解时间。  

针对备用终点会造成机器人的等待问题，将在6.0版本中进行解决，目前的想法是将补充算法提前至返回原终点的位置，即在返回原终点的同时，补充返回原点的路径。  

目前对于8机器人的求解时间：391s

![images](https://github.com/HUOJIAXI/GraduationProject/blob/master/results/version5.1.png)

# Version 6.0 13/02/2020
Author: HUO JIAXI

6.0版本完善了启发式算法，并且将MASPP_IP_div中的代码代码模块化。通过代码模块化，可以减少一定的延时。

6.0版本将备用终点会造成机器人等待的问题修复，在机器人规划至备用终点后，会调用启发式路径规划将路径规划回原始终点，这样不但可以绕开被占用的终点，也可以节省运行时间，避免造成等待。

另外将启发式路径修复引入到非交叉冲突中，在7机器人的环境中运行良好，相对于未引入启发式修复算法，仅仅引入了启发式原始路径算法的5.0版本，节省了50s左右的运行时间，仍然有改进的空间，启发式算法所框定的范围还可以更小，子图分割仍然可以更细。

7-机器人：  

![images](https://github.com/HUOJIAXI/GraduationProject/blob/master/results/version6.0_7ROBOT.png)

8-机器人:  

![images](https://github.com/HUOJIAXI/GraduationProject/blob/master/results/version6.0_robot8.png)

## Version 6.1 14/02/2020
Author: HUO JIAXI  

6.1版本将启发式算法的应用进行了扩展，将交叉冲突和非交叉冲突的冲突处理都引入了启发式算法，并且对于通过图分割对模型规模进行缩减的启发式算法进行了进一步的验证，发现能够适用于中部发生的碰撞，对于这样的碰撞，求解器只需要求解一部分的环境，而非整个环境。对于在边角部位发生的碰撞，由于6.1版本的启发式算法还无法应用于边角部位，对于边角部位的冲突，仍然需要求解整个模型，但是依然可以开发不同的启发式算法。从仿真结果中可以看出启动启发式算法的冲突解除耗时远小于未调用启发式算法的冲突。即使如此，6.1版本对于8机器人的环境仍然降低了运算时间，目前未273.193s，比6.0版本减小了68秒的时间。  

对于之后的版本可以考虑引入单行线法则，在原始路径规划阶段避免一些不必要的冲突，减少调用冲突解决算法的次数，这样可以更进一步降低运行时间。  

8-机器人：  

![images](https://github.com/HUOJIAXI/GraduationProject/blob/master/results/result6.1.png)

运行时间：273.193s  
最长机器人路径：18

## Version 6.2 15/02/2020
Author: HUO JIAXI  

6.2版本解决了6.1版本中未解决的在非交叉冲突解除中出现的bug（在备用终点启用后可能导致无法回到原始起点的bug，以及在Modify_Path中出现的若起点终点在同一点时会报错的bug）。并且增加两台机器人，目前能够求解10台机器人，总用时337.43s。  

![images](https://github.com/HUOJIAXI/GraduationProject/blob/master/results/result6.2_10robots.png)

运行时间：337.43s
系统总消耗时刻：19
系统总路程（不包含重复经过的点）：110

![images](https://github.com/HUOJIAXI/GraduationProject/blob/master/results/result6.2_10robot_2.png)

运行时间：390.15s

系统总消耗时刻：20

系统总路程（不包含重复经过的点）：90

## Version 6.3 23/02/2020
Author: HUO JIAXI  

6.3版本通过调试解决了之前版本中一直没有察觉到的一些bug，比如解决非交叉冲突中出现的对启发式解法op_modify_sup中输入参数错误的问题，这在调试中得到了修复。 

另外提高了环境密度，目前环境中有12台机器人，但是可能存在一定的稳定性问题，在冲突密集发生处可能会出现卡死现象，限制了环境密度的升高。  

另外对改进对于边界处发生的冲突的解决方案，初步提供了一个框架，可以在7.0之后的版本中进行实现，尝试降低运行时间。  

![images](https://github.com/HUOJIAXI/GraduationProject/blob/master/results/result6.3_12robots.png)  

运行时间：469.84s

系统总消耗时刻：20

系统总路程（不包含重复经过的点）：102

## Version 7.0 24/02/2020
Author: HUO JIAXI

7.0版本通过拓展环境优化了边界处冲突，将冲突解决时间大大缩短，是优化前的1/2不到，但是运行时间会出现一定的浮动，对于12个机器人的环境，运行时间在200s左右。

但是7.0版本的稳定性不是很高，会出现跳跃的情况，可能需要在求解器中添加约束，需要在之后的版本中加以改进优化。  

![images](https://github.com/HUOJIAXI/GraduationProject/blob/master/results/result7.0_12robots.png)

运行时间：211.37s  

系统总消耗时刻：15

系统总路程：107 

## Version 7.1 25/02/2020
Author: HUO JIAXI

7.1版本改进了7.0版本中出现的跳跃的情况，并且解决了可能会发生的在某点卡死的情况，在13个机器人的环境下，运行时间进一步减少，运行时间为195.06s。  

但是7.1版本还存在着出现子循环的情况，造成了一定的资源浪费，但是最后能够到达最终目的地，还需要在之后的版本中进行改进。

另外在op_modify_path中可能存在重复顶点未去除的bug，需要在后面的版本中加以修复，其他的冲突处理算法运行正常。  

![images](https://github.com/HUOJIAXI/GraduationProject/blob/master/results/result7.1_13robots.png)

## Version 8.0 26/02/2020
Author: HUO JIAXI

8.0版本修复了7.1版本中出现的子循环的情况，并且提高了机器人密度，通过增加密度解决了之前没有发现的一些bug，对于16机器人的环境，能够完成求解。运行时间为257s左右，由于机器人密度的上升，所以发生冲突的情况也更加复杂，因此会修改更多的路径，需要调用不同的启发式方法，但是到目前为止，方法证明是正确的。  

目前的版本支持继续扩展密度。  

14机器人：  

![images](https://github.com/HUOJIAXI/GraduationProject/blob/master/results/Version8.0_14robots.png)  

运行时间：114.46s  

系统总消耗时间：25

系统总路程：96  

16机器人：

![images](https://github.com/HUOJIAXI/GraduationProject/blob/master/results/Version8.0_16robots.png)  

运行时间：257.75s

系统总消耗时间：25

系统总路程：106    

## Version 8.1 削减式算法最终版
Author: HUO JIAXI

8.1版本为削减式优化算法的最终版本，更新了暂停等待的情况下机器人进入货架等待的情况（在一定情况下防止堵塞），并且时间在180s左右（在16台机器人的情况下）。但是削减式优化算法无法解决死锁的问题：可能将可行路径在优化过程中被阻断，导致无法求解。

对于死锁问题，通过了郑老师的讨论，可以尝试新的方法：单行道法则，这是新的方向，因此削减式算法将暂时不再更新，在论文的撰写过程中可作为一个模块，与单行道法则进行对比。  

16个机器人的路径：

![images](https://github.com/HUOJIAXI/GraduationProject/blob/master/results/Vesion8.1.jpg)

# 单行线法则
## Version 1.1 单行线法则1.1版本
Author: HUO JIAXI

单行道规划的1.1版本实现了3X3一障碍物环境的双机器人无碰撞规划，将单机器人整数规划模型扩展为多机器人，并且添加了多个约束，使得在该环境下，环境中的巷道均为单行道形式，但是有些约束不具有通用性，在障碍物增加的情况下可能无法实现求解。

![images](https://github.com/HUOJIAXI/GraduationProject/blob/master/Result_one_way/Version1.1_one_way.jpg)

## Version 1.2
Author: HUO JIAXI

1.2版本提高了通用性，改变了方向的表达方式，由1.1版本的1和-1改为1和2，以方便巷道方向的框定：每个巷道点对于所有机器人（目前为2）的方向之和不等于3，即机器人在同一巷道中不可反向。并且通过增加机器人的个数证明其通用性。运行时间比双机器人时增加了1.5s左右，在可接受范围内。

1.2版本修复了视频生成中存在的帧数会发生变化的问题。

![images](https://github.com/HUOJIAXI/GraduationProject/blob/master/Result_one_way/Version1.2_one_way.jpg)

双机器人： 

![images](https://github.com/HUOJIAXI/GraduationProject/blob/master/Result_one_way/Version1.1.png)

三机器人：

![images](https://github.com/HUOJIAXI/GraduationProject/blob/master/Result_one_way/Version1.2.png)  

## Version 2.0
Author: HUO JIAXI

2.0版本扩大了环境，将3*3的环境扩展到5*5的四机器人空间，并且添加了针对不同的交叉点避免同进同出的限制，以及单行道方向限制，完成时间为70s。

2.0版本会出现在保证单行道方向的时候两个机器人可能会交叉进入对方的巷道，这个问题需要征求老师的建议，是否需要添加新的对于单行道方向的限制。

![images](https://github.com/HUOJIAXI/GraduationProject/blob/dev_one_way/Result_one_way/Version2.0_4Rob.png)

四机器人：

![images](https://github.com/HUOJIAXI/GraduationProject/blob/dev_one_way/Result_one_way/Version2.0_one_way_4Rob.jpg)

## Version 2.1
Author: HUO JIAXI

2.1版本通过将模型规模进一步扩大，扩大为7*7的环境，并且有9个机器人，通过求解，发现之前所设定的约束条件可以满足要求，因此初步判定这些约束条件具有通用性。

3。0版本尝试将起点终点的选取随机化，以得出一般性结论，需要用到：r = a + (b-a).*rand([m n])

![images](https://github.com/HUOJIAXI/GraduationProject/blob/master/Result_one_way/Version2.1_one_way_9Robot.jpg)

九机器人：

![images](https://github.com/HUOJIAXI/GraduationProject/blob/master/Result_one_way/Version2.1.jpg)

## Version 3.0
Author: HUO JIAXI

3.0版本添加了随机初始点和终点：通过输入机器人个数，即可生成一组对应每个机器人的随机起点和终点，并且随机起点和终点不会出现在障碍物处，并且机器人之间的起点不会重合，终点也不会重合。

根据对模型的随机测试，我们得到了如下结果，为了节省时间，我们安排了三个机器人：
	运行时间（s）	最佳路径长度

1.    46				13

2.    49				20

3.	  53				17

通过三组随机测试，可以发现模型的求解时间变化在可接受的范围内，但是测试的过程中电脑会发热，会影响一定的求解速度。

![images](https://github.com/HUOJIAXI/GraduationProject/blob/master/Result_one_way/Version3.0_9Rob.jpg)

九机器人：

![images](https://github.com/HUOJIAXI/GraduationProject/blob/master/Result_one_way/Version3.0.png)

接下来的任务是尝试利用切割的方法，减小求解规模。

## Version 4.0
Author: HUO JIAXI

4.0版本由拓扑结构扩展为由1*3的货架组成的7*13的环境中，并且引入IP模型。

并且在调试过程中发现之前版本的单行线规则中的保持路径方向的限制存在问题，会存在方向出现问题的问题，会出现冲突的情况，由于在拓扑图结构中不会出现冲突，因此在扩展的13x7环境中可以通过调节机器人的速度实现避障。

由于单行线规则复杂度变高，因此求解时间变长（可能是电脑温度太高，导致电脑运行速度变慢），还需要改进IP模型。

![images](https://github.com/HUOJIAXI/GraduationProject/blob/master/Result_one_way/Version4.0.png)

![images](https://github.com/HUOJIAXI/GraduationProject/blob/master/Result_one_way/Version4.0_9Rob.jpg)

## Vesion 4.1
Author: HUO JIAXI

4.1版本处理了4.0版本存在的由原环境转拓扑图存在的导致起点终点偏移的问题。

4.1版本存在IP模型限制存在的问题，对于控制巷道方向会失效。

![images](https://github.com/HUOJIAXI/GraduationProject/blob/master/Result_one_way/Version4.1.jpg)

## Version 4.2
Author: HUO JIAXI

4.2版本修复了4.1版本中IP模型存在的巷道方向控制失效的问题，但是存在限制过多的问题，会导致求解速度很慢，对于7x13的需要求解将近40分钟的时间。因此需要改进模型中的限制。

![images](https://github.com/HUOJIAXI/GraduationProject/blob/master/Result_one_way/Version4.2_9rob.png)

## Version 4.2 ops
Author: HUO JIAXI

4.2改进版本改进了4.2版本的限制，主要针对巷道方向的控制限制。降低了限制数量，缩小了模型规模，8机器人环境求解时间在290s左右，9机器人260s，相对于4.2版本的40分钟由一定的提高，并且由于求解次数过多，可能会导致计算机运行速度变慢，因此在初始状态下运行可能会得到更快的运行时间。

![images](https://github.com/HUOJIAXI/GraduationProject/blob/master/Result_one_way/Version4.2_9rob_ops.png)

![images](https://github.com/HUOJIAXI/GraduationProject/blob/master/Result_one_way/Version4.2_ops.jpg)

## Version 5.0
Author: HUO JIAXI

5.0版本完善了单行道法则，添加了起点终点检查功能，防止出现随机起点终点选取错误问题，因此能够执行随机任务。

随着模型规模的增大，对于某些起点终点的选择，会出现不存在路径的情况，但是是少数情况，正常情况下都能够正确求解，并且路径不存在冲突。

![images](https://github.com/HUOJIAXI/GraduationProject/blob/master/Result_one_way/Version5.0.png)

![images](https://github.com/HUOJIAXI/GraduationProject/blob/master/Result_one_way/Version5.0.jpg)

## Version 5.1
Author: HUO JIAXI

5.1版本是对于动态整数规划方法的改进，改进了原始路径的规划方法，由于原始路径不考虑机器人之间的冲突，因此可以不用调用通用求解器，并且由于是规则的环境，因此可以我设计了"直线移动法"进行初始路径规划，规划速度非常快，在1s以内，并且用随机起点终点进行了算法可行性验证。接下来需要将其应用于整体算法中，预计可以节约一大半的运行时间。

![images](https://github.com/HUOJIAXI/GraduationProject/blob/master/results/ops_ori.jpg)

## Version Final dyanmic
Author: HUO JIAXI

本版本将动态多机器人仓储环境整体路径规划设计完善，对于初始路径规划以及最后的补充规划，调用适用于障碍物成块规则出现环境的“直线移动”最短路径规划方法，成功将16机器人的13x13环境的求解时间压缩在不超过160s，并且对于不同的随机起点终点时间浮动不大，时间稳定性较高。但是依然会出现死锁的问题，而在毕设的第二个版本“整体单行道整数规划”方法中，死锁问题得到了解决。

![images](https://github.com/HUOJIAXI/GraduationProject/blob/master/results/VersionFinal_dynamic_ops.png)

![images](https://github.com/HUOJIAXI/GraduationProject/blob/master/results/VersionFinal_dynamic_ops.jpg)

## Version V2 oneway 1.0
Author: HUO JIAXI

由于在V1版本中的单行道模型需要很长的时间进行求解，并且随着机器人个数增加，求解时间成指数上升。因此开发V2版本，尝试避免指数上升问题。

1.0版本完成了可求解的单行道模型约束，并且求解成功，但是求解时间较长，但是经过测试，在求解过程的前半段求解器已经求解出了最优解，但是求解器并没有停止，仍然在继续检索，因此在1.0版本中对运行时间进行了限制，能够正常求解。

之后的版本将对求解器利用assign和usex0进行初始值的赋值，尝试降低求解时间。

## Version V2 oneway 2.0
Author: HUO JIAXI

2.0版本更新了1.0版本的模型限制，发现了之前版本中限制存在的问题，对于交汇节点的入边出边限制，1.0版本可能会出现去除一些可能存在的可行路径的问题。

2.0版本可以完成高密度环境的模型求解，可以解决16机器人的模型，但是求解时间会随着随机起点终点的选取而变化（Gurobi检索规模会随着起点终点的选取变化而变化）。

2.0版本引入了初始解，针对dir_way决策变量赋以一个可行的单行道绕行方向，在一定程度上可以降低求解时间。

该版本仍需进行优化，降低求解时间。

![images](https://github.com/HUOJIAXI/GraduationProject/blob/master/Result_one_way_V2/simulation_9ROB_V2_indi.jpg)

![images](https://github.com/HUOJIAXI/GraduationProject/blob/master/Result_one_way_V2/simulation_9ROB_V2.jpg)

![images](https://github.com/HUOJIAXI/GraduationProject/blob/master/Result_one_way_V2/simulation_16ROB_V2_indi.jpg)

![images](https://github.com/HUOJIAXI/GraduationProject/blob/master/Result_one_way_V2/simulation_16ROB_V2.jpg)

## Version V2 oneway 2.2
Author: HUO JIAXI

2.2版本加入了对dir_way以及dir_rob两个方向变量的取值限制，限制为0-3之间，这样可以有效缩减Gurobi的求解规模，因此求解时间得到了一定的降低，对于16机器人的环境下，平均求解时间在300s左右，最快可以达到100s左右（得益于启发式初始解）。

由于Gurobi约束的限制，求解结果会在小几率下出现结果为一个无限接近于实际正确值的小数的问题，但是并不影响求解，

需要进一步引入启发式方法降低求解时间。

![images](https://github.com/HUOJIAXI/GraduationProject/blob/master/Result_one_way_V2/simulation_16ROB_V2.2_indi.jpg)

![images](https://github.com/HUOJIAXI/GraduationProject/blob/master/Result_one_way_V2/simulation_16ROB_V2.2.png)

## Version V3 oneway
Author: HUO JIAXI

3.0版本将模型优化为可适应任意规模的拓扑图结构环境（可应用之前方法中的拓扑图拓展方法拓展）。

将模型规模扩展，将7x7环境增大为13x9，容纳16台机器人。在8机器人环境下，求解模型需要花费700s左右，16机器人则需要花费2800s左右，时间增长的原因是8机器人在13x7环境下不容易发生冲突，容易满足限制，而在16机器人环境下，冲突比较复杂，需要更多的时间求解模型，但是随着机器人个数上升，模型规模是线性上升的。

对于7x7环境和13x9环境，由于环境规模增加了3倍，会导致模型规模快速增长。

![images](https://github.com/HUOJIAXI/GraduationProject/blob/master/Result_one_way_V3/simulation_8ROB_V3.png)

![images](https://github.com/HUOJIAXI/GraduationProject/blob/master/Result_one_way_V3/simulation_8ROB_V3_indi.jpg)

![images](https://github.com/HUOJIAXI/GraduationProject/blob/master/Result_one_way_V3/simulation_16ROB_V3.png)

![images](https://github.com/HUOJIAXI/GraduationProject/blob/master/Result_one_way_V3/simulation_16ROB_V3_indi.jpg)

## Version 3.1 oneway
Author: HUO JIAXI

3.1版本通过移植V1版本的拓扑图拓展方法，将环境扩展成为一个大型的仓库环境，规则排布了24个1x3的货架，以及拥有16个机器人，求解时间在2400s左右。但是对于4机器人的环境中来说，则求解时间很短，因为对于4个机器人来说可以免除一些约束，可以提高求解效率。对于16机器人，在环境中发生的冲突会比较复杂，并且约束变量很多，因此需要较长的求解时间。

## Version test
Author: HUO JIAXI

test版本主要对在相同环境下不同机器人个数模型求解时间进行分析。test版本优化了初始解，在增加机器人个数的过程中，将上一次运行生成的结果作为启发式初始解输入IP模型，使得求解时间得到一定程度的降低，并且可以发现在16个机器人之前，求解时间随着机器人个数的增加而线性增长，而机器人过多会导致模型过于复杂，求解时间则会大幅上升。

![images](https://github.com/HUOJIAXI/GraduationProject/blob/master/res_test/simulation_20ROB_test.gif)

## Version 3.2 oneway
Author: HUO JIAXI

本版本主要对初始解算法进行了探究，发现初始解算法成功求解的条件比较苛刻，因为初始解很难满足单行线法则中的所有条件，因此初始解会被忽略。

>User MIP start did not produce a new incumbent solution

>User MIP start violates constraint R14176 by 1.000000000

初始解能够在去除一台机器人以及在机器人的状态没有发生变化的情况下会起效：

>User MIP start produced solution with objective 27 (0.20s)
>Loaded user MIP start with objective 27

在机器人的初始情况发生变化的情况下，初始解很难满足条件，因此会被求解器忽略

## Version 4.0 oneway
Author: HUO JIAXI

本版本将初始化解启发式方法进行了改进，启发式方法在一定概率上能够在小扰动情况下起效，可以降低求解时间。另外能够在系统中增加机器人的情况下起效。能够降低求解时间。并且通过测试，发现在扩大环境的情况下，使用启发式方法能够在大环境中降低大约200s的求解时间。

## Version ops heuristic start oneway
Author: HUO JIAXI

本版本为单行线法则的优化版本，主要针对启发性初始解的设计。由于之前的对于单行线路线方向的启发式解无法良好使用，因此我基于初始的绕行单行线路线，设计了对边占用x变量的启发式解。经过大规模的测试（10000组），启发式初始解设计方法能够正常求解出启发式初始解。

在调用启发式初始解之后，求解器能够从必为可行解出发，进行节点的检索，最终找到最优解，求解时间能够降低很多，对于机器人密集的情况下优化效果尤为明显，能够根据需要，控制最大求解时间，来取得与最优解不同接近程度的可行解，或是最终得到最优解。


