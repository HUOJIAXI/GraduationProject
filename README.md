# GraduationProject 项目日志 
# Version 8.1 削减式算法最终版(具有在货架下躲避冲突的功能)
# Version 8.1 最终效果：
![images](https://github.com/HUOJIAXI/GraduationProject/blob/master/results/simulation_16ROB_COLI_version8.1.gif)
# 当前进行：单行线法则版本 Version 1.2
## 仿真平台：Intel I7 RAM 16G - Matlab for mac
## 到目前为止的效果：  

![images](https://github.com/HUOJIAXI/GraduationProject/blob/dev_one_way/Result_one_way/simulation_9ROB_Version2.1.gif)

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


