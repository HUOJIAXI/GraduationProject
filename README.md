# GraduationProject 项目日志
## 到目前为止的效果：  

![images](https://github.com/HUOJIAXI/GraduationProject/blob/master/results/simulation_6ROB_COLIANTI.gif)

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


