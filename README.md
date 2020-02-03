# GraduationProject 项目日志
## Version:02/02/2020
Author: HUO JIAXI. 

已完成单机器人的IP模型构建，通过调用基于CPLEX的Yalmip线性求解器成功求解最短路径，但是由于CPLEXcommunity版变量个数限制，环境规模被限制在5*5

## Version:03/02/2020
Autuor: HUO JIAXI

调用Gurobi求解器，成功增大模型规模，目前暂时定为12*12，由于gurobi不限制变量个数，有增大模型规模的可能性，但是会造成比较长的延时。  

模型被成功求解，得到单个最短路径  

耗时24.957595秒.  

为多机器人系统搭建环境  

引入按次序执行路径规划的多机器人IP模型，求解成功，并且不会发生冲突，但是空间时间效率低，需要引入同步模型。
耗时为单个机器人运行时间的叠加，并且路径被单个机器人直接占用，未被释放，效率低，需改进。

![images](https://github.com/HUOJIAXI/GraduationProject/blob/master/results/Map12*12.jpg)
![images](https://github.com/HUOJIAXI/GraduationProject/blob/master/results/Result12*12.jpg)  

1-AS   

![images](https://github.com/HUOJIAXI/GraduationProject/blob/master/results/3-MAS_12*12.jpg)  

3-MAS

## 对于优化MAS路径规划的一点想法：  
动态规划，暂时先不考虑启发式算法。
1. 先对每个机器人执行不考虑其他机器人存在的单机器人路径规划，将各机器人的最短路径存储在PATH.mat中，作为优先路径选择集合。
2. 正式开始启动系统，每个机器人按照优先路径行动，（for循环）每次行动存储实际所在位置在temp.mat中（暂定），并判断下一步节点是否与其他机器人优先路径集合中的下一节点重合（head-on-head collision），或是和已经达到目的地的机器人的实际节点重合，若重合，则重新调用一次IP_solver求解器，求出无撞撞的路径，并对所有的机器人执行，直到所有的机器人达到目的节点，得到一个最终的路径cell集合pathcellfinal.matrix。



