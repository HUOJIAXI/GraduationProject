# GraduationProject 项目日志
## Version:02/02/2020
Author: HUO JIAXI. 

已完成单机器人的IP模型构建，通过调用基于CPLEX的Yalmip线性求解器成功求解最短路径，但是由于CPLEXcommunity版变量个数限制，环境规模被限制在5*5

## Version:03/02/2020
Autuor:HUO JIAXI

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





