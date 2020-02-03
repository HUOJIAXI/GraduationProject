# GraduationProject 项目日志
## Version:02/02/2020
HUO JIAXI
已完成单机器人的IP模型构建，通过调用基于CPLEX的Yalmip线性求解器成功求解最短路径，但是由于CPLEXcommunity版变量个数限制，环境规模被限制在5*5

## Version:03/02/2020
调用Gurobi求解器，成功增大模型规模，目前暂时定为10*10。
模型被成功求解，得到单个最短路径

