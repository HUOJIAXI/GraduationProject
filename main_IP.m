%% 传统整数规划模型多机器人系统路径规划方法 多变量

clear;
clc;

yalmip('clear') % Yalmip接口清空内存

numrobot=9;

D = load('distmap.txt'); 

[obs,nobs]=count_obstacle(D); 
n_obs=length(obs);          % 清点障碍物数量

size_D_m=size(D,1);
size_D_n=size(D,2);

PATH=cell(numrobot,1);
Path=cell(numrobot,1);

lenob=1;

[r_Goal,r_start]=rand_Goal_Start(D,numrobot,lenob);

[path_rob,runtime_indi]=IP_tradion_way(D,numrobot,r_Goal,r_start);

plot_ind(D,numrobot,size_D_n,size_D_m,r_start,r_Goal,path_rob)

save('9_9_13.mat')









      
