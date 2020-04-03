%% 传统整数规划模型多机器人系统路径规划方法 多变量

clear;
clc;

yalmip('clear') % Yalmip接口清空内存

numrobot=5;
MAXINT=1000; %大数

[obs,nobs]=count_obstacle(D); 
n_obs=length(obs);          % 清点障碍物数量

PATH=cell(numrobot,1);
Path=cell(numrobot,1);

T=MAXINT;

D = load('distmap.txt'); 

N=intvar(numrobot,1,'full'); % 总运行时间，暂时作为一个决策变量

state_rob=intvar(numrobot,2,T,'full'); %机器人在t时刻的状态，三位维决策变量，第一维表示机器人序号，第二维表示x和y，第三维表示时刻

u_rob=binvar(numrobot,2,T,'full'); %机器人在t时刻的前进状态，三维决策变量，第一维表示机器人序号，第二维表示在x方向前进还是在y方向前进

fun_obj=0;

for i = 1:numrobot
    fun_obj = fun_obj+sum(N(:));
end

A=[1,0;0,1]; % 状态转移矩阵

B=[1,0;0,1]; % 速度矩阵

C=[]; %约束集合

% 机器人状态

% 状态转移

for t = 1:MAXINT-1
    state_rob(:,:,t+1)=A.*state_rob(:,:,t)+B.*u_rob(:,:,t);
end

%% 约束1 对于每次转移，x和y只能选取一个前进一个单位
C=[C, sum(u_rob,2)<=ones(numrobot,1,MAXINT)]; % 循环向量化，对行求和
      
