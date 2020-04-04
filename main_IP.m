%% 传统整数规划模型多机器人系统路径规划方法 多变量

clear;
clc;

yalmip('clear') % Yalmip接口清空内存

numrobot=5;
MAXINT=100; %大数

D = load('distmap.txt'); 

[obs,nobs]=count_obstacle(D); 
n_obs=length(obs);          % 清点障碍物数量

PATH=cell(numrobot,1);
Path=cell(numrobot,1);

T=MAXINT;

lenob=1;

[r_Goal,r_start]=rand_Goal_Start(D,numrobot,lenob);
size_D_m=size(D,1);
size_D_n=size(D,2);

[r_Goal_x,r_Goal_y]=spread(r_Goal,size_D_n);
[r_start_x,r_start_y]=spread(r_start,size_D_n);
[obs_x,obs_y]=spread(obs,size_D_n);

state_start=cat(2,r_start_x',r_start_y');

state_goal=cat(2,r_Goal_x',r_Goal_y');

state_obs=cat(2,obs_x',obs_y');


% N=intvar(numrobot,1,'full'); % 总运行时间，暂时作为一个决策变量

state_rob=intvar(numrobot,2,T,'full'); %机器人在t时刻的状态，三位维决策变量，第一维表示机器人序号，第二维表示x和y，第三维表示时刻

u_rob=binvar(numrobot,2,T,'full'); %机器人在t时刻的前进状态，三维决策变量，第一维表示机器人序号，第二维表示在x方向前进还是在y方向前进

% t_juge=binvar(numrobot,n_obs,T,4,'full');
b_juge=binvar(numrobot,numrobot,T,4);

fun_obj=0;
for rob=1:numrobot
    for t =1:T
        fun_obj=fun_obj+(squeeze(state_rob(rob,:,t))-state_goal(rob,:))*[1,1]';
    end
end


A=[1,0;0,1]; % 状态转移矩阵

B=[1,0;0,1]; % 速度矩阵

C=[]; %约束集合

% 机器人状态

% 状态转移

%% 约束1 对于每次转移，x和y只能选取一个前进一个单位
C=[C, sum(u_rob,2)<=ones(numrobot,1,MAXINT)]; % 循环向量化，对行求和

%% 约束2 机器人不会离开区域
C=[C,state_rob>0,state_rob(:,1,:)<=size_D_m,state_rob(:,2,:)<=size_D_n];

%% 约束3 机器人需按照转移函数前进
for t = 1:T-1
    for rob=1:numrobot
        C= [C, state_rob(rob,:,t+1)'-(A*state_rob(rob,:,t)'+B*u_rob(rob,:,t)')==0];
    end
end

%% 约束4 机器人的起点终点需在规定区域
%起点
C=[C,squeeze(state_rob(:,:,1))-state_start==0];
%终点
for rob=1:numrobot
    C=[C,state_rob(rob,:,T)-state_goal(rob,:)==0];
end

%% 约束5 机器人不可与障碍物碰撞
for t = 1:T-1
    for rob=1:numrobot
        rob_s=state_rob(rob,2,t)+(state_rob(rob,1,t)-1)*size_D_m;
        for count_obs=1:length(obs)
            C=[C,(obs(count_obs)-rob_s)~=0];
        end
    end
end     %有多重循环 可能造成内存分配时间长

%% 约束6 机器人之间没有冲突
for rob=1:numrobot
    for rob_col=rob+1:numrobot
        for t =1:T
            C=[C,(state_rob(rob,:,t)-state_rob(rob_col,:,t))~=0];
        end
    end
end

ops = sdpsettings('verbose',1,'solver','gurobi');
% 求解
tic
result  = optimize(C,fun_obj,ops);
toc

runtime_indi=toc;
if result.problem== 0
%    value(z)
%     disp(value(dir_rob))
%    text=' 系统总最优路径长度：';
%    disp([text,num2str(value(z))]);
    disp('系统总最优路径长度：Best objective');
else
    disp('Finish ! ');
%     disp(value(dir_rob))
end










      
