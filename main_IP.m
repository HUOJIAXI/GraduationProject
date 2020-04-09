%% 传统整数规划模型多机器人系统路径规划方法 多变量

clear;
clc;

yalmip('clear') % Yalmip接口清空内存

numrobot=9;

% D = load('distmap.txt'); 
D = load('tsp_dist_broad.txt'); 
[Goal_ori,Start_ori,r_start_ori,r_Goal_ori]=rand_Goal_Start_op(D,numrobot,3);

[Start,Goal,start_sp,goal_sp,D_reduit] = reduit(r_start_ori,r_Goal_ori,D);
size_D=size(D_reduit,2);

[Start_new,Goal_new,RobotNum_new,err,num]=test_reduce_coincidence(Start,Goal,numrobot);

if err==1
    numrobot=num;
end

[obs,nobs]=count_obstacle(D_reduit); 
n_obs=length(obs);          % 清点障碍物数量

size_D_m=size(D,1);
size_D_n=size(D,2);

PATH=cell(numrobot,1);
Path=cell(numrobot,1);

% lenob=1;

% [r_Goal,r_start]=rand_Goal_Start(D,numrobot,lenob);

[path_rob,runtime_indi]=IP_tradion_way(D_reduit,numrobot,Goal_new,Start_new);
Path_num=cell(numrobot,1);
disp(runtime_indi)

[Path_new]=treatment_arrive(path_rob,numrobot,Goal_new,D_reduit);

for rob=1:numrobot
    Path_num{rob,1}=[Path_num{rob,1}, Path_new{rob,1}(:,2)+(Path_new{rob,1}(:,1)-1)*size_D];
end

[Path_new,Path_num] = rebuild_path(numrobot,Start,Goal,Start_new,Goal_new,Path_new,Path_num);

[PathStore_new,Path_num_new]=broaden(Path_new,D,numrobot,r_start_ori,r_Goal_ori);

exam(Path_num_new,r_start_ori,r_Goal_ori,numrobot);


% plot_ind_tig(D,numrobot,size_D_n,size_D_m,r_start,r_Goal,path_rob)

plot_ind(D,numrobot,size_D_n,size_D_m,Start_ori,Goal_ori,PathStore_new)

plotdynamic_tes(D,PathStore_new,Path_num_new,numrobot,Start_ori,Goal_ori);

save('9_9_13.mat')









      
