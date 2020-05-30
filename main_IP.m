%% 传统整数规划模型多机器人系统路径规划方法 多变量

clear;
clc;

yalmip('clear') % Yalmip接口清空内存

% 
flag_test=1;
% D = load('distmap.txt'); 
if flag_test==1
    numrobot=9;
    D = load('tsp_map.txt'); 
    [Goal_ori,Start_ori,r_start_ori,r_Goal_ori]=rand_Goal_Start_op(D,numrobot);
else
    load('Path_num_test_h3.mat')
    r_Goal_ori=Start;
    r_start_ori=Goal;
    Start_ori=r_Goal;
    Goal_ori=r_start;
end
% [Start,Goal,start_sp,goal_sp,D_reduit] = reduit(r_start_ori,r_Goal_ori,D);
size_D=size(D,2);

% [Start_new,Goal_new,RobotNum_new,err,num]=test_reduce_coincidence(Start,Goal,numrobot);

% if err==1
%     numrobot=num;
% end

[obs,nobs]=count_obstacle(D); 
n_obs=length(obs);          % 清点障碍物数量

size_D_m=size(D,1);
size_D_n=size(D,2);

PATH=cell(numrobot,1);
Path=cell(numrobot,1);

% lenob=1;

% [r_Goal,r_start]=rand_Goal_Start(D,numrobot,lenob);

[path_rob,runtime_indi]=IP_tradion_way(D,numrobot,r_Goal_ori,r_start_ori);
load dat result

disp(runtime_indi)

[Path_new,sum_dist]=treatment_arrive(path_rob,numrobot,r_Goal_ori,D);

disp(['总路径长度：',num2str(sum_dist)]);

Path_num=cell(numrobot,1);
for rob=1:numrobot
    Path_num{rob,1}=[Path_num{rob,1}, Path_new{rob,1}(:,2)+(Path_new{rob,1}(:,1)-1)*size_D];
end

exam(Path_num,r_start_ori,r_Goal_ori,numrobot);

% plot_ind_tig(D,numrobot,size_D_n,size_D_m,r_start,r_Goal,path_rob)

% plot_ind(D,numrobot,size_D_n,size_D_m,Goal_ori,Start_ori,Path_new)

plot_ind(D,numrobot,size_D_n,size_D_m,Start_ori,Goal_ori,Path_new)

% plotdynamic_tes(D,Path_new,Path_num,numrobot,Start_ori,Goal_ori);

save('test_final_2.mat')









      
