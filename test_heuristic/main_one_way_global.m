%% 约束点条件不必要考虑，由于通过节点占用推出的机器人行驶方向已经框定了机器人不会同时进入一个交汇点（否则无法离开节点，这和约束1相违背，因此不会出现这个情况）

while 1
    clear;
    clc;
    D = load('tsp_dist_broad.txt'); 
    m = size(D,1);
    n = size(D,2);
    RobotNum=30; %22非上限若不考虑交汇点约束 30达到容量上限
    [Goal_ori,Start_ori,r_start_ori,r_Goal_ori]=rand_Goal_Start_op(D,RobotNum,3);
    %RobotNum = size(Start,2);

    [Start,Goal,start_sp,goal_sp,D_reduit] = reduit(r_start_ori,r_Goal_ori,D);
    % disp(D_reduit)
    size_D=size(D_reduit,2);

    diary('res.txt');

    disp(datestr(now));

    [Start_new,Goal_new,RobotNum_new,err]=test_reduce_coincidence(Start,Goal,RobotNum);
    if err==1
        continue
    else
        [ini_Path_num,ini_PathStore]=initial_x_way(D_reduit,RobotNum,Start_new,Goal_new);
        break
    end
end

disp('已完成启发式初始解设定')

ini_x_value=[];
for i = 1:RobotNum
    [ini_x_value]=initial_guess_heuristic(ini_Path_num{i},ini_x_value,D_reduit);
end
% D_reduit(m,:)=[];
% 
% disp(D_reduit)
% disp(Start_new)
% disp(Goal_new)
 [PathStore,Path_num,dir_way,runtime_indi]=IP_solver_single_way_V3_res(D_reduit,Start_new,Goal_new,RobotNum_new,size_D,ini_x_value);
%  [PathStore,Path_num]=IP_solver_single_way_V2(D,Start_ori,Goal_ori,RobotNum,size_D);
disp('运行时间')
disp(runtime_indi)
diary('off');

disp(dir_way)

[PathStore,Path_num] = rebuild_path(RobotNum,Start,Goal,Start_new,Goal_new,PathStore,Path_num);

[PathStore_new,Path_num_new]=broaden(PathStore,D,RobotNum,r_start_ori,r_Goal_ori);

exam(Path_num_new,r_start_ori,r_Goal_ori,RobotNum);

% plotdynamic(D,PathStore,Path_num,RobotNum,Start_ori,Goal_ori);
%

plot_ind(D,RobotNum,n,m,Start_ori,Goal_ori,PathStore_new,dir_way)

% pause(5);

% plotdynamic_tes(D,PathStore_new,Path_num_new,RobotNum,Start_ori,Goal_ori,r_start_ori,r_Goal_ori,dir_way);

save('Path_num.mat')