%% 约束点条件不必要考虑，由于通过节点占用推出的机器人行驶方向已经框定了机器人不会同时进入一个交汇点（否则无法离开节点，这和约束1相违背，因此不会出现这个情况）

clear;
clc;
D = load('tsp_dist_broad.txt'); 
m = size(D,1);
n = size(D,2);
RobotNum=4; %22非上限若不考虑交汇点约束 30达到容量上限
[Goal_ori,Start_ori,r_start_ori,r_Goal_ori]=rand_Goal_Start_op(D,RobotNum,3);
%RobotNum = size(Start,2);

[Start,Goal,start_sp,goal_sp,D_reduit] = reduit(r_start_ori,r_Goal_ori,D);
% disp(D_reduit)
size_D=size(D_reduit,2);
diary('res.txt');

disp(datestr(now));

[Start_new,Goal_new,RobotNum_new]=test_reduce_coincidence(Start,Goal,RobotNum);
ini_x_value=[];
for i = 1:RobotNum
    [ini_x_value]=initial_guess(ini_x_value,Start_new(i),Goal_new(i),D_reduit);
end
% D_reduit(m,:)=[];
% 
% disp(D_reduit)
% disp(Start_new)
% disp(Goal_new)
tic
 [PathStore,Path_num,dir_way]=IP_solver_single_way_V3_res(D_reduit,Start_new,Goal_new,RobotNum_new,size_D,ini_x_value);
%  [PathStore,Path_num]=IP_solver_single_way_V2(D,Start_ori,Goal_ori,RobotNum,size_D);
toc
diary('off');

disp(dir_way)

[PathStore,Path_num] = rebuild_path(RobotNum,Start,Goal,Start_new,Goal_new,PathStore,Path_num);

[PathStore_new,Path_num_new]=broaden(PathStore,D,RobotNum,r_start_ori,r_Goal_ori);

exam(Path_num_new,r_start_ori,r_Goal_ori,RobotNum);

% plotdynamic(D,PathStore,Path_num,RobotNum,Start_ori,Goal_ori);
% 
plotdynamic_tes(D,PathStore_new,Path_num_new,RobotNum,Start_ori,Goal_ori,r_start_ori,r_Goal_ori,dir_way);

mapdesigner(fliplr(D),2);

show=ceil(sqrt(RobotNum));

for i = 1:RobotNum
        mapdesigner_show(fliplr(D),i,show,0); % 最后一个参数控制行数 是否需要-1
        axis equal
        xlim([0 n])
        ylim([0 m])
        plot((PathStore_new{i}(:,2)-1/2),(PathStore_new{i}(:,1)-1/2),'-ks','MarkerFaceColor','r','MarkerSize',10) ;% 将所有机器人的路径显示在图中。
        str=['robot=',num2str(i)];
        title(str);
end

save('PathStore.mat')
save('Path_num.mat')