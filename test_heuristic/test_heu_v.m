flag_test=1;
while 1
%     clear;
    clc;
    D = load('tsp_dist_broad.txt'); 
    m = size(D,1);
    n = size(D,2);
    RobotNum=14; %22非上限若不考虑交汇点约束 30达到容量上限
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
%         break
    end
    flag_test=flag_test+1;
end