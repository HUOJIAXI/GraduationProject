function exam(Path_num_new,Start_ori,Goal_ori,RobotNum)
flag_goal = 0;
flag_start = 0;

for i = 1:RobotNum
    if Path_num_new{i}(1) - Start_ori(i) ~=0
        flag_start = 1;
        disp(i)
        break
    end
end

if flag_start == 0
    disp('机器人起点检查结束。成功')
else
    disp('机器人未正常从起点出发')
end

for i = 1:RobotNum
    if Path_num_new{i}(size(Path_num_new{i},1)) - Goal_ori(i) ~=0
        flag_goal = 1;
        disp(i)
        break
    end
end

if flag_goal == 0
    disp('机器人已全部达到终点。成功')
else
    disp('机器人未正常达到目的地')
end