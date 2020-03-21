function [PathStore,Path_num] = rebuild_path(RobotNum,Start,Goal,Start_new,Goal_new,PathStore,Path_num)

PathStore_re=cell(RobotNum,1);
Path_num_re=cell(RobotNum,1);

for i = 1:RobotNum
    
    start_find = find(Start_new==Start(i));
    goal_find = find(Goal_new==Goal(i));
    
    for j = 1:length(start_find)
        for m = 1:length(goal_find)
            if start_find(j)==goal_find(m)
                findindex=start_find(j);
            end
        end
    end
 
       PathStore_re{i,1}=PathStore{findindex,1};
       Path_num_re{i,1}=Path_num{findindex,1};
    
end

PathStore=PathStore_re;
Path_num=Path_num_re;

