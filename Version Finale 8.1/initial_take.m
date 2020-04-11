function [PathStore,Path_num]=initial_take(r_start_ori,r_Goal_ori,D,RobotNum)
Path_num=cell(RobotNum,1);
sizeD=size(D,2);

[Start,Goal,~,~,D_reduit] = reduit(r_start_ori,r_Goal_ori,D);

[PathStore_ini,~] = ori_path_am(Start,Goal,RobotNum,D_reduit);

% disp(PathStore_ini{1})

[PathStore,~]=broaden(PathStore_ini,D,RobotNum,r_start_ori,r_Goal_ori);

for i =1:RobotNum
    Path_num{i,1}=(PathStore{i,1}(:,2)+(PathStore{i,1}(:,1)-1)*sizeD)';
end
disp(Path_num)