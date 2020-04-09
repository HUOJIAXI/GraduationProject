function [Path_new]=treatment_arrive(Path,RobotNum,Goal,D)

size_n=size(D,2);

Path_num=cell(RobotNum,1);
Path_new=cell(RobotNum,1);

for rob=1:RobotNum
    Path_num{rob,1}=[Path_num{rob,1}, Path{rob,1}(:,2)+(Path{rob,1}(:,1)-1)*size_n];
end

for rob =1:RobotNum
    index_goal=find(Path_num{rob,1}==Goal(rob));
    index_g=index_goal(1);
    Path{rob,1}(index_g+1:end,:)=[];
    Path_new{rob,1}=Path{rob,1};
end