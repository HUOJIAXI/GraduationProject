function [Path_new,sum_dist]=treatment_arrive(Path,RobotNum,Goal,D)
size_n=size(D,2);

Path_num=cell(RobotNum,1);
Path_new=cell(RobotNum,1);

for rob=1:RobotNum
    Path_num{rob,1}=[Path_num{rob,1}, Path{rob,1}(:,2)+(Path{rob,1}(:,1)-1)*size_n];
end

sum_dist=0;
for rob =1:RobotNum
    index_goal=find(Path_num{rob,1}==Goal(rob));
    for i = 1:length(index_goal)
        res=Path_num{rob,1}(index_goal(i)+1:end,:)-Goal(rob);
%         disp(res)
        if all(res==0)
            index_g=index_goal(i);
            break
        end
    end
%     index_g=index_goal(1);
    Path{rob,1}(index_g+1:end,:)=[];
    Path_num{rob,1}(index_g+1:end,:)=[];
    Path_new{rob,1}=Path{rob,1};
    test_path=unique(Path_num{rob,1});
%     disp(test_path)
    sum_dist=sum_dist+size(test_path,1);
end