function [sum_dist,Path_new]=dis_cal(RobotNum,Goal,Path_num,PathStore)

% dis_total=0;
% 
% for i=1:RobotNum
%     test=Path_num{i}-Goal(i);
%     final=find(test==0);
%     PathStore{i,1}(final(1)+1:end,:)=[];
% end
% 
% for i=1:RobotNum
%     path_temp=PathStore{i,1};
% %     path_temp=unique(path_temp,'stable');
%     dis_total=dis_total+size(path_temp,1);
% end
sum_dist=0;
Path_new=cell(RobotNum,1);
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
    PathStore{rob,1}(index_g+1:end,:)=[];
    Path_num{rob,1}(index_g+1:end,:)=[];
    Path_new{rob,1}=PathStore{rob,1};
    test_path=unique(Path_num{rob,1});
%     disp(test_path)
    sum_dist=sum_dist+length(test_path);
end