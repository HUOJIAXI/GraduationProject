function [PathStore]=biais_goal(PathStore,Path_num,RobotNum,~,Goal_ori,~,Goal,n)
%     [X_start,Y_start]=spread(start_ori,n);
    [X_goal,Y_goal]=spread(Goal_ori,n);
    
    for i=1:RobotNum
        test=Path_num{i}-Goal(i);
        final=find(test==0);
        PathStore{i,1}(final+1:end,:)=[];
    end
    
    for i = 1:RobotNum
        
      
%         temp=PathStore{i,1};
%         temp_goal=temp(length(temp),:);
%         
%         if mod(temp_goal(1),2)==1
%             if temp_goal(1)==1
%                 temp_add=[temp_goal(1)+1 temp_goal(2)];
%                 PathStore{i,1}=cat(1,PathStore{i,1},temp_add);
%                 continue
%             else
%                 temp_add=[temp_goal(1)-1 temp_goal(2)];
%                 PathStore{i,1}=cat(1,PathStore{i,1},temp_add);
%                 continue
%             end
%         end
%         
%         if mod(temp_goal(1),2)==0
%             if temp_goal(2)==1
%                 temp_add=[temp_goal(1) temp_goal(2)+1];
%                 PathStore{i,1}=cat(1,PathStore{i,1},temp_add);
%                 continue
%             else
%                 temp_add=[temp_goal(1) temp_goal(2)-1];
%                 PathStore{i,1}=cat(1,PathStore{i,1},temp_add);
%                 continue
%             end
%         end
%         PathStore{i,1}=cat(1,[X_start(i) Y_start(i)],PathStore{i,1});
        PathStore{i,1}=cat(1,PathStore{i,1},[X_goal(i) Y_goal(i)]);
%         disp(PathStore{i,1})
        
    end