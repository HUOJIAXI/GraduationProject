function [PathStore_new,Path_num_new]=broaden(PathStore,D,RobotNum,Start_ori,Goal_ori)
% 先将路径扩展
PathStore_new=cell(RobotNum,1);
sizeD=size(D,2);

[X_start,Y_start]=spread(Start_ori,sizeD);
[X_goal,Y_goal]=spread(Goal_ori,sizeD);

    for i = 1:RobotNum
        for j = 1:size(PathStore{i},1) 
                PathStore{i}(j,2)=PathStore{i}(j,2)+(PathStore{i}(j,2)-1);
        end
    end
   
    for i = 1:RobotNum
        
        for j = 1:size(PathStore{i},1)
            
            if  j <= size(PathStore{i},1)-1 
                if (PathStore{i}(j+1,2)-PathStore{i}(j,2)) == -2
                    PathStore_new{i}=[PathStore_new{i}; PathStore{i}(j,:)];
                    
                    PathStore_new{i}=[PathStore_new{i}; [PathStore{i}(j,1) (PathStore{i}(j+1,2)+PathStore{i}(j,2))/2]];
                    
                elseif (PathStore{i}(j+1,2)-PathStore{i}(j,2)) == 2 
                    PathStore_new{i}=[PathStore_new{i}; PathStore{i}(j,:)];
                    
                    PathStore_new{i}=[PathStore_new{i}; [PathStore{i}(j,1) (PathStore{i}(j+1,2)+PathStore{i}(j,2))/2]];
                    
                else
                    PathStore_new{i}=[PathStore_new{i}; PathStore{i}(j,:)];
                
                end
            else
                
                PathStore_new{i}=[PathStore_new{i}; PathStore{i}(j,:)];
                
            end
                
            
        end
        
    end
% 开始处理起点终点
% 起点
for i = 1:RobotNum
    if (PathStore_new{i}(1,1)-X_start(i)) ~= 0 || (PathStore_new{i}(1,2)-Y_start(i)) ~= 0
        if (PathStore_new{i}(2,1)-X_start(i))==0 && (PathStore_new{i}(2,2)-Y_start(i)) == 0
            PathStore_new{i}(1,:)=[];
        
        elseif (PathStore_new{i}(2,1)-X_start(i))~=0 || (PathStore_new{i}(2,2)-Y_start(i)) ~= 0
            PathStore_new{i}=[[X_start(i) Y_start(i)]; PathStore_new{i}];
        end
        
    end
end

%终点
for i = 1:RobotNum
    if (PathStore_new{i}(size(PathStore_new{i},1),1)-X_goal(i)) ~= 0 || (PathStore_new{i}(size(PathStore_new{i},1),2)-Y_goal(i)) ~= 0
        if (PathStore_new{i}(size(PathStore_new{i},1)-1,1)-X_goal(i))==0 && (PathStore_new{i}(size(PathStore_new{i},1)-1,2)-Y_goal(i)) == 0
            PathStore_new{i}(size(PathStore_new{i},1),:)=[];
        
        elseif (PathStore_new{i}(size(PathStore_new{i},1)-1,1)-X_goal(i))~=0 || (PathStore_new{i}(size(PathStore_new{i},1)-1,2)-Y_goal(i)) ~= 0
            PathStore_new{i}=[PathStore_new{i};[X_goal(i) Y_goal(i)]];
        end
        
    end
end

for i = 1:RobotNum
    Path_num_new{i}=PathStore_new{i}(:,2)+(PathStore_new{i}-1)*sizeD;
end
% 
% z=[];
% for i = 1:RobotNum
%     z=[z Path_num_new{i}(length(Path_num_new{i}))-Goal_ori(i)];
% end




