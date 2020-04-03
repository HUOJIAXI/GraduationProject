function [PathStore_inserted]=insert_value(PathStore,RobotNum)

PathStore_inserted=cell(RobotNum,1);

for i = 1:RobotNum
    for j = 1:size(PathStore{i,1},1)-1
        
        if PathStore{i,1}(j,1) == PathStore{i,1}(j+1,1) % 横向行走，保持纵坐标不变
            temp=zeros(11,2);
            temp(1,2)=PathStore{i,1}(j,2);
            temp(1,1)=PathStore{i,1}(j,1);
            temp(11,2)=PathStore{i,1}(j+1,2);
            temp(11,1)=PathStore{i,1}(j+1,1);
            for k = 2:10
                if temp(1,2)<temp(11,2)
                    temp(k,2)=temp(k-1,2)+0.1;
                    temp(k,1)=temp(k-1,1);
                else
                    temp(k,2)=temp(k-1,2)-0.1;
                    temp(k,1)=temp(k-1,1);
                end
            end
            PathStore_inserted{i,1}=[PathStore_inserted{i,1};temp];
        end
        
        if PathStore{i,1}(j,2) == PathStore{i,1}(j+1,2) % 纵向行走，保持横坐标不变
            temp(1,2)=PathStore{i,1}(j,2);
            temp(1,1)=PathStore{i,1}(j,1);
            temp(11,2)=PathStore{i,1}(j+1,2);
            temp(11,1)=PathStore{i,1}(j+1,1);
            for k = 2:10
                if temp(1,1)<temp(11,1)
                    temp(k,1)=temp(k-1,1)+0.1;
                    temp(k,2)=temp(k-1,2);
                else
                    temp(k,1)=temp(k-1,1)-0.1;
                    temp(k,2)=temp(k-1,2);
                end
            end
            PathStore_inserted{i,1}=[PathStore_inserted{i,1};temp];
        end
        
    end

end

for i=1:RobotNum

    PathStore_inserted{i,1}=unique(PathStore_inserted{i,1},'rows','stable');

end