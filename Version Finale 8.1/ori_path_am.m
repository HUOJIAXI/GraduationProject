function [PathStore,Path_num] = ori_path_am(Start,Goal,numrobot,D)
PathStore=cell(numrobot,1);
Path_num=cell(numrobot,1);
sizeD=size(D,1);

for i = 1:numrobot
    [Goal_x,Goal_y] = spread_sin(Goal(i),sizeD);
    [Start_x,Start_y] = spread_sin(Start(i),sizeD);
    
    if Start_x ~= Goal_x && Goal_y ~= Start_y
        path_g=zeros(abs(Start_y-Goal_y)+1,2);
        path_s=zeros(abs(Start_x-Goal_x)+1,2);
        path_g(:,2)=linspace(Start_y,Goal_y,abs(Start_y-Goal_y)+1)';
        path_g(:,1)=linspace(Goal_x,Goal_x,abs(Start_y-Goal_y)+1)';
        path_s(:,2)=linspace(Start_y,Start_y,abs(Start_x-Goal_x)+1)';
        path_s(:,1)=linspace(Start_x,Goal_x,abs(Start_x-Goal_x)+1)';

        flag_s=0;
        flag_g=0;

        for j = 1:size(path_s(:,2),1)
            if D(path_s(j,1),path_s(j,2)) == 1
                flag_s = 1;
                break
            end
        end

        for j = 1:size(path_g(:,2),1)
            if D(path_g(j,1),path_g(j,2)) == 1
                flag_g = 1;
                break
            end
        end

        if flag_s==0 && flag_g==0
            path_g(1,:)=[];
            PathStore{i,1}=[path_s;path_g];
            continue
        end
        
        if flag_s==1 && flag_g==1
            path_g=zeros(abs(Start_x-Goal_x)+1,2);
            path_s=zeros(abs(Start_y-Goal_y)+1,2);
            
            path_g(:,2)=linspace(Goal_y,Goal_y,abs(Start_x-Goal_x)+1)';
            path_g(:,1)=linspace(Start_x,Goal_x,abs(Start_x-Goal_x)+1)';
            path_s(:,2)=linspace(Start_y,Goal_y,abs(Start_y-Goal_y)+1)';
            path_s(:,1)=linspace(Start_x,Start_x,abs(Start_y-Goal_y)+1)';
            
            path_g(1,:)=[];
            PathStore{i,1}=[path_s;path_g];
            continue       
        end
        
        flag_m=0;
        
        if flag_s ~= flag_g
            if flag_g == 1
                    path_g(:,1)=path_g(:,1)-1;
                for j = 1:size(path_g(:,1),1)
                    if D(path_g(j,1),path_g(j,2)) == 1
                        path_g(:,1)=path_g(:,1)+2;
                        flag_m=1;
                        break
                    end
                end
%                 path_g_sup=zeros(2,2);
%                 path_g_sup(:,1)=linspace(path_g(end,1),Goal_x,2);
%                 path_g_sup(:,2)=linspace(Goal_y,Goal_y,2);
%          %       path_g(1,:)=[];
%                 path_g=[path_g;path_g_sup];

                if flag_m==0
                    if Start_x < Goal_x
                         path_s(end,:)=[];
                         path_s(end,:)=[];
                    else
                        
                    end
%                      path_g=[[path_g(1,1)-1 path_g(1,2)];path_g];
                elseif flag_m == 1
                    
                    if Start_x > Goal_x
                         path_s(end,:)=[];
                         path_s(end,:)=[];
                    else
                        
                    end
%                    path_g(1,:)=[];
%                     path_s(end,:)=[];
%                     path_g=[[Goal_x Start_y];path_g];
                end
                PathStore{i,1}=[path_s;path_g;[Goal_x Goal_y]];
                continue
            end
            
            if flag_s == 1
                if Start_y < Goal_y
                    path_s(:,2)=path_s(:,2)+1;
                    path_s=[[Start_x Start_y];path_s];
                    path_g(1:2,:)=[];
                    PathStore{i,1}=[path_s;path_g];   
                    continue
                end

                if Start_y > Goal_y
                    path_s(:,2)=path_s(:,2)-1;
                    path_g(1:2,:)=[];
                    path_s=[[Start_x Start_y];path_s];
                    PathStore{i,1}=[path_s;path_g];   
                    
                    continue
                end
                
            end
        end
        
        
    end
    
    
    if Start_x == Goal_x  
        flag_p=0;
        path_g=zeros(abs(Start_y-Goal_y)+1,2);
        path_g(:,2)=linspace(Start_y,Goal_y,abs(Start_y-Goal_y)+1)';
        path_g(:,1)=linspace(Goal_x,Goal_x,abs(Start_y-Goal_y)+1)';
        
        for j = 1:size(path_g(:,2),1)
            if D(path_g(j,1),path_g(j,2)) == 1
                flag_p=1;
                break
            end
        end
        
        if flag_p==1
            if  mod(Start_x,3) == 0 
                path_g(:,1)=path_g(:,1)+1;
                PathStore{i,1}=[[Start_x Start_y];path_g;[Goal_x Goal_y]];
                continue

            elseif mod(Start_x,3) ~= 0
                path_g(:,1)=path_g(:,1)-1;
                PathStore{i,1}=[[Start_x Start_y]; path_g; [Goal_x Goal_y]];
                continue
            end
        else
            PathStore{i,1}=path_g;
            continue
        end
        
        
        
    end
    
    if Goal_y == Start_y
        path_s=zeros(abs(Start_x-Goal_x)+1,2);
        path_s(:,2)=linspace(Start_y,Start_y,abs(Start_x-Goal_x)+1)';
        path_s(:,1)=linspace(Start_x,Goal_x,abs(Start_x-Goal_x)+1)';
        
        flag_p=0;
       for j = 1:size(path_s(:,2),1)
            if D(path_s(j,1),path_s(j,2)) == 1
                flag_p=1;
                break
            end
       end
        
        if flag_p==1
        path_s(:,1)=path_s(:,1)-1;
        PathStore{i,1}=[[Start_x Start_y]; path_s; [Goal_x Goal_y]]; 
        continue
        else
            PathStore{i,1}=path_s;
            continue
        end
    end

end

for i =1:numrobot
    Path_num{i,1}=(PathStore{i,1}(:,2)+(PathStore{i,1}(:,1)-1)*sizeD)';
end

%disp('原始路径规划完成')

% for i =1:numrobot
%     disp(Path_num{i,1}(end)-Goal(i))
% end
