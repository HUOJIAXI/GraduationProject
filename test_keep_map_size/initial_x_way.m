function [PathStore,Path_num]=initial_x_way(D,RobotNum,Start,Goal)

[ini_dir_way] = initial(D);

size_m=size(D,1);
size_n=size(D,2);

PathStore=cell(RobotNum,1);
Path_num=cell(RobotNum,1);

for i = 1:RobotNum
    
    PathStore{i}(1)=Start(i);
    
    [goal_x,goal_y]=spread_sin(Goal(i),size_n);
    
    count=1;
    
    while PathStore{i}(count) ~= Goal(i)
        
        if ini_dir_way( PathStore{i}(count)) ==1 %若所在巷道为从西到东的方向或从北到南
            
            [temp_x,~]=spread_sin(PathStore{i}(count),size_n);
            
            if mod(temp_x,2)==0 % 前进第一步进入交汇点
                PathStore{i}=[PathStore{i}, PathStore{i}(count)+size_n];
                count=count+1;
            else
                PathStore{i}=[PathStore{i}, PathStore{i}(count)+1];
                count=count+1;
            end
        elseif ini_dir_way( PathStore{i}(count)) ==3 %若所在巷道为从东到西的方向或从南到北
            [temp_x,~]=spread_sin(PathStore{i}(count),size_n);
            if mod(temp_x,2)==0 % 前进第一步进入交汇点
                PathStore{i}=[PathStore{i}, PathStore{i}(count)-size_n];
                count=count+1;
            else
                PathStore{i}=[PathStore{i}, PathStore{i}(count)-1];
                count=count+1;
            end
        end
            
            [temp_x,temp_y]=spread_sin(PathStore{i}(count),size_n);
            
                        % 前进第二步进入另一个巷道
%% 若在边缘地带
            if temp_x==1||temp_x==size_m ||temp_y==1||temp_y==size_n
                
                if (temp_x==1&&temp_y==1)||(temp_x==size_m&&temp_y==1)
                    PathStore{i}=[PathStore{i}, PathStore{i}(count)+1];
                    count=count+1;
                elseif (temp_x==1&&temp_y==size_n)||(temp_x==size_m&&temp_y==size_n)
                    PathStore{i}=[PathStore{i}, PathStore{i}(count)-1];
                    count=count+1;
                else
                    
                    if goal_x<=temp_x&&goal_y<=temp_y  %终点在左上角
                        if temp_x==1
                            if ini_dir_way(PathStore{i}(count)-1)==3
                                PathStore{i}=[PathStore{i}, PathStore{i}(count)-1];
                                count=count+1;
                            elseif ini_dir_way(PathStore{i}(count)+size_n)==1
                                PathStore{i}=[PathStore{i}, PathStore{i}(count)+size_n];
                                count=count+1;                         
                            end
                        elseif temp_x==size_m
                            if ini_dir_way(PathStore{i}(count)-size_n)==3
                                PathStore{i}=[PathStore{i}, PathStore{i}(count)-size_n];
                                count=count+1;    
                            elseif ini_dir_way(PathStore{i}(count)-1)==3
                                PathStore{i}=[PathStore{i}, PathStore{i}(count)-1];
                                count=count+1;   
                            end
                        elseif temp_y==1
                            if ini_dir_way(PathStore{i}(count)-size_n)==3
                                PathStore{i}=[PathStore{i}, PathStore{i}(count)-size_n];
                                count=count+1;    
                            elseif ini_dir_way(PathStore{i}(count)+1)==1
                                PathStore{i}=[PathStore{i}, PathStore{i}(count)+1];
                                count=count+1;   
                            end
                        elseif temp_y==size_n
                            if ini_dir_way(PathStore{i}(count)-size_n)==3
                                PathStore{i}=[PathStore{i}, PathStore{i}(count)-size_n];
                                count=count+1;    
                            elseif ini_dir_way(PathStore{i}(count)-1)==3
                                PathStore{i}=[PathStore{i}, PathStore{i}(count)-1];
                                count=count+1;   
                            end
                        end
                    end
                    
                    if goal_x>=temp_x&&goal_y<=temp_y %终点在左下角
                        if temp_x==1
                            if ini_dir_way(PathStore{i}(count)+size_n)==1
                                PathStore{i}=[PathStore{i}, PathStore{i}(count)+size_n];
                                count=count+1;
                            elseif ini_dir_way(PathStore{i}(count)-1)==3
                                PathStore{i}=[PathStore{i}, PathStore{i}(count)-1];
                                count=count+1;                         
                            end
                        elseif temp_x==size_m
                            if ini_dir_way(PathStore{i}(count)-1)==3
                                PathStore{i}=[PathStore{i}, PathStore{i}(count)-1];
                                count=count+1;    
                            elseif ini_dir_way(PathStore{i}(count)-size_n)==3
                                PathStore{i}=[PathStore{i}, PathStore{i}(count)-size_n];
                                count=count+1;   
                            end
                        elseif temp_y==1
                            if ini_dir_way(PathStore{i}(count)+size_n)==1
                                PathStore{i}=[PathStore{i}, PathStore{i}(count)+size_n];
                                count=count+1;    
                            elseif ini_dir_way(PathStore{i}(count)+1)==1
                                PathStore{i}=[PathStore{i}, PathStore{i}(count)+1];
                                count=count+1;   
                            end
                        elseif temp_y==size_n
                            if ini_dir_way(PathStore{i}(count)+size_n)==1
                                PathStore{i}=[PathStore{i}, PathStore{i}(count)+size_n];
                                count=count+1;    
                            elseif ini_dir_way(PathStore{i}(count)-1)==3
                                PathStore{i}=[PathStore{i}, PathStore{i}(count)-1];
                                count=count+1;   
                            end
                        end
                    end
                    
                    if goal_x<=temp_x&&goal_y>=temp_y %终点在右上角
                        if temp_x==1
                            if ini_dir_way(PathStore{i}(count)+1)==1
                                PathStore{i}=[PathStore{i}, PathStore{i}(count)+1];
                                count=count+1;
                            elseif ini_dir_way(PathStore{i}(count)+size_n)==1
                                PathStore{i}=[PathStore{i}, PathStore{i}(count)+size_n];
                                count=count+1;                         
                            end
                        elseif temp_x==size_m
                            if ini_dir_way(PathStore{i}(count)-size_n)==3
                                PathStore{i}=[PathStore{i}, PathStore{i}(count)-size_n];
                                count=count+1;    
                            elseif ini_dir_way(PathStore{i}(count)+1)==1
                                PathStore{i}=[PathStore{i}, PathStore{i}(count)+1];
                                count=count+1;   
                            end
                        elseif temp_y==1
                            if ini_dir_way(PathStore{i}(count)-size_n)==3
                                PathStore{i}=[PathStore{i}, PathStore{i}(count)-size_n];
                                count=count+1;    
                            elseif ini_dir_way(PathStore{i}(count)+1)==1
                                PathStore{i}=[PathStore{i}, PathStore{i}(count)+1];
                                count=count+1;   
                            end
                        elseif temp_y==size_n
                            if ini_dir_way(PathStore{i}(count)-size_n)==3
                                PathStore{i}=[PathStore{i}, PathStore{i}(count)-size_n];
                                count=count+1;    
                            elseif ini_dir_way(PathStore{i}(count)-1)==3
                                PathStore{i}=[PathStore{i}, PathStore{i}(count)-1];
                                count=count+1;   
                            end
                        end
                    end
                    
                    if goal_x>=temp_x&&goal_y>=temp_y %终点在右下角
                        if temp_x==1
                            if ini_dir_way(PathStore{i}(count)+size_n)==1
                                PathStore{i}=[PathStore{i}, PathStore{i}(count)+size_n];
                                count=count+1;
                            elseif ini_dir_way(PathStore{i}(count)+1)==1
                                PathStore{i}=[PathStore{i}, PathStore{i}(count)+1];
                                count=count+1;                         
                            end
                        elseif temp_x==size_m
                            if ini_dir_way(PathStore{i}(count)+1)==1
                                PathStore{i}=[PathStore{i}, PathStore{i}(count)+1];
                                count=count+1;    
                            elseif ini_dir_way(PathStore{i}(count)-size_n)==3
                                PathStore{i}=[PathStore{i}, PathStore{i}(count)-size_n];
                                count=count+1;   
                            end
                        elseif temp_y==1
                            if ini_dir_way(PathStore{i}(count)+1)==1
                                PathStore{i}=[PathStore{i}, PathStore{i}(count)+1];
                                count=count+1;    
                            elseif ini_dir_way(PathStore{i}(count)+size_n)==1
                                PathStore{i}=[PathStore{i}, PathStore{i}(count)+size_n];
                                count=count+1;   
                            end
                        elseif temp_y==size_n
                            if ini_dir_way(PathStore{i}(count)+size_n)==1
                                PathStore{i}=[PathStore{i}, PathStore{i}(count)+size_n];
                                count=count+1;    
                            elseif ini_dir_way(PathStore{i}(count)-1)==3
                                PathStore{i}=[PathStore{i}, PathStore{i}(count)-1];
                                count=count+1;   
                            end
                        end
                    end                 
                end
            else
            %%  在中心部分
            if goal_x<=temp_x&&goal_y<=temp_y %终点在左上角
               if ini_dir_way(PathStore{i}(count)-1)==3
                           PathStore{i}=[PathStore{i}, PathStore{i}(count)-1];
                           count=count+1;
               elseif ini_dir_way(PathStore{i}(count)-size_n)==3
                           PathStore{i}=[PathStore{i}, PathStore{i}(count)-size_n];
                           count=count+1;
               end
            end
            
            if goal_x>=temp_x&&goal_y<=temp_y %终点在左下角
                if ini_dir_way(PathStore{i}(count)-1)==3
                           PathStore{i}=[PathStore{i}, PathStore{i}(count)-1];
                           count=count+1;
                elseif ini_dir_way(PathStore{i}(count)+size_n)==1
                           PathStore{i}=[PathStore{i}, PathStore{i}(count)+size_n];
                           count=count+1;
                end
            end
            
            if goal_x<=temp_x&&goal_y>=temp_y %终点在右上角
                if ini_dir_way(PathStore{i}(count)+1)==1
                           PathStore{i}=[PathStore{i}, PathStore{i}(count)+1];
                           count=count+1;
                elseif ini_dir_way(PathStore{i}(count)-size_n)==3
                           PathStore{i}=[PathStore{i}, PathStore{i}(count)-size_n];
                           count=count+1;
                end              
            end
            
            if goal_x>=temp_x&&goal_y>=temp_y %终点在右下角
                if ini_dir_way(PathStore{i}(count)+1)==1
                           PathStore{i}=[PathStore{i}, PathStore{i}(count)+1];
                           count=count+1;
                elseif ini_dir_way(PathStore{i}(count)+size_n)==1
                           PathStore{i}=[PathStore{i}, PathStore{i}(count)+size_n];
                           count=count+1;
                end
            end
                
            end

        
    end  
end

for i =1:RobotNum
    [Path_x,Path_y]=spread(PathStore{i},size_n);
    Path_num{i}=cat(2,Path_x',Path_y');
end

