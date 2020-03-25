function arrowdesign(dir_way,D)

m=size(D,1);
n=size(D,2);

count=1;

for i = 1:m*n
    [temp_x,temp_y]=spread_sin(i,n);
%     disp(count)
    if mod(temp_x,2)==1
        if mod(temp_y+1,4)==0
            
            if dir_way(count) == 1
         
                arrow([temp_y-1,temp_x-0.5], [temp_y+1,temp_x-0.5])
                count=count+1;
                continue
                
            end
            
            if dir_way(count) == 3
                arrow([temp_y+1,temp_x-0.5], [temp_y-1,temp_x-0.5])
                count=count+1;
                continue
            end
            
            if dir_way(count) == 0
                count=count+1;
                continue
            end
            
        end
    end
    
    if mod(temp_x,2)==0
        if mod(temp_y-1,4)==0
            
            if dir_way(count) == 1
                arrow([temp_y-0.5,temp_x-1], [temp_y-0.5,temp_x])
                count=count+1;
                continue
            end
            
            if dir_way(count) == 3
                arrow([temp_y-0.5,temp_x], [temp_y-0.5,temp_x-1])
                count=count+1;
                continue
            end
            
            if dir_way(count) == 0
                count=count+1;
                continue
            end
        end
        
    end

end