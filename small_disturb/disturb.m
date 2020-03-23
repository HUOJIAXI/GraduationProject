function [Start_test,Goal_test,dis_num,ini_dir_way,ini_dir_rob,ini_x_value,ini_u_value]=disturb(Start_test,Goal_test,D,RobotNum_total,ini_dir_way,ini_dir_rob,ini_x_value,ini_u_value)

D_size=size(D,1);

dis_num=1+round(rand(1,1)*(RobotNum_total-1));

dis_flag=round(rand(1,1));
% 
% ini_u_value(dis_num,:)=0;
% ini_x_value(:,:,dis_num)=0;
% ini_dir_rob(dis_num,:)=0;
% 
% [ini_dir_way_res] = initial(D_size);
% ini_dir_way=ini_dir_way_res;

if dis_flag==0
    flag=0;
    while flag==0
        [rand_x,rand_y]=spread_sin(Start_test(dis_num),D_size);

        if mod(rand_x,2) ==1
            if rand_y<D_size
                rand_y = rand_y + 1;
            elseif rand_y == D_size
                rand_y = rand_y - 1;
            end

        end

        if mod(rand_x,2) == 0
            if rand_x<D_size
                rand_x = rand_x + 1;
            elseif rand_x == D_size
                rand_x = rand_x - 1;
            end

        end

        Start_test(dis_num) = rand_y + (rand_x-1)*D_size;
        
        if  Start_test(dis_num) == Goal_test(dis_num)
            continue
        else
            flag=1;
        end
    end
    
    
else
    flag = 0;
    while flag ==0
        [rand_x,rand_y]=spread_sin(Goal_test(dis_num),D_size);

        if mod(rand_x,2) ==1
            if rand_y<D_size
                rand_y = rand_y + 1;
            elseif rand_y == D_size
                rand_y = rand_y - 1;
            end

        end

        if mod(rand_x,2) == 0
            if rand_x<D_size
                rand_x = rand_x + 1;
            elseif rand_x == D_size
                rand_x = rand_x - 1;
            end

        end

        Goal_test(dis_num) = rand_y + (rand_x-1)*D_size;
        
        if  Start_test(dis_num) == Goal_test(dis_num)
            continue
        else
            flag=1;
        end
    end
    
end
