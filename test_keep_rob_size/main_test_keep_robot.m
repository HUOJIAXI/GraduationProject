clear;
clc;

disp('控制机器人个数，改变环境大小')

size_max=15;
RobotNum_total=4;

run_time=zeros(1,(size_max-1)/2);

i=1;
for size_D_index = 1 : size_max
    
    if mod(size_D_index,2)==1 && size_D_index > 1
        
        [D]=construct_D(size_D_index);
        
        size_D=size(D,1);

        [Start_ori,Goal_ori]=rand_Goal_Start(D,RobotNum_total); % 随机生成一组测试集
        
        ini_x_value=[];
        for i = 1:RobotNum_total
            [ini_x_value]=initial_guess(ini_x_value,Start_ori(i),Goal_ori(i),D);
        end

        disp('===================================');
        disp(['执行规模：',num2str(size_D_index)])

        [ini_dir_way] = initial(size_D_index);
        
        tic

            [PathStore,Path_num,ini_dir_way,ini_dir_rob,ini_x_value,ini_u_value]=IP_solver_single_way_ini_keep_rob(D,Start_ori,Goal_ori,RobotNum_total,size_D,ini_dir_way,ini_x_value);    % 将上一次求解所得ini_dir_way作为原始解输入

        toc

        run_time(i)=toc;
        disp(['执行机器人个数：',num2str(RobotNum_total)])
        disp(['运行时间: ',num2str(toc)])
        i = i+1;        
    end

end

save('run_time.mat')

figure(2)
plot(run_time,'-o')
hold on
p = polyfit((1:(size_max-1)/2),run_time,2);
y1=polyval(p,(1:(size_max-1)/2));
plot(y1)