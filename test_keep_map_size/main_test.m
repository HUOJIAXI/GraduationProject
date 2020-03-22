clear;
clc;
D = load('tsp_dist.txt'); 
size_D=size(D,1);
RobotNum_total=20;

[Start_ori,Goal_ori]=rand_Goal_Start(D,RobotNum_total); % 随机生成一组测试集

run_time=zeros(1,RobotNum_total);

[ini_dir_way] = initial();

for i = 1:RobotNum_total
    disp('===================================');
    disp(['执行次数：',num2str(i)])
    Start_test = Start_ori(1:i);
    Goal_test = Goal_ori(1:i);
    if i == 1
    tic

        [PathStore,Path_num,ini_dir_way,ini_dir_rob,ini_x_value,ini_u_value]=IP_solver_single_way_ini(D,Start_test,Goal_test,i,size_D,ini_dir_way);    % 将上一次求解所得ini_dir_way作为原始解输入

    toc
    else
        tic
         [PathStore,Path_num,ini_dir_way,ini_dir_rob,ini_x_value,ini_u_value]=IP_solver_single_way_test(D,Start_test,Goal_test,i,size_D,ini_dir_way,ini_dir_rob,ini_x_value,ini_u_value);
        toc
    end
    run_time(i)=toc;
    disp(['执行机器人个数：',num2str(i)])
    disp(['运行时间: ',num2str(toc)])
end

plotdynamic(D,PathStore,Path_num,RobotNum_total,Start_ori,Goal_ori);

save('run_time.mat')

figure(2)
plot(run_time,'-o')
hold on
p = polyfit((1:RobotNum_total),run_time,1);
y1=polyval(p,(1:RobotNum_total));
plot(y1)