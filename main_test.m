clear;
clc;
D = load('tsp_dist.txt'); 
size_D=size(D,1);
RobotNum_total=20;

[Start_ori,Goal_ori]=rand_Goal_Start(D,RobotNum_total); % 随机生成一组测试集

run_time=zeros(1,RobotNum_total-3);

[ini_dir_way] = initial();

for i = 4:RobotNum_total
    disp(['执行次数：',num2str(i-3)])
    Start_test = Start_ori(1:i);
    Goal_test = Goal_ori(1:i);
    if i == 4
    tic

        [PathStore,Path_num,ini_dir_way,ini_dir_rob]=IP_solver_single_way_ini(D,Start_test,Goal_test,i,size_D,ini_dir_way);    % 将上一次求解所得ini_dir_way作为原始解输入

    toc
    else
        tic
         [PathStore,Path_num,ini_dir_way,ini_dir_rob]=IP_solver_single_way_test(D,Start_test,Goal_test,i,size_D,ini_dir_way,ini_dir_rob);
        toc
    end
    run_time(i-3)=toc;
    disp(['机器人序号：',i])
    disp(['运行时间: ',num2str(toc)])
end

save('run_time.mat')

figure(1)
plot(run_time,'-o')
hold on
p = polyfit((1:RobotNum_total-3),run_time,1);
y1=polyval(p,(1:RobotNum_total-3));
plot(y1)