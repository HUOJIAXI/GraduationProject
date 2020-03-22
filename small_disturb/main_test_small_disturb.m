clear;
clc;

size_D_index=9;
[D]=construct_D(size_D_index);
size_D=size(D,1);
RobotNum_total=6;

[Start_test,Goal_test]=rand_Goal_Start(D,RobotNum_total); % 随机生成一组测试集

run_time=zeros(1,2);

[ini_dir_way] = initial(size_D);

diary('res.txt')

disp(datestr(now));

disp('===================================');


tic

    [PathStore_1,Path_num_1,ini_dir_way,ini_dir_rob,ini_x_value,ini_u_value]=IP_solver_single_way_ini_test_rob_disturb(D,Start_test,Goal_test,RobotNum_total,size_D,ini_dir_way);    % 将上一次求解所得ini_dir_way作为原始解输入

toc

run_time(1)=toc;

figure(1)
show_graph(D,RobotNum_total,PathStore_1)

disp('发生扰动前起点')
disp(Start_test)
disp('发生扰动前终点')
disp(Goal_test)

disp('===================================');

disp('发生小扰动')

[Start_test,Goal_test,dis_num]=disturb(Start_test,Goal_test,D,RobotNum_total);

disp('===================================');

disp('发生扰动后起点')
disp(Start_test)
disp('===================================');
disp('发生扰动后终点')
disp(Goal_test)

disp('===================================');

tic
 [PathStore_2,Path_num_2,ini_dir_way,ini_dir_rob,ini_x_value,ini_u_value]=IP_solver_single_way_test(D,Start_test,Goal_test,RobotNum_total,size_D,ini_dir_way,ini_dir_rob,ini_x_value,ini_u_value);
toc
run_time(2)=toc;

figure(2)
show_graph(D,RobotNum_total,PathStore_2)

disp('===================================');
disp(['产生扰动机器人序号：',num2str(dis_num)])
disp(['运行时间差: ',num2str(abs(run_time(1)-run_time(2)))])

diary('off');

% plotdynamic(D,PathStore,Path_num,RobotNum_total,Start_ori,Goal_ori);

save('run_time.mat')

figure(3)
plot(run_time,'-o')
% hold on
% p = polyfit((1:2),run_time,1);
% y1=polyval(p,(1:2));
% plot(y1)