clear;
clc;

size_D_index=11;
test_disturb=5;
[D]=construct_D(size_D_index);
size_D=size(D,1);
RobotNum_total=10;
num_test=10;
m = size(D,1);
n = size(D,2);

[Start_test,Goal_test]=rand_Goal_Start_obs(D,RobotNum_total); % 随机生成一组测试集

run_time=zeros(1+test_disturb,1);
var_runtime=zeros(test_disturb+1,1);

run_time_test=zeros(test_disturb,num_test);

% [ini_dir_way] = initial(size_D);

diary('res.txt')

disp(datestr(now));

disp('===================================');
[ini_Path_num,~]=initial_x_way(D,RobotNum_total,Start_test,Goal_test);
disp('已完成初始解设定')
ini_x_value=[];
for i = 1:RobotNum_total
    [ini_x_value]=initial_guess_heuristic(ini_Path_num{i},ini_x_value,D);
end


    [runtime,PathStore_1,Path_num_1,ini_dir_way,ini_dir_rob,~]=IP_solver_single_way_ini_test_rob_disturb(D,Start_test,Goal_test,RobotNum_total,size_D,ini_x_value);    % 将上一次求解所得ini_dir_way作为原始解输入


run_time(1)=runtime;

% figure(1)
% plot_ind(D,RobotNum_total,n,m,Start_test,Goal_test,PathStore_1,ini_dir_way)

disp('发生扰动前起点')
disp(Start_test)
disp('发生扰动前终点')
disp(Goal_test)

disp('===================================');
    Start_ori=Start_test;
    Goal_ori=Goal_test;
    
for num_test_ind=1:num_test
    
    
    
    disp('发生小扰动')

    [Start_test,Goal_test,dis_num]=Test_disturb(Start_ori, Goal_ori,D);
    
    for disturb=1:test_disturb

        disp('===================================');

        disp('发生扰动后起点')
        disp(Start_test)
        disp('===================================');
        disp('发生扰动后终点')
        disp(Goal_test)

        disp('===================================');

        [ini_Path_num,ini_PathStore]=initial_x_way(D,RobotNum_total,Start_test,Goal_test);
        disp('已完成初始解设定')
        ini_x_value=[];
        for i = 1:RobotNum_total
            [ini_x_value]=initial_guess_heuristic(ini_Path_num{i},ini_x_value,D);
        end

         [runtime,PathStore_2,Path_num_2,ini_dir_way,~,ini_x_value]=IP_solver_single_way_ini_test_rob_disturb(D,Start_test,Goal_test,RobotNum_total,size_D,ini_x_value);
        %  [PathStore_2,Path_num_2,ini_dir_way,ini_dir_rob,ini_x_value,ini_u_value]=IP_solver_single_way_test(D,Start_test,Goal_test,RobotNum_total,size_D,ini_dir_way,ini_dir_rob,ini_x_value,ini_u_value);

        run_time_test(disturb,num_test_ind)=runtime;

        % figure(3)
        % plot_ind(D,RobotNum_total,n,m,Start_test,Goal_test,PathStore_2,ini_dir_way)

        disp('===================================');
        disp(['产生扰动机器人序号：',num2str(dis_num)])
        disp(['运行时间差: ',num2str(run_time_test(disturb,num_test_ind)-run_time(1))])
        disp(['测试次数：',num2str(num_test_ind)])
        disp(['扰动次数：',num2str(disturb)])
        [Start_test,Goal_test,dis_num]=Test_disturb(Start_test,Goal_test,D);
    end
    
end

diary('off');

var_runtime(1)=0;
for i = 1:test_disturb
    run_time(i+1)=mean(run_time_test(i,:));
    var_runtime(i+1)= std(run_time_test(i,:));
end
    
% plotdynamic(D,PathStore,Path_num,RobotNum_total,Start_ori,Goal_ori);

save('run_time9.mat')

figure(3)

% figure(4)
rob=1:test_disturb+1;
e=errorbar(rob,run_time,var_runtime,'-o');
e.Color=[0.6350 0.0780 0.1840];
hold on
plot(run_time(1),'Marker','*','MarkerSize',10,'MarkerEdgeColor','b')
hold on 
xlim([0,test_disturb+2])
ylim([0,2*max(run_time)])
yticks(0:1:2*max(run_time))
% xlim([0.5,2.5])
% ylim([min(run_time(1)-var_runtime(2),run_time(2)-var_runtime(2)-var_runtime(2)),max(run_time(1)+var_runtime(2),run_time(2)+var_runtime(2)+var_runtime(2))])
title(['控制环境大小 对起点终点进行扰动 测试环境大小' num2str(size_D_index) 'X' num2str(size_D_index) ' 每测试集测试次数' num2str(num_test) ' 测试机器人个数' num2str(RobotNum_total) ' 发生连续扰动次数' num2str(test_disturb) ])
xticklabels({'0','扰动前','第1次扰动','第2次扰动','第3次扰动','第4次扰动','第5次扰动'})
xlabel('发生连续扰动')
ylabel('求解时间/s')
hold off
% hold on
% p = polyfit((1:2),run_time,1);
% y1=polyval(p,(1:2));
% plot(y1)