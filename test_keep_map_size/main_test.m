clear;
clc;
D = load('tsp_dist.txt'); 
size_D=size(D,1);
RobotNum_total=10;
test_para=10;

test_size=3;

[Start_ori,Goal_ori]=rand_Goal_Start(D,RobotNum_total*test_size); % 随机生成一组测试集

run_time_global=cell(RobotNum_total,1);

% run_time=zeros(1,RobotNum_total);

moyen=zeros(1,RobotNum_total);

[ini_dir_way] = initial();
diary('res_keep_map.txt')
disp(datestr(now));
for i = 1:RobotNum_total
    
%     if i > 1
%     [ini_x_value]=initial_guess(ini_x_value,Start_test(i),Goal_test(i),D);
%     else
%         ini_x_value=[];
%         [ini_x_value]=initial_guess(ini_x_value,Start_test(i),Goal_test(i),D);
%     end
%         
    for j = 1:test_para
        disp('===================================');
        disp(['执行机器人个数：',num2str(i)])
        disp(['执行次数：',num2str(j)])
        
        test_choix=randperm(RobotNum_total*test_size,i);
        Start_test = Start_ori(test_choix);
        Goal_test = Goal_ori(test_choix);

        ini_x_value=[];
        
        for k = 1:i
            [ini_x_value]=initial_guess(ini_x_value,Start_test(i),Goal_test(i),D);
        end
        
        if i == 1
        tic

            [PathStore,Path_num,ini_dir_way,ini_dir_rob,ini_x_value]=IP_solver_single_way_ini(D,Start_test,Goal_test,i,size_D,ini_dir_way,ini_x_value);    % 将上一次求解所得ini_dir_way作为原始解输入

        toc
        else
            tic
             [PathStore,Path_num,ini_dir_way,ini_dir_rob,ini_x_value]=IP_solver_single_way_test(D,Start_test,Goal_test,i,size_D,ini_dir_way,ini_dir_rob,ini_x_value);
            toc
        end

        run_time_global{i,1}(j)=toc;
        disp(['运行时间: ',num2str(toc)])
    end
    
    moyen(i)=mean(run_time_global{i,1});
    disp('===================================');
    disp(['平均运行时间: ',num2str(moyen(i))])
    
end

% plotdynamic(D,PathStore,Path_num,RobotNum_total,Start_ori,Goal_ori);

var_runtime=zeros(RobotNum_total,1);

for i = 1 :RobotNum_total
    var_runtime(i)= std(run_time_global{i,1});
end

rob=1:1:RobotNum_total;

errorbar(rob,moyen,var_runtime,'-o');

xlabel('机器人个数');ylabel('运行时间/s')

axis([0,11,0,max(moyen)+2*max(var_runtime)])

diary('off')

save('run_time_global.mat')

% figure(2)
% plot(run_time,'-o')
% hold on
% p = polyfit((1:RobotNum_total),run_time,1);
% y1=polyval(p,(1:RobotNum_total));
% plot(y1)