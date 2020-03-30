%% 机器人规模测试：13*17

clear;
clc;
D = load('tsp_dist_broad.txt'); 
RobotNum_total=15;
RobotNum_start=5;
test_para=1; %% 测试次数

nobs=[];
obs=[];
m_D=size(D,1);
n_D=size(D,2);
for i = 1:m_D
    for j = 1:n_D
        if D(i,j) == 1
            obs=[obs j+(i-1)*n_D]; % 障碍物所在位置
%             [obs_x,obs_y]=spread(obs,m);
        else
            nobs=[nobs j+(i-1)*n_D];
        end
    end
end


test_size=length(obs);

% [Start_ori,Goal_ori]=rand_Goal_Start(D,RobotNum_total*test_size); % 随机生成一组测试集
[Goal_ori,Start_ori,r_start_ori,r_Goal_ori]=rand_Goal_Start_op(D,test_size,3);

run_time_global=cell(RobotNum_total,1);

% run_time=zeros(1,RobotNum_total);

moyen=zeros(1,RobotNum_total);

[ini_dir_way] = initial();
diary('res_keep_map.txt')
disp(datestr(now));
for i = RobotNum_start:RobotNum_total
    
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
        
        err=1;
        while err==1
            
            test_choix=randperm(test_size,i);
            [Start_test,Goal_test,start_sp,goal_sp,D_reduit] = reduit(r_start_ori(test_choix),r_Goal_ori(test_choix),D);

            size_D=size(D_reduit,2);

            ini_x_value=[];

            for k = 1:i
                [ini_x_value]=initial_guess(ini_x_value,Start_test(i),Goal_test(i),D_reduit);
            end
                    
             [run_time_indi,err,PathStore,Path_num,ini_dir_way,ini_dir_rob,ini_x_value]=IP_solver_single_way_test(D_reduit,Start_test,Goal_test,i,size_D,ini_dir_way,ini_x_value);
%                 disp(err)
            if err == 0
                break
            end
            
        end


        run_time_global{i,1}(j)=run_time_indi;
        disp(['运行时间: ',num2str(run_time_indi)])
    end
    
    moyen(i)=mean(run_time_global{i,1});
    disp('===================================');
    disp(['平均运行时间: ',num2str(moyen(i))])
    
    [PathStore_new,Path_num_new]=broaden(PathStore,D,i,r_start_ori(test_choix),r_Goal_ori(test_choix));
    
end

plotdynamic_tes(D,PathStore_new,Path_num_new,RobotNum_total,Start_ori(test_choix),Goal_ori(test_choix),r_start_ori(test_choix),r_Goal_ori(test_choix),ini_dir_way);

var_runtime=zeros(RobotNum_total,1);

for i = 1 :RobotNum_total
    
    moyen(i)=mean(run_time_global{i,1});
    var_runtime(i)= std(run_time_global{i,1});
end

figure(2)

rob=1:1:RobotNum_total;

errorbar(rob,moyen,var_runtime,'-o');

xlabel('机器人个数');ylabel('运行时间/s')

axis([0,RobotNum_total+1,0,max(moyen)+2*max(var_runtime)])

diary('off')

save('run_time_global.mat')

% figure(2)
% plot(run_time,'-o')
% hold on
% p = polyfit((1:RobotNum_total),run_time,1);
% y1=polyval(p,(1:RobotNum_total));
% plot(y1)