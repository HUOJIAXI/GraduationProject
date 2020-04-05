%% 机器人规模测试：13*17

clear;
clc;
D = load('tsp_dist_broad.txt'); 
RobotNum_total=15;
RobotNum_start=1;
test_para=10; %% 测试次数

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
            
            [ini_Path_num,ini_PathStore]=initial_x_way(D_reduit,i,Start_test,Goal_test);
            
            for k = 1:i
               [ini_x_value]=initial_guess_heuristic(ini_Path_num{k},ini_x_value,D_reduit);
            end
            
            disp('已完成初始解调用')
                    
             [err,PathStore,Path_num,dir_way,runtime_indi]=IP_solver_single_way_V3_res(D_reduit,Start_test,Goal_test,i,size_D,ini_x_value);
%                 disp(err)
            if err == 0
                break
            end
            
        end


        run_time_global{i,1}(j)=runtime_indi;
        disp(['运行时间: ',num2str(runtime_indi)])
    end
    
    moyen(i)=mean(run_time_global{i,1});
    disp('===================================');
    disp(['平均运行时间: ',num2str(moyen(i))])
    
    [PathStore_new,Path_num_new]=broaden(PathStore,D,i,r_start_ori(test_choix),r_Goal_ori(test_choix));
    
end

var_runtime=zeros(RobotNum_total,1);

for i = 1 :RobotNum_total
    
    moyen(i)=mean(run_time_global{i,1});
    var_runtime(i)= std(run_time_global{i,1});
end

figure(2)

rob=1:1:RobotNum_total;

e=errorbar(rob,moyen,var_runtime,'-o');

e.Color=[0.8500 0.3250 0.0980];

xlabel('机器人个数');ylabel('运行时间/s')

axis([0,RobotNum_total+1,0,max(moyen)+2*max(var_runtime)])

title(['控制环境大小，改变机器人个数，测试环境大小' num2str(m_D) 'X' num2str(n_D) ' 每测试集测试次数' num2str(test_para)])

diary('off')

save('run_time_global_10.mat')


% plotdynamic_tes(D,PathStore_new,Path_num_new,RobotNum_total,Start_ori(test_choix),Goal_ori(test_choix),r_start_ori(test_choix),r_Goal_ori(test_choix),ini_dir_way);

% figure(2)
% plot(run_time,'-o')
% hold on
% p = polyfit((1:RobotNum_total),run_time,1);
% y1=polyval(p,(1:RobotNum_total));
% plot(y1)