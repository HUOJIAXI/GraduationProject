
clc;
clear;

D = load('tsp_map.txt'); 
m=size(D,1);
n=size(D,2);

RobotNum_test=15;
RobotNum_start=1;
test_para=10;
disp('选取测试集')

nobs=[];
obs=[];

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

[Goal_ori,Start_ori,r_start_ori,r_Goal_ori]=rand_Goal_Start_op(D,test_size);

disp('测试集生成')

run_time_oneway=cell(RobotNum_test-RobotNum_start+1,1);

run_time_oneway_heu=cell(RobotNum_test-RobotNum_start+1,1);

disp('测试开始')

disp(datestr(now));

for i = RobotNum_start:RobotNum_test
    for j = 1:test_para
        
        disp('===================================');
        disp(['执行机器人个数：',num2str(i)])
        disp(['执行次数：',num2str(j)])
        
        err_gen=1;
        while err_gen==1        
            
            test_choix=randperm(test_size,i);
           
             
            disp('===================================');
            disp('单行线法则模型测试开始')

            [Start_test,Goal_test,D_reduit] = reduit(r_start_ori(test_choix),r_Goal_ori(test_choix),D);
            size_D=size(D_reduit,2);

             [err,PathStore,Path_num,dir_way,runtime_indi]=IP_solver_single_way_V3_res_noheu(D_reduit,Start_test,Goal_test,i,size_D);
%                 disp(err)
            if err == 1
                yalmip('clear')
                err_gen=1;
                continue
            end
            
            [PathStore_oneway,Path_num_oneway]=broaden(PathStore,D,i,r_start_ori(test_choix),r_Goal_ori(test_choix));
            
            run_time_oneway{i,1}(j)=runtime_indi;
            
            disp('===================================');
            disp('启发式单行线法则模型测试开始')
            
             ini_x_value=[];

            [ini_Path_num,ini_PathStore]=initial_x_way(D_reduit,i,Start_test,Goal_test);

            for k = 1:i
               [ini_x_value]=initial_guess_heuristic(ini_Path_num{k},ini_x_value,D_reduit);
            end
            disp('已完成初始解调用')
            
             [err,PathStore,Path_num,dir_way,runtime_indi]=IP_solver_single_way_V3_res(D_reduit,Start_test,Goal_test,i,size_D,ini_x_value);
%                 disp(err)
            if err == 1
                yalmip('clear')
                err_gen=1;
                continue
            end
            
            [PathStore_oneway,Path_num_oneway]=broaden(PathStore,D,i,r_start_ori(test_choix),r_Goal_ori(test_choix));
            
            run_time_oneway_heu{i,1}(j)=runtime_indi;
            
            err_gen=0;
        end
    end
end

save('test_heu.mat')
var_time_oneway=zeros(RobotNum_test-RobotNum_start+1,1);
var_time_oneway_heu=zeros(RobotNum_test-RobotNum_start+1,1);

moyen_oneway=zeros(RobotNum_test-RobotNum_start+1,1);
moyen_oneway_heu=zeros(RobotNum_test-RobotNum_start+1,1);

for i = RobotNum_start :RobotNum_test
      
    moyen_oneway(i)=mean(run_time_oneway{i,1});
    var_time_oneway(i)= std(run_time_oneway{i,1});

    moyen_oneway_heu(i)=mean(run_time_tra{i,1});
    var_time_oneway_heu(i)= std(run_time_tra{i,1});
    
end

figure(1)

rob=RobotNum_start:1:RobotNum_test;

e1=errorbar(rob,moyen_oneway_heu,var_time_oneway_heu,'-o');

e1.Color=[0.8500 0.3250 0.0980];

hold on;

e2=errorbar(rob,moyen_oneway,var_time_oneway,'-o');

e2.Color=[0 0.4470 0.7410];

hold on;

axis([0,RobotNum_test+1,min(moyen_oneway,moyen_oneway_heu)-2*max(var_time_oneway),max(moyen_oneway,moyen_oneway_heu)+2*max(var_time_oneway_heu)])

title(['OWIP算法求解时间测试：控制环境大小，改变机器人个数，测试环境大小' num2str(m_D) 'X' num2str(n_D) ' 每测试集测试次数' num2str(test_para)])
xlabel('机器人个数');ylabel('求解时间/s')

legend('启发初始解下OWIP','OWIP')

hold off;



