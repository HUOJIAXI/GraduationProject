clc;
clear;

D=load('tsp_map.txt');
load('test_result_15.mat')
m=size(D,1);
n=size(D,2);

RobotNum_test=10;
RobotNum_start=1;
test_para=5;
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

% [Goal_ori,Start_ori,r_start_ori,r_Goal_ori]=rand_Goal_Start_op(D,test_size);

disp('测试集生成')

run_time_dyn=cell(RobotNum_test-RobotNum_start+1,1);

dis_dyn=cell(RobotNum_test-RobotNum_start+1,1);

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
            disp('动态算法测试开始')
             try
                 Start_test=r_start_ori(test_choix);
                 Goal_test=r_Goal_ori(test_choix);
                 tic
                 encarde=3;
                 [PathStore_dyn,Path_num_dyn]=MASPP_IP_div_op(D,i,Start_test,Goal_test,encarde);
                 toc
             catch
                 yalmip('clear')
                 err_gen=1;
                 continue
             end
             
             [dis_total,Path_new_dyn]=dis_cal(i,Goal_test,Path_num_dyn,PathStore_dyn);
             
            run_time_dyn{i,1}(j)=toc;
            dis_dyn{i,1}(j)=dis_total;
             
            disp('动态测试已完成')
            err_gen=0;
        end
    end
end

var_time_dyn=zeros(RobotNum_test-RobotNum_start+1,1);

moyen_dyn=zeros(RobotNum_test-RobotNum_start+1,1);

for i = RobotNum_start :RobotNum_test
    
    moyen_dyn(i)=mean(run_time_dyn{i,1});
    var_time_dyn(i)= std(run_time_dyn{i,1});

end

save('test_result_dyn.mat')

figure(1)

rob=RobotNum_start:1:RobotNum_test;

e1=errorbar(rob,moyen_dyn,var_time_dyn,'-o');

e1.Color=[0.8500 0.3250 0.0980];

hold on;

e2=errorbar(rob,moyen_oneway,var_time_oneway,'-o');

e2.Color=[0 0.4470 0.7410];

hold on;

e3=errorbar(rob,moyen_tra,var_time_tra,'-o');

e3.Color=[0.6350 0.0780 0.1840];

axis([0,RobotNum_test+1,min(moyen_tra)-2*max(var_time_tra),max(moyen_tra)+2*max(var_time_tra)])

title(['运行时间测试：控制环境大小，改变机器人个数，测试环境大小' num2str(m_D) 'X' num2str(n_D) ' 每测试集测试次数' num2str(test_para)])
xlabel('机器人个数');ylabel('运行时间/s')

legend('动态算法','改进单行线模型','传统IP模型')

hold off;

plot_dis(RobotNum_test,RobotNum_start,dis_dyn,dis_oneway,dis_tra,m_D,n_D,test_para)
plot_dis_reel(RobotNum_test,RobotNum_start,dis_dyn,dis_oneway,dis_tra,m_D,n_D,test_para)
