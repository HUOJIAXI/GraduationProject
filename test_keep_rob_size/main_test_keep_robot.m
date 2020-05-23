clear;
clc;

disp('控制机器人个数，改变环境大小')

disp(datestr(now));

size_max=13;
RobotNum_total=15;
% test_size=2;
run_fois=5;

% run_time=zeros(1,(size_max-1)/2);

run_time_global=cell((size_max-1)/2-1,1);

run_time_global_non=cell((size_max-1)/2-1,1);

moyen=zeros(1,(size_max-1)/2-1);


for size_D_index = 5 : size_max
    
    if mod(size_D_index,2)==1 && size_D_index >= 5
        
        [D]=construct_D(size_D_index);

        m=size(D,1);
        n=size(D,2);
        % a=floor(m/2);
        % b=floor(n/2);
        %rng shuffle

        nobs=[];
        obs=[];
        for i = 1:m
            for j = 1:n
                if D(i,j) == 1
                    obs=[obs j+(i-1)*n];
        %             [obs_x,obs_y]=spread(obs,m);
                else
                    nobs=[nobs j+(i-1)*n];
                end
            end
        end

        count_nobs=length(nobs);
       
        size_D=size(D,1);
        
            [Goal_ori,Start_ori]=rand_Goal_Start_e(D,count_nobs);
        
        for j =1:run_fois
             disp('===================================');
             disp(['执行次数：',num2str(j)])
             
             err=1;
             while err==1
                 if size_D_index==3
                      test_choix=randperm(8,RobotNum_total);
                 else
                      test_choix=randperm(count_nobs,RobotNum_total);
                 end
                
                Start_test=Start_ori(test_choix);
%                 disp(Start_test)
                Goal_test=Goal_ori(test_choix);
                
                [err_heu,ini_Path_num,ini_PathStore]=initial_x_way(D,RobotNum_total,Start_test,Goal_test);
                
                if err_heu==1
                    continue
                end
                
                ini_x_value=[];
                
                for i = 1:RobotNum_total                  
                        [ini_x_value]=initial_guess_heuristic(ini_Path_num{i},ini_x_value,D);
%                     else
%                         [ini_x_value]=initial_guess(ini_x_value,Start_test(i),Goal_test(i),D);
%                     end
                end
                
                disp('===================================');
                disp(['执行规模：',num2str(size_D_index)])
                disp('已完成初始解设定')
                [run_time_indi,err,PathStore,Path_num,ini_dir_way,ini_dir_rob,ini_x_value]=IP_solver_single_way_ini_keep_rob(D,Start_test,Goal_test,RobotNum_total,size_D,ini_x_value);    % 将上一次求解所得ini_dir_way作为原始解输入
                
      
%             disp(['执行机器人个数：',num2str(RobotNum_total)])
                disp(['运行时间: ',num2str(run_time_indi)])

                run_time_global{(size_D_index-1)/2-1,1}(j)=run_time_indi;
            
                
                disp('===================================');
                disp(['执行规模：',num2str(size_D_index)])
                disp('已完成初始解设定')
                [run_time_indi_non,err_non,~,~,~,~,~]=IP_solver_single_way_ini_keep_rob_non(D,Start_test,Goal_test,RobotNum_total,size_D,ini_x_value);               
                
                disp(['运行时间: ',num2str(run_time_indi_non)])
            
                run_time_global_non{(size_D_index-1)/2-1,1}(j)=run_time_indi_non;

                if err==0&&err_non==0
                    break
                end
                
             end     
        end
        
        moyen((size_D_index-1)/2-1)=mean(run_time_global{(size_D_index-1)/2-1,1});
        disp('===================================');
        disp(['平均运行时间: ',num2str(moyen((size_D_index-1)/2-1))])
    end

end

var_runtime=zeros((size_max-1)/2-1,1);

for i = 1 :(size_max-1)/2-1
    
    moyen(i)=mean(run_time_global{i,1});
    var_runtime(i)= std(run_time_global{i,1});
end

figure(2)


rob=1:1:(size_max-1)/2-1;

e1=errorbar(rob,moyen,var_runtime,'-o'); e1.Color='r';

xlabel('环境规模');ylabel('求解时间/s')

title(['控制机器人个数，改变环境大小，测试机器人个数' num2str(RobotNum_total)])

axis([0,(size_max-1)/2-1+1,0,max(moyen)+2*max(var_runtime)])

xticklabels({'0','5X5','7X7','9X9','11X11','13X13'})
%%
hold on

var_runtime_non=zeros((size_max-1)/2-1,1);

for i = 1 :(size_max-1)/2-1
    
    moyen(i)=mean(run_time_global_non{i,1});
    var_runtime_non(i)= std(run_time_global{i,1});
end

rob=1:1:(size_max-1)/2-1;

e2=errorbar(rob,moyen,var_runtime_non,'-o');e2.Color='b';

xlabel('环境规模');ylabel('求解时间/s')

title(['环境规模鲁棒性测试：控制机器人个数，改变环境大小，测试机器人个数' num2str(RobotNum_total)])

axis([0,(size_max-1)/2-1+1,0,max(moyen)+2*max(var_runtime)])

xticklabels({'0','5X5','7X7','9X9','11X11','13X13'})

ylim('auto')

legend('启发初始解下OWIP','未调用启发初始解的OWIP')

% save('run_time_sup.mat')

save('run_time_sup.mat')