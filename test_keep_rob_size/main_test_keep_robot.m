clear;
clc;

disp('控制机器人个数，改变环境大小')

disp(datestr(now));

size_max=21;
RobotNum_total=10;
% test_size=2;
run_fois=5;

run_time=zeros(1,(size_max-1)/2);

run_time_global=cell((size_max-1)/2-3,1);

moyen=zeros(1,(size_max-1)-3/2);

for size_D_index = 19 : size_max
    
    if mod(size_D_index,2)==1 && size_D_index >= 9
        
        [D]=construct_D(size_D_index);
       
        size_D=size(D,1);
        
            [Goal_ori,Start_ori]=rand_Goal_Start_e(D,round(sqrt(size_D)/2)*RobotNum_total);
        
        for j =1:run_fois
             disp('===================================');
             disp(['执行次数：',num2str(j)])
             
             err=1;
             while err==1
                 if size_D_index==3
                      test_choix=randperm(8,RobotNum_total);
                 else
                      test_choix=randperm(round(sqrt(size_D)/2)*RobotNum_total,RobotNum_total);
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
                
                if err==0
                    break
                end
                
             end           
%             disp(['执行机器人个数：',num2str(RobotNum_total)])
            disp(['运行时间: ',num2str(run_time_indi)])
            
            run_time_global{(size_D_index-1)/2-3,1}(j)=run_time_indi;
            
        end
        
        moyen((size_D_index-1)/2-3)=mean(run_time_global{(size_D_index-1)/2-3,1});
        disp('===================================');
        disp(['平均运行时间: ',num2str(moyen((size_D_index-1)/2))])
    end

end

var_runtime=zeros((size_max-1)/2-3,1);

for i = 1 :(size_max-1)/2-3
    
    moyen(i)=mean(run_time_global{i,1});
    var_runtime(i)= std(run_time_global{i,1});
end

figure(2)


rob=1:1:(size_max-1)/2-3;

errorbar(rob,moyen,var_runtime,'-o');

xlabel('环境规模');ylabel('运行时间/s')

title(['控制机器人个数，改变环境大小，测试机器人个数' num2str(RobotNum_total)])

axis([0,(size_max-1)/2-3+1,0,max(moyen)+2*max(var_runtime)])

xticklabels({'0','9X9','11X11','13X13','15X15','17X17','19X19'})


save('run_time.mat')