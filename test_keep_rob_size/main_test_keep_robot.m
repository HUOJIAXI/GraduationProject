clear;
clc;

disp('控制机器人个数，改变环境大小')

disp(datestr(now));

size_max=15;
RobotNum_total=6;
% test_size=2;
run_fois=10;

run_time=zeros(1,(size_max-1)/2);

run_time_global=cell((size_max-1)/2,1);

moyen=zeros(1,(size_max-1)/2);

for size_D_index = 1 : size_max
    
    if mod(size_D_index,2)==1 && size_D_index > 1
        
        [D]=construct_D(size_D_index);
       
        size_D=size(D,1);
        if size_D_index == 3
        [Start_ori,Goal_ori]=rand_Goal_Start_e(D,8); % 随机生成一组测试集
        else
            [Start_ori,Goal_ori]=rand_Goal_Start_e(D,(size_D_index-round(RobotNum_total/3))*RobotNum_total);
        end
        
        for j =1:run_fois
             disp('===================================');
             disp(['执行次数：',num2str(j)])
             
             err=1;
             while err==1
                 if size_D_index==3
                      test_choix=randperm(8,RobotNum_total);
                 else
                      test_choix=randperm((size_D_index-round(RobotNum_total/3))*RobotNum_total,RobotNum_total);
                 end
                
                Start_test=Start_ori(test_choix);
%                 disp(Start_test)
                Goal_test=Goal_ori(test_choix);
                
                ini_x_value=[];
                
                for i = 1:RobotNum_total
                    [ini_x_value]=initial_guess(ini_x_value,Start_test(i),Goal_test(i),D);
                end

                disp('===================================');
                disp(['执行规模：',num2str(size_D_index)])

                [run_time_indi,err,PathStore,Path_num,ini_dir_way,ini_dir_rob,ini_x_value]=IP_solver_single_way_ini_keep_rob(D,Start_test,Goal_test,RobotNum_total,size_D,ini_x_value);    % 将上一次求解所得ini_dir_way作为原始解输入
                
                if err==0
                    break
                end
                
             end           
%             disp(['执行机器人个数：',num2str(RobotNum_total)])
            disp(['运行时间: ',num2str(run_time_indi)])
            
            run_time_global{(size_D_index-1)/2,1}(j)=run_time_indi;
            
        end
        
        moyen((size_D_index-1)/2)=mean(run_time_global{(size_D_index-1)/2,1});
        disp('===================================');
        disp(['平均运行时间: ',num2str(moyen((size_D_index-1)/2))])
    end

end

var_runtime=zeros((size_max-1)/2,1);

for i = 1 :(size_max-1)/2
    
    moyen(i)=mean(run_time_global{i,1});
    var_runtime(i)= std(run_time_global{i,1});
end

figure(2)


rob=1:1:(size_max-1)/2;

errorbar(rob,moyen,var_runtime,'-o');

xlabel('环境规模');ylabel('运行时间/s')

title(['控制机器人个数，改变环境大小，测试机器人个数' num2str(RobotNum_total)])

axis([0,(size_max-1)/2+1,0,max(moyen)+2*max(var_runtime)])

xticklabels({'0','3X3','5X5','7X7','9X9','11X11','13X13','15X15'})


save('run_time.mat')