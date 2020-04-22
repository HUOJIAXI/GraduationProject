function plot_dis(RobotNum_test,RobotNum_start,dis_dyn,dis_oneway,dis_tra,m_D,n_D,test_para)

figure(2)
ecart_dis_dyn=cell(RobotNum_test-RobotNum_start+1,1);
ecart_dis_oneway=cell(RobotNum_test-RobotNum_start+1,1);
per_dis_dyn=cell(RobotNum_test-RobotNum_start+1,1);
per_dis_oneway=cell(RobotNum_test-RobotNum_start+1,1);
moyen_dyn=zeros(RobotNum_test-RobotNum_start+1,1);
moyen_oneway=zeros(RobotNum_test-RobotNum_start+1,1);



for i = RobotNum_start:RobotNum_test
    
    ecart_dis_dyn{i,1}=dis_dyn{i,1}-dis_tra{i,1};
    ecart_dis_oneway{i,1}=dis_oneway{i,1}-dis_tra{i,1};
end

ecart_dis_dyn_mat = cell2mat(ecart_dis_dyn);
ecart_dis_oneway_mat=cell2mat(ecart_dis_oneway);

for i = RobotNum_start:RobotNum_test
    per_dis_dyn{i,1}=abs(ecart_dis_dyn_mat(i,:)./dis_dyn{i,1});
    per_dis_oneway{i,1}=abs(ecart_dis_oneway_mat(i,:)./dis_oneway{i,1});
end

var_dis_dyn=zeros(RobotNum_test-RobotNum_start+1,1);

var_dis_oneway=zeros(RobotNum_test-RobotNum_start+1,1);

for i = RobotNum_start:RobotNum_test
    
    moyen_dyn(i)=mean(per_dis_dyn{i,1});
    var_dis_dyn(i)= std(per_dis_dyn{i,1});
    
    moyen_oneway(i)=mean(per_dis_oneway{i,1});
    var_dis_oneway(i)= std(per_dis_oneway{i,1});
    
end

rob=RobotNum_start:1:RobotNum_test;

e1=errorbar(rob,moyen_dyn,var_dis_dyn,'-o');
e1.Color=[0.8500 0.3250 0.0980];
hold on;
e2=errorbar(rob,moyen_oneway,var_dis_oneway,'-o');
e2.Color=[0 0.4470 0.7410];

moyen_min_dyn=min(moyen_dyn);
moyen_min_oneway=min(moyen_oneway);
moyen_max_dyn=max(moyen_dyn);
moyen_max_oneway=max(moyen_oneway);

var_dis_oneway_max=min(var_dis_oneway);
var_dis_dyn_max=max(var_dis_dyn);

axis([0,RobotNum_test+1,min(moyen_min_dyn,moyen_min_oneway)-2*max(var_dis_dyn_max,var_dis_oneway_max),max(moyen_max_dyn,moyen_max_oneway)+2*max(var_dis_dyn_max,var_dis_oneway_max)])

title(['最优损失比测试：控制环境大小，改变机器人个数，测试环境大小' num2str(m_D) 'X' num2str(n_D) ' 每测试集测试次数' num2str(test_para)])
xlabel('机器人个数');ylabel('最优损失比')

legend('动态算法DSIP','改进单行线模型OWIP')

hold off;