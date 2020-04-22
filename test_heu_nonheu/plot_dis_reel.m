function plot_dis_reel(RobotNum_test,RobotNum_start,dis_dyn,dis_oneway,dis_tra,m_D,n_D,test_para)

figure(3)
moyen_dyn=zeros(RobotNum_test-RobotNum_start+1,1);
moyen_oneway=zeros(RobotNum_test-RobotNum_start+1,1);
moyen_tra=zeros(RobotNum_test-RobotNum_start+1,1);


dis_dyn_mat = cell2mat(dis_dyn);
dis_oneway_mat=cell2mat(dis_oneway);
dis_tra_mat=cell2mat(dis_tra);

for rob =RobotNum_start:RobotNum_test
    dis_dyn_mat(rob,:)=dis_dyn_mat(rob,:)/rob;
    dis_oneway_mat(rob,:)=dis_oneway_mat(rob,:)/rob;
    dis_tra_mat(rob,:)=dis_tra_mat(rob,:)/rob;
end

var_dis_dyn=zeros(RobotNum_test-RobotNum_start+1,1);

var_dis_oneway=zeros(RobotNum_test-RobotNum_start+1,1);

var_dis_tra=zeros(RobotNum_test-RobotNum_start+1,1);

for i = RobotNum_start:RobotNum_test
    
    moyen_dyn(i)=mean(dis_dyn_mat(i,:));
    var_dis_dyn(i)= std(dis_dyn_mat(i,:));
    
    moyen_oneway(i)=mean(dis_oneway_mat(i,:));
    var_dis_oneway(i)= std(dis_oneway_mat(i,:));
    
    moyen_tra(i)=mean(dis_tra_mat(i,:));
    var_dis_tra(i)= std(dis_tra_mat(i,:));
    
end

rob=RobotNum_start:1:RobotNum_test;

e1=errorbar(rob,moyen_dyn,var_dis_dyn,'-o');
e1.Color=[0.8500 0.3250 0.0980];
hold on;
e2=errorbar(rob,moyen_oneway,var_dis_oneway,'-o');
e2.Color=[0 0.4470 0.7410];
hold on;
e3=errorbar(rob,moyen_tra,var_dis_tra,'-o');
e3.Color=[0.6350 0.0780 0.1840];
hold on;


moyen_min_dyn=min(moyen_dyn);
moyen_min_oneway=min(moyen_oneway);
moyen_max_dyn=max(moyen_dyn);
moyen_max_oneway=max(moyen_oneway);

var_dis_oneway_max=min(var_dis_oneway);
var_dis_dyn_max=max(var_dis_dyn);

axis([0,RobotNum_test+1,min(moyen_min_dyn,moyen_min_oneway)-2*max(var_dis_dyn_max,var_dis_oneway_max),max(moyen_max_dyn,moyen_max_oneway)+2*max(var_dis_dyn_max,var_dis_oneway_max)])

title(['最优路径测试：控制环境大小，改变机器人个数，测试环境大小' num2str(m_D) 'X' num2str(n_D) ' 每测试集测试次数' num2str(test_para)])
xlabel('机器人个数');ylabel('平均机器人最优路径')

legend('动态算法DSIP','改进单行线模型OWIP','传统IP模型TIP')

hold off;