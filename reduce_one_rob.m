function [Start_test,Goal_test,dis_num,ini_dir_rob,ini_x_value,ini_u_value]=reduce_one_rob(Start_test,Goal_test,RobotNum_total,ini_dir_rob,ini_x_value,ini_u_value)

dis_num=1+round(rand(1,1)*(RobotNum_total-1));

Start_test(dis_num)=[];

Goal_test(dis_num)=[];

ini_dir_rob(dis_num,:)=[];

ini_x_value(:,:,dis_num)=[];

ini_u_value(dis_num,:)=[];




