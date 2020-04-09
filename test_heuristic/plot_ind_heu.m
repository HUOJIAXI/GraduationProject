function plot_ind_heu(D,RobotNum,n,m,Start_ori,Goal_ori,PathStore_new,dir_way)

show=ceil(sqrt(RobotNum));

for i = 1:RobotNum
        mapdesigner_show(fliplr(D),i,show,0); % 最后一个参数控制行数 是否需要-1
        axis equal
        xlim([0 n])
        ylim([0 m])
        
        [start_ori_x,start_ori_y]=spread_sin(Start_ori(i),n);
        [Goal_ori_x,Goal_ori_y]=spread_sin(Goal_ori(i),n);
    
        PathStore_new{i}=cat(1,[start_ori_x,start_ori_y],PathStore_new{i});
        PathStore_new{i}=cat(1,PathStore_new{i},[Goal_ori_x,Goal_ori_y]);
        
        plot((PathStore_new{i}(:,2)-1/2),(PathStore_new{i}(:,1)-1/2),'-ks','MarkerFaceColor','r','MarkerSize',4) ;
        
        plot((Goal_ori_y-1/2),(Goal_ori_x-1/2),'-ks','MarkerFaceColor','g','MarkerSize',4)
        plot((start_ori_y-1/2),(start_ori_x-1/2),'-ks','MarkerFaceColor','y','MarkerSize',4)        
       
        str=['robot=',num2str(i)];
        title(str);
        hold off
end

mapdesigner_global(fliplr(D),2);
title('绕行方向')
arrowdesign(dir_way,D)