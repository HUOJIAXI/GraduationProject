function show_graph(D,RobotNum,PathStore_new)
show=ceil(sqrt(RobotNum));
for i = 1:RobotNum
        mapdesigner_show(fliplr(D),i,show,0); % 最后一个参数控制行数 是否需要-1
        plot((PathStore_new{i}(:,2)-1/2),(PathStore_new{i}(:,1)-1/2),'-ks','MarkerFaceColor','r','MarkerSize',10) ;% 将所有机器人的路径显示在图中。
        str=['robot=',num2str(i)];
        title(str);
end