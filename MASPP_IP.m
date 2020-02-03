function PathStore = MASPP_IP(D,RobotNum,Start,Goal)
temp = D;
PathStore=cell(RobotNum,1);

for i = 1:RobotNum
    [PATH,temp]=IP_solver(temp,Start(i),Goal(i),i);
    PathStore{i} = PATH;
end

mapdesigner(fliplr(D));
hold on;

for i = 1:RobotNum
    plot((PathStore{i}(:,2)-1/2),(PathStore{i}(:,1)-1/2),'-ks','MarkerFaceColor','r','MarkerSize',10)
    hold on;
end



