function PathStore = MASPP_IP(D,RobotNum,Start,Goal)
temp = D;
PathStore=cell(RobotNum,1);

for i = 1:RobotNum
    [PATH,temp]=IP_solver(temp,Start,Goal);
    PathStore{i} = PATH;
end

