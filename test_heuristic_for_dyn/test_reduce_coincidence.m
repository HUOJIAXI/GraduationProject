function [Start_new,Goal_new,RobotNum_new,err]=test_reduce_coincidence(Start,Goal,RobotNum)
test = cat(2,Start',Goal');
equal=[];
Start_new=[];
Goal_new=[];
for i = 1:RobotNum-1
    test_o=test(i,:);
    if ~ismember(i,equal)
        for j = i+1:RobotNum

                test_o_i=test(j,:);
                if (test_o-test_o_i)==0

                    equal=[equal j];

                end
        end
    end
end

for j =1:RobotNum

    if ~ismember(j,equal)
        Start_new = [Start_new Start(j)];
        Goal_new  = [Goal_new Goal(j)];
    end

end

RobotNum_new=length(Start_new);

if length(Start_new)-length(Start)==0
    disp('不存在重合起点终点');
    err=0;
else
    disp('存在重合起点终点');
    err=1;
end
