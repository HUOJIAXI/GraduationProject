%% 寻找替代终点

function[Goal,flgn,m] = GOAL_RESERVE(temp,Goal_ori,SD)

%disp('终点被包围，启用备用终点');
flgn=0;
disp('备用终点启用');
for m = 1:SD*SD-Goal_ori
[X_fin_pos,Y_fin_pos]=spread_sin(Goal_ori+m,SD);
    if temp(X_fin_pos,Y_fin_pos)==0
        Goal=Goal_ori+m;
        flgn=1;
        break;
    end
end

if flgn==0
    for m = 1:Goal_ori-1
        [X_fin_neg,Y_fin_neg]=spread_sin(Goal_ori-m,SD);
        if temp(X_fin_neg,Y_fin_neg)==0
            Goal=Goal_ori-m;
            flgn=2;
            break;
        end
    end
end   