function [obs,nobs]=count_obstacle(D)

m_D=size(D,1);
n_D=size(D,2);

nobs=[];
obs=[];
for i = 1:m_D
    for j = 1:n_D
        if D(i,j) == 1
            obs=[obs j+(i-1)*n_D]; % 障碍物所在位置
%             [obs_x,obs_y]=spread(obs,m);
        else
            nobs=[nobs j+(i-1)*n_D];
        end
    end
end