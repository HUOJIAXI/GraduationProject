function [PathStore_apres,PathStore_apres_sep]=traitement_arrete(Path_num,D,numrobot)

disp('进行等待处理')
croise=[];

m_size=size(D,1);
n_size=size(D,2);

for x= 1:m_size
    for y=1:n_size
%     [x,y]=spread_sin(i,n_size);
        if x==1 
            if D(x+1,y)==0
                croise=cat(1,croise,y+(x-1)*n_size);
                continue
            end
        elseif x==m_size
            if D(x-1,y)==0
                croise=cat(1,croise,y+(x-1)*n_size);
                continue
            end
        elseif y ==1
             if D(x,y+1)==0
                 croise=cat(1,croise,y+(x-1)*n_size);
                 continue
             end
        elseif y==n_size
            if D(x,y-1)==0
                croise=cat(1,croise,y+(x-1)*n_size);
                continue
            end
        else
            if D(x-1,y)==0 && D(x+1,y)==0  && D(x,y-1)==0&&D(x,y+1)==0
                croise=cat(1,croise,y+(x-1)*n_size);
                continue
            end
        end
    end
end

croise=unique(croise);

PathStore_apres=Path_num;

flag=1;
while flag==1
    flag=0;
    for rob=1:numrobot
        for rob_col=rob+1:numrobot
            len_rob=length(PathStore_apres{rob});
            len_rob_col=length(PathStore_apres{rob_col});
            len=min(len_rob,len_rob_col);
            col=PathStore_apres{rob}(1:len)-PathStore_apres{rob_col}(1:len);
            col_index=find(col==0); 
            if ~isempty(col_index)
%                 disp(col_index)
                flag=1;
%                 index_set=find(PathStore_apres{rob}==col_index(1));
%                 disp(index_set)
%                 index=index_set(1);
%                 disp(index)
%                 disp(rob)
%                 disp(rob_col)
                index=col_index(1);
%                 disp(index)
                temp=PathStore_apres{rob_col}(index:end);
                PathStore_apres{rob_col}(index:end)=[];
                PathStore_apres{rob_col}=[PathStore_apres{rob_col};PathStore_apres{rob_col}(index-1)];

                
                PathStore_apres{rob_col}=[PathStore_apres{rob_col}; temp];
            end
        end
    end
end

PathStore_apres_sep=cell(numrobot,1);

for rob=1:numrobot
    
    [PathStore_apres_sep{rob}(1,:),PathStore_apres_sep{rob}(2,:)]=spread(PathStore_apres{rob},n_size);
    PathStore_apres_sep{rob}= PathStore_apres_sep{rob}';
end

disp('等待处理已完成')