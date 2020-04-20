function [PathStore_apres,PathStore_apres_sep]=traitement_arrete_high_density(Path_num,D,numrobot)

% disp('进行等待处理')
% croise=[];
% 
% m_size=size(D,1);
% n_size=size(D,2);
% 
% for x= 1:m_size
%     for y=1:n_size
% %     [x,y]=spread_sin(i,n_size);
%         if x==1 
%             if D(x+1,y)==0
%                 croise=cat(1,croise,y+(x-1)*n_size);
%                 continue
%             end
%         elseif x==m_size
%             if D(x-1,y)==0
%                 croise=cat(1,croise,y+(x-1)*n_size);
%                 continue
%             end
%         elseif y ==1
%              if D(x,y+1)==0
%                  croise=cat(1,croise,y+(x-1)*n_size);
%                  continue
%              end
%         elseif y==n_size
%             if D(x,y-1)==0
%                 croise=cat(1,croise,y+(x-1)*n_size);
%                 continue
%             end
%         else
%             if D(x-1,y)==0 && D(x+1,y)==0  && D(x,y-1)==0&&D(x,y+1)==0
%                 croise=cat(1,croise,y+(x-1)*n_size);
%                 continue
%             end
%         end
%     end
% end
% 
% croise=unique(croise);
% 
% PathStore_apres=Path_num;
% roblist=[];
% % robclist=[];
% % flag=1;
% % while flag==1
% %     flag=0;
%     for cro=1:length(croise)
%         for rob=1:numrobot
%                 indexlist=find(PathStore_apres{rob,1}==croise(cro)); %取出机器人在交汇点处的index
%                 if ~isempty(indexlist)
%                     roblist=cat(1,roblist,[indexlist(1),rob]); %机器人编号
%                     flag_n=0;
%                 else
%                     flag_n=1;
%                     continue
%                 end
%         end
%          
%         if  flag_n==0
%             
%             robclist=roblist(:,1);
%             
%            
% %             disp(1)
%             if length(robclist)~=length(unique(robclist))
%                 flag=1;
%                 b=unique(robclist,'stable');
% %                 disp(b)
% %                 disp(robclist)
%                 d=[];
% %                 e=[];
%                 for i=(1:length(b))
%                 c=length(find(robclist==b(i)));
%                 if c>1
%                 d=[d,b(i)]; % 所有重复的index
%                 
% %                 e=[e,roblist(i)]
%                 end
%                 end
%                 
%                 index_re=[];
% %                 disp(d)
%                 for i = 1:length(d)
%                         m=find(roblist(:,1)==d(i));
%     %                     disp(m)
%                         for j = 1:length(m)
%                             index_re=[index_re;roblist(m(j),:)]; %在某交汇点处重复的机器人
%                         end
% 
%     %                 disp(index_re)
%                     for rob = 1:size(index_re,1)
% 
%                         index=index_re(rob,1);
%     %                     disp(index)
%                         temp=PathStore_apres{index_re(rob,2)}(index:end);
%                         PathStore_apres{index_re(rob,2)}(index:end)=[];
% %                         disp(rob)
% 
%                         for l =1:rob
%                             PathStore_apres{index_re(rob,2)}=[PathStore_apres{index_re(rob,2)};PathStore_apres{index_re(rob,2)}(index-1)];
%                         end
% 
%                         PathStore_apres{index_re(rob,2)}=[PathStore_apres{index_re(rob,2)}; temp];
% 
%                     end
%                 end
% 
% 
%             else
%                 continue
%             end
%         end
%         
%         
%     end     
%                 
% 
% % end
% 
% 
% PathStore_apres_sep=cell(numrobot,1);
% 
% for rob=1:numrobot
%     
%     [PathStore_apres_sep{rob}(1,:),PathStore_apres_sep{rob}(2,:)]=spread(PathStore_apres{rob},n_size);
%     PathStore_apres_sep{rob}= PathStore_apres_sep{rob}';
% end
% 
% disp('等待处理已完成')

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
                
%                 index_set=find(PathStore_apres{rob}==col_index(1));
%                 disp(index_set)
%                 index=index_set(1);
%                 disp(index)
%                 disp(rob)
%                 disp(rob_col)
                index=col_index(1);
%                 disp(index)

                if ~ismember(PathStore_apres{rob_col}(index),croise)
                    continue
                end
                flag=1;
%                 disp 1
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