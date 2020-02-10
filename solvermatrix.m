function [path]=solvermatrix(a,l,t)
%初始时，从第一行开始
%if flag==0
%    z=1;
    path = [];
%     for i=l:size(a,1)
%          %寻找当前行k中的1所在的位置
%          for j=1:size(a,2)
%             if(a(j,i) == 1)
%                 path(z)=j;
%                 z=z+1;
%                 path(z)=i;
%                 z=z+1;
%                 break;
%             end
%          end
%     end
    path = [path l];
    k=l;
    q=1;
    while path(end)~=t
        if a(k,q)==1
            path = [path q];
            k=q;
            q=1;
            continue;
        else
            q=q+1;
        end
    end
    



