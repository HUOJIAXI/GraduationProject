function [PathStore,Path_num]=traite_pause(D,PathStore,Path_num,res,flag)
SD=size(D,1);
flag_stop=1;
if flag_stop == 0
    PATH= PathStore(res-1:size(PathStore,1),:);
    Path_num_MAJ=Path_num(res-1:size(Path_num,2));

    PathStore(res:size(PathStore,1),:)=[];
    Path_num(res:size(Path_num,2))=[];

    PathStore=[PathStore ; PATH];
    Path_num=[Path_num Path_num_MAJ];
    
else
    if PathStore(res-1,2)-1~=0 || PathStore(res-1,1)-1~=0
        if PathStore(res-1,2)-1~=0 && D(PathStore(res-1,1),PathStore(res-1,2)-1)==1 % 在货架处躲避

            M=PathStore(res-1,2)-1;
            N=M+(PathStore(res-1,1)-1)*SD;
            
            PATH= PathStore(res-1:size(PathStore,1),:);
            Path_num_MAJ=Path_num(res-1:size(Path_num,2));
             
            Path_num_MAJ(1)=N;
            PATH(1,2)=M;
%             Path_num_MAJ=[N Path_num_MAJ];
%             PATH=[[M PathStore(res-1,1)];PATH];
            
            if flag==1
                PathStore(res:size(PathStore,1),:)=[];
                Path_num(res:size(Path_num,2))=[];

                PathStore=[PathStore ; PATH];
                Path_num=[Path_num Path_num_MAJ];
            end

        elseif PathStore(res-1,1)-1~=0 && D(PathStore(res-1,1)-1,PathStore(res-1,2))==1
            
            M=PathStore(res-1,1)-1;
            N=PathStore(res-1,2)+(M-1)*SD;
            
            PATH= PathStore(res-1:size(PathStore,1),:);
            Path_num_MAJ=Path_num(res-1:size(Path_num,2));
             
%             Path_num_MAJ(1)=N;
%             PATH(1,1)=M;
            Path_num_MAJ=[N Path_num_MAJ];
            PATH=[[M PathStore(res-1,2)];PATH];
            
            if flag==1
                PathStore(res:size(PathStore,1),:)=[];
                Path_num(res:size(Path_num,2))=[];

                PathStore=[PathStore ; PATH];
                Path_num=[Path_num Path_num_MAJ];
            end
            
        end

    else
        if D(PathStore(res-1,1),PathStore(res-1,2)+1)==1
            
            M=PathStore(res-1,2)+1;
            N=M+(PathStore(res-1,1)-1)*SD;
            
            PATH= PathStore(res-1:size(PathStore,1),:);
            Path_num_MAJ=Path_num(res-1:size(Path_num,2));
             
            Path_num_MAJ(1)=N;
            PATH(1,2)=M;
%             Path_num_MAJ=[N Path_num_MAJ];
%             PATH=[[M PathStore(res-1,2)];PATH];
            
            if flag==1
                PathStore(res:size(PathStore,1),:)=[];
                Path_num(res:size(Path_num,2))=[];

                PathStore=[PathStore ; PATH];
                Path_num=[Path_num Path_num_MAJ];
            end

        elseif D(PathStore(res-1,1)+1,PathStore(res-1,2))==1

            M=PathStore(res-1,1)+1;
            N=PathStore(res-1,2)+(M-1)*SD;
            
            PATH= PathStore(res-1:size(PathStore,1),:);
            Path_num_MAJ=Path_num(res-1:size(Path_num,2));
             
            Path_num_MAJ(1)=N;
            PATH(1,1)=M;
%             Path_num_MAJ=[N Path_num_MAJ];
%             PATH=[[M PathStore(res-1,2)];PATH];
%             
            if flag==1
                PathStore(res:size(PathStore,1),:)=[];
                Path_num(res:size(Path_num,2))=[];

                PathStore=[PathStore ; PATH];
                Path_num=[Path_num Path_num_MAJ];
            end
            
        else 
            disp('机器人被包围，出现死锁，机器人停止等待处理')
            PATH= PathStore(res-1:size(PathStore,1),:);
            Path_num_MAJ=Path_num(res-1:size(Path_num,2));

            if flag==1
                PathStore(res:size(PathStore,1),:)=[];
                Path_num(res:size(Path_num,2))=[];

                PathStore=[PathStore ; PATH];
                Path_num=[Path_num Path_num_MAJ];
            end
        end
    end
end
