function mapdesigner_global(G,num)
MM = size(G,2);
NN = size(G,1);
G = rot90(G,2);

figure(num)

axis equal;
for i=1:NN
    for j=1:MM
        if G(i,j)==1
            x1=j-1;y1=NN-i;
            x2=j;y2=NN-i;
            x3=j;y3=NN-i+1;
            x4=j-1;y4=NN-i+1;
            fill([x1,x2,x3,x4],[y1,y2,y3,y4],[0.3,0.3,0.3]);
            hold on

        else
            x1=j-1;y1=NN-i;
            x2=j;y2=NN-i;
            x3=j;y3=NN-i+1;
            x4=j-1;y4=NN-i+1;
            fill([x1,x2,x3,x4],[y1,y2,y3,y4],[1,1,1]);
            hold on
       end
    end
end

xlim([0 MM])
ylim([0 NN])