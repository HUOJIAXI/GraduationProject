function [X,Y]=spread(Path,m)
for i = 1:length(Path)
    if mod(Path(i),m)==0
        X(i) = m;
        Y(i) = Path(i)/m;
    else
        X(i) = mod(Path(i),m);
        Y(i) = floor(Path(i)/m)+1;
    end
end

