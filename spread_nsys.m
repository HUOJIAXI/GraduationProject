function [X,Y]=spread_nsys(Path,D)
m=size(D,1);
n=size(D,2);
for i = 1:length(Path)
    if mod(Path(i),n)==0
        Y(i) = n;
        X(i) = Path(i)/n;
    else
        Y(i) = mod(Path(i),n);
        X(i) = floor(Path(i)/n)+1;
    end
end