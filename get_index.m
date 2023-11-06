function [ind,error]=get_index(x,y,xset,yset,cyaw)  
N=length(xset)-1;
least=sqrt((xset(1,1)-x)^2+(yset(1,1)-y)^2);  % calculate the distance between the first reference point and current predictive location
for i=1:N
    dist=sqrt((xset(1,i)-x)^2+(yset(1,i)-y)^2);
    if least>=dist
        least=dist;  % find the nearest point
        ind=i;
    else
        break;
    end
end
error=dist;
end