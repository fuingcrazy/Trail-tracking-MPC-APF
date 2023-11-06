function [x,y,yaw]=Obstacle(x,y,Obs_x,Obs_y,yaw,target_v,Obs)
P0=2;   % Effective distance of obstacle
if Obs==0    % No obstacle
return
else    % Obstacle exists
    dist=sqrt((Obs_x-x)^2+(Obs_y-y)^2);  % Calculate the distance from current position
    if dist<P0   % Need to start avoiding obstacle
    X=[x,y];   
    k=4;    % Gain(Attractive Force)
    m=3;   % Gain(Repulsive Force)
    v=target_v*0.7;   % Driving speed
    search=50;   % Maximum query times
    ang1=atan((Obs_y-y)/(Obs_x-x));  % angle
    ang2=yaw;  % Vehicle heading angle
    if ang1<=ang2   % Adjust the gravity source position appropriately
        dest=[Obs_x+30,Obs_y+50];   
    else
        dest=[Obs_x+30,Obs_y-50];  
    end
    theta=zeros(1,2);
    theta(1,1)=atan(yaw);
    for i=1:search
        Dis_Y=sqrt((X(1)-dest(1))^2+(X(2)-dest(2))^2);% Current distance from the objective
        U1=dest-X;  % Direction vector(Attractive force)
        F_Y=k*Dis_Y*U1/sqrt(U1(1)^2+U1(2)^2);  
        Dis_C=1/sqrt((X(1)-Obs_x)^2+(X(2)-Obs_y)^2);
        U2=X-[Obs_x Obs_y];  % Direction vector(Repulsive Force)
        if Dis_C<1/P0 % Distance > bound
           break;
        else
            F_C=m*Dis_C*U2/sqrt(U2(1)^2+U2(2)^2);  
            F_H=F_Y+F_C;  % Combination
            delta_pos=F_H/sqrt(F_H(1)^2+F_H(2)^2)*v;
            theta(1,i+1)=atan(F_H(2)/F_H(1));
            X(1)=X(1)+delta_pos(1);
            X(2)=X(2)+delta_pos(2);
            x=X(1);y=X(2);
            yaw=yaw+theta(1,i+1)-theta(1,i);  % Yaw change
            plot(x,y,'bo',Color='blue');
            pause(0.01);
        end
    end
    end
end







