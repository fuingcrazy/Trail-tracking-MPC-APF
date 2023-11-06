clc
clear

Nc=30; % Control step size
Np=20;
Nx=3;  % Dimensions of status variables
Nu=2;  % Dimensions of control variables
Q=100*eye(Np*Nx,Np*Nx);  % Error weight matrice
R=eye(Nc*Nu);  % Any non-zero input will increase the cost
max_steer=sqrt(3)/6;  % Maximum steering speed
target_v=0.3;  % Final speed
Obs=1;   % 0:No obstacle 1:Obstacle
Obs_x=15;Obs_y=11;  % Obstacle position
[xset,yset]=Curve;  % Draw the reference curve and get dataset
hold on;
if Obs==1
    scatter(Obs_x,Obs_y,'k','filled','d');
%     legend("Reference Lines","Reference Dots","Obstacle Point",'Location','best')
end

p=[xset',yset'];
pd=zeros(length(xset)-1,1);
pdd=zeros(length(xset)-1,1);
k=zeros(length(xset)-1,1);
dpd=zeros(length(xset)-1,1);
for i=1:length(xset)-1
    pd(i,1)=(p(i+1,2)-p(i,2))/(p(i+1,1)-p(i,1));  % Calculate the first derivative
    if i>=2
        dpd(i,1)=atan(pd(i,1))-atan(pd(i-1,1));   %Find the yawing angular change at each interval point
    end
end
dpd(1,1)=dpd(2,1);
cyaw=atan(pd);   
index=0;
x=0;y=0;yaw=sqrt(3)/3;  % Initial status
U=[0.2;0];  % Initial control quantities
vd1_p=0;vd2_p=0;
vd_p=[vd1_p;vd2_p];

i=1;
posx=zeros(length(xset),1);
posy=zeros(length(yset),1);
phi=zeros(length(xset),1);
while index<length(xset)-1
    if sqrt((x-Obs_x)^2+(y-Obs_y)^2)<2  && Obs==1
    [x,y,yaw]=Obstacle(x,y,Obs_x,Obs_y,yaw,target_v,Obs);
    %[delta,v,index,e,U,vd_p]=MPC_Control(x,y,yaw,xset,yset,cyaw,dpd,target_v,Q,R,U,1);
    else
    [delta,v,index,e,U,vd_p]=MPC_Control(x,y,yaw,xset,yset,cyaw,dpd,target_v,Q,R,U,0);
    end
    if abs(e)>7
        fprintf('Deviation too big!');
        break;
    end
    [x,y,yaw,v]=Refresh(x,y,yaw,v,delta,max_steer);
    posx(i,1)=x;
    posy(i,1)=y;
    phi(i,1)=180*yaw/pi;
    i=i+1;
    plot(posx(i-1),posy(i-1),'bo',Color='blue');
    xlabel('X Position')
    ylabel('Y Position')
    title('Vehicle Track')
    pause(0.01);
    hold on;
end
    figure(2)
    plot(phi);
    xlabel('Xset')
    ylabel("Delta(Degree)")
    title("Vihecle's facing angle")