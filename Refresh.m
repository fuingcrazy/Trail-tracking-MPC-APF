function [x,y,yaw,v]=Refresh(x,y,yaw,v,delta,max_steer)  %Refresh the current status
delta=max(min(max_steer,delta),-max_steer);
yaw=yaw+delta;
x=x+v*cos(yaw);
y=y+v*sin(yaw);
v=v;
end