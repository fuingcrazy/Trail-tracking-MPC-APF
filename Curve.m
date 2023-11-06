function[xset,yset] = Curve  %Bezier Curve generator
P0=[0,0];
P1=[30,30/sqrt(3)];
P2=[30,40];   % Define three control points
P=[P0;
    P1;
    P2];
plot(P(:,1),P(:,2),'k',color='red')
hold on
k=1;
xset=zeros(1,110);
yset=zeros(1,110);
for t=0:0.01:1
    P_t_1=(1-t) * P0 + t * P1;
    P_t_2=(1-t) * P1 + t * P2;
    P_t_3=(1-t) * P_t_1 + t * P_t_2;

    xset(1,k)=P_t_3(1);  
    yset(1,k)=P_t_3(2);  
    k=k+1;
    
    scatter(P_t_3(1),P_t_3(2),300,'.r'); 
end
end