function [delta,v,index,e,U,vd_p]=MPC_Control(x,y,yaw,xset,yset,cyaw,dpd,target_v,Q,R,U,Obs)
u = [x,y,yaw];
[index, e] = get_index(x,y,xset,yset,cyaw); % find the nearest reference point on the curve
vd1=target_v ;    % speed of the target point 
vd2=dpd(index,1);   % Yawing angle of the target point

r = [xset(index), yset(index), cyaw(index)];
Nx=3;%Number of status quantities
Nu =2;%Number of control quantities
Np =20;% Prediction step size
Nc=30;% Control step size
Row=10;% Relaxation factor
deg =u(3);% Radian
     
kesi=zeros(Nx+Nu,1);
kesi(1)=u(1)-r(1);%u(1)==X(1)
kesi(2)=u(2)-r(2);%u(2)==X(2)
kesi(3)=deg-r(3); %u(3)==X(3)
kesi(4)=U(1);
kesi(5)=U(2);
fprintf('Update start, u(1)=%4.2f\n',U(1))
fprintf('Update start, u(2)=%4.2f\n',U(2))
    
%Initialize matrices  
    
A=[1    0   -vd1*sin(deg);
   0    1   vd1*cos(deg);
   0    0   1;];
B=[cos(deg)   0;
   sin(deg)   0;
    0         1];
A_cell=cell(2,2);
B_cell=cell(2,1);
A_cell{1,1}=A;
A_cell{1,2}=B;
A_cell{2,1}=zeros(Nu,Nx);
A_cell{2,2}=eye(Nu);
B_cell{1,1}=B;
B_cell{2,1}=eye(Nu);
A=cell2mat(A_cell);
B=cell2mat(B_cell);
C=[1 0 0 0 0;0 1 0 0 0;0 0 1 0 0;];
PHI_cell=cell(Np,1);
THETA_cell=cell(Np,Nc);
for j=1:1:Np
  PHI_cell{j,1}=C*A^j;
    for k=1:1:Nc
         if k<=j
             THETA_cell{j,k}=C*A^(j-k)*B;
         else 
             THETA_cell{j,k}=zeros(Nx,Nu);
         end
    end
end
PHI=cell2mat(PHI_cell);%size(PHI)=[Nx*Np Nx+Nu]
THETA=cell2mat(THETA_cell);%size(THETA)=[Nx*Np Nu*(Nc+1)]
H_cell=cell(2,2);
H_cell{1,1}=THETA'*Q*THETA+R;
H_cell{1,2}=zeros(Nu*Nc,1);
H_cell{2,1}=zeros(1,Nu*Nc);
H_cell{2,2}=Row;
H=0.5*cell2mat(H_cell);
H=(H+H')/2;
error=PHI*kesi;
f_cell=cell(1,2);
f_cell{1,1}=2*error'*Q*THETA;
f_cell{1,2}=0;
f=cell2mat(f_cell);
    
 % Inequation contraints 
A_t=zeros(Nc,Nc);
for p=1:1:Nc
    for q=1:1:Nc
        if q<=p 
            A_t(p,q)=1;
        else 
            A_t(p,q)=0;
        end
    end 
end 
A_I=kron(A_t,eye(Nu));% Crone product
Ut=kron(ones(Nc,1),U);
umin=[-0.6;-0.54;];% dimensions = number of control variables
umax=[0.6;0.35];
delta_umin = [ -0.2 ; -0.44];
delta_umax = [  0.2 ;  0.44];
Umin=kron(ones(Nc,1),umin);
Umax=kron(ones(Nc,1),umax);
A_cons_cell={A_I zeros(Nu*Nc,1);-A_I zeros(Nu*Nc,1)};
b_cons_cell={Umax-Ut;-Umin+Ut};
A_cons=cell2mat(A_cons_cell);
b_cons=cell2mat(b_cons_cell);
   % status variables contraints
M=10; 
delta_Umin=kron(ones(Nc,1),delta_umin);
delta_Umax=kron(ones(Nc,1),delta_umax);
lb=[delta_Umin;0];% status variable lower boundary
ub=[delta_Umax;M];% status variable upper boundary

    %% start solving
%     options = optimset('Algorithm','interior-point-convex');
options = optimoptions('quadprog','Display','iter','MaxIterations',100,'TolFun',1e-16);
[X,fval,exitflag]=quadprog(H,f,A_cons,b_cons,[],[],lb,ub,[],options);   % Solvers for quadratic objective functions with linear constraints
    
%     U(1)=X(1)+kesi(4);
%     U(2)=X(2)+kesi(5); 
if Obs==1  %There is an obstacle
    delta= X(2)+0.2*(kesi(5)+vd2);
    v = X(1)+0.2*(kesi(4)+vd1);   %Control quantity+reference quantity+latest error
    vd_p  = [vd1;vd2 ];
else
    delta= X(2)+(kesi(5)+vd2);
    v = X(1)+(kesi(4)+vd1);   
    vd_p  = [vd1;vd2 ];
end
end