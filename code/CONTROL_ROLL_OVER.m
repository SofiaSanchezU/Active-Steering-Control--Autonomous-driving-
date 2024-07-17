clear all;
clc
close all;
format short;

%% ------------------Select the operation with 1--------------------------%
Structural_properties = 0;
LMI_Lyapunov_stability = 0;
LMI_regionf = 0;
H_2_controller=0;
H_inf_controller=0;
L_1_controller=0;
LQ_controller=0;
H_2_H_inf_controller=0;
comparation=0;
Robust_control_H_inf=0;

%% Data...................................................................
m=14300;
m2=12487;
cf=582000;
cr=783000;
mu=1;
lf=1.95;
lr=1.54;
Jz=34917;
J2x=24201;
g=9.81;
dfip=100000;
cfi=457000;
T=1.99;
h=1.15;     %                         % CG heigh for optimal control
%c=10000;     %%%%%%%%%%%%%%%%%%%%% not necesary
%k=457000;   %%%%%%%%%%%%%%%%%%%%%%% not necesary
hr=0.68;
Ifi=J2x+(m2*(h^2));%
Yb=(cf+cr)*mu;%
Nb=((cf*lf)+(cr*lr))*mu;%
Nr=((cf*(lf^2))+(cr*(lr^2)))*mu;%
sigma=(m2*g*h)-cfi;%
v=60;                                % velocity for optimal control

%% ...............State Space Representation...............................
E=[1,0,0,0;0,m,0,-h*m2;0,0,Jz,0;0,h*m2,0,Ifi];
E1=inv(E);
F=[0,0,0,1;0,-Yb/v,-((Nb/v)-(m*v)),0;0,-Nb/v,-Nr/v,0;sigma,0,m2*h*v,-dfip];
A=E1*F;
G=[0;cf*mu;cf*lf*mu;0];
B=E1*G;
W=B;
C=eye(4);
D=zeros(4,1);

%% ................... System analisys.....................................
%.....................Structural Properties ...............................
if Structural_properties==1
x0=[0.02, 0.05, 0.01, 0.06];
ctrb(A,B);
obsv(A,C);
ragg=rank(ctrb(A,B));          % Reachability
oss=rank(obsv(A,C));           % Obsevability 
eigenvalues=eig(A);            % check the stability
end
%% LMI Lyapunov stability..................................................
if LMI_Lyapunov_stability==1
Q = LMI_stability(A,B);
eig_s = eig(Q)
isnegdef = all(eig_s < 0)
if isnegdef == 1
        disp('-------------------------------------------')
	    disp('the LMI constrain matrix is negative define')
        disp('-------------------------------------------')
else
        disp('-------------------------------------------')
	    disp('the LMI constrain matrix is positive define')
        disp('-------------------------------------------')
end
end
%% LMI Region
if LMI_regionf==1
    eigenvalues=eig(A);
    alfa=-max(real(eigenvalues(1))) %% because is the least negative
    theta=max(angle(eigenvalues(1)))
    r=max(abs(eigenvalues(1)))
    [M P] = LMI_region(A,B,alfa,theta,r);% M matrix condition that use P matrix use for proof stability
    eig_M=eigs(M);
    isnegdef = all(eig_M < 0);
    if isnegdef == 1
        disp('-----------------------------------------------')
	    disp('--Exist a matrix P>0 s.t. proof de S-stability_')
        disp('-----------------------------------------------')
    else
        disp('-----------------------------------------------')
	    disp('No exist a matrix P>0 s.t. proof de S-stability')
        disp('-----------------------------------------------')
    end
end
%%  H2 Optimal Control 
if H_2_controller==1
    rho=200e-3;
    D22=sqrt(rho)*[0;0;0;0;1];
    B2=W;
    C2=[1 0 0 0 ;0 1 0 0;0 0 1 0;0 0 0 1; 0 0 0 0];
    % H2 gain
    k=H_2_gain(A,B,B2,C2,D22)
    eigv_H2=eig(A+B*k)
end
%% H_inf Optimal control
if H_inf_controller==1
    C1=[1 0 0 0;0 1 0 0; 0 0 1 0; 0 0 0 1; 0 0 0 0];
    rho=200e-2;
    D12=sqrt(rho)*[0;0;0;0;1];
    D11=zeros(5,1);
    B2=W;
    k = H_inf(A,B,B2,C1,D12,D11)
    eigH_inf=eig(A+B*k)
end
%% L_1 controller
if L_1_controller==1
    rho=20e-3;
    lambda=2.51;
    B2=W;
    C3=[1 0 0 0;0 1 0 0;0 0 1 0 ;0 0 0 1;0 0 0 0];
    D32=sqrt(rho)*[0;0;0;0;1];
    D31=zeros(5,1);
    % k_1 gain
    k=L_1(A,B,B2,C3,D32,D31,lambda)
    eigL_1=eig(A+B*k)
end
%% LQ controller
if LQ_controller==1
    Cz=[1 0 0 0;0 1 0 0;0 0 1 0;0 0 0 1;0 0 0 0];
    rho=20e-3;
    Dzu=sqrt(rho)*[0;0;0;0;1];
    % LQ gain
    k=LQ(Cz,Dzu,A,B,0)
    eigLQ=eig(A+B*k)
end
%% H_2_H_inf_controller
if H_2_H_inf_controller==1
    C1=[1 0 0 0 ;0 1 0 0;0 0 1 0;0 0 0 1; 0 0 0 0];
    rho=25e-4;
    D12=sqrt(rho)*[0;0;0;0;1];
    D11=zeros(5,1);
    B2=W;
    D22=sqrt(rho)*[0;0;0;0;1];
    C2=[1 0 0 0 ;0 1 0 0;0 0 1 0;0 0 0 1; 0 0 0 0];
    a=0.8;
    b=(1-a);
    k=H_2_H_inf(A,B,B2,C2,D22,C1,D12,D11,a,b)
    eigH2_INF=eig(A+B*k)
end
%% comparation
if comparation==1
    rho=20e-3;
    D22=sqrt(rho)*[0;0;0;0;1];
    B2=W;
    C2=[1 0 0 0 ;0 1 0 0;0 0 1 0;0 0 0 1; 0 0 0 0];
    % H2 gain
    k_h2=H_2_gain(A,B,B2,C2,D22);

    C1=[1 0 0 0;0 1 0 0; 0 0 1 0; 0 0 0 1; 0 0 0 0];
    rho=200e-2;
    D12=sqrt(rho)*[0;0;0;0;1];
    D11=zeros(5,1);
    B2=W;
    % k_inf gain
    k_inf= H_inf(A,B,B2,C1,D12,D11);

    rho=20e-3;
    lambda=2.51;
    B2=W;
    C3=[1 0 0 0;0 1 0 0;0 0 1 0 ;0 0 0 1;0 0 0 0];
    D32=sqrt(rho)*[0;0;0;0;1];
    D31=zeros(5,1);
    % k_1 gain
    k_l1=L_1(A,B,B2,C3,D32,D31,lambda);

    Cz=[1 0 0 0;0 1 0 0;0 0 1 0;0 0 0 1;0 0 0 0];
    rho=20e-3;
    Dzu=sqrt(rho)*[0;0;0;0;1];
    % LQ gain
    k_LQ=LQ(Cz,Dzu,A,B,0);

    a=0.8;
    b=(1-a);
    % Multiobjective gain
    kH2Hinf=H_2_H_inf(A,B,B2,C2,D22,C1,D12,D11,a,b);
    % sim('comparation');
end
%% Robust control H inf 
if Robust_control_H_inf==1
     
    C1=[1 0 0 0;0 1 0 0; 0 0 1 0; 0 0 0 1; 0 0 0 0];
    rho=2e-10;
    D12=sqrt(rho)*[0;0;0;0;1];
    D11=zeros(5,1);

    % Matrixes for vertix one
    v=20;                                
    h=0.77; 
    E1=[1,0,0,0;0,m,0,-h*m2;0,0,Jz,0;0,h*m2,0,Ifi];
    E11=inv(E1);
    F1=[0,0,0,1;0,-Yb/v,-((Nb/v)-(m*v)),0;0,-Nb/v,-Nr/v,0;sigma,0,m2*h*v,-dfip];
    A1=E11*F1;
    G1=[0;cf*mu;cf*lf*mu;0];
    B1=E1*G1;
    W1=B1;

    % Matrixes for vertix two
    v=100;                                
    h=0.77; 
    E2=[1,0,0,0;0,m,0,-h*m2;0,0,Jz,0;0,h*m2,0,Ifi];
    E12=inv(E2);
    F2=[0,0,0,1;0,-Yb/v,-((Nb/v)-(m*v)),0;0,-Nb/v,-Nr/v,0;sigma,0,m2*h*v,-dfip];
    A2=E12*F2;
    G2=[0;cf*mu;cf*lf*mu;0];
    B2=E2*G2;
    W2=B2;


    % Matrixes for vertix three
    v=20;                                
    h=1.53; 
    E3=[1,0,0,0;0,m,0,-h*m2;0,0,Jz,0;0,h*m2,0,Ifi];
    E13=inv(E3);
    F3=[0,0,0,1;0,-Yb/v,-((Nb/v)-(m*v)),0;0,-Nb/v,-Nr/v,0;sigma,0,m2*h*v,-dfip];
    A3=E13*F3;
    G3=[0;cf*mu;cf*lf*mu;0];
    B3=E3*G3;
    W3=B3;


    % Matrixes for vertix four 
    v=100;                                
    h=1.53; 
    E4=[1,0,0,0;0,m,0,-h*m2;0,0,Jz,0;0,h*m2,0,Ifi];
    E14=inv(E4);
    F4=[0,0,0,1;0,-Yb/v,-((Nb/v)-(m*v)),0;0,-Nb/v,-Nr/v,0;sigma,0,m2*h*v,-dfip];
    A4=E14*F4;
    G4=[0;cf*mu;cf*lf*mu;0];
    B4=E4*G4;
    W4=B4;
  

    k = H_inf_robust(A1,A2,A3,A4,B1,B2,B3,B4,W1,W2,W3,W4,C1,D12,D11)
    eigH_inf=eig(A+B*k)

end

