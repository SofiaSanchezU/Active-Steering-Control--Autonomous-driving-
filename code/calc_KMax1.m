function KM1 = calc_KMax1(v,h)
h=h/100;
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
hr=0.68;
Ifi=J2x+(m2*(h^2));%
Yb=(cf+cr)*mu;%
Nb=((cf*lf)+(cr*lr))*mu;%
Nr=((cf*(lf^2))+(cr*(lr^2)))*mu;%
sigma=(m2*g*h)-cfi;%


%% ...............State Space Representation...............................
E=[1,0,0,0;0,m,0,-h*m2;0,0,Jz,0;0,h*m2,0,Ifi];
E1=inv(E);
F=[0,0,0,1;0,-Yb/v,-((Nb/v)-(m*v)),0;0,-Nb/v,-Nr/v,0;sigma,0,m2*h*v,-dfip];
A=E1*F;
G=[0;cf*mu;cf*lf*mu;0];
B=E1*G;
W=B

%% H_inf_controller
    C1=[1 0 0 0;0 1 0 0; 0 0 1 0; 0 0 0 1; 0 0 0 0];
    rho=900;
    D12=sqrt(rho)*[0;0;0;0;1];
    D11=zeros(5,1);
    B2=W;
dim = size(A);
    n = dim(1);
dim = size(B);
    m = dim(2);
dim = size(B2);
    mw = dim(2);
dim = size(D11);
    nw = dim(1);

P = sdpvar(n,n);
Y = sdpvar(m,n);
gamma=sdpvar(1,1);

% Constrains to solve

F1 = ([(A*P+B*Y)+(A*P+B*Y)' B2 (C1*P+D12*Y)';
    B2' -gamma*eye(mw) D11';
    C1*P+D12*Y D11 -gamma*eye(nw)]<= 0); 

F2 = ([gamma]>=0);
F3 = ([P]>=0);
F = F1+F2+F3;

% Risoluzione delle LMI

opts=sdpsettings('solver','sedumi','verbose',0);
solvesdp(F,gamma,opts);

% control gain

KM1=double(Y)*inv(double(P));
