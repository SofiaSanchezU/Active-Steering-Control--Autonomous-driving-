function K_inf_robust = H_inf_robust(A1,A2,A3,A4,B1,B2,B3,B4,W1,W2,W3,W4,C1,D12,D11)
format long

dim = size(A1);
    n = dim(1);
dim = size(B2);
    m = dim(2);
dim = size(W1);
    mw = dim(2);
dim = size(D11);
    nw = dim(1);

P = sdpvar(n,n);
Y = sdpvar(m,n);
gamma=sdpvar(1,1);

% Constrains to solve

F1 = ([(A1*P+B1*Y)+(A1*P+B1*Y)' W1 (C1*P+D12*Y)';
    W1' -gamma*eye(mw) D11';
    C1*P+D12*Y D11 -gamma*eye(nw)]<= 0); 

F2 = ([(A2*P+B2*Y)+(A2*P+B2*Y)' W2 (C1*P+D12*Y)';
    W2' -gamma*eye(mw) D11';
    C1*P+D12*Y D11 -gamma*eye(nw)]<= 0); 

F3 = ([(A3*P+B3*Y)+(A3*P+B3*Y)' W3 (C1*P+D12*Y)';
    W3' -gamma*eye(mw) D11';
    C1*P+D12*Y D11 -gamma*eye(nw)]<= 0); 

F4 = ([(A4*P+B4*Y)+(A4*P+B4*Y)' W4 (C1*P+D12*Y)';
    W4' -gamma*eye(mw) D11';
    C1*P+D12*Y D11 -gamma*eye(nw)]<= 0); 

F5 = ([gamma]>=0);
F6 = ([P]>=0);
F = F1+F2+F3+F4+F5+F6;

% Risoluzione delle LMI

opts=sdpsettings('solver','sedumi','verbose',0);
solvesdp(F,gamma,opts);

% control gain

K_inf_robust=double(Y)*inv(double(P));

end
