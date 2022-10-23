function [u_opt,y_opt,problem_status] = DeeP_LCC(Up,Yp,Uf,Yf,Ep,Ef,...
    uini,yini,eini,Q,R,r,lambda_g,lambda_y,u_limit,s_limit)
% =========================================================================
%            Centralized Distributed DeeP-LCC Formulation
%
% Up --> Ef:                data hankel matrices
% u_ini --> e_ini:          past trajectory before time t
% Q & R:                    penalty for the cost function
% r:                        reference trajectory
% lambda_g & lambda_y:      penalty for the regularization in the cost function
% u_limit & s_limit:        box constraint for control and spacing
% =========================================================================


if nargin < 15           % whether there exists input/output constraints and initial optimization point
    constraint_bool = 0;
else
    constraint_bool = 1;
end

% --------------------
% Problem size
% --------------------
m        = size(uini,1);
p        = size(yini,1);
Tini     = size(Up,1)/m;
N        = size(Uf,1)/m;
T        = size(Up,2) + Tini + N - 1;

% --------------------
% Construct past data and penality matrices
% --------------------
uini_col = reshape(uini,[m*Tini,1]);
yini_col = reshape(yini,[p*Tini,1]);
eini_col = reshape(eini,[Tini,1]);
r_col    = reshape(r,[p*N,1]);

Q_blk    = zeros(p*N);
R_blk    = zeros(m*N); 
for i = 1:N
    Q_blk((i-1)*p+1:i*p,(i-1)*p+1:i*p) = Q; 
    R_blk((i-1)*m+1:i*m,(i-1)*m+1:i*m) = R; 
end

% ---------------------
% Standard QP in MATLAB
% [x,fval,exitflag,output,lambda]=quadprog(H,f,A,b,B,c,l,u,x0,options)
% minimize     0.5*x'*H*x+f'*x    
% subject to         A*x          <= b 
%                    B*x           = c
%                    l <= x <= u 
% ---------------------

% Coefficient
H       = Yf'*Q_blk*Yf + Uf'*R_blk*Uf + lambda_g*eye(T-Tini-N+1) + lambda_y*Yp'*Yp;
f       = -lambda_y*Yp'*yini_col;

B       = [Up;Ep;Ef];
c       = [uini_col;eini_col;zeros(N,1)];

if constraint_bool % there exists input/output constraints
    Sf = [zeros(m,p-m),eye(m)];
    Sf_blk = Sf;
    for i = 2:N
        Sf_blk = blkdiag(Sf_blk,Sf); 
    end
    A = [Uf;-Uf;Sf_blk*Yf;-Sf_blk*Yf];
    b = [max(u_limit)*ones(m*N,1);-min(u_limit)*ones(m*N,1);...
        max(s_limit)*ones(m*N,1);-min(s_limit)*ones(m*N,1)];
else
    A = [];
    b = [];
end



options = optimoptions('quadprog','Algorithm','interior-point-convex','OptimalityTolerance',1e-3);
% Optimization
[g_opt,fval,exitflag,output,lambda] = quadprog(H,f,A,b,B,c,[],[],[],options);

% Solution
u_opt   = Uf*g_opt;
y_opt   = Yf*g_opt;
problem_status = exitflag;

% % For infeasible cases
% if exitflag ~= 1
%     u_opt = previous_u_opt;
% end

end