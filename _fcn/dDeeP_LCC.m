function [u_opt,g_opt,mu_opt,eta_opt,phi_opt,theta_opt,real_iter_num] = dDeeP_LCC(Uip,Yip,Uif,Yif,Eip,Eif,ui_ini,yi_ini,ei_ini,Qi,Ri,...
    lambda_gi,lambda_yi,u_limit,s_limit,rho,mu_initial,eta_initial,g_initial,phi_initial,theta_initial,KKT_vert,Hz_vert)
% =========================================================================
%                   Distributed DeeP-LCC Formulation
%
% Uip --> Eif:                  data hankel matrices
% ui_ini --> ei_ini:            past trajectory before time t
% Qi & Ri:                      penalty for the cost function
% lambda_gi & lambda_yi:        penalty for the regularization in the cost function
% u_limit & s_limit:            box constraint for control and spacing
% rho:                          penality value for ADMM (Augmented Lagrangian)
% mu_initial --> theta_initial: initial value for variables in ADMM
% KKT_vert:                     pre-calculated inverse matrix for the KKT system
% Hz_vert:                      pre-calculated value of Hz^-1
% =========================================================================

% --------------------
% Stopping criterion
% --------------------
iteration_num       = 300;
error_absolute      = 0.1;
error_relative      = 1e-3;
% error_fix           = 1;

% varying penalty parameter
% tau_incr = 2;
% tau_decr = 2;
% mu_rho   = 10;

% --------------------
% Problem size
% --------------------
n_cav    = size(ui_ini,1);

for i = 1:n_cav
    m(i) = size(ui_ini{i},1);
    p(i) = size(yi_ini{i},1);
end

% --------------------
% Recover time horizon
% --------------------
Tini     = size(Uip{1},1)/m(1);
N        = size(Uif{1},1)/m(1);
T        = size(Uip{1},2) + Tini + N - 1;


% --------------------
% Parameters in ADMM
% --------------------
K               = cell(n_cav,1);
P               = cell(n_cav,1);
Hg              = cell(n_cav,1);
Aeqg            = cell(n_cav,1);
beqg            = cell(n_cav,1);
Hz              = cell(n_cav,1);
fg              = cell(n_cav,1);
fz              = cell(n_cav,1);


for i = 1:n_cav
    
    K{i} = kron(eye(N),[zeros(1,p(i)-2),1,0]);
    P{i} = kron(eye(N),[zeros(1,p(i)-1),1]);
    
    ui_ini{i} = reshape(ui_ini{i},[m(i)*Tini,1]);
    yi_ini{i} = reshape(yi_ini{i},[p(i)*Tini,1]);
    ei_ini{i} = reshape(ei_ini{i},[Tini,1]);
    
    if i == 1
        beqg{i}  = [ui_ini{1};ei_ini{1};zeros(N,1)];
    else
        beqg{i}  = [ui_ini{i};ei_ini{i}];
    end
    
end

% --------------------
% Initial value for variables
% --------------------
g               = g_initial;
z               = g_initial;
mu              = mu_initial;
eta             = eta_initial;
phi             = phi_initial;
theta           = theta_initial;
for i = 1:n_cav
    s{i} = P{i}*Yif{i}*g{i};
    u{i} = Uif{i}*g{i};
end

g_plus          = cell(n_cav,1);
z_plus          = cell(n_cav,1);
mu_plus         = cell(n_cav,1);
eta_plus        = cell(n_cav,1);
s_plus          = cell(n_cav,1);
u_plus          = cell(n_cav,1);
phi_plus        = cell(n_cav,1);
theta_plus      = cell(n_cav,1);



% figure(1);
% set(gca,'XLim',[0,iteration_num]);
% set(gcf,'Position',[250 600 600 400]);
%
% figure(2);
% set(gca,'XLim',[0,iteration_num]);
% set(gcf,'Position',[1050 600 600 400]);



for k = 1:iteration_num
    
    % error for primal and dual problems
    error_pri1      = 0;
    error_pri2      = 0;
    error_pri3      = 0;
    error_pri4      = 0;
    error_dual1     = 0;
    error_dual2     = 0;
    error_dual3     = 0;
    error_dual4     = 0;
    
    
    tolerence_pri1  = 0;
    tolerence_pri2  = 0;
    tolerence_pri3  = 0;
    tolerence_pri4  = 0;
    tolerence_dual1 = 0;
    tolerence_dual2 = 0;
    tolerence_dual3 = 0;
    tolerence_dual4 = 0;
    
    % --------------------
    % Update g
    % --------------------
    for i = 1:n_cav
        if i == 1
            fg{i}    = -lambda_yi(i)*Yip{i}'*yi_ini{i} + mu{i}/2 - rho*z{i}/2 ...
                - Yif{i}'*P{i}'*phi{i}/2 - Uif{i}'*theta{i}/2 - rho*Yif{i}'*P{i}'*s{i}/2 - rho*Uif{i}'*u{i}/2;
        else
            fg{i}    = -lambda_yi(i)*Yip{i}'*yi_ini{i} + mu{i}/2 - rho*z{i}/2 + Eif{i}'*eta{i-1}/2 - rho/2*Eif{i}'*K{i-1}*Yif{i-1}*z{i-1} ...
                - Yif{i}'*P{i}'*phi{i}/2 - Uif{i}'*theta{i}/2 - rho*Yif{i}'*P{i}'*s{i}/2 - rho*Uif{i}'*u{i}/2;
        end
    end
    
    %     parfor i = 1:n_cav
    for i = 1:n_cav
        temp = KKT_vert{i}*[-fg{i};beqg{i}];
        g_plus{i} = temp(1:T-Tini-N+1,:);
    end
    
    % --------------------
    % Update z, mu, eta & Calculate errors
    % --------------------
    %     parfor i = 1:n_cav-1
    for i = 1:n_cav-1
        fz{i}           = -mu{i}/2 - rho/2*g_plus{i} - Yif{i}'*K{i}'*eta{i}/2 - rho/2*Yif{i}'*K{i}'*Eif{i+1}*g_plus{i+1};
        
        z_plus{i}       = -Hz_vert{i}*fz{i};
        s_plus{i}       = min(max(P{i}*Yif{i}*g_plus{i} - phi{i}/rho,min(s_limit{i})*ones(N,1)),max(s_limit{i})*ones(N,1));
        u_plus{i}       = min(max(Uif{i}*g_plus{i} - theta{i}/rho,min(u_limit)*ones(N,1)),max(u_limit)*ones(N,1));
        mu_plus{i}      = mu{i} + rho*(g_plus{i} - z_plus{i});
        eta_plus{i}     = eta{i} + rho*(Eif{i+1}*g_plus{i+1} - K{i}*Yif{i}*z_plus{i});
        phi_plus{i}     = phi{i} + rho*(s_plus{i} - P{i}*Yif{i}*g_plus{i});
        theta_plus{i}   = theta{i} + rho*(u_plus{i} - Uif{i}*g_plus{i});
        
        error_pri1      = error_pri1 + norm(g_plus{i}-z_plus{i},2);
        tolerence_pri1  = tolerence_pri1 + sqrt(size(g_plus{i},1))*error_absolute + error_relative*max(norm(g_plus{i},2),norm(z_plus{i},2));
        error_dual1     = error_dual1 + norm(rho*(z_plus{i}-z{i}),2);
        tolerence_dual1 = tolerence_dual1 + sqrt(size(rho*(z_plus{i}-z{i}),1))*error_absolute + error_relative*norm(mu{i},2);
        error_pri2      = error_pri2 + norm(Eif{i+1}*g_plus{i+1} - K{i}*Yif{i}*z_plus{i},2);
        tolerence_pri2  = tolerence_pri2 + sqrt(size(Eif{i+1}*g_plus{i+1},1))*error_absolute + error_relative*max(norm(Eif{i+1}*g_plus{i+1},2),norm(K{i}*Yif{i}*z_plus{i},2));
        error_dual2     = error_dual2 + norm(rho*Eif{i+1}'*K{i}*Yif{i}*(z_plus{i}-z{i}),2);
        tolerence_dual2 = tolerence_dual2 + sqrt(size(Eif{i+1}'*K{i}*Yif{i}*(z_plus{i}-z{i}),1))*error_absolute + error_relative*norm(Eif{i+1}'*eta{i},2);
        error_pri3      = error_pri3 + norm(s_plus{i}-P{i}*Yif{i}*g_plus{i},2);
        tolerence_pri3  = tolerence_pri3 + sqrt(size(s_plus{i},1))*error_absolute + error_relative*max(norm(s_plus{i},2),norm(P{i}*Yif{i}*g_plus{i},2));
        error_dual3     = error_dual3 + norm(rho*(P{i}*Yif{i})'*(s_plus{i}-s{i}),2);
        tolerence_dual3 = tolerence_dual3 + sqrt(size(rho*(z_plus{i}-z{i}),1))*error_absolute + error_relative*norm((P{i}*Yif{i})'*phi{i},2);
        error_pri4      = error_pri4 + norm(u_plus{i}-Uif{i}*g_plus{i},2);
        tolerence_pri4  = tolerence_pri4 + sqrt(size(u_plus{i},1))*error_absolute + error_relative*max(norm(u_plus{i},2),norm(Uif{i}*g_plus{i},2));
        error_dual4     = error_dual4 + norm(rho*Uif{i}'*(u_plus{i}-u{i}),2);
        tolerence_dual4 = tolerence_dual4 + sqrt(size(rho*(u_plus{i}-u{i}),1))*error_absolute + error_relative*norm(Uif{i}'*theta{i},2);
    end
    
    
    i               = n_cav;
    
    fz{i}           = -mu{i}/2 - rho/2*g_plus{i};
    z_plus{i}       = -Hz_vert{i}^(-1)*fz{i};
    s_plus{i}       = min(max(P{i}*Yif{i}*g_plus{i} - phi{i}/rho,min(s_limit{i})*ones(N,1)),max(s_limit{i})*ones(N,1));
    u_plus{i}       = min(max(Uif{i}*g_plus{i} - theta{i}/rho,min(u_limit)*ones(N,1)),max(u_limit)*ones(N,1));
    mu_plus{i}      = mu{i} + rho*(g_plus{i} - z_plus{i});
    phi_plus{i}     = phi{i} + rho*(s_plus{i} - P{i}*Yif{i}*g_plus{i});
    theta_plus{i}   = theta{i} + rho*(u_plus{i} - Uif{i}*g_plus{i});
    
    error_pri1      = error_pri1 + norm(g_plus{i}-z_plus{i},2);
    tolerence_pri1  = tolerence_pri1 + sqrt(size(g_plus{i},1))*error_absolute + error_relative*max(norm(g_plus{i},2),norm(z_plus{i},2));
    error_dual1     = error_dual1 + norm(rho*(z_plus{i}-z{i}),2);
    tolerence_dual1 = tolerence_dual1 + sqrt(size(rho*(z_plus{i}-z{i}),1))*error_absolute + error_relative*norm(mu{i},2);
    error_pri3      = error_pri3 + norm(s_plus{i}-P{i}*Yif{i}*g_plus{i},2);
    tolerence_pri3  = tolerence_pri3 + sqrt(size(s_plus{i},1))*error_absolute + error_relative*max(norm(s_plus{i},2),norm(P{i}*Yif{i}*g_plus{i},2));
    error_dual3     = error_dual3 + norm(rho*(P{i}*Yif{i})'*(s_plus{i}-s{i}),2);
    tolerence_dual3 = tolerence_dual3 + sqrt(size(rho*(z_plus{i}-z{i}),1))*error_absolute + error_relative*norm((P{i}*Yif{i})'*phi{i},2);
    error_pri4      = error_pri4 + norm(u_plus{i}-Uif{i}*g_plus{i},2);
    tolerence_pri4  = tolerence_pri4 + sqrt(size(u_plus{i},1))*error_absolute + error_relative*max(norm(u_plus{i},2),norm(Uif{i}*g_plus{i},2));
    error_dual4     = error_dual4 + norm(rho*Uif{i}'*(u_plus{i}-u{i}),2);
    tolerence_dual4 = tolerence_dual4 + sqrt(size(rho*(u_plus{i}-u{i}),1))*error_absolute + error_relative*norm(Uif{i}'*theta{i},2);
    
    
    fprintf('Iteration step: %d\n',k);
    %     fprintf('Satisfy: %d; Primal residuals: %10.8f; Tolerance: %10.8f \n',...
    %         error_pri1 <= tolerence_pri1,error_pri1,tolerence_pri1);
    %     fprintf('Satisfy: %d; Primal residuals: %10.8f; Tolerance: %10.8f \n',...
    %         error_pri2 <= tolerence_pri2,error_pri2,tolerence_pri2);
    %     fprintf('Satisfy: %d; Primal residuals: %10.8f; Tolerance: %10.8f \n',...
    %         error_pri3 <= tolerence_pri3,error_pri3,tolerence_pri3);
    %     fprintf('Satisfy: %d; Primal residuals: %10.8f; Tolerance: %10.8f \n',...
    %         error_pri4 <= tolerence_pri4,error_pri4,tolerence_pri4);
    %     fprintf('Satisfy: %d; Dual residuals: %10.8f; Tolerance: %10.8f \n',...
    %         error_dual1 <= tolerence_dual1,error_dual1,tolerence_dual1);
    %     fprintf('Satisfy: %d; Dual residuals: %10.8f; Tolerance: %10.8f \n',...
    %         error_dual2 <= tolerence_dual2,error_dual2,tolerence_dual2);
    %     fprintf('Satisfy: %d; Dual residuals: %10.8f; Tolerance: %10.8f \n',...
    %         error_dual3 <= tolerence_dual3,error_dual3,tolerence_dual3);
    %     fprintf('Satisfy: %d; Dual residuals: %10.8f; Tolerance: %10.8f \n',...
    %         error_dual4 <= tolerence_dual4,error_dual4,tolerence_dual4);
    
    % update penalty parameter
    %     residual_ratio = (error_pri1 + error_pri2) / (error_dual1 + error_dual2);
    %     if residual_ratio > mu_rho
    %         rho = tau_incr * rho;
    %     elseif residual_ratio < 1/mu_rho
    %         rho = 1/tau_decr * rho;
    %     end
    
    
    
    % --------------------
    % Plot convergence process
    % --------------------
    plot_bool = 0;
    if plot_bool
        figure(1);
        set(gcf,'Position',[500 200 1000 500]);
        subplot(2,4,1);
        title(['Primal 1 error: ',num2str(error_pri1)],'Interpreter','latex');
        scatter(k,error_pri1); hold on;
        subplot(2,4,2);
        title(['Primal 2 error: ',num2str(error_pri2)],'Interpreter','latex');
        scatter(k,error_pri2); hold on;
        subplot(2,4,3);
        title(['Primal 3 error: ',num2str(error_pri3)],'Interpreter','latex');
        scatter(k,error_pri3); hold on;
        subplot(2,4,4);
        title(['Primal 4 error: ',num2str(error_pri4)],'Interpreter','latex');
        scatter(k,error_pri4); hold on;
        
        subplot(2,4,5);
        title(['Dual 1 error: ',num2str(error_dual1)],'Interpreter','latex');
        scatter(k,error_dual1); hold on;
        subplot(2,4,6);
        title(['Dual 2 error: ',num2str(error_dual2)],'Interpreter','latex');
        scatter(k,error_dual2); hold on;
        subplot(2,4,7);
        title(['Dual 3 error: ',num2str(error_dual3)],'Interpreter','latex');
        scatter(k,error_dual3); hold on;
        subplot(2,4,8);
        title(['Dual 4 error: ',num2str(error_dual4)],'Interpreter','latex');
        scatter(k,error_dual4); hold on;
    end
    
    
    g       = g_plus;
    z       = z_plus;
    u       = u_plus;
    s       = s_plus;
    mu      = mu_plus;
    eta     = eta_plus;
    phi     = phi_plus;
    theta   = theta_plus;
    
    % --------------------
    % Check stopping criterion
    % --------------------
    if error_pri1 <= tolerence_pri1 && error_dual1 <= tolerence_dual1 ...
            && error_pri2 <= tolerence_pri2 && error_dual2 <= tolerence_dual2 ...
            && error_pri3 <= tolerence_pri3 && error_dual3 <= tolerence_dual3 ...
            && error_pri4 <= tolerence_pri4 && error_dual4 <= tolerence_dual4
        break;
    end
    
    %     if error_pri1 <= error_fix && error_pri2 <= error_fix ...
    %             && error_dual1 <= error_fix && error_dual2 <= error_fix
    %        break;
    %     end
    
    
end

% --------------------
% Record optimal value
% --------------------
real_iter_num   = k;
g_opt           = g;
mu_opt          = mu;
eta_opt         = eta;
phi_opt         = phi;
theta_opt       = theta;

u_opt           = zeros(n_cav*N,1);
for i = 1:n_cav
    u_opt(i:n_cav:end,1) = Uif{i}*g{i};
end

if plot_bool
    close(figure(1));
end

end