% =========================================================================
%               Large-scale Traffic Simulation
%               100 Vehicles with 20 CAVs
%               The head vehicle has a braking perturbation
%
% *Attention*: need pre-collected trajectory data to run this simulation. 
% If there are no pre-collected data, please run dataCollection_LargeScale 
% first.
% =========================================================================

clc; close all; clear;
addpath('_fcn');
warning off;

% whether output data
output_bool             = 1;

% whether mixed traffic flow
mix                     = 1;                    

% Type of the controller
controller_type         = 3;    
% 1. centralized DeeP-LCC  2. MPC  3.distributed DeeP-LCC

% Whether update equilibrium velocity
% If not, the CAV has a prescribed equilibrium velocity
update_equilibrium_bool = 0;

% length of data samples
switch controller_type
    case 1
        T               = 1200;      
    case 3
        T               = 600;
end

% -------------------------------------------------------------------------
%   Parameter setup
% -------------------------------------------------------------------------

h_wait = waitbar(0,'please wait');

% Number of data sets for simulation
data_number         = 1;
% Data set
data_str            = '1';  % 1. random ovm  2. manual ovm  3. homogeneous ovm
% Perturbation amplitude
per_type            = 3;    % 1. sinuoid perturbation 2. small brake perturbation 3. large brake perturbation
                            % 4. larger brake perturbation
                            % 5. Perturbation on a vehicle in the middle of the platoon
sine_amp            = 4; % amplitidue of sinuoid perturbation
brake_amp           = 5; % brake amplitude of brake perturbation

constraint_bool     = 1; % Whether there exist constraints

% Type for HDV car-following model
hdv_type            = 1;    % 1. OVM   2. IDM
% Uncertainty for HDV behavior
acel_noise          = 0.1;  % A white noise signal on HDV's original acceleration

% Parameters in Simulation
total_time          = 200;              % Total Simulation Time
Tstep               = 0.05;             % Time Step
total_time_step     = total_time/Tstep;

% Index for one experiment
computation_time    = zeros(total_time_step,1);
iteration_num       = zeros(total_time_step,1);

% Average index for all the experiments
Collected_computation_time  = zeros(data_number,1);
Collected_iteration_num     = zeros(data_number,1);

% DeeP-LCC Formulation
Tini                = 20;        % length of past data
N                   = 50;        % length of predicted horizon


% Weight coefficients
weight_choice       = 3; 
% case for weight choice in centralized DeeP-LCC

switch weight_choice
    case 1
        weight_v     = 1;        % weight coefficient for velocity error
        weight_s     = 0.5;      % weight coefficient for spacing error
        weight_u     = 0.1;      % weight coefficient for control input
        lambda_g     = 10;       % penalty on ||g||_2^2 in objective
        lambda_y     = 1e4;      % penalty on ||sigma_y||_2^2 in objective
    case 2
        weight_v     = 4;        % weight coefficient for velocity error
        weight_s     = 2;        % weight coefficient for spacing error
        weight_u     = 0.4;      % weight coefficient for control input
        lambda_g     = 10;       % penalty on ||g||_2^2 in objective
        lambda_y     = 1e4;      % penalty on ||sigma_y||_2^2 in objective
 case 3
        weight_v     = 1;        % weight coefficient for velocity error
        weight_s     = 0.5;      % weight coefficient for spacing error
        weight_u     = 0.1;      % weight coefficient for control input
        lambda_g     = 50;       % penalty on ||g||_2^2 in objective
        lambda_y     = 1e4;      % penalty on ||sigma_y||_2^2 in objective
end

% penality parameter in ADMM
rho          = 1;


% ------------------------------------------
% Parameters in Mixed Traffic
% ------------------------------------------
load('_data/ID_LargeScale.mat'); % record ID
% 1: CAV  0: HDV
ID_str      = num2str(ID);
ID_str(find(ID_str==' ')) = '';

pos_cav     = find(ID==1);          % position of CAVs
n_vehicle   = length(ID);           % number of vehicles
n_cav       = length(pos_cav);      % number of CAVs
n_hdv       = n_vehicle-n_cav;      % number of HDVs

v_star      = 15;                   % Equilibrium velocity
s_star      = 20;                   % Equilibrium spacing for CAV

% Constraints
acel_max        = 2;
dcel_max        = -5;
spacing_max     = 40;
spacing_min     = 5;
u_limit         = [dcel_max,acel_max];
s_limit         = [spacing_min,spacing_max]-s_star;

if data_str == '1'
    % Random setup for OVM
    load(['_data/hdv_ovm_random_largescale.mat']);
elseif data_str == '3'
    % Homegeneous setup for OVM
    load(['_data/hdv_ovm_homogeneous_',ID_str,'.mat']);
end




% What is measurable
% for measure_type    = 2:3
measure_type = 3;
% 1. Only the velocity errors of all the vehicles are measurable;
% 2. All the states, including velocity error and spacing error are measurable;
% 3. Velocity error and spacing error of the CAVs are measurable,
%    and the velocity error of the HDVs are measurable.


% ------------------
%  size in DeeP-LCC
% ------------------

n_ctr = 2*n_vehicle;    % number of state variables
m_ctr = n_cav;          % number of input variables
switch measure_type     % number of output variables
    case 1
        p_ctr = n_vehicle;
    case 2
        p_ctr = 2*n_vehicle;
    case 3
        p_ctr = n_vehicle + n_cav;
end

% ------------------
%  size in Distributed DeeP-LCC
% ------------------
ni_vehicle = zeros(1,n_cav);    % number of vehicles in each LCC subsystem

for i_cav = 1:n_cav-1
    ni_vehicle(i_cav) = pos_cav(i_cav+1) - pos_cav(i_cav);
end
ni_vehicle(n_cav) = n_vehicle - pos_cav(end) + 1;


% -------------------------------------------------------------------------
%   First Loop: Run simulation for different data sets
% -------------------------------------------------------------------------

for i_data = 1:data_number
    
    % Load trajectory data
    load(['_data\trajectory_data_collection\','LargeScale','_data',data_str,'_',num2str(i_data),...
    '_T_',num2str(T),'_',num2str(Tini),'_',num2str(N),'_noiseLevel_',num2str(acel_noise),'.mat']);
    
    
    % ---------------------------------------
    %   Scenario initialization
    %----------------------------------------
    
    % There is one head vehicle at the very beginning
    S           = zeros(total_time_step,n_vehicle+1,3);
    S(1,1,1)    = 0;
    for i = 2 : n_vehicle+1
        S(1,i,1) = S(1,i-1,1) - hdv_parameter.s_star(i-1);
    end
    S(1,:,2)    = v_star * ones(n_vehicle+1,1);
    
    
    % ------------------
    %  Centralized DeeP-LCC Formulation
    % ------------------
    Q_v         = weight_v*eye(n_vehicle);         % penalty for velocity error
    Q_s         = weight_s*eye(p_ctr-n_vehicle);   % penalty for spacing error
    Q           = blkdiag(Q_v,Q_s);                % penalty for trajectory error
    R           = weight_u*eye(m_ctr);             % penalty for control input
    
    u           = zeros(m_ctr,total_time_step); % control input
    x           = zeros(n_ctr,total_time_step); % state variables
    y           = zeros(p_ctr,total_time_step); % output variables
    pr_status   = zeros(total_time_step,1);     % problem status
    e           = zeros(1,total_time_step);     % external input
    
    
    % ------------------
    %  Distributed DeeP-LCC Formulation
    % ------------------
    if controller_type == 3 
        si_limit        = cell(n_cav,1);
        Qi              = cell(n_cav,1);
        Ri              = cell(n_cav,1);
        lambda_gi       = zeros(n_cav,1);
        lambda_yi       = zeros(n_cav,1);
        g_initial       = cell(n_cav,1);
        mu_initial      = cell(n_cav,1);
        eta_initial     = cell(n_cav,1);
        phi_initial     = cell(n_cav,1);
        theta_initial   = cell(n_cav,1);
        ui_ini          = cell(n_cav,1);
        yi_ini          = cell(n_cav,1);
        ei_ini          = cell(n_cav,1);
        
        for i = 1:n_cav
            Qi{i}        = blkdiag(weight_v*eye(ni_vehicle(i)),weight_s);      % penalty for trajectory error
            Ri{i}        = weight_u;                                           % penalty for control input
            lambda_gi(i) = lambda_g/n_cav;
            lambda_yi(i) = lambda_y;
            
            % Initial value
            g_initial{i}   = zeros(T-Tini-N+1,1);
            mu_initial{i}  = zeros(T-Tini-N+1,1);
            eta_initial{i} = zeros(N,1);
            phi_initial{i} = zeros(N,1);
            theta_initial{i} = zeros(N,1);
        end
        
        % ------------------
        %  Precalculation for those parameters that are fixed during the
        %  distributed DeeP-LCC algorithm
        % ------------------
    
        K           = cell(n_cav,1);    % Parameter in coupling constraint
        P           = cell(n_cav,1);    % Parameter in output constraint
        Hg          = cell(n_cav,1);    % Hg
        Aeqg        = cell(n_cav,1);
        beqg        = cell(n_cav,1);
        Qi_stack    = cell(n_cav,1);
        Ri_stack    = cell(n_cav,1);
        Hz          = cell(n_cav,1);
        KKT_vert    = cell(n_cav,1);    % Inverse matrix in KKT system
        Hz_vert     = cell(n_cav,1);    % Inverse matrix in Hz
        
        
        for i = 1:n_cav
            
            p(i) = ni_vehicle(i)+1;
            
            K{i} = kron(eye(N),[zeros(1,p(i)-2),1,0]);
            P{i} = kron(eye(N),[zeros(1,p(i)-1),1]);
            
            Qi_stack{i} =[];
            Ri_stack{i} =[];
            
            for k = 1:N
                Qi_stack{i}   = blkdiag(Qi_stack{i},Qi{i});
                Ri_stack{i}   = blkdiag(Ri_stack{i},Ri{i});
            end
            
            if i == 1
            Hg{i}    = Yif{i}'*Qi_stack{i}*Yif{i} + Uif{i}'*Ri_stack{i}*Uif{i} + lambda_gi(i)*eye(T-Tini-N+1) + lambda_yi(i)*Yip{i}'*Yip{i} +...
                rho/2*(eye(T-Tini-N+1) + Yif{i}'*P{i}'*P{i}*Yif{i} + Uif{i}'*Uif{i});
            Aeqg{i}  = [Uip{1};Eip{1};Eif{1}];
        else
            Hg{i}    = Yif{i}'*Qi_stack{i}*Yif{i} + Uif{i}'*Ri_stack{i}*Uif{i} + lambda_gi(i)*eye(T-Tini-N+1) + lambda_yi(i)*Yip{i}'*Yip{i} + ...
                rho/2*(eye(T-Tini-N+1) + Eif{i}'*Eif{i} + Yif{i}'*P{i}'*P{i}*Yif{i} + Uif{i}'*Uif{i});
            Aeqg{i}  = [Uip{i};Eip{i}];
        end
        
        KKT_vert{i} = inv([Hg{i},Aeqg{i}';Aeqg{i},zeros(size(Aeqg{i},1))]);
        
        if i ~= n_cav
            Hz{i}    = rho/2*eye(T-Tini-N+1) + rho/2*Yif{i}'*K{i}'*K{i}*Yif{i};
        else
            Hz{i}    = rho/2*eye(T-Tini-N+1);
        end
        Hz_vert{i} = inv(Hz{i});
        end
    end
    
    

    
    % ------------------
    %  Reference trajectory
    % ------------------
    r       = zeros(p_ctr,total_time_step+N);            % stabilization
    
    % ---------------------------------------------------------------------
    %   Simulation starts here
    %----------------------------------------------------------------------
    tic
    
    % ------------------
    %  Initial trajectory
    % ------------------
    uini = zeros(m_ctr,Tini);
    eini = zeros(1,Tini);
    yini = zeros(p_ctr,Tini);
    
    for k = 1:Tini-1
        % Update acceleration
        acel         =  HDV_dynamics(S(k,:,:),hdv_parameter) ...
            -acel_noise + 2*acel_noise*rand(n_vehicle,1);
        
        S(k,1,3)           = 0;               % the head vehicle
        S(k,2:end,3)       = acel;            % all the vehicles using HDV model
        S(k,pos_cav+1,3)   = uini(:,k);       % the CAV
        
        S(k+1,:,2) = S(k,:,2) + Tstep*S(k,:,3);
        S(k+1,1,2) = eini(k) + v_star;          % the velocity of the head vehicle
        S(k+1,:,1) = S(k,:,1) + Tstep*S(k,:,2);
        
        yini(:,k) = measure_mixed_traffic(S(k,2:end,2),S(k,:,1),ID,v_star,s_star,measure_type);
        
    end
    
    k_end = k+1;
    yini(:,k_end) = measure_mixed_traffic(S(k_end,2:end,2),S(k_end,:,1),ID,v_star,s_star,measure_type);
    
    
    u(:,1:Tini) = uini;
    e(:,1:Tini) = eini;
    y(:,1:Tini) = yini;
    
    % For MPC and DeeP-LCC with constraints
    previous_u_opt = 0;
    
    
    % ------------------
    %  simulation starts here
    % ------------------
    for k = Tini:total_time_step-1
        
        % Update acceleration
        acel         =  HDV_dynamics(S(k,:,:),hdv_parameter) ...
            -acel_noise + 2*acel_noise*rand(n_vehicle,1);
        
        S(k,2:end,3) = acel;     % all the vehicles using HDV model
        if min(min(yini)) < -15
            temp = 1;
        end
   
        if mix
            switch controller_type
                case 1
                    tic
                    % Calculate control input via centralized DeeP-LCC
                    if constraint_bool
                        [u_opt,y_opt,pr] = DeeP_LCC(Up,Yp,Uf,Yf,Ep,Ef,uini,yini,eini,Q,R,r(:,k:k+N-1),...
                            lambda_g,lambda_y,u_limit,s_limit);
                    else
                        [u_opt,y_opt,pr] = DeeP_LCC(Up,Yp,Uf,Yf,Ep,Ef,uini,yini,eini,Q,R,r(:,k:k+N-1),...
                            lambda_g,lambda_y);
                    end
                    toc
                case 2
                    % Calculate control input via MPC
                    if constraint_bool
                        [u_opt,y_opt,pr] = qpMPC(ID,Tstep,hdv_type,measure_type,v_star,uini,yini,N,Q,R,r(:,k:k+N-1),u_limit,s_limit,previous_u_opt);
                        previous_u_opt   = u_opt;
                    else
                        [u_opt,y_opt,pr] = qpMPC(ID,Tstep,hdv_type,measure_type,v_star,uini,yini,N,Q,R,r(:,k:k+N-1));
                    end
                    %                 if pr~=1
                    %                     break;
                    %                 end
                case 3
                    % Calculate control input via distributed DeeP-LCC
                    % ------------------
                    %  construct distributed data
                    % ------------------
                    
                    for i=1:n_cav
                        ui_ini{i}                 = uini(i,:);
                        
                        yi_ini{i}                 = zeros(ni_vehicle(i)+1,Tini);
                        if i~= n_cav
                            yi_ini{i}(1:end-1,:)  = yini(pos_cav(i):pos_cav(i+1)-1,:);
                        else
                            yi_ini{i}(1:end-1,:)  = yini(pos_cav(i):n_vehicle,:);
                        end
                        yi_ini{i}(end,:)          = yini(n_vehicle+i,:);
                        
                        if i==1
                            ei_ini{i}             = eini;
                        else
                            ei_ini{i}             = yini(pos_cav(i)-1,:);
                        end
                    end
                    
                    % -------------
                    % Update equilibrium
                    % -------------
                    
                    if update_equilibrium_bool == 1
                        % Estimated equilibrium velocity
                        v_star_estimated = zeros(n_cav,1);
                        s_star_estimated = zeros(n_cav,1);
                        % Estimate equilibrium velocity for each subsystem
                        id_cav = 1;
                        for i = 1:n_vehicle
                            if ID(i) == 1
                                % the average velocity of the vehicle ahead of the CAV
                                if k>Tini
                                    v_star_estimated(id_cav) = mean(S(k-Tini+1:k,i,2));
                                else
                                    v_star_estimated(id_cav) = mean(S(k-Tini+2:k,i,2));
                                end
                                % Design equilibrium spacing
                                %s_star_estimated(id_cav) = spacing_min + 1*v_star_estimated(id_cav);
                                s_star_estimated(id_cav) = acos(1-v_star_estimated(id_cav)/30*2)/pi*(35-5) + 5;
                                % Update spacing constraint
                                si_limit{id_cav} = [spacing_min,spacing_max]-s_star_estimated(id_cav);
                                id_cav = id_cav+1;
                            end
                            
                        end
                        
                        % Update equilibrium velocity for each past data
                        for i = 1:n_cav
                            yi_ini{i}(1:end-1,:)    = yi_ini{i}(1:end-1,:) + v_star - v_star_estimated(i);
                            yi_ini{i}(end,:)        = yi_ini{i}(end,:) + s_star - s_star_estimated(i);
                            ei_ini{i}   = ei_ini{i} + v_star - v_star_estimated(i);
                        end
                    else
                        for i = 1:n_cav
                            si_limit{i} = s_limit;
                        end
                    end
                    
                    tic
                    % distributed Deep-LCC
                    [u_opt,g_opt,mu_opt,eta_opt,phi_opt,theta_opt,real_iter_num] = dDeeP_LCC(Uip,Yip,Uif,Yif,Eip,Eif,ui_ini,yi_ini,ei_ini,Qi,Ri,...
                        lambda_gi,lambda_yi,u_limit,si_limit,rho,mu_initial,eta_initial,g_initial,phi_initial,theta_initial,KKT_vert,Hz_vert);
                    toc
                    
                    g_initial   = g_opt;
                    mu_initial  = mu_opt;
                    eta_initial = eta_opt;
                    
            end
            if controller_type == 3 %|| controller_type == 4 || controller_type == 5
                t_compute = toc/n_cav;
            else
                t_compute = toc;
            end
            computation_time(k) = t_compute;
            if controller_type == 3 %|| controller_type == 4 || controller_type == 5
                iteration_num(k)    = real_iter_num;
            end
            fprintf('Average computation time: %6.4f \n',t_compute);
            % One-step formulation
            u(:,k) = u_opt(1:m_ctr,1);
            % Update accleration for the CAV
            S(k,pos_cav+1,3)   = u(:,k);
            % Judge whether SD system commands to brake
            brake_vehicle_ID = find(acel==dcel_max);                % the vehicles that need to brake
            brake_cav_ID     = intersect(brake_vehicle_ID,pos_cav); % the CAVs that need to brake
            if ~isempty(brake_cav_ID)
                S(k,brake_cav_ID+1,3) = dcel_max;
            end
            % Record problem status
            %             pr_status(k) = pr;
        end
        
        % Update state
        S(k+1,:,2) = S(k,:,2) + Tstep*S(k,:,3);
        
        % -------------
        % Perturbation for the head vehicle
        % -------------
        switch per_type
            case 1
                S(k+1,1,2) = v_star + sine_amp*sin(2*pi/(10/Tstep)*(k-Tini));
                S(k+1,:,1) = S(k,:,1) + Tstep*S(k,:,2);
            case 2
                if (k-Tini)*Tstep <= brake_amp/5
                    S(k+1,1,3) = -5;
                elseif (k-Tini)*Tstep <= brake_amp/5+5
                    S(k+1,1,3) = 1;
                else
                    S(k+1,1,3) = 0;
                end
                S(k+1,:,2) = S(k,:,2) + Tstep*S(k,:,3);
                S(k+1,:,1) = S(k,:,1) + Tstep*S(k,:,2);
            case 3
                if (k-Tini)*Tstep <= brake_amp/5
                    S(k+1,1,3) = -5;
                elseif (k-Tini)*Tstep <= brake_amp/5+3
                    S(k+1,1,3) = 0;
                elseif (k-Tini)*Tstep <= brake_amp/5+3+5
                    S(k+1,1,3) = 1;
                else
                    S(k+1,1,3) = 0;
                end
                S(k+1,:,2) = S(k,:,2) + Tstep*S(k,:,3);
                S(k+1,:,1) = S(k,:,1) + Tstep*S(k,:,2);
            case 4
                if (k-Tini)*Tstep <= 2
                    S(k+1,1,3) = -5;
                elseif (k-Tini)*Tstep <= 2+5
                    S(k+1,1,3) = 2;
                end
                S(k+1,:,2) = S(k,:,2) + Tstep*S(k,:,3);
                S(k+1,:,1) = S(k,:,1) + Tstep*S(k,:,2);
            case 5
                if (k-Tini)*Tstep <= brake_amp/2
                    S(k,4,3) = -2;
                end
                S(k+1,:,2) = S(k,:,2) + Tstep*S(k,:,3);
                S(k+1,:,1) = S(k,:,1) + Tstep*S(k,:,2);
        end
        
        % Record output
        y(:,k) = measure_mixed_traffic(S(k,2:end,2),S(k,:,1),ID,v_star,s_star,measure_type);
        e(k)   = S(k,1,2) - v_star;
        
        % update past data in control process
        uini = u(:,k-Tini+1:k);
        yini = y(:,k-Tini+1:k);
        eini = S(k-Tini+1:k,1,2) - v_star;
        
        
        fprintf('Simulation number: %d  |  process... %2.2f%% \n',i_data,k/total_time_step*100);
        fprintf('Current spacing of the first CAV: %4.2f \n',S(k,pos_cav(1),1)-S(k,pos_cav(1),1));
        
        str=['Num:', num2str(i_data),' | Processing: ',num2str(k/total_time_step*100),'%'];
        waitbar(k/total_time_step,h_wait,str);
    end
    k_end = k+1;
    y(:,k_end) = measure_mixed_traffic(S(k_end,2:end,2),S(k_end,:,1),ID,v_star,s_star,measure_type);
    
    tsim = toc;
    
    fprintf('Simulation ends at %6.4f seconds \n', tsim);
    

    

% -------------------------------------------------------------------------
%   Plot Results
%--------------------------------------------------------------------------
% Simulation Time
begin_time       = 0.05;
end_time         = total_time;

color_gray  = [190 190 190]/255;
color_red   = [244, 53, 124]/255;
color_blue  = [67, 121, 227]/255;
color_black = [0 0 0];
color_orange = [255,132,31]/255;
label_size  = 18;
total_size  = 14;
line_width  = 2;

load('_data/ColorMap_RedWhiteBlue.mat');

% Velocity
figure;
id_cav = 1;
plot(begin_time:Tstep:end_time,S(begin_time/Tstep:end_time/Tstep,1,2),'Color',color_black,'linewidth',line_width-0.5); hold on;
for i = 1:n_vehicle
    if ID(i) == 0
        plot(begin_time:Tstep:end_time,S(begin_time/Tstep:end_time/Tstep,i+1,2),'Color',color_gray,'linewidth',line_width-0.5); hold on; % line for velocity of HDVs
    end
end
for i = 1:n_vehicle
    if ID(i) == 1
            plot(begin_time:Tstep:end_time,S(begin_time/Tstep:end_time/Tstep,i+1,2),'Color',mymap_red_white_blue(end,:),'linewidth',line_width); hold on; % line for velocity of CAVs
            id_cav  = id_cav+1;
    end
end
grid on;
set(gca,'TickLabelInterpreter','latex','fontsize',total_size);
set(gca,'YLim',[5 25]);
set(gca,'XLim',[0 total_time]);

xl = xlabel('$t$ [$\mathrm{s}$]','fontsize',label_size,'Interpreter','latex','Color','k');
yl = ylabel('Velocity [$\mathrm{m/s}$]','fontsize',label_size,'Interpreter','latex','Color','k');

set(gcf,'Position',[250 150 500 300]);
fig = gcf;
fig.PaperPositionMode = 'auto';
if output_bool
    if mix
        print(gcf,['figures/Controller_',num2str(controller_type),'_data_',num2str(i_data),'_T_',num2str(T),'_Tini_',num2str(Tini),'_N_',num2str(N),'_Trajectory'],'-dpng','-r300');
    else
        print(gcf,'figures/HDVs_Trajectory','-dpng','-r300');
    end
end

% Spacing
figure;
id_cav = 1;
for i = 1:n_vehicle
    if ID(i) == 1
            plot(begin_time:Tstep:end_time,S(begin_time/Tstep:end_time/Tstep,i,1)-S(begin_time/Tstep:end_time/Tstep,i+1,1),'Color',mymap_red_white_blue(end,:),'linewidth',line_width); hold on; % line for velocity of CAVs
            id_cav  = id_cav+1;
    end
end
grid on;
set(gca,'TickLabelInterpreter','latex','fontsize',total_size);
set(gca,'YLim',[0 30]);
set(gca,'XLim',[0 total_time]);

xl = xlabel('$t$ [$\mathrm{s}$]','fontsize',label_size,'Interpreter','latex','Color','k');
yl = ylabel('Spacing [$\mathrm{m}$]','fontsize',label_size,'Interpreter','latex','Color','k');

set(gcf,'Position',[250 450 500 300]);
fig = gcf;
fig.PaperPositionMode = 'auto';
if output_bool
    if mix
        print(gcf,['figures/Controller_',num2str(controller_type),'_data_',num2str(i_data),'_T_',num2str(T),'_Tini_',num2str(Tini),'_N_',num2str(N),'_Spacing'],'-dpng','-r300');
    else
        print(gcf,'figures/HDVs_Spacing','-dpng','-r300');
    end
end

% Computation time
figure;
plot(begin_time:Tstep:end_time,computation_time,'Color',color_black,'linewidth',line_width-0.5); hold on;
set(gca,'TickLabelInterpreter','latex','fontsize',total_size);

title(['Average time: ',num2str(mean(computation_time(Tini:total_time_step-1)))],'fontsize',label_size,'Interpreter','latex','Color','k')
set(gcf,'Position',[650 150 500 300]);
fig = gcf;
fig.PaperPositionMode = 'auto';
if output_bool
    if mix
        print(gcf,['figures/Controller_',num2str(controller_type),'_data_',num2str(i_data),'_T_',num2str(T),'_Tini_',num2str(Tini),'_N_',num2str(N),'_ComputationTime'],'-dpng','-r300');
    end
end

% Iteration number
if controller_type == 3 %|| controller_type == 4 || controller_type == 5
    figure;
    plot(begin_time:Tstep:end_time,iteration_num,'Color',color_black,'linewidth',line_width-0.5); hold on;
    set(gca,'TickLabelInterpreter','latex','fontsize',total_size);
    
    title(['Average iteration: ',num2str(mean(iteration_num(Tini:total_time_step-1)))],'fontsize',label_size,'Interpreter','latex','Color','k')
    set(gcf,'Position',[1050 150 500 300]);
    fig = gcf;
    fig.PaperPositionMode = 'auto';
    if output_bool
        if mix
            print(gcf,['figures/Controller_',num2str(controller_type),'_data_',num2str(i_data),'_T_',num2str(T),'_Tini_',num2str(Tini),'_N_',num2str(N),'_IterationNumber'],'-dpng','-r300');
        end
    end
end

% Record data
Collected_computation_time(i_data)  = mean(computation_time(Tini:total_time_step-1));
Collected_iteration_num(i_data)     = mean(iteration_num(Tini:total_time_step-1));

% close all;

if output_bool
    if mix
        save(['_data\simulation_data\','LargeScale','_',num2str(i_data),'_ControllerType_',num2str(controller_type),...
            '_PerType_',num2str(per_type),'_T_',num2str(T),'_UpdateEquilibrium_',num2str(update_equilibrium_bool),...
            '_Weight_',num2str(weight_choice),'.mat'],...
            'hdv_type','acel_noise','S','T','Tini','N','ID','Tstep','v_star','computation_time','iteration_num');
    else
        save(['_data\simulation_data\LargeScale_HDVs','_PerType_',num2str(per_type),'.mat'],...
            'hdv_type','acel_noise','S','T','Tini','N','ID','Tstep','v_star');
    end
end

end

close(h_wait);
