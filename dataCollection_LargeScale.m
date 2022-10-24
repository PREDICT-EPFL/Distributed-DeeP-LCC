% =========================================================================
%               Data collection for large-scale traffic
% =========================================================================

clc; close all; clear;
addpath('_fcn');

% How many data sets to collect
data_total_number = 1;

h_wait = waitbar(0,'please wait');

for i_data = 1:data_total_number

% -------------------------------------------------------------------------
%   Parameter setup
% -------------------------------------------------------------------------

% Type for HDV car-following model
hdv_type        = 1;    % 1. OVM   2. IDM
% Uncertainty for HDV behavior
acel_noise      = 0.1;  % A white noise signal on HDV's original acceleration
% Data set
data_str        = '1';  % 1. random ovm  2. manual ovm  3. homogeneous ovm

% Parameters in Simulation
total_time       = 40;              % Total Simulation Time
Tstep            = 0.05;            % Time Step
total_time_step  = total_time/Tstep;


% ------------------------------------------
% DeeP-LCC Parameters 
% ------------------------------------------
% T       = 1200;      % length of data samples
T               = 600;       % Second choise

Tini            = 20;        % length of past data
N               = 50;        % length of predicted horizon

weight_v        = 1;        % weight coefficient for velocity error
weight_s        = 0.5;      % weight coefficient for spacing error   
weight_u        = 0.1;      % weight coefficient for control input

lambda_g        = 10;        % penalty on ||g||_2^2 in objective
lambda_y        = 1e4;      % penalty on ||sigma_y||_2^2 in objective


% ------------------------------------------
% Parameters in Mixed Traffic
% ------------------------------------------
load('ID_LargeScale.mat'); % record ID
ID_str      = num2str(ID);
ID_str(find(ID_str==' ')) = '';

pos_cav     = find(ID==1);          % position of CAVs
n_vehicle   = length(ID);           % number of vehicles
n_cav       = length(pos_cav);      % number of CAVs
n_hdv       = n_vehicle-n_cav;      % number of HDVs

mix         = 1;                    % whether mixed traffic flow

v_star      = 15;                   % Equilibrium velocity
s_star      = 20;                   % Equilibrium spacing for CAV

if data_str == '1'
    % Random setup for OVM
    load(['_data/hdv_ovm_random_largescale.mat']);
elseif data_str == '3'
    % Homegeneous setup for OVM
    load(['_data/hdv_ovm_homogeneous_',ID_str,'.mat']);
end

acel_max = 2;
dcel_max = -5;
    
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
%   Scenario initialization
%-------------------------------------------------------------------------- 

% There is one head vehicle at the very beginning
S           = zeros(total_time_step,n_vehicle+1,3);
S(1,1,1)    = 0;
for i = 2 : n_vehicle+1
    S(1,i,1) = S(1,i-1,1) - hdv_parameter.s_star(i-1);
end
S(1,:,2)    = v_star * ones(n_vehicle+1,1);

% -------------------------------------------------------------------------
%   Data collection
%-------------------------------------------------------------------------- 

% ------------------
%  persistently exciting input data
% ------------------
ud          = -1+2*rand(m_ctr,T);
ed          = -1+2*rand(1,T);
yd          = zeros(p_ctr,T);


% ------------------
%  generate output data
% ------------------
for k = 1:T-1
    % Update acceleration
    acel               = HDV_dynamics(S(k,:,:),hdv_parameter) ...
                         -acel_noise + 2*acel_noise*rand(n_vehicle,1);
    
    S(k,1,3)           = 0;         % the head vehicle
    S(k,2:end,3)       = acel;      % all the vehicles using HDV model
    S(k,pos_cav+1,3)   = ud(:,k);   % the CAVs
    
    S(k+1,:,2) = S(k,:,2) + Tstep*S(k,:,3);
    S(k+1,1,2) = ed(k) + v_star;   % the velocity of the head vehicle
    S(k+1,:,1) = S(k,:,1) + Tstep*S(k,:,2);    
    
    yd(:,k) = measure_mixed_traffic(S(k,2:end,2),S(k,:,1),ID,v_star,s_star,measure_type);
end
k = k+1;
yd(:,k) = measure_mixed_traffic(S(k,2:end,2),S(k,:,1),ID,v_star,s_star,measure_type);

% ------------------
%  construct distributed data
% ------------------

for i=1:n_cav
    ui_d{i}                 = ud(i,:);
    
    yi_d{i}                 = zeros(ni_vehicle(i)+1,T);
    if i~= n_cav
        yi_d{i}(1:end-1,:)  = yd(pos_cav(i):pos_cav(i+1)-1,:);
    else
        yi_d{i}(1:end-1,:)  = yd(pos_cav(i):n_vehicle,:);
    end
    yi_d{i}(end,:)          = yd(n_vehicle+i,:);
    
    if i==1
        ei_d{i}             = ed;
    else
        ei_d{i}             = yd(pos_cav(i)-1,:);
    end
end

% ------------------
%  data Hankel matrices
% ------------------
% For centralized DeeP-LCC
U   = hankel_matrix(ud,Tini+N);
Up  = U(1:Tini*m_ctr,:);
Uf  = U((Tini*m_ctr+1):end,:);

E   = hankel_matrix(ed,Tini+N);
Ep  = E(1:Tini,:);
Ef  = E((Tini+1):end,:);

Y   = hankel_matrix(yd,Tini+N);
Yp  = Y(1:Tini*p_ctr,:);
Yf  = Y((Tini*p_ctr+1):end,:);

% For distributed DeeP-LCC
for i=1:n_cav
    Ui{i}   = hankel_matrix(ui_d{i},Tini+N);
    Uip{i}  = Ui{i}(1:Tini,:);
    Uif{i}  = Ui{i}(Tini+1:end,:);
    
    Ei{i}   = hankel_matrix(ei_d{i},Tini+N);
    Eip{i}  = Ei{i}(1:Tini,:);
    Eif{i}  = Ei{i}(Tini+1:end,:);
    
    Yi{i}   = hankel_matrix(yi_d{i},Tini+N);
    Yip{i}  = Yi{i}(1:Tini*(ni_vehicle(i)+1),:);
    Yif{i}  = Yi{i}(Tini*(ni_vehicle(i)+1)+1:end,:);
end

str=['Processing...',num2str(i_data/data_total_number*100),'%'];
    waitbar(i_data/data_total_number,h_wait,str);

save(['_data\trajectory_data_collection\','LargeScale','_data',num2str(data_str),'_',num2str(i_data),...
    '_T_',num2str(T),'_',num2str(Tini),'_',num2str(N),...
    '_noiseLevel_',num2str(acel_noise),'.mat'],...
    'hdv_type','acel_noise','Up','Yp','Uf','Yf','Ep','Ef','T','Tini','N','ID','Tstep','v_star','Uip','Uif','Eip','Eif','Yip','Yif');

end

close(h_wait);