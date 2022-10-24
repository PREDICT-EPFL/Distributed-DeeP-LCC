% =========================================================================
%                 Process moderate-scale simulation results
%                 Plot trajectory from one simulation
%
% *Attention*: need simulation data to run this code. 
% If there are no simulation results yet, please run main_ModerateScale 
% first.             
% =========================================================================

clc; clear; close all;

% Data set
data_str         = '1';  % 1. random ovm  2. manual ovm  3. homogeneous ovm
% Mix or not
mix              = 0;    % 0. all HDVs; 1. mix
% Type of the controller
controller_type  = 3;    % 1. centralized DeeP-LCC  2. MPC  3.distributed DeeP-LCC
% Type for HDV car-following model
hdv_type         = 1;    % 1. OVM   2. IDM
% Perturbation type
per_type         = 1; % 1. sinuoid perturbation 2. small brake perturbation 3. large brake perturbation
                      % 4. larger brake perturbation
                      % 5. Perturbation on a vehicle in the middle of the platoon
% Time horizon for DeeP-LCC
switch controller_type
    case 1
        T                = 1200; % For centralized DeeP-LCC
    case 3
        T                = 300;  % For distributed DeeP-LCC
end

% data number
i_data           = 1;

% Whether print figures
output_bool      = 1;

% ID
ID          = [1,0,0,1,0,0,1,0,0,1,0,0,1,0,0];    % ID of vehicle types
ID_str      = num2str(ID);
ID_str(find(ID_str==' ')) = '';

pos_cav     = find(ID==1);          % position of CAVs
n_vehicle   = length(ID);           % number of vehicles
n_cav       = length(pos_cav);      % number of CAVs
n_hdv       = n_vehicle-n_cav;      % number of HDVs

% Parameters in Simulation
total_time       = 30;              % Total Simulation Time
Tstep            = 0.05;            % Time Step
total_time_step  = total_time/Tstep;

if mix
    load(['_data\simulation_data\',ID_str,'_',num2str(i_data),'_ControllerType_',num2str(controller_type),'_PerType_',num2str(per_type),'_T_',num2str(T),'.mat']);
else
    load(['_data\simulation_data\HDVs','_PerType_',num2str(per_type),'.mat']);
end




% -------------------------------------------------------------------------
%   Plot Results
%--------------------------------------------------------------------------
% Simulation Time
begin_time       = 0.05;
end_time         = 30;

color_gray  = [190 190 190]/255;
color_red   = [244, 53, 124]/255;
color_blue  = [67, 121, 227]/255;
color_black = [0 0 0];
color_orange = [255,132,31]/255;
label_size  = 18;
total_size  = 16;
line_width  = 2;

load('_data/ColorMap_RedWhiteBlue.mat');
color_cav = mymap_red_white_blue(linspace(108,76,5),:);

% Velocity
figure;
id_cav = 1;
plot(begin_time:Tstep:end_time,S(begin_time/Tstep:end_time/Tstep,1,2),'Color',color_black,'linewidth',line_width-0.5); hold on;
for i = 1:n_vehicle
    if ~mix
        plot(begin_time:Tstep:end_time,S(begin_time/Tstep:end_time/Tstep,i+1,2),'Color',color_gray,'linewidth',line_width-0.5); hold on; % line for velocity of HDVs
    elseif ID(i) == 0
        plot(begin_time:Tstep:end_time,S(begin_time/Tstep:end_time/Tstep,i+1,2),'Color',color_gray,'linewidth',line_width-0.5); hold on; % line for velocity of HDVs
    end
end
if mix
for i = 1:n_vehicle
    if ID(i) == 1
            velocity_plot(id_cav) = plot(begin_time:Tstep:end_time,S(begin_time/Tstep:end_time/Tstep,i+1,2),'Color',color_cav(id_cav,:),'linewidth',line_width); hold on; % line for velocity of CAVs
            id_cav  = id_cav+1;
    end
end
end
grid on;

if mix
    l = legend(velocity_plot,'CAV 1','CAV 2','CAV 3','CAV 4','CAV 5');
    l.Interpreter = 'latex';
    l.FontSize = total_size;
    l.Box = 'off';
    l.Orientation = 'horizontal';
    l.Location = 'north';
end

set(gca,'TickLabelInterpreter','latex','fontsize',total_size);
set(gca,'YLim',[8 22]);
set(gca,'XLim',[5 30]);

xl = xlabel('$t$ [$\mathrm{s}$]','fontsize',label_size,'Interpreter','latex','Color','k');
yl = ylabel('Velocity [$\mathrm{m/s}$]','fontsize',label_size,'Interpreter','latex','Color','k');

set(gcf,'Position',[250 150 750 300]);
fig = gcf;
fig.PaperPositionMode = 'auto';
if output_bool
    if mix
        print(gcf,['figures/SinusoidPerturbation_Controller_',num2str(controller_type),'_data_',num2str(i_data),'_T_',num2str(T),'_Tini_',num2str(Tini),'_N_',num2str(N),'_Trajectory'],'-painters','-depsc2','-r300');
    else
        print(gcf,'figures/SinusoidPerturbation_HDVs_Trajectory','-painters','-depsc2','-r300');
    end
end

% Spacing
figure;
id_cav = 1;
for i = 1:n_vehicle
    if ID(i) == 1
            plot(begin_time:Tstep:end_time,S(begin_time/Tstep:end_time/Tstep,i,1)-S(begin_time/Tstep:end_time/Tstep,i+1,1),'Color',color_cav(id_cav,:),'linewidth',line_width); hold on; % line for velocity of CAVs
            id_cav  = id_cav+1;
    end
end
grid on;
set(gca,'TickLabelInterpreter','latex','fontsize',total_size);
set(gca,'YLim',[0 30]);
set(gca,'XLim',[0 end_time]);

xl = xlabel('$t$ [$\mathrm{s}$]','fontsize',label_size,'Interpreter','latex','Color','k');
yl = ylabel('Spacing [$\mathrm{m}$]','fontsize',label_size,'Interpreter','latex','Color','k');

set(gcf,'Position',[250 450 500 300]);
fig = gcf;
fig.PaperPositionMode = 'auto';
% if output_bool
%     if mix
%         print(gcf,['figures/Controller_',num2str(controller_type),'_data_',num2str(i_data),'_T_',num2str(T),'_Tini_',num2str(Tini),'_N_',num2str(N),'_Spacing'],'-dpng','-r300');
%     else
%         print(gcf,'figures/HDVs_Spacing','-dpng','-r300');
%     end
% end

if mix
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
end