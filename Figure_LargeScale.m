% =========================================================================
%               Analysis for simulation results under same data sample
% =========================================================================

clc; clear; close all;

% Data set
data_str         = '1';  % 1. random ovm  2. manual ovm  3. homogeneous ovm
% Mix or not
mix              = 1;    % 0. all HDVs; 1. mix
% Type of the controller
controller_type  = 3;    % 1. centralized DeeP-LCC  2. MPC  3.distributed DeeP-LCC
% Choise of the weight coefficients
weight_choice    = 3;
% Whether update equilibrium velocity
update_equilibrium_bool = 0;

% Type for HDV car-following model
hdv_type         = 1;    % 1. OVM   2. IDM
% Perturbation type
per_type         = 3; % 1. sinuoid perturbation 2. small brake perturbation 3. large brake perturbation
% 4. larger brake perturbation
% 5. Perturbation on a vehicle in the middle of the platoon
% Time horizon for DeeP-LCC
T                = 600;

% data number
i_data           = 1;

% Whether print figures
output_bool      = 1;

% ID
load('ID_LargeScale.mat'); % record ID
ID_str      = num2str(ID);
ID_str(find(ID_str==' ')) = '';

pos_cav     = find(ID==1);          % position of CAVs
n_vehicle   = length(ID);           % number of vehicles
n_cav       = length(pos_cav);      % number of CAVs
n_hdv       = n_vehicle-n_cav;      % number of HDVs

% Parameters in Simulation
total_time       = 200;              % Total Simulation Time
Tstep            = 0.05;            % Time Step
total_time_step  = total_time/Tstep;

if mix
    load(['_data\simulation_data\','LargeScale','_',num2str(i_data),'_ControllerType_',...
        num2str(controller_type),'_PerType_',num2str(per_type),'_T_',num2str(T),...
        '_UpdateEquilibrium_',num2str(update_equilibrium_bool),'_Weight_',num2str(weight_choice),'.mat']);
    
else
    load(['_data\simulation_data\LargeScale_HDVs','_PerType_',num2str(per_type),'.mat']);
end




% -------------------------------------------------------------------------
%   Plot Results
%--------------------------------------------------------------------------
% Simulation Time
begin_time       = 0.05;
end_time         = 200;
display_time     = 150;

color_gray  = [190 190 190]/255;
color_red   = [244, 53, 124]/255;
color_blue  = [67, 121, 227]/255;
color_black = [0 0 0];
color_orange = [255,132,31]/255;
label_size  = 18;
total_size  = 16;
line_width  = 2;
line_width_3d = 0.7;

load('ColorMap_RedWhiteBlue.mat');
color_cav = mymap_red_white_blue(linspace(108,76,5),:);

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
        velocity_plot(id_cav) = plot(begin_time:Tstep:end_time,S(begin_time/Tstep:end_time/Tstep,i+1,2),'Color',color_blue,'linewidth',line_width); hold on; % line for velocity of CAVs
        id_cav  = id_cav+1;
    end
end
grid on;

% if mix
%     l = legend(velocity_plot,'CAV 1','CAV 2','CAV 3','CAV 4','CAV 5');
%     l.Interpreter = 'latex';
%     l.FontSize = total_size;
%     l.Box = 'off';
%     l.Orientation = 'horizontal';
%     l.Location = 'north';
% end

set(gca,'TickLabelInterpreter','latex','fontsize',total_size);
set(gca,'YLim',[8 22]);
set(gca,'XLim',[0 end_time]);

xl = xlabel('$t$ [$\mathrm{s}$]','fontsize',label_size,'Interpreter','latex','Color','k');
yl = ylabel('Velocity [$\mathrm{m/s}$]','fontsize',label_size,'Interpreter','latex','Color','k');

set(gcf,'Position',[250 150 750 300]);



% Spacing
figure;
id_cav = 1;
for i = 1:n_vehicle
    if ID(i) == 1
        plot(begin_time:Tstep:end_time,S(begin_time/Tstep:end_time/Tstep,i,1)-S(begin_time/Tstep:end_time/Tstep,i+1,1),'Color',color_blue,'linewidth',line_width); hold on; % line for velocity of CAVs
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
    plot(begin_time:Tstep:display_time,computation_time(begin_time/Tstep:display_time/Tstep),'Color',color_black,'linewidth',line_width/3); hold on;
    set(gca,'TickLabelInterpreter','latex','fontsize',total_size);
    plot(begin_time:Tstep:display_time,ones(length(begin_time:Tstep:display_time))*mean(computation_time(begin_time/Tstep:display_time/Tstep-1)),'--','Color',color_blue,'linewidth',line_width);
    
    set(gca,'YLim',[0 0.12]);
    set(gca,'XLim',[1.5 display_time]);
    grid on;
    xl = xlabel('$t$ [$\mathrm{s}$]','fontsize',label_size,'Interpreter','latex','Color','k');
    ylabel('Computation Time [$\mathrm{s}$]','fontsize',label_size,'Interpreter','latex','Color','k');
    
    %title(['Average time: ',num2str(mean(computation_time(begin_time/Tstep:display_time/Tstep-1)))],'fontsize',label_size,'Interpreter','latex','Color','k')
    set(gcf,'Position',[650 150 750 300]);
    fig = gcf;
    fig.PaperPositionMode = 'auto';
    
    fprintf('Mean computation time is %6.4f s\n', mean(computation_time(begin_time/Tstep:display_time/Tstep-1)));
    if output_bool
        print(gcf,'figures/LargeScale_ComputationTime','-painters','-dpng','-r600');
    end
    
    % Iteration number
    if controller_type == 3 %|| controller_type == 4 || controller_type == 5
        figure;
        plot(begin_time:Tstep:display_time,iteration_num(begin_time/Tstep:display_time/Tstep),'Color',color_black,'linewidth',line_width/3); hold on;
        set(gca,'TickLabelInterpreter','latex','fontsize',total_size);
        
        plot(begin_time:Tstep:display_time,ones(length(begin_time:Tstep:display_time))*mean(iteration_num(begin_time/Tstep:display_time/Tstep-1)),'--','Color',color_blue,'linewidth',line_width);
        
        set(gca,'YLim',[0 12]);
        set(gca,'XLim',[1.5 display_time]);
        grid on;
        xl = xlabel('$t$ [$\mathrm{s}$]','fontsize',label_size,'Interpreter','latex','Color','k');
        ylabel('Iteration number','fontsize',label_size,'Interpreter','latex','Color','k');
        
        %title(['Average iteration: ',num2str(mean(iteration_num(Tini:display_time/Tstep-1)))],'fontsize',label_size,'Interpreter','latex','Color','k')
        set(gcf,'Position',[1050 150 750 300]);
        fig = gcf;
        fig.PaperPositionMode = 'auto';
        
        fprintf('Mean iteration number is %6.4f s\n', mean(iteration_num(begin_time/Tstep:display_time/Tstep-1)));
        
        if output_bool
            print(gcf,'figures/LargeScale_IterationNumber','-painters','-dpng','-r600');
        end
    end
    
    
end

% Shock wave figure
% Type 1
figure;
for i=1:n_vehicle
    scatter((Tstep:Tstep:end_time)',S(:,i,1),15,S(:,i,2),'filled');
    hold on;
end
colormap(jet);
colormap(flipud(colormap));
caxis([5 15]);
hcb = colorbar;
hcb.TickLabelInterpreter = 'latex';
hcb.FontSize = total_size;
hcb.Label.String = 'Velocity [$\mathrm{m/s}$]';
hcb.Label.FontSize = label_size;
hcb.Label.Interpreter = 'latex';

set(gca,'TickLabelInterpreter','latex','fontsize',total_size);
set(gca,'XLim',[0 120]);
set(gca,'YLim',[-1000 500]);

xl = xlabel('$t$ [$\mathrm{s}$]','fontsize',label_size,'Interpreter','latex','Color','k');
yl = ylabel('Position [$\mathrm{m}$]','fontsize',label_size,'Interpreter','latex','Color','k');
grid on;

set(gcf,'Position',[1550 850 550 400]);
fig = gcf;
fig.PaperPositionMode = 'auto';
if output_bool
    if mix
        print(gcf,'figures/LargeScale_CAVs_Wave','-painters','-dpng','-r600');
    else
        print(gcf,'figures/LargeScale_HDVs_Wave','-painters','-dpng','-r600');
    end
end

% Type 2
% figure;
%
% [t_grid,p_grid] = meshgrid(0:0.1:120,-1000:1:500);
% t_vector        = repmat(Tstep:Tstep:end_time,1,n_vehicle);
% p_vector        = S(:,1,1)';
% for i = 2:n_vehicle
%     p_vector     = [p_vector S(:,i,1)'];
% end
% v_vector        = S(:,1,2)';
% for i = 2:n_vehicle
%     v_vector     = [v_vector S(:,i,2)'];
% end
% v_grid   = griddata(t_vector,p_vector,v_vector,t_grid,p_grid);
% mesh(t_grid,p_grid,v_grid);
% colormap(jet);
% colormap(flipud(colormap));
% caxis([5 15]);
% view([0,90]);
%
%
% hcb = colorbar;
% hcb.TickLabelInterpreter = 'latex';
% hcb.FontSize = total_size;
% hcb.Label.String = 'Velocity [$\mathrm{m/s}$]';
% hcb.Label.FontSize = label_size;
% hcb.Label.Interpreter = 'latex';
%
% set(gca,'TickLabelInterpreter','latex','fontsize',total_size);
% set(gca,'XLim',[0 120]);
% set(gca,'YLim',[-1000 500]);
%
% xl = xlabel('$t$ [$\mathrm{s}$]','fontsize',label_size,'Interpreter','latex','Color','k');
% yl = ylabel('Position [$\mathrm{m}$]','fontsize',label_size,'Interpreter','latex','Color','k');
%
% set(gcf,'Position',[1550 750 550 400]);


% Type 3
figure;

id_cav = 1;
plot3(zeros(total_time_step,1),begin_time:Tstep:end_time,S(begin_time/Tstep:end_time/Tstep,1,2),'Color',color_black,'linewidth',line_width/2); hold on;
for i = 1:n_vehicle
    if ~mix
        plot3(i*ones(total_time_step,1),begin_time:Tstep:end_time,S(begin_time/Tstep:end_time/Tstep,i+1,2),'Color',color_gray,'linewidth',line_width_3d); hold on; % line for velocity of HDVs
    elseif ID(i) == 0
        hdv_velocity_plot = plot3(i*ones(total_time_step,1),begin_time:Tstep:end_time,S(begin_time/Tstep:end_time/Tstep,i+1,2),'Color',color_gray,'linewidth',line_width_3d); hold on; % line for velocity of HDVs
    end
end
if mix
    for i = 1:n_vehicle
        if ID(i) == 1
            cav_velocity_plot = plot3(i*ones(total_time_step,1),begin_time:Tstep:end_time,S(begin_time/Tstep:end_time/Tstep,i+1,2),'Color',color_blue,'linewidth',line_width_3d); hold on; % line for velocity of CAVs
            id_cav  = id_cav+1;
        end
    end
end
grid on;
set(gca,'YLim',[0 150]);
set(gca,'ZLim',[0 30]);

set(gca,'TickLabelInterpreter','latex','fontsize',total_size);

xl = xlabel('Vehicle No.','fontsize',label_size,'Interpreter','latex','Color','k');
yl = ylabel('$t$ [$\mathrm{s}$]','fontsize',label_size,'Interpreter','latex','Color','k');
zl = zlabel('Velocity [$\mathrm{m/s}$]','fontsize',label_size,'Interpreter','latex','Color','k');
xl.Rotation = 30;
yl.Position = [-13 75 -5];
yl.Rotation = -10;

if mix
    l = legend([hdv_velocity_plot,cav_velocity_plot],'HDV','CAV');
    l.Interpreter = 'latex';
    l.FontSize = total_size;
    l.Box = 'off';
    l.Orientation = 'horizontal';
    l.Position = [0.65 0.7 0.1 0.1];
end
view([301 46]);

set(gcf,'Position',[650 850 800 400]);
fig = gcf;
fig.PaperPositionMode = 'auto';
if output_bool
    if mix
        print(gcf,'figures/LargeScale_CAVs_Velocity','-painters','-dpng','-r600');
    else
        print(gcf,'figures/LargeScale_HDVs_Velocity','-painters','-dpng','-r600');
    end
end

% ======================
% Calculate fuel consumption
% ======================


fuel = 0;
for i=begin_time/Tstep:display_time/Tstep
    R           = 0.333 + 0.00108*S(i,2:end,2).^2 + 1.2*S(i,2:end,3);
    F           = 0.444 + 0.09*R.*S(i,2:end,2) + 0.054 * max(0,S(i,2:end,3)).^2.*S(i,2:end,2);
    F(R <= 0)   = 0.444;
    fuel        = fuel + sum(F)*Tstep;
    
end

fprintf('Fuel consumption is %6.4f mL \n', fuel);