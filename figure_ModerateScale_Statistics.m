% =========================================================================
%                 Process moderate-scale simulation results
%                 Calculate statistics from all the random simulations
%
% *Attention*: need simulation data to run this code. 
% If there are no simulation results yet, please run main_ModerateScale 
% first.             
% =========================================================================
clc; clear; close all;

% Data set
data_str         = '1';  % 1. random ovm  2. manual ovm  3. homogeneous ovm
% Mix or not
mix              = 1;    % 0. all HDVs; 1. mix
% Perturbation type
per_type         = 1; % 1. sinuoid perturbation 2. small brake perturbation 3. large brake perturbation
                      % 4. larger brake perturbation
                      % 5. Perturbation on a vehicle in the middle of the platoon


% data number
data_number      = 100;

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

if data_str == '1'
    % Random setup for OVM
    load(['_data/hdv_ovm_random_',ID_str,'.mat']);
elseif data_str == '3'
    % Homegeneous setup for OVM
    load(['_data/hdv_ovm_homogeneous_',ID_str,'.mat']);
end

weight_v     = 1;        % weight coefficient for velocity error
weight_s     = 0.5;      % weight coefficient for spacing error
weight_u     = 0.1;      % weight coefficient for control input


s_star     = 20;
% -------------------------------------------------------------------------
%   Calculate statistics
% -------------------------------------------------------------------------

% -------------------------------------------------------------------------
%   Calculate statistics
%--------------------------------------------------------------------------
real_cost             = zeros(data_number,3);
time                  = zeros(data_number,3);
 

% Type of the controller
for controller_type = 1:2:3 

for i_data          = 1:data_number
    
    switch controller_type
        case 1
            T                = 1200;
        case 3
            T                = 300;
    end
    load(['_data\simulation_data\',ID_str,'_',num2str(i_data),'_ControllerType_',num2str(controller_type),'_PerType_',num2str(per_type),'_T_',num2str(T),'.mat']);

for i = 1:n_vehicle
    if ID(i) == 1
        real_cost(i_data,controller_type) = real_cost(i_data,controller_type) + ...
                                          weight_v*sum((S(:,i+1,2)-v_star).^2) + ...
                                          weight_s*sum((S(:,i,1)-S(:,i+1,1)-s_star).^2) + ...
                                          weight_u*sum(S(:,i+1,3).^2);
    else
        real_cost(i_data,controller_type) = real_cost(i_data,controller_type) + ...
                                          weight_v*sum((S(:,i+1,2)-v_star).^2);
    end
end

time(i_data,controller_type) = mean(computation_time(50:end));

end

end

average_cost = mean(real_cost);
average_time = mean(time);
% -------------------------------------------------------------------------
%   Plot Results
% -------------------------------------------------------------------------

color_gray  = [190 190 190]/255;
color_red   = [244, 53, 124]/255;
color_blue  = [67, 121, 227]/255;
color_black = [0 0 0];
color_blue_2 = [61, 90, 128]/255;
color_red_2  = [238, 108, 77]/255;
label_size  = 18;
total_size  = 16;
line_width  = 1;

% -----------------
% Real Cost
% -----------------
figure;

p1 = plot(real_cost(:,1),'Color',color_blue_2,'Linewidth',line_width); hold on;
plot(average_cost(1)*ones(1,data_number),'--','Color',color_blue_2,'Linewidth',line_width*2); hold on;
p2 = plot(real_cost(:,3),'Color',color_red_2,'Linewidth',line_width); hold on;
plot(average_cost(3)*ones(1,data_number),'--','Color',color_red_2,'Linewidth',line_width*2); hold on;


grid on;
l = legend([p1 p2],'Centralized DeeP-LCC','Distributed DeeP-LCC');
l.Interpreter = 'latex';
l.FontSize = label_size;
l.Box = 'off';

set(gca,'TickLabelInterpreter','latex','fontsize',total_size);
set(gca,'YLim',[0.8e4,2.6e4]);

grid on;

xl = xlabel('Pre-collected Data Sets','fontsize',label_size,'Interpreter','latex','Color','k');
yl = ylabel('Real Cost','fontsize',label_size,'Interpreter','latex','Color','k');

set(gcf,'Position',[250 150 750 300]);
fig = gcf;
fig.PaperPositionMode = 'auto';

print(gcf,'.\figures\SinusoidPerturbation_RealCost','-painters','-depsc2','-r300');

fprintf('Average Cost:   Centrzlied DeeP-LCC  |    Distributed DeeP-LCC    \n');
fprintf('             %4.2f  |  %4.2f  \n',average_cost(1),average_cost(3));

% -----------------
% Computation time
% -----------------
figure;

p1 = plot(time(:,1),'Color',color_blue_2,'Linewidth',line_width); hold on;
plot(average_time(1)*ones(1,data_number),'--','Color',color_blue_2,'Linewidth',line_width*2); hold on;
p2 = plot(time(:,3),'Color',color_red_2,'Linewidth',line_width); hold on;
plot(average_time(3)*ones(1,data_number),'--','Color',color_red_2,'Linewidth',line_width*2); hold on;


grid on;
l = legend([p1 p2],'Centralized DeeP-LCC','Distributed DeeP-LCC');
l.Interpreter = 'latex';
l.FontSize = label_size;
l.Box = 'off';

set(gca,'TickLabelInterpreter','latex','fontsize',total_size);
set(gca,'YLim',[-0.4,2.4]);

grid on;

xl = xlabel('Pre-collected Data Sets','fontsize',label_size,'Interpreter','latex','Color','k');
yl = ylabel('Computation Time','fontsize',label_size,'Interpreter','latex','Color','k');

set(gcf,'Position',[250 150 750 300]);
fig = gcf;
fig.PaperPositionMode = 'auto';

print(gcf,'.\figures\SinusoidPerturbation_ComputationTime','-painters','-depsc2','-r300');

fprintf('Average Cost:   Centrzlied DeeP-LCC  |    Distributed DeeP-LCC    \n');
fprintf('             %4.4f  |  %4.4f  \n',average_time(1),average_time(3));