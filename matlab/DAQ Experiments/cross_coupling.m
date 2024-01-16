clear all; clc; close all;

% Connect to device
dq = daq("ni");
dq.Rate = 10000;
t_read = 20;

% Define output channels
ch1 = addinput(dq, "Dev1", "ai0", "Voltage");
ch2 = addinput(dq, "Dev1", "ai4", "Voltage");
ch3 = addinput(dq, "Dev1", "ai5", "Voltage");
ch4 = addinput(dq, "Dev1", "ai6", "Voltage");
ch1.TerminalConfig = "SingleEnded";
ch2.TerminalConfig = "SingleEnded";
ch3.TerminalConfig = "SingleEnded";
ch4.TerminalConfig = "SingleEnded";

save_data = true;

%% Collect data

data = read(dq, seconds(t_read), "OutputFormat", "Matrix");
t = 0:(1/dq.Rate):(t_read-(1/dq.Rate));
t = t';

% Apply threshold to data
data(data<2.5) = 0;
data(data>2.5) = 1;

chA = data(:,1);
chB = data(:,2);
chC = data(:,3);
chD = data(:,4);

input_pos = decodeQuadrature(chA,chB).*(2*pi/1024);
output_pos = decodeQuadrature(chC,chD).*(2*pi/1024);

figure(1); clf; hold on
plot(t,output_pos);
plot(t,input_pos);


if save_data
    save('theta_12.mat','input_pos','output_pos', 't');
end

%% PLotting function
data = load('theta_12.mat');
export_fig = true;
fig_s = 3;
% fonts
ax_font_size = 9*fig_s;
legend_font_size = 9*fig_s;

fig = figure(1); clf; hold on;
grid on
map = brewermap(9,'Set1');

motor_on = find(abs(data.input_pos)>0);
data.t = data.t(motor_on:end,:);
data.t = data.t - data.t(1);
data.input_pos = data.input_pos(motor_on:end,:);
data.output_pos = data.output_pos(motor_on:end,:);

plot(data.t,data.input_pos./(2*pi),'LineWidth',3,'color',map(1,:),"DisplayName","")
plot(data.t,data.output_pos./(2*pi),'LineWidth',3,'color',map(2,:),"DisplayName","")

% figure formatting
set(gcf,'color','w');
set(fig, 'Units', 'inches');
width = 2.25;
height = 1.75;
set(fig, 'Position', [0, 0, width*fig_s, height*fig_s]);

% axis formatting
set(findobj(gcf,'type','axes'),'FontSize',ax_font_size,'LineWidth',1.5);
xlim([0 15])
ylim([-5 25])

lg = legend('interpreter','latex','Location','northwest');
lg.FontSize = legend_font_size;
set(lg,'Box','off')
lg.ItemTokenSize(1) = 20;

% export fig
if export_fig
    exportgraphics(gcf,'./cross-coupling.png','Resolution',300*fig_s)
end

%% Function for decoding signal from encoders
function pos_array = decodeQuadrature(signalA, signalB)
    pos_array = zeros(size(signalA));
    position = 0;
    lastA = signalA(1);
    lastB = signalB(1);

    for i = 2:length(signalA)
        currentA = signalA(i);
        currentB = signalB(i);

        if lastA == 0 && currentA == 1
            % A rising edge
            if currentB == 0
                position = position + 1; % Forward step
            else
                position = position - 1; % Backward step
            end
        end
        pos_array(i) = position; 
        lastA = currentA;
        lastB = currentB;
    end
end
