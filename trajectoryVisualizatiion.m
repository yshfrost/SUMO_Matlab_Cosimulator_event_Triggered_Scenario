%%
clear
close all
clc
%% Data Generation
% vehTrajectory = load('E:\2022_WirelessCommunicationProjectCode\V2X_Simulator\integratedSumoMatlabSimulator\sumoMatlabCosimulator_Ver20240625_JoinAndLeaveEvent\trafficData\rho20_CaccDrivingTrafficData\iter1_seed218926.mat').vehTrajectory;
% vehTrajectoryTemp = load('E:\2022_WirelessCommunicationProjectCode\V2X_Simulator\integratedSumoMatlabSimulator\sumoMatlabCosimulator_Ver20240625_JoinAndLeaveEvent\trafficData\rho20_CaccDrivingTrafficData\iter2_seed172239.mat').vehTrajectory;

%%
f = figure;hold on;
ax1 = plot([0], [0], 'LineStyle','none', 'Color', 'Red', 'Marker', 'o', 'MarkerSize', 10, 'MarkerFaceColor', 'red');
% ax1_1 = plot([0], [0], 'LineStyle','none', 'Color', 'Red', 'Marker', 'o', 'MarkerSize', 15, 'MarkerFaceColor', 'none');

ax2 = plot([0], [0], 'LineStyle','none', 'Color', 'Blue', 'Marker', 'o', 'MarkerSize', 10, 'MarkerFaceColor', 'blue');
% ax2_1 = plot([0], [0], 'LineStyle','none', 'Color', 'Blue', 'Marker', 'o', 'MarkerSize', 15, 'MarkerFaceColor', 'none');

ax3 = plot([0], [0], 'LineStyle','none', 'Color', 'Black', 'Marker', 'o', 'MarkerSize', 10, 'MarkerFaceColor', 'black');
% ax3_1 = plot([0], [0], 'LineStyle','none', 'Color', 'Black', 'Marker', 'o', 'MarkerSize', 15, 'MarkerFaceColor', 'none');
xlim([0, 2000])
ylim([-10, 12])
hold off;

for timeIdx = 1:size(vehTrajectory, 3)
% for timeIdx = 1
    ax1.XData = vehTrajectory(1, 2, timeIdx);
    ax1.YData = vehTrajectory(1, 3, timeIdx);

    % ax1_1.XData = vehTrajectoryTemp(1, 2, timeIdx);
    % ax1_1.YData = vehTrajectoryTemp(1, 3, timeIdx);

    ax2.XData = vehTrajectory(2:5, 2, timeIdx);
    ax2.YData = vehTrajectory(2:5, 3, timeIdx);

    % ax2_1.XData = vehTrajectoryTemp(2:5, 2, timeIdx);
    % ax2_1.YData = vehTrajectoryTemp(2:5, 3, timeIdx);

    ax3.XData = vehTrajectory(6:end, 2, timeIdx);
    ax3.YData = vehTrajectory(6:end, 3, timeIdx);
    
    % ax3_1.XData = vehTrajectoryTemp(6:end, 2, timeIdx);
    % ax3_1.YData = vehTrajectoryTemp(6:end, 3, timeIdx);
    
    pause(0.001)

end

%%
return
sampleXMLfile = './sumoTrace.xml';
MyStruct = readstruct(sampleXMLfile);

timeStepArray = cell2mat({MyStruct.timestep(:).timeAttribute});

% timeStepArray = {MyStruct.timestep(1).vehicle(:).idAttribute};

totVehNum = 40;

vehTrajectoryTemp = zeros(totVehNum, 3, 0);

for timeStepIdx = 1:length(timeStepArray)
    idList = cell2mat({MyStruct.timestep(timeStepIdx).vehicle(:).idAttribute});
    xPosition = cell2mat({MyStruct.timestep(timeStepIdx).vehicle(:).xAttribute});
    yPosition = cell2mat({MyStruct.timestep(timeStepIdx).vehicle(:).yAttribute});
    vehTrajectoryTemp(idList, :, timeStepIdx) = [idList', xPosition', yPosition'];
end




