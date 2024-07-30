%% TRACI TEST: LAS PALMAS MULTIMODAL SCENARIO
try
    traci.close()
end

clear
close all
clc



rng(2020314097)


[scenarioPath,~,~] = fileparts(which(mfilename));

cd(scenarioPath);

scenarioPath = [scenarioPath, '/highway_scenario.sumocfg'];
%% Traffic Setting
roadVehicleMax = 2000;
roadTruckMax = 500;
roadVehicleMin = 0;
rhoArray = 20:20:400;
% rhoArray = 400;
totVehNumArray = 2*rhoArray;
iterPerRho = 100;

totIterNum = 500000;

seedArray = randsample(totIterNum, totIterNum,false);
seedIdx = 1;

for totVehIdx = 1:length(totVehNumArray)
    totVehNum = totVehNumArray(totVehIdx);
    rho = rhoArray(totVehIdx);
    caseName = ['rho',num2str(rho), '_CaccDrivingTrafficData'];
    saveDirName = ['./trafficData/', caseName];
    mkdir(saveDirName)
    saveDir = dir(saveDirName);

    for iterIdx = 1:iterPerRho
        clearvars MyStruct

        seed = seedArray(seedIdx);
        rng(seed)
        import traci.constants
        delete sumoTrace.xml
        traci.start(['sumo-gui -c ' '"' scenarioPath '"' ' --start --lateral-resolution 1 --quit-on-end --fcd-output sumoTrace.xml']);
        
        vehIDs = 1:totVehNum;
        laneIDs = repmat(1:6, 1, 10000);
        laneIDs = laneIDs(1:length(vehIDs));
        rpArray = randperm(length(laneIDs));
        laneIDs = laneIDs(rpArray);
        posVehicleArray = linspace(roadVehicleMin, roadVehicleMax, length(laneIDs));
        posVehicleArray = randsample(posVehicleArray, length(laneIDs), false);
        
        truckIDs = 1:5;
        
        posTruckArray = flip(50 + 20*(truckIDs - 1));
        
        for truckID = truckIDs
            if truckID == 1
                traci.vehicle.add(num2str(truckID), 'r_0', 'typeTruckLeader', '', '0', num2str(posTruckArray(truckID)), '0')
                disp(['Vehicle ID: ', num2str(truckID), ' successful add'])
            else
                traci.vehicle.add(num2str(truckID), 'r_0', 'typeTruckFollow', '', '0', num2str(posTruckArray(truckID)), '13')
                disp(['Vehicle ID: ', num2str(truckID), ' successful add'])
            end
        end
        
        vehIDs(truckIDs) = [];
        velRange = [7, 15];
        
        for vehID = vehIDs
            if laneIDs(vehID) == 1
                
                traci.vehicle.add(num2str(vehID), 'r_0', 'typeNormal', '', '1')
                traci.vehicle.moveToXY(num2str(vehID), '-E1', 1, posVehicleArray(vehID), 8)
                disp(['Vehicle ID: ', num2str(vehID), ' successful add'])
        
            elseif laneIDs(vehID) == 2
                
                traci.vehicle.add(num2str(vehID), 'r_0', 'typeNormal', '', '2')
                traci.vehicle.moveToXY(num2str(vehID), '-E1', 2, posVehicleArray(vehID), 5)
                disp(['Vehicle ID: ', num2str(vehID), ' successful add'])
        
            elseif laneIDs(vehID) == 3
                
                traci.vehicle.add(num2str(vehID), 'r_0', 'typeNormal', '', '3')
                traci.vehicle.moveToXY(num2str(vehID), '-E1', 3, posVehicleArray(vehID), 2)
                disp(['Vehicle ID: ', num2str(vehID), ' successful add'])
        
            elseif laneIDs(vehID) == 4
                
                traci.vehicle.add(num2str(vehID), 'r_1', 'typeNormal', '', '1')
                traci.vehicle.moveToXY(num2str(vehID), 'E1', 1, posVehicleArray(vehID), -8)
                disp(['Vehicle ID: ', num2str(vehID), ' successful add'])
        
            elseif laneIDs(vehID) == 5
                
                traci.vehicle.add(num2str(vehID), 'r_1', 'typeNormal', '', '2')
                traci.vehicle.moveToXY(num2str(vehID), 'E1', 2, posVehicleArray(vehID), -5)
                disp(['Vehicle ID: ', num2str(vehID), ' successful add'])
        
            elseif laneIDs(vehID) == 6
                
                traci.vehicle.add(num2str(vehID), 'r_1', 'typeNormal', '', '3')
                traci.vehicle.moveToXY(num2str(vehID), 'E1', 3, posVehicleArray(vehID), -2)
                disp(['Vehicle ID: ', num2str(vehID), ' successful add'])
        
            end
            traci.vehicle.setMaxSpeed(num2str(vehID), 11);
        end
        
        simulTotStep = 1000000;
        
        veh1SpeedArray = [];
        veh2SpeedArray = [];
        veh3SpeedArray = [];
        veh4SpeedArray = [];
        veh5SpeedArray = [];
        startIndicator = false;
        firstIndicator = true;
        leavingEventIndicator = false;
        joiningEventIndicator1 = false;
        joiningEventIndicator2 = false;
        joiningEventIndicator3 = false;
        joiningEventIndicator4 = false;
        
        saveStartIndicator = false;
        endIndicator = false;
        truckVelDiff = Inf;
        
        vehTrajectory = zeros(totVehNum, 2, 0);
        vehPosition = zeros(totVehNum, 2);
        saveIter = 1;
        velVarRange = [80*(1000/3600), 120*(1000/3600)];
        
        
        for simulStep = 1:simulTotStep
            if endIndicator
                break
            end
            
            traci.simulation.step();
            vehicleIdArray = traci.vehicle.getIDList;
            
            targetTruckID = 3;
            targetLocalLeaderID = targetTruckID - 1;
            targetLocalFollowID = targetTruckID + 1;
            
            if length(vehicleIdArray) == totVehNum && (firstIndicator)
                for vehID = vehIDs
                    traci.vehicle.setMaxSpeed(num2str(vehID), 36.11);
                end
                firstIndicator = false;
                currentStep = simulStep;
            end
        
            if ~firstIndicator
                vehPosition = traci.vehicle.getPosition(num2str(targetTruckID));
                if ((vehPosition(1) >= 300) && (vehPosition(1) <= 1700)) && (vehPosition(2) > 0) && leavingEventIndicator
                    traci.vehicle.setRouteID(num2str(targetTruckID), 'r_2')
            
                    leavingEventIndicator = false;
                end
                
                targetTruckRoadID = traci.vehicle.getRoadID(num2str(targetTruckID));
                
                if strcmp(targetTruckRoadID, 'E2') && ~joiningEventIndicator1 && ~joiningEventIndicator2 && ~joiningEventIndicator3
                    traci.vehicle.setMaxSpeed(num2str(targetTruckID), 0.001);
                    joiningEventIndicator1 = true;
                end
                
                targetLocalLeaderPos = traci.vehicle.getPosition(num2str(targetLocalLeaderID));
                
                targetLeaderRoadID = traci.vehicle.getRoadID(num2str(1));
                targetLeaderPos = traci.vehicle.getPosition(num2str(1));
        
                targetTruckPos = traci.vehicle.getPosition(num2str(targetTruckID));
            
                if ((targetLocalLeaderPos(2) > 0) && (targetLeaderPos(1) < 1700 + 150)) && joiningEventIndicator1 && strcmp(targetLeaderRoadID, '-E11')
                    traci.vehicle.setMaxSpeed(num2str(targetTruckID), 27.77)
                    joiningEventIndicator1 = false;
                    joiningEventIndicator2 = true;
                end
            
                if ((targetLocalLeaderPos(2) > 0) && (targetLeaderPos(1) < 1700 + 50)) && joiningEventIndicator2
                    % binStr = '011011'; % 27
                    % binStr = '100100'; % 36
                    % binStr = '100001'; % 33
                    binStr = '100101'; % 37
                    % binStr = '100000'; % 32
                    % binStr = '011111'; % 31 (default)
                    D = bin2dec(binStr);
                    traci.vehicle.setSpeedMode(num2str(targetTruckID), D)
                    tempSpeed = 14;
                    traci.vehicle.setMaxSpeed(num2str(1), tempSpeed )
                    traci.vehicle.setMaxSpeed(num2str(2), tempSpeed )
                    traci.vehicle.setMaxSpeed(num2str(4), tempSpeed )
                    traci.vehicle.setMaxSpeed(num2str(5), tempSpeed )
        
                    % traci.vehicle.setRouteID(num2str(targetTruckID), 'r_0')
                    traci.vehicle.setMinGap(num2str(targetLocalFollowID), 20)
                    % traci.vehicle.setMinGap(num2str(1), 25)
                    traci.vehicle.setMinGap(num2str(2), 1)
                    traci.vehicle.setMinGap(num2str(3), 1)
                    % traci.vehicle.setMinGap(num2str(5), 25)
                    joiningEventIndicator2 = false;
                    joiningEventIndicator3 = true;
                end
                
                targetTruckRoadID = traci.vehicle.getRoadID(num2str(targetTruckID));
                if strcmp(targetTruckRoadID, '-E10') && joiningEventIndicator3
                    traci.vehicle.setRouteID(num2str(targetTruckID), 'r_0')
                    
                    tempSpeed = 14;
                    maxSpeed = 36.11;
                    traci.vehicle.setMaxSpeed(num2str(1), tempSpeed )
                    traci.vehicle.setMaxSpeed(num2str(2), maxSpeed )
                    traci.vehicle.setMaxSpeed(num2str(3), maxSpeed )
                    traci.vehicle.setMaxSpeed(num2str(4), maxSpeed )
                    traci.vehicle.setMaxSpeed(num2str(5), maxSpeed )
        
                    tempMiniGap = 2.5;
                    traci.vehicle.setMinGap(num2str(1), tempMiniGap)
                    traci.vehicle.setMinGap(num2str(2), tempMiniGap)
                    traci.vehicle.setMinGap(num2str(3), tempMiniGap)
                    traci.vehicle.setMinGap(num2str(4), tempMiniGap)
                    traci.vehicle.setMinGap(num2str(5), tempMiniGap)
        
                    joiningEventIndicator3 = false;
                    joiningEventIndicator4 = true;
                    step = 0;
                end
                truckVel = [];
                truckGap = [];
                if joiningEventIndicator4
                    step = step + 1;
                end
                
                if strcmp(targetTruckRoadID, '-E10') && joiningEventIndicator4 && (step >= 3500)
                    traci.vehicle.setRouteID(num2str(targetTruckID), 'r_0')
        
                    tempSpeed = 22.22;
                    maxSpeed = 27.77;
                    traci.vehicle.setMaxSpeed(num2str(1), tempSpeed )
                    traci.vehicle.setMaxSpeed(num2str(2), maxSpeed )
                    traci.vehicle.setMaxSpeed(num2str(3), maxSpeed )
                    traci.vehicle.setMaxSpeed(num2str(4), maxSpeed )
                    traci.vehicle.setMaxSpeed(num2str(5), maxSpeed )
        
                    joiningEventIndicator4 = false;
                    leavingEventIndicator = true;
                end
        
                if simulStep >= currentStep + 10/0.01
                    if strcmp(traci.vehicle.getRoadID(num2str(5)), '-E10') && ~saveStartIndicator
                        saveStartIndicator = true;
                        saveStartStep = simulStep;
                    end
                    % if strcmp(traci.vehicle.getRoadID(num2str(1)), '-E1')
                    %     saveStartIndicator = false;
                    %     endIndicator = true;
                    %     saveEndStep = simulStep;
                    % end
                    if saveStartIndicator
                        for vehID = [truckIDs, vehIDs]
                            vehTrajectory(vehID, :, saveIter) = traci.vehicle.getPosition(num2str(vehID));
                        end
                        saveIter = saveIter + 1;
                        if strcmp(traci.vehicle.getRoadID(num2str(1)), '-E1')
                            saveStartIndicator = false;
                            endIndicator = true;
                            saveEndStep = simulStep;
                        end
                    end
                    stop = 0;
                end
            end
        end
        
        maxtries = 100;
        failcount = 0;
        
        try
            traci.close()
        end

        for k = 1:maxtries
            try
                sampleXMLfile = './sumoTrace.xml';
                % DOMnode = xmlread(sampleXMLfile);
                MyStruct = readstruct(sampleXMLfile);
                break;
            catch
                failcount = failcount + 1;
            end
        end
        if failcount == 100
            error('Error')
        end
        
        timeStepArray = cell2mat({MyStruct.timestep(:).timeAttribute});
        
        vehTrajectoryTemp = zeros(totVehNum, 3, 0);
        
        for timeStepIdx = 1:length(timeStepArray)
            idList = cell2mat({MyStruct.timestep(timeStepIdx).vehicle(:).idAttribute});
            xPosition = cell2mat({MyStruct.timestep(timeStepIdx).vehicle(:).xAttribute});
            yPosition = cell2mat({MyStruct.timestep(timeStepIdx).vehicle(:).yAttribute});
            vehTrajectoryTemp(idList, :, timeStepIdx) = [idList', xPosition', yPosition'];
        end
        vehTrajectoryTemp = vehTrajectoryTemp(:, :, saveStartStep:saveEndStep);
        
        save([saveDir(1).folder, filesep, 'iter',num2str(iterIdx),'_seed',num2str(seed), '.mat'],'vehTrajectory', 'vehTrajectoryTemp', '-v7.3');
        
        seedIdx = seedIdx + 1;
    end
end

traci.close()