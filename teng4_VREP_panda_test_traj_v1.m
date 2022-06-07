clear
close all;
clc;

%%
% mex gettable.cpp
% mex settable.cpp

%%
Q=dlmread('exciting_traj_positions.txt');
Q=reshape(Q,7,[]); %every 7 is one column; reshape a vec to matrix, is generating column by column.
Q_des=Q';

disp('Q_des joint traj data imported!')

if 0 %output and save data as .mat file.
    size(Q_des) 
    save('Q_des.mat','Q_des');
    
    disp('Q_des.mat saved!');
    Q_des(100:105,:) %display several rows.
end

%%
% first need to OPEN & RUN scene "teng4_panda.ttt" in VREP.
% for a new scene, you must add code "simRemoteApi.start(19999)" into the
% very beginning of the scene script first.
% make sure to run the "teng4_panda_test" scene first in VREP. (only drag panda to a new scene is NOT ok! need modify the script!)
% - by teng4.20210521

vrep=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
vrep.simxFinish(-1); % just in case, close all opened connections
clientID=vrep.simxStart('127.0.0.1',19999,true,true,5000,5); %1999 need to be added into scene script.

if (clientID>-1)
    disp('Connected to remote API server');
    %     %create some joints pos
    %     joint_pos1 = [0,pi/4,0,0,0,0,0];
    %     joint_pos2 = [pi/3,pi/4,0,0,0,0,0];
    %     joint_pos3 = [pi/2,pi/4,0,0,0,0,0];
    
    %joint handles
    h=[0,0,0,0,0,0,0];
    %https://www.coppeliarobotics.com/helpFiles/en/remoteApiFunctionsMatlab.htm
    %[number returnCode,number handle]=simxGetObjectHandle(number clientID,string objectName,number operationMode)
    [r,h(1)] = vrep.simxGetObjectHandle(clientID,'Franka_joint1',vrep.simx_opmode_blocking);
    [r,h(2)] = vrep.simxGetObjectHandle(clientID,'Franka_joint2',vrep.simx_opmode_blocking);
    [r,h(3)] = vrep.simxGetObjectHandle(clientID,'Franka_joint3',vrep.simx_opmode_blocking);
    [r,h(4)] = vrep.simxGetObjectHandle(clientID,'Franka_joint4',vrep.simx_opmode_blocking);
    [r,h(5)] = vrep.simxGetObjectHandle(clientID,'Franka_joint5',vrep.simx_opmode_blocking);
    [r,h(6)] = vrep.simxGetObjectHandle(clientID,'Franka_joint6',vrep.simx_opmode_blocking);
    [r,h(7)] = vrep.simxGetObjectHandle(clientID,'Franka_joint7',vrep.simx_opmode_blocking);
    
    for itr=1:length(Q_des)
        %get q_des at time t.
        joint_pos1 = Q_des(itr,:);
        
        for i=1:7
            %simxSetJointTargetPosition(number clientID,number jointHandle,number targetPosition,number operationMode)
            vrep.simxSetJointTargetPosition(clientID, h(i), joint_pos1(i), vrep.simx_opmode_streaming); %simx_opmode_oneshot or simx_opmode_streaming
        end
        pause(0.0001);   
        if mod(itr,1000)==0
            disp({'itr', itr});
        end
    end
    disp('end of for loop.');
    
else
    disp('Failed connecting to remote API server');
end
vrep.delete(); % call the destructor!

disp('Program ended');



