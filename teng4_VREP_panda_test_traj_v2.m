% video source:
%  https://www.youtube.com/watch?v=7Z01cRw_i5E
% Remote API functions (Matlab):
%  file:///C:/Program%20Files/CoppeliaRobotics/CoppeliaSimEdu/helpFiles/index.html

%%
clear
close all;
clc;

%%
% mex gettable.cpp
% mex settable.cpp

%%
Q_des=dlmread('exciting_traj_positions.txt');
Q_des=reshape(Q_des,7,[]); %every 7 is one column; reshape a vec to matrix, is generating column by column.
Q_des=Q_des';

TAU_des=dlmread('exciting_traj_torques.txt');
TAU_des=reshape(TAU_des,7,[]); %every 7 is one column; reshape a vec to matrix, is generating column by column.
TAU_des=TAU_des';

disp('Q_des joint traj data imported!');
disp('TAU_des joint traj data imported!');

if 0 %output and save data as .mat file.
    size(Q_des)
    save('Q_des.mat','Q_des');    
    disp('Q_des.mat saved!');
    Q_des(100:105,:) %display several rows.
    
    size(TAU_des)
    save('TAU_des.mat','TAU_des');    
    disp('TAU_des.mat saved!');
    TAU_des(100:105,:) %display several rows.
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
disp(['Your VREP clientID=', num2str(clientID)]);

q=[0;0;0;0;0;0;0];
Q=[]; %actual q joint positions wrt. time
tau=[0;0;0;0;0;0;0];
TAU=[]; %actual joint torques wrt. time

% simx_opmode_blocking (or simx_opmode_oneshot_wait)
% Blocking mode. The command is sent, and the function will "wait for the actual reply and return it" (if the function doesn't time out).
% simx_opmode_buffer:
% Non-blocking mode. "A previous reply" to the same command is returned (if available). The command is not send, nor does the function wait for the actual reply.
% simx_opmode_streaming + alpha:
% Non-blocking mode. The command is sent and "a previous reply" to the same command returned (if available).

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
[r,h8] = vrep.simxGetObjectHandle(clientID,'Franka_connection',vrep.simx_opmode_blocking);
[r,h9] = vrep.simxGetObjectHandle(clientID,'Franka_link8',vrep.simx_opmode_blocking);

%Initialize streaming, otherwise, unexpected data will be stored at the begining.
for i=1:7
    [r1,test1]=vrep.simxGetJointPosition(clientID,h(i),vrep.simx_opmode_streaming);
    [r2,test2]=vrep.simxGetJointForce(clientID,h(i),vrep.simx_opmode_streaming);
end
[r3,X]=vrep.simxGetObjectGroupData(clientID,h8,19,vrep.simx_opmode_streaming);
pause(1);


%%%%%%%%%%%%%%%%%%%%%%%%
if (clientID>-1)
    disp('Connected to remote API server');
    
    for itr=1:length(Q_des)
        %%%
        %receive actual q joint positions from VREP PANDA in real time
        % Retrieves the intrinsic position of a joint. This function cannot be used with spherical joints (use simxGetJointMatrix instead). See also simxSetJointPosition and simxGetObjectGroupData.
        %[number returnCode,number position]=simxGetJointPosition(number clientID,number jointHandle,number operationMode)
        [r1,q(1)]=vrep.simxGetJointPosition(clientID,h(1),vrep.simx_opmode_buffer);
        [r1,q(2)]=vrep.simxGetJointPosition(clientID,h(2),vrep.simx_opmode_buffer);
        [r1,q(3)]=vrep.simxGetJointPosition(clientID,h(3),vrep.simx_opmode_buffer);
        [r1,q(4)]=vrep.simxGetJointPosition(clientID,h(4),vrep.simx_opmode_buffer);
        [r1,q(5)]=vrep.simxGetJointPosition(clientID,h(5),vrep.simx_opmode_buffer);
        [r1,q(6)]=vrep.simxGetJointPosition(clientID,h(6),vrep.simx_opmode_buffer);
        [r1,q(7)]=vrep.simxGetJointPosition(clientID,h(7),vrep.simx_opmode_buffer);        
        %pause(0.0001);
        Q(itr,:)=q'; %store all q wrt. time.
        
        %%%
        %receive joint torques from VREP PANDA in real time
        %Retrieves the force or torque applied to a joint along/about its active axis. This function retrieves meaningful information only if the joint is prismatic or revolute, and is dynamically enabled. With the Bullet engine, this function returns the force or torque applied to the joint motor (torques from joint limits are not taken into account). With the ODE or Vortex engine, this function returns the total force or torque applied to a joint along/about its z-axis. See also simxSetJointMaxForce, simxReadForceSensor and simxGetObjectGroupData.
        %[number returnCode,number force]=simxGetJointForce(number clientID,number jointHandle,number operationMode)
        [r2,tau(1)]=vrep.simxGetJointForce(clientID,h(1),vrep.simx_opmode_buffer);
        [r2,tau(2)]=vrep.simxGetJointForce(clientID,h(2),vrep.simx_opmode_buffer);
        [r2,tau(3)]=vrep.simxGetJointForce(clientID,h(3),vrep.simx_opmode_buffer);
        [r2,tau(4)]=vrep.simxGetJointForce(clientID,h(4),vrep.simx_opmode_buffer);
        [r2,tau(5)]=vrep.simxGetJointForce(clientID,h(5),vrep.simx_opmode_buffer);
        [r2,tau(6)]=vrep.simxGetJointForce(clientID,h(6),vrep.simx_opmode_buffer);
        [r2,tau(7)]=vrep.simxGetJointForce(clientID,h(7),vrep.simx_opmode_buffer);        
        %pause(0.0001);
        TAU(itr,:)=tau'; %store all tau wrt. time.
        
        %%% not yet ok.
        %Simultaneously retrieves data of various objects in a CoppeliaSim scene.
        %[number returnCode,array handles,array intData,array floatData,array stringData]=simxGetObjectGroupData(number clientID,number objectType,number dataType,number operationMode)
        %3: retrieves the absolute object positions (in floatData. There are 3 values for each object (x,y,z))
        %17: retrieves the object linear velocity (in floatData. There are 3 values for each object (vx,vy,vz))
        %19: retrieves the object linear and angular velocity (in floatData. There are 6 values for each object (vx,vy,vz,dAlpha,dBeta,dGamma))
        %[returnCode1,handles1,intData1,floatData1,stringData1]=vrep.simxGetObjectGroupData(clientID,h8,17,vrep.simx_opmode_streaming); %not yet ok.
        %[returnCode2,handles2,intData2,floatData2,stringData2]=vrep.simxGetObjectGroupData(clientID,h9,3,vrep.simx_opmode_streaming); %not yet ok.

        %%%
        %get q_des at time t.
        joint_pos1 = Q_des(itr,:);        
        %send q_des command
        for i=1:7
            %simxSetJointTargetPosition(number clientID,number jointHandle,number targetPosition,number operationMode)
            vrep.simxSetJointTargetPosition(clientID, h(i), joint_pos1(i), vrep.simx_opmode_streaming); %simx_opmode_oneshot or simx_opmode_streaming
        end
        
        %pause(0.0001);
        if mod(itr,1000)==0
            disp({'itr', itr});
        end      

    end
    
    
    %%%
    % Now retrieve streaming data (i.e. in a non-blocking fashion):
    % detect mouse position in VREP windows.
    t=clock;
    startTime=t(6);
    currentTime=t(6);
    vrep.simxGetIntegerParameter(clientID,vrep.sim_intparam_mouse_x,vrep.simx_opmode_streaming); % Initialize streaming
    while (currentTime-startTime < 3) %<3s
        if currentTime-startTime==0; disp('Move your mouse into VREP windows...'); end
        
        [returnCode,data]=vrep.simxGetIntegerParameter(clientID,vrep.sim_intparam_mouse_x,vrep.simx_opmode_buffer); % Try to retrieve the streamed data
        if (returnCode==vrep.simx_return_ok) % After initialization of streaming, it will take a few ms before the first value arrives, so check the return code
            if mod(currentTime-startTime,2)==0 %selective display
                fprintf('Mouse position x: %d\n',data); % Mouse position x is actualized when the cursor is over CoppeliaSim's window
            end
        end
        t=clock;
        currentTime=t(6);
    end
    disp('Mouse detection ended!');
    
    %%%
    % Now try to retrieve data in a blocking fashion (i.e. a service call):
    [res,objs]=vrep.simxGetObjects(clientID,vrep.sim_handle_all,vrep.simx_opmode_blocking);
    if (res==vrep.simx_return_ok)
        fprintf('Number of objects in the scene: %d\n',length(objs));
    else
        fprintf('Remote API function call returned with error code: %d\n',res);
    end
    
    %%%
    % Now send some data to CoppeliaSim in a non-blocking fashion:
    vrep.simxAddStatusbarMessage(clientID,'Hello CoppeliaSim! This is a msg from MATLAB sent by teng4',vrep.simx_opmode_oneshot);
    
    %%%
    % Before closing the connection to CoppeliaSim, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
    vrep.simxGetPingTime(clientID);    
    % Now close the connection to CoppeliaSim:
    vrep.simxFinish(clientID);    
    disp('end of for loop.');
else
    disp('Failed connecting to remote API server');
end

figure
plot(1:itr,Q_des(:,1),1:itr,Q(:,1))
legend({'q\_des','q'});
grid on;

figure
plot(1:itr,TAU_des(:,1),1:itr,TAU(:,1))
legend({'tau\_des','tau'});
grid on;


%%
vrep.delete(); % call the destructor!

disp('Program ended');



