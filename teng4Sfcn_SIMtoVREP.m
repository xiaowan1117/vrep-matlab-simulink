
function [sys,x0,str,ts] = spacemodel(t,x,u,flag) %x=[q1 dq1 q2 dq2];u=tol

switch flag,
    case 0,
        [sys,x0,str,ts]=mdlInitializeSizes;
    case 3,
        sys=mdlOutputs(t,u);
    case {2,4,9}
        sys=[];
    otherwise
        error(['Unhandled flag = ',num2str(flag)]);
end

function [sys,x0,str,ts]=mdlInitializeSizes
global vrep clientID loopt1
sizes = simsizes;
sizes.NumOutputs     = 7; %output is tol=[tau1 tau2].
sizes.NumInputs      = 7;  %q=[q1 q2 q3 q4 q5 q6 q7]
sizes.DirFeedthrough = 1;
sizes.NumSampleTimes = 1;
sys = simsizes(sizes);
% %about ts
% %The valid sample time pairs for a Level-2 MATLAB S-function are
% [0 offset]                            % Continuous sample time
% [discrete_sample_time_period, offset] % Discrete sample time
% [-1, 0]                               % Inherited sample time
% [-2, 0]                               % Variable sample time
% %where variable names in italics indicate that a real value is required.
% %When using a continuous sample time,
% %an offset of 1 indicates the output is fixed in minor integration time steps.
% %An offset of 0 indicates the output changes at every minor integration time step.
x0  = [];
str = [];
%ts  = [0 0];
ts  = [0 1];

vrep=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
vrep.simxFinish(-1); % just in case, close all opened connections
clientID=vrep.simxStart('127.0.0.1',19999,true,true,5000,5); %1999 need to be added into scene script.
loopt1=0;



function sys=mdlOutputs(t,u) %u=[q1_d q2_d q1 q2 dq1 dq2]; u is input; output is tol.;x=[q1 q2 dq1 dq2];
global vrep clientID loopt1

if loopt1==0
    if (clientID>-1)
        disp('Connected to remote API server');
    else
        disp('Failed connecting to remote API server');
    end
    %vrep.delete(); % call the destructor!
    loopt1 = loopt1+1;
end

%get q_des from input.
q1=u(1);
q2=u(2);
q3=u(3);
q4=u(4);
q5=u(5);
q6=u(6);
q7=u(7);
q=[q1;q2;q3;q4;q5;q6;q7];
joint_pos1 = q;

if (clientID>-1)   
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
    
    for i=1:7
        %simxSetJointTargetPosition(number clientID,number jointHandle,number targetPosition,number operationMode)
        vrep.simxSetJointTargetPosition(clientID, h(i), joint_pos1(i), vrep.simx_opmode_streaming); %simx_opmode_oneshot or simx_opmode_streaming
    end
    
else
    disp('Failed connecting to remote API server');
end


%disp('loopt ended');

sys(1)=q(1); %S-file output1
sys(2)=q(2); %S-file output2
sys(3)=q(3);
sys(4)=q(4);
sys(5)=q(5);
sys(6)=q(6);
sys(7)=q(7);


