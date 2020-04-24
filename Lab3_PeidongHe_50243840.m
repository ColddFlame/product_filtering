% This is a simple script to manipualte the objects in VREP
%
% Make sure to have the server side running in V-REP: 
% in a child script of the V-REP scene (Initialization part), add the 
% following command to be executed just once, at simulation start:
%
% simExtRemoteApiStart(19999)
%
%
% Hint:
% To get the output of the vision sensor, you just need to
% - first retrieve the handle of the vision sensor using "simxGetObjectHandle"
% - then use this handle as one of the input paramters of "simxGetVisionSensorImage2"
%   to recieve the matrix of rgb image inside the while loop and and
%   finally draw the image using "imshow"
% Please refer to "Matlab remote API functions.pdf" on ublearns for list of
% V-REP functions in matlab
% 

clc; clear all; close all;

%% Introduce the path of V-REP APIs to matlab
 addpath('C:\Program Files\V-REP3\V-REP_PRO_EDU\programming\remoteApiBindings\matlab\matlab');
 addpath('C:\Program Files\V-REP3\V-REP_PRO_EDU\programming\remoteApiBindings\lib');
 addpath('C:\Program Files\V-REP3\V-REP_PRO_EDU\programming\remoteApiBindings\lib\lib\Windows\64Bit');


%% Begin the program  by connecting to V-REP using remote api and clientID
vrep = remApi('remoteApi'); % get the main programming object to work with V-REP
disp('Program started')

vrep.simxFinish(-1); % close any previous connection just in case

% connect to V-REP running on the local machine with IP (='127.0.0.1') on the port (=19999)
clientID = vrep.simxStart('127.0.0.1',19997,true,false,10000,5); 


%% Now get handles for Cylinder or any objects 
[res1A,Sensor_A] = vrep.simxGetObjectHandle(clientID,'Proximity_sensor1',vrep.simx_opmode_oneshot_wait);
[res1B,Sensor_B] = vrep.simxGetObjectHandle(clientID,'Proximity_sensor2',vrep.simx_opmode_oneshot_wait);
[res1C,Joint_A] = vrep.simxGetObjectHandle(clientID,'Prismatic_joint',vrep.simx_opmode_oneshot_wait);
%simxGetVisionSensorImage
[resl4,Vision_sensorA] = vrep.simxGetObjectHandle(clientID,'Vision_sensor',vrep.simx_opmode_oneshot_wait);
%% Initialize parameters 
t = 0;
tic 

state1 = 0;
%% Main loop of the program for real-time interactions between Matlab and V-REP
if clientID > -1 % check for proper connection
    format
    pvstate2 = [];
    pvVal1 = [];
    pvstate1 = [];
    flag = [];
    s2 = 0;
    v11 = 0;
    v12 = 0;
    s11 = 0;
    s12 = 0;    
    while t < 100
        t = toc;        
        [resPx1, state1, sensorVal1]= vrep.simxReadProximitySensor(clientID,Sensor_A,vrep.simx_opmode_oneshot_wait);
        [resPx2, state2, sensorVal2]= vrep.simxReadProximitySensor(clientID,Sensor_B,vrep.simx_opmode_oneshot_wait);
        [returncode,resolution,image] = vrep.simxGetVisionSensorImage2(clientID,Vision_sensorA,0,vrep.simx_opmode_oneshot_wait);
        imshow(image);
        cs = [];
       
        color = 0;
        I = rgb2gray(image);
        Mask = 0*I;        Mask=Mask(:);
        x = 1:size(I,2);
        y = 1:size(I,1);
        [xx,yy]=meshgrid(x,y);
        xx = xx(:);                 yy = yy(:);
        cc = 0;
        if state1 == 1
            for j = 1:size(image,3)
                cs(j) = sum(sum(image(:,:,j)));
            end   
            [~,cjudge] = max(cs);
            if cjudge == 1
                color = 1;
            end
            
            
            conn = bwconncomp(I,4);
            LObj = zeros(1,conn.NumObjects);
            curid = conn.PixelIdxList{1};
            Mask(curid)=1;
            xObj = xx(curid);
            yObj = yy(curid);
            xcObj = mean(xObj);
            ycObj = mean(yObj);
            dx = max(xObj)-min(xObj);
            dy = max(yObj)-min(yObj);
            r = (dx + dy)/4;
            criterion = (xObj-xcObj).^2+(yObj-ycObj).^2 < r^2;
            cc = sum(criterion)/length(criterion);
        end
        cuboid = 0;
        if cc < 0.90
            cuboid = 1;
        end
        rc = 0;
        if cuboid && color
            rc = 1;
        elseif ~cuboid && color
            pause(1);
        end
%         pvVal1 = [pvVal1;sensorVal1];
%         pvstate1 = [pvstate1,state1];
%         pvstate2 = [pvstate2,state2];
%         if size(pvVal1,1) > 2 
%             v11 = pvVal1(end,3) - pvVal1(end-1,3);
%             v12 = pvVal1(end-1,3) - pvVal1(end-2,3);
%         end   
%         if length(pvstate1) > 2
%             s12 = pvstate1(end) - pvstate1(end-1);
%             s11 = pvstate1(end) - pvstate1(end-2);
%         end        
%         if size(pvstate2,2) > 2
%             s2 = pvstate2(end) - pvstate2(end-1);
%         end         
%     if size(pvVal1,1)>2
%         if v12 > pvVal1(end - 2,3)*10^(30) && v11 <0 && s11 > 0
%             flag = [flag,1];            
%         elseif s11 > 0 && s12 == 0
%             flag = [flag,0];            
%         end        
%             if state2 == 1 && flag(1) == 0 && s2 > 0      
    if rc == 1 && state1 == 1
        [resPs1]= vrep.simxSetJointPosition(clientID,Joint_A,-1,vrep.simx_opmode_oneshot_wait);
        [resPs1]= vrep.simxSetJointPosition(clientID,Joint_A,0,vrep.simx_opmode_oneshot_wait);
%         flag(1) = []; 
%         elseif state2 == 1 && flag(1) == 1 && s2 > 0
%         flag(1) = [];         
%             end               
%     end
%     end
    end
end
end

%% Close the connection with V-REP
vrep.simxFinish(clientID); % disconnect from the V-REP software
vrep.delete(); % call the destructor to delete vrep object!