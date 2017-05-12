
% refer to the image attached in the folder for clarity

time = 0;
timeResolution = 0.1;

% Arbitrarily chosen PID parameters: Kp, Ki, Kd
% for velocity and angle modification
KiVy = 0.2;
KpVy = 2;
KiW = 0.1;
KpW = 1;
KiVx = 0.2;
KpVx = 2;

% Origin -> AUV position ; target -> target position
target = [1,2,2]; % x,y,z

% Scaling factor (of 2) to translate real distances to camera distances
camera = target./2;

% Vector that stores what the camera sees at every resolution point
C = camera;

% Vectors for the AUV's starting position and orientation
currentPosition = [0,0,0];
currentDirection = [1,0,0];

% Integral error (for Ki). The "translation function" is 0.1/(1+Zerror^2)
IntY = 0;
IntX = 0;
IntX_1 = 0;

% Global (x,y,z) corresponds to the camera's (Z,Y,X) axes

for time = (0:1:100)
    Zerror = camera(3);     % corresponds to X error of camera
    Yerror = camera(2);     % corresponds to Y error of camera
    Z_1error = 0.1/(1+Zerror^2);    % corresponds to Z error (into camera)
    
    % modify Integral error at every timestep
    IntX = IntX + Zerror*timeResolution;
    IntY = IntY + Yerror*timeResolution;
    IntX_1 = IntX_1 + Z_1error*timeResolution;

    % V = Kp*d is implicity an integral controller (as shown in the image 
    % pinned in the folder)
    
    Vy = KpVy*Yerror ; %+ KiVy*IntX_1;      % upward velocity
    W = KpW*Zerror ; %+ KiW*IntX;           % omega
    Vx = KpVx*Z_1error ; %+ KiVx*IntX_1;    % radial velocity
    
    % o -> angle of the camera (orientation of the AUV)
    o = W*timeResolution;
    
    % matrix to convert the local X,Y,Z velocities to motion in the global axes
    R = [[cos(o), 0, sin(o)];[0, 1, 0];[-sin(o), 0, cos(o)]];
    
    % Compute translations in the X,Y and Z coordinates
    tcow = [target(1)-Vx*timeResolution,target(2)-Vy*timeResolution,target(3)];
    target = tcow;
    
    % Convert these translations into the global coordinates
    target = target*R';
    
    % The camera sees a small error, 
    if ((camera(3))^2 + (camera(2))^2 > 0.0001)
        ResPt = camera;
        T = time;
    else
        break;
    end
    
    % complete the loop by changing the camera's view
    camera = target./2;
    C = [C;camera];
end

% plotting the camera's view
figure
scatter3(C(:,1),C(:,2),C(:,3),1);
hold on;
scatter3(0.5,1,1, 'b');
scatter3(ResPt(1),ResPt(2),ResPt(3), 'r');
hold off;
legend('Radial Error is less than 0.1 units :: @5.3 second mark','Location','Northeast');
title('Performance of simultaneous radial & directional PID (10s, 10Hz update rate)');
xlabel('X axis');
ylabel('Y axis');
zlabel('Z axis');
set(gca,'XLim',[0 2],'YLim',[0 2],'ZLim',[0 2]);
