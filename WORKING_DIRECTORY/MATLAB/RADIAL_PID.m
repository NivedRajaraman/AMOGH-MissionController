time = 0;
timeResolution = 0.1;

KiVy = 0.2;
KpVy = 2;
KiW = 0.1;
KpW = 1;
KiVx = 0.2;
KpVx = 2;

target = [1,2,2]; % x,y,z
camera = target./2;
C = camera;
currentPosition = [0,0,0];
currentDirection = [1,0,0];


IntY = 0;
IntX = 0;
IntX_1 = 0;
for time = (0:1:100)
    Zerror = camera(3);
    Yerror = camera(2);
    Z_1error = 0.1/(1+Zerror^2);
    IntX = IntX + Zerror*timeResolution;
    IntY = IntY + Yerror*timeResolution;
    IntX_1 = IntX_1 + Z_1error*timeResolution;

    Vy = KpVy*Yerror ; %+ KiVy*IntX_1;   % upward velocity
    W = KpW*Zerror ; %+ KiW*IntX;          % omega
    Vx = KpVx*Z_1error ; %+ KiVx*IntX_1;   % radial velocity
    
    o = W*timeResolution;
    
    R = [[cos(o), 0, sin(o)];[0, 1, 0];[-sin(o), 0, cos(o)]];
    
    tcow = [target(1)-Vx*timeResolution,target(2)-Vy*timeResolution,target(3)];
    target = tcow;
    target = target*R';
    
    if ((camera(3))^2 + (camera(2))^2 > 0.0001)
        ResPt = camera;
        T = time;
    end
    
    camera = target./2;
    C = [C;camera];
end

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
