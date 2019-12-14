% Plot the results and analysis data after execution of the Kalman filter
% localization algorithm implemented in MagnetLoc.m

global samplingFrequency ;
global t tRes tMagnetDetection  X Xodom ;
global Xodo vOdo wOdo travDistance totalTravDistance Xres sigx sigy sigtheta ;
global update dMahaAll nbMagnetsDetected ;
global estMagnetPos exactMagnetPos ;

% Script for plotting results for analysis

% Plot robot path, Kalman fiter estimation and odometry only estimation

figure; 
plot( Xres(1,:), Xres(2,:) , 'b' , 'LineWidth', 2 ) ;
hold on ;
plot( Xodo(1,:), Xodo(2,:) , 'r' ) ;
zoom on ; grid on; axis('equal');
title('Estimated path EKF (blue) and odometry (red)');
xlabel('x (mm)');
ylabel('y (mm)');

% On top of the path, indicate estimated and real magnet positions.

hold on;
plot( estMagnetPos(1,:), estMagnetPos(2,:) , 'g+' ) ;
hold on;
plot( exactMagnetPos(1,:), exactMagnetPos(2,:) , 'k.' ) ;

% Plot odometry-estimated speed and rotation speed

figure; 
subplot(2,1,1);
plot( t,vOdo );
xlabel('t (s)')
ylabel('v (mm/s)');
title('Odometry-estimated speed');
zoom on ; grid on;
subplot(2,1,2);
plot( t,wOdo*180/pi );
xlabel('t (s)')
ylabel('w (deg/s)');
title('Odometry-estimated rotation speed');
zoom on ; grid on;

% Plot estimated variances

figure;
subplot(3,1,1);
maximum = max(sigx) ;
for k=1:numel(update) ,
    if update(k)==1 ,
       line([tRes(k) tRes(k)],[0 maximum],'Color','g','LineStyle',':');      
    end
end
hold on ;
plot( tRes,sigx );
xlabel('t (s)')
ylabel('sigma_x (mm)');
title('Estimated variances');
zoom on ; grid on;
subplot(3,1,2);
maximum = max(sigy) ;
for k=1:numel(update) ,
    if update(k) ,
       line([tRes(k) tRes(k)],[0 maximum],'Color','g','LineStyle',':');      
    end
end
hold on ;
plot( tRes,sigy );
xlabel('t (s)')
ylabel('sigma_y (mm)');
zoom on ; grid on;
subplot(3,1,3);
maximum = max(sigtheta) ;
for k=1:numel(update) ,
    if update(k) ,
       line([tRes(k) tRes(k)],[0 maximum],'Color','g','LineStyle',':');      
    end
end
hold on ;
plot( tRes,sigtheta*180/pi );
xlabel('t (s)')
ylabel('sigma_{theta} (deg.)');
zoom on ; grid on;

% Plot Mahalanobis distances. Green dots are for closest magnet,
% red dots are for neighbor magnets.

figure; 
plot( tMagnetDetection , dMahaAll(1,:) , 'g.' ) ;
for k = 2:5 ,
    hold on; 
    plot( tMagnetDetection , dMahaAll(k,:) , 'r.' ) ;
end
hold on;
plot( tMagnetDetection , mahaThreshold*ones(1,size(dMahaAll,2)) , 'b' ) ; 
xlabel('t (s)');
ylabel('Mahalanobis distance (no dimension).');
title('Mahalanobis distances: closest magnet (green) and neighbors (red)');
zoom on; grid on; 

% Plot x, y and theta as functions of time

figure; 
subplot(3,1,1);
plot( tRes,Xres(1,:) );
xlabel('t (s)')
ylabel('x (mm)');
title('Position and heading as functions of time.');
zoom on ; grid on;
subplot(3,1,2);
plot( tRes,Xres(2,:) );
xlabel('t (s)')
ylabel('y (mm)');
zoom on ; grid on;
subplot(3,1,3);
plot( tRes,Xres(3,:)*180/pi );
xlabel('t (s)')
ylabel('theta (deg.)');
zoom on ; grid on;

% Plot number of measurements at each time step.

figure; 
plot(travDistance,nbMagnetsDetected,'.') ;
xlabel('Travelled distance (mm)');
title('Number of magnets detected at each step.');
zoom on; grid on;

% Calculate odometry error:

disp('Travelled distance: '); totalTravDistance
finalOdometryErrorInPercent = (norm(X(1:2)-Xodom(1:2)) / totalTravDistance )*100 

% Calculate percentage of rejected closest magnets:

disp('Percentage of closest magnets rejected:');
100*numel(find(dMahaAll(1,:) > mahaThreshold ))/numel(dMahaAll(1,:))

disp('Percentage of neighbor magnets under threshold:');
100*numel(find(dMahaAll(2:5,:) <= mahaThreshold ))/numel(dMahaAll(2:5,:))

% Plot raw sensor measurements

rawMeas = zeros( nbReedSensors , numel(sensorReadings) ) ;
for k = 1 : numel(sensorReadings) ,
    rawMeas( : , k ) = bitget( sensorReadings(k) , 1:8 ) ;
end
figure; 
for n = 1 : nbReedSensors ,
    for k = 1 : numel(sensorReadings) ,
        if rawMeas(n,k) == 0 ;
            hold on ;
            line([travDistance(k) travDistance(k)],[n-0.5 n+0.5],'Color','b','LineStyle','-');
        end
    end
end
xlabel('Travelled distance (mm)');
ylabel('State of Reed sensors');
grid on ; zoom on;