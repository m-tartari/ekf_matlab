% ShowOdometry: Plot the odometry-estimated path corresponding to a data
% file. 
% Usage: ShowOdometry( <FileName.ext> )
% Example: ShowOdometry( 'oneloop.txt' ) 
% ShowOdometry automatically searches for the file in folder data/
% Author: G. Garcia
% 13 March 2013.

RobotAndSensorDefinition ;

Xodom = [ 0 0 0*pi/180 ].' ;    % Set this according to robot initial position.
%Load the data file
dataFile = uigetfile('data/*.txt','Select data file') ;
if isunix ,
    eval(['load data/' , dataFile]) ;
else
    eval(['load data\' , dataFile]) ;
end
dataFile = strrep(dataFile, '.txt', '')
eval(['data = ',dataFile,'; clear ',dataFile]) ;
travDistance = 0 ;

% Skip motionless parts of the data at beginning and end of the experiment
% and return only meaningful data, with wheel rotations in radians.
% Also reduce encoder resolution and frequency according to factors
% set in RobotDefinition.m

[nbLoops,t,qL,qR,sensorReadings] = PreprocessData(data) ;
nbLoops

Xodo = zeros(3,nbLoops) ;
vOdo = zeros(1,nbLoops) ;
wOdo = zeros(1,nbLoops) ;

Xodo(:,1) = Xodom ;

wbHandle = waitbar(0,'Computing...') ;

for mainLoopIndex = 2 : nbLoops ,
    
    waitbar(mainLoopIndex/nbLoops) ;

    % Calculate input vector from proprioceptive sensors
    deltaq = [ qR(mainLoopIndex) - qR(mainLoopIndex-1) ; 
               qL(mainLoopIndex) - qL(mainLoopIndex-1) ] ;
    U = jointToCartesian * deltaq ;  % joint speed to Cartesian speed.
    
    % Predic state (here odometry)
    Xodom = EvolutionModel( Xodom , U ) ;
    Xodo(:,mainLoopIndex) = Xodom ;
    vOdo(mainLoopIndex) = U(1)/( t(mainLoopIndex)-t(mainLoopIndex-1) ) ;
    wOdo(mainLoopIndex) = U(2)/( t(mainLoopIndex)-t(mainLoopIndex-1) ) ;
    
end

close(wbHandle) ;

% Plot robot path, Kalman fiter estimation and odometry only estimation

figure; 
plot( Xodo(1,:), Xodo(2,:) , 'r' ) ;
zoom on ; grid on; axis('equal');
title('Path estimated by odometry');
xlabel('x (mm)');
ylabel('y (mm)');

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
