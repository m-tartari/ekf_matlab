% Localization by detection of magnets at known locations in the ground.
% -----
% Usage: 
%    - Set the characteristics of the robot and sensor in 
%      RobotAndSensorDefinition.m
%    - Set noise levels in DefineVariances.m
%    - Set data file name and robot initial position in the present file.
%    - Execute this file.
%    - Then, execute PlotResults.m to get the plots for analysis.
% -----
% Project history:
%    - Project initiator and principle of the sensor: Gaëtan Garcia
%    - Sensor manufacturing and test: Joël Rousseau.
%    - Characterization of the sensor: EMARO1/ARIA1 project, Hendry Chame
% and Marcelo Gaudenzi de Faria. Supervision: Gaëtan Garcia
%    - First implementation (Matlab): EMARO1/ARIA1 project, Filip Cichosz
% and Marteen Samuel. Supervision: Gaëtan Garcia.
%    - This program (using Samuel and Cichosz's work): Gaëtan Garcia

global mainLoopIndex nbLoops ;
global t X P U Y Qbeta Qgamma Xodom totalTravDistance oTm mTo ;
global dMaha
global measures oPest oPmagnet ;
global xSpacing ySpacing ;

RobotAndSensorDefinition ;
DefineVariances ;

X = [ 0 0 0*pi/180 ].' ;    % Set this according to robot initial position.
%Load the data file
dataFile = uigetfile('data/*.txt','Select data file') ;
if isunix ,
    eval(['load data/' , dataFile]) ;
else
    eval(['load data\' , dataFile]) ;
end
dataFile = strrep(dataFile, '.txt', '')
eval(['data = ',dataFile,'; clear ',dataFile]) ;

P = Pinit ; 
Xodom = X ;
totalTravDistance = 0 ;

% Skip motionless parts of the data at beginning and end of the experiment
% and return only meaningful data, with wheel rotations in radians.
% Also reduce encoder resolution and frequency according to factors
% set in RobotDefinition.m. We want want to make the odometry even worse
% than it already is...

[nbLoops,t,qL,qR,sensorReadings] = PreprocessData(data) ;

PrepareVectorsAndMatricesForStorageOfResults ;

wbHandle = waitbar(0,'Computing...') ;

for mainLoopIndex = 2 : nbLoops ,
    
    waitbar(mainLoopIndex/nbLoops) ;

    % Calculate input vector from proprioceptive sensors
    deltaq = [ qR(mainLoopIndex) - qR(mainLoopIndex-1) ; 
               qL(mainLoopIndex) - qL(mainLoopIndex-1) ] ;
    U = jointToCartesian * deltaq ;  % joint speed to Cartesian speed.
    
    % Calculate linear approximation of the system equation
    A=[1,0,-U(1)*sin(X(3));
        0,1,U(1)*cos(X(3));
        0,0,1];
    B=[cos(X(3)),0;
       sin(X(3)),0;
       0,1]; 
        
    % Predic state (here odometry)
    X = EvolutionModel( X , U ) ;
    
    % Error propagation
    P = A*P*(A.') + B*Qbeta*(B.') + Qalpha ;
    
    % Vector of measurements. Size is zero if no magnet was detected.
    measures = ExtractMeasurements( sensorReadings(mainLoopIndex) ) ;
    CalculateAndStoreResultsForAnalysis('prediction');
        
    % When two or more magnets are detected simultaneously, they are taken
    % as independant measurements, for the sake of simplicity.
    
    for measNumber = 1 : numel(measures) ,
        
        % Calculate homogeneous transform of the robot with respect to the world frame
        oTm = [cos(X(3)),-sin(X(3)),X(1);
                sin(X(3)),cos(X(3)),X(2);
                0,0,1] ;
        mTo = inv(oTm) ;
        
        % Measurement vector in homogeneous coordinates
        Y = [ sensorPosAlongXm ; 
              sensorRes*( measures(measNumber) - sensorOffset ) ;                
              1 ] ;
          
        % Now we determine which magnet has been detected it should be the magnet closest 
        % to the estimated world position of the measurement point.
        
        % Y is the measurement point in robot frame. Transfer to world frame.
        oPest = oTm * Y ;
        
        % Which actual magnet is closest to the estimated position?
        oPmagnet = round( oPest ./ [xSpacing ; ySpacing ; 1] ) .* [xSpacing ; ySpacing ; 1] ;

        % The position of the magnet in robot frame is the expected measurement Yhat
        Yhat = mTo * oPmagnet ;
        Pmag= [ cos(X(3))*(oPmagnet(1)-X(1))+sin(X(3))*(oPmagnet(2)-X(2));
                cos(X(3))*(oPmagnet(2)-X(2))-sin(X(3))*(oPmagnet(1)-X(1));
                1];
        C = [ -cos(X(3)),    -sin(X(3)),	-sin(X(3))*(oPmagnet(1)-X(1))+cos(X(3))*(oPmagnet(2)-X(2)) ;
               sin(X(3)),    -cos(X(3)),    -sin(X(3))*(oPmagnet(2)-X(2))-cos(X(3))*(oPmagnet(1)-X(1))];
                      
        innov = Y(1:2) - Yhat(1:2) ;   % Not in homogeneous coordinates.
        dMaha = sqrt( innov.' * inv( C*P*C.' + Qgamma) * innov ) ;
        
        CalculateAndStoreResultsForAnalysis( 'measurement' ) ;
        
        % Now perform update only if the candidate magnet is indeed coherent
        % with the measurement. In "CalculateAndStore..." script, the
        % calculation is also performed for the four nearest neighbors of
        % the candidate magnet. Those should not be coherent with the
        % measurement, otherwise it means we're pretty much lost.
        
        if dMaha <= mahaThreshold ,
            K = P * C.' * inv( C*P*C.' + Qgamma) ;
            X = X + K*innov ;
            P = (eye(numel(X)) - K*C) * P ;
            CalculateAndStoreResultsForAnalysis( 'update' ) ;
        end
        
    end

end

close(wbHandle) ;
