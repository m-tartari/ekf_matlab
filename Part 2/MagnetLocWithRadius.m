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

global period trackGauge rwheel ;
global mainLoopIndex nbLoops ;
global t X P U Uodo Y Qbeta Qgamma Xodom totalTravDistance oTm mTo ;
global dMaha
global measures oPest oPmagnet ;
global xSpacing ySpacing ;
global travDistance nbMagnetsDetected ;

RobotAndSensorDefinition ;
DefineVariances ;

e=trackGauge;  %for later calculations

% Set this according to robot initial position.
% Introduce a reasonable error on the wheel radius.
r_r=rwheel*1.07;                    %added 14% wheel difference
r_l=rwheel*0.93;
X = [ 0 0 0*pi/180 r_r r_l ].' ;
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
Xodom = X(1:3) ;
totalTravDistance = 0 ;

% Skip motionless parts of the data at beginning and end of the experiment
% and return only meaningful data, with wheel rotations in radians.
% Also reduce encoder resolution and frequency according to factors
% set in RobotDefinition.m

[nbLoops,t,qL,qR,sensorReadings] = PreprocessData(data) ;

PrepareVectorsAndMatricesForStorageOfResults ;

wbHandle = waitbar(0,'Computing...') ;

for mainLoopIndex = 2 : nbLoops ,
    
    waitbar(mainLoopIndex/nbLoops) ;

    % Calculate input vector from proprioceptive sensors
    
    % Elementary wheel rotations
    deltaq = [ qR(mainLoopIndex) - qR(mainLoopIndex-1) ; 
               qL(mainLoopIndex) - qL(mainLoopIndex-1) ] ;
    % Elem. transl. and rotation for standard odometry.     
    Uodo = jointToCartesian * deltaq ;  
    
    % Beware! In this context, U=(qdot_r,qdot_l)'. Cannot be (v,w)' 
    % since we want to have the radii in the equations...

    U = deltaq / period ;
    
    v = (1/2)*( X(4)*U(1) + X(5)*U(2) )             ;
    w = X(4)/trackGauge*U(1) - X(5)/trackGauge*U(2) ;

    % Calculate linear approximation of the system equation
    A=[ 1,  0,  -(X(4)*U(1)*period+X(5)*U(2)*period)/2*sin(X(3)),     U(1)/2*cos(X(3))*period,       U(2)/2*cos(X(3))*period ;
        0,  1,   (X(4)*U(1)*period+X(5)*U(2)*period)/2*cos(X(3)),     U(1)/2*sin(X(3))*period,       U(2)/2*sin(X(3))*period ;
        0,  0,                                                 1,               U(1)/e*period,                -U(2)/e*period ;
        0,  0,                                                 0,                           1,                             0 ;
        0,  0,                                                 0,                           0,                             1];
    
    B=period*[ X(4)/2*cos(X(3)),   X(5)/2*cos(X(3)) ;
               X(4)/2*sin(X(3)),   X(5)/2*sin(X(3)) ;
                          X(4)/e,            -X(5)/e ;
                               0,                  0 ; 
                               0,                  0]; 
   
    % Predic state (here odometry)
    X = EvolutionModel( X , U ) ;
    
    % Error propagation
    P = A*P*(A.') + B*Qbeta*(B.') + Qalpha %; for printing
    
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
        
        % Measurement vector iX(1:2)n homogeneous coordinates
        Y = [ sensorPosAlongXm ; 
              sensorRes*( measures(measNumber) - sensorOffset ) ;                
              1 ] ;
                
        % Y is the measurement point in robot frame. Transfer to world
        % frame
        oPest = oTm * Y ;
        
        % Which actual magnet is closest to the estimated position?
        oPmagnet = round( oPest ./ [xSpacing ; ySpacing ; 1] ) .* [xSpacing ; ySpacing ; 1] ;

        % The position of the magnet in robot frame is the expected measurement Yhat
        Yhat = mTo * oPmagnet ;
        
        C = [ -cos(X(3)),    -sin(X(3)),	-sin(X(3))*(oPmagnet(1)-X(1))+cos(X(3))*(oPmagnet(2)-X(2)), 0,  0 ;
               sin(X(3)),    -cos(X(3)),    -sin(X(3))*(oPmagnet(2)-X(2))-cos(X(3))*(oPmagnet(1)-X(1)), 0,  0];
                      
        innov = Y(1:2) - Yhat(1:2) ;   % Not in homogeneous coordinates.
        dMaha = sqrt( innov.' * inv( C*P*C.' + Qgamma) * innov ) ;
        
        CalculateAndStoreResultsForAnalysis( 'measurement' ) ;
        
        if dMaha <= mahaThreshold ,
            K = P * C.' * inv( C*P*C.' + Qgamma) ;
            X = X + K*innov ;
            P = (eye(numel(X)) - K*C) * P,; 
            CalculateAndStoreResultsForAnalysis( 'update' ) ;
        end
        
    end

end

close(wbHandle) ;
