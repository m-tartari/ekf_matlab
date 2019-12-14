% Set the parameters which have the "Determined by student" comment to tune
% the Kalman filter. Do not modify anything else in this file.

global Pinit Qgamma Qbeta mahaThreshold ;

% Uncertainty on initial position of the robot.

sigmaX     =2 ;   % We estimate 2mm of standard deviation
sigmaY     = 2 ;   % We estimate 2mm of standard deviation
sigmaTheta = 0.0349 ;   % We estimate 2deg of standard deviation
Pinit = diag( [sigmaX^2 sigmaY^2 sigmaTheta^2] ) ;


% Measurement noise.

sigmaXmeasurement = 5.7735 ;  % approximate to gaussian noise using uniform ditribution
                              % 20/sqrt(12), uniform distribution
sigmaYmeasurement = 2.8868 ;  % 10/sqrt(12), uniform distribution
Qgamma = diag( [sigmaXmeasurement^2 sigmaYmeasurement^2] ) ;


% Input noise

sigmaWheels = 0.07; 
Qwheels = sigmaWheels^2 * eye(2) ;
Qbeta   = jointToCartesian * Qwheels * jointToCartesian.' ; 

% State noise
 
Qalpha = zeros(3) ;

% Mahalanobis distance threshold

%mahaThreshold = *** ;  % Determined by student
mahaThreshold = chi2inv(0.9,2) ;  % chi2inv(probability (we choosed 0.9),number of dof)
