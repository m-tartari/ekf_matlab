% Set the parameters which have the "Determined by student" comment to tune
% the Kalman filter. Do not modify anything else in this file.

global Pinit Qgamma Qbeta mahaThreshold ;

% Uncertainty on initial position of the robot.

sigmaX     = 2 ;            % Determined by student
sigmaY     = 2 ;            % Determined by student
sigmaTheta = 2 *pi/180;     % Determined by student
sigmaRR    = 2 ;         	% 2mm tollerance from real value
sigmaRL    = sigmaRR ;
Pinit = diag( [ sigmaX^2 sigmaY^2 sigmaTheta^2 ...
                sigmaRR^2 sigmaRL^2 ] ) ;


% Measurement noise.

sigmaXmeasurement = 5.7735 ;  % approximate to gaussian noise using uniform ditribution
                              % 20/sqrt(12), uniform distribution
sigmaYmeasurement = 2.8868 ;  % 10/sqrt(12), uniform distribution
Qgamma = diag( [sigmaXmeasurement^2 sigmaYmeasurement^2] ) ;


% Input noise

sigmaWheels = 0.05 ;   % Determined by student
Qwheels = sigmaWheels^2 * eye(2) ;
Qbeta   = Qwheels; 

% State noise
 
Qalpha = zeros(5) ;   % Determined by student

% Mahalanobis distance threshold

mahaThreshold = chi2inv(0.9,2) ;  % chi2inv(probability (we choosed 0.9),number of dof)
