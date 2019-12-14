% Reserves storage for the data that is stored during execution of the
% localization algorithm.

global indRes indMagnetDetection nbLoops ;
global tRes ;
global Xodo vOdo wOdo Xres sigx sigy sigtheta update ;

% Define the vectors/matrices for storage of results.
% Define vectors to store results for display and analysis

% For odometry and number of magnets detected at each step, we need exactly nbLoops elements.
Xodo = zeros(3,nbLoops) ;
vOdo = zeros(1,nbLoops) ;
wOdo = zeros(1,nbLoops) ;
travDistance = zeros(1,nbLoops) ;
nbMagnetsDetected = zeros(1,nbLoops) ;

% For these we need at least nbLoops elements, more due to updates, exact
% number unknown. Rest will be allocated dynamically.

tRes = zeros(1,nbLoops)     ;
Xres = zeros(3,nbLoops)     ;
sigx = zeros(1,nbLoops)     ;  
sigy = zeros(1,nbLoops)     ;   
sigtheta = zeros(1,nbLoops) ;
update   = zeros(1,nbLoops) ;
indRes = 1 ;
indMagnetDetection = 1 ;