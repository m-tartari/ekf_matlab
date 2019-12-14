% Implements the evolution model of the system. Here, this model is simply
% the equations of odometry.

function Xnew = EvolutionModel( Xold , U )

% X=(x,y,theta,rR,rL)  U=(qRdot,qLdot)

global trackGauge period ;
dD=(Xold(4)*U(1)+Xold(5)*U(2))/2*period;
dT=(Xold(4)*U(1)-Xold(5)*U(2))/trackGauge*period;

Xnew=zeros(5,1);
Xnew(1)=Xold(1)+dD*cos(Xold(3));
Xnew(2)=Xold(2)+dD*sin(Xold(3));
Xnew(3)=Xold(3)+dT;
Xnew(4)=Xold(4);    
Xnew(5)=Xold(5);
          
return          
