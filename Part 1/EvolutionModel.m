function Xnew=EvaluationModel(Xold,U)
% Evaluation Model of the robot

Xnew=zeros(3,1);
Xnew(1)=Xold(1)+U(1)*cos(Xold(3));
Xnew(2)=Xold(2)+U(1)*sin(Xold(3));
Xnew(3)=Xold(3)+U(2);
end

