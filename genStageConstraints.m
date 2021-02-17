function [Dt,Et,bt] = genStageConstraints(A,B,D,cl,ch,ul,uh)
    sU=size(ul,1);
    sA=size(A,2);
    sB=size(B,2);
    pD=D*A;
    Dt=[pD;-pD;zeros(sU,sA);zeros(sU,sA)];
    pE=D*B;
    Et=[pE;-pE;eye(sU);-eye(sU)];
    bt=[ch;-cl;uh;-ul];
end

