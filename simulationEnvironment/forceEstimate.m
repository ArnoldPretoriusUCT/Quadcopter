function f = forceEstimate(sig,ec,w)

rho = 1.2041;
c = .02;
n = 2;
Cl = 4.8;
% w = 600;
r = 0.103;
% ec = 0;

a1 = 1/6*rho*c*n*Cl*w^2*r^3;
a2 = 1/8*rho*c*n*Cl*w*r^2*ec+1/64/pi*rho*c^2*n^2*Cl^2*w^2*r^2;
a3 = 1/32*rho*c*n*Cl*w*r^2;
a4 = 16/3/pi*c*n*Cl*w^2*r;
a5 = 16*ec^2+8/rho/pi^2/r^2*c*n*Cl*w*ec+1/4/pi^2*c^2*n^2*Cl^2*w^2;

sig = sig/6700;

if(sig>=0)
    f = a1*sig+a2-a3*sqrt( a4*abs(sig)+a5);
else
    f = a1*sig-a2+a3*sqrt( a4*abs(sig)+a5);
end