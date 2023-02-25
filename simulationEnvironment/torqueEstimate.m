function tau = torqueEstimate(sig,ec,w)



% sig = sig/6700;

a0 = 5.8e-3;
a1 = 1.376e-7;
a2 = 1.237e-9;
a3 = 1.937e-12;

tau = abs( a0+a1*sig+a2*sig^2+a3*sig^3 );