function dxdt = ship_responding(t,x)
%TEST_ODE Summary of this function goes here
%   Detailed explanation goes here
%x(1) = r; x(2) = delta;
global ship
T      = ship.T;
k      = ship.k;
n3     = ship.n3;
r      = x(1);
delta  = x(2);
dxdt   = zeros(2,1);
dxdt(1)= (k*delta-n3*r^3-r)/T;
end