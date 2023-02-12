
syms t angle theta(t) phi(t) psi(t) x(t) y(t) z(t) m l g f1 f2 m1 m2 u1 u2
% x, y, z: Position
% psi, theta, phi: yaw, pitch, roll
% p, q, q: Angular velocities in body frame
% g, m, l: gravity, mass, arm-length
% f1, f2: thrusts
% m1, m2: moments, u2=[L(f2-f4), L(f3-f1), gamma(f1-f2)+gamma(f3-f4)]

syms Ixx Iyy Izz

Rx(angle) = [cos(angle), -sin(angle); 
             sin(angle), cos(angle)];

%u1 = f1+f2
%u2 = l*(f1 - f2)
r = [x(t);y(t)]
            
Rab = simplify(Rx(phi(t))); % A -> B (Inertial to Body)
Rba = transpose(Rab); % B -> A
Wab = diff(phi, t) 
#simplify( [0;0;m*g] - Rab*[0;0;f1+f2+f3+f4] )