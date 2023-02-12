clc
clear all
syms t theta(t) phi(t) psi(t) x(t) y(t) z(t) 
syms m l g p(t) q(t) r(t) 
syms f1 f2 f3 f4 u1 u2 u3 u4
syms Ixx Iyy Izz Ixy Ixz Iyz

% x, y, z: Position
% psi, theta, phi: yaw, pitch, roll
% p, q, q: Angular velocities in body frame
% g, m, l: gravity, mass, arm-length
% f1, f2, f3, f4: thrusts, u1=f1+f2+f3+f4
% u1 = f1 + f2 + f3 + f4 -> Thrust
% u2 = f4 - f2 -> Roll
% u3 = f1 - f3 -> Pitch
% u4 = f1 - f2 + f3 - f4 -> Yaw
% Ixx Iyy Izz Ixy Ixz Iyz: Inertia tensor
% Wab = R(phi)R(theta)[0;0;dpsi] + R(phi)[0;dtheta;0]+[dphi;0;0]
# Wba = simplify(inv(Wab))
% A -> B (Inertial to Body)
% B -> A (Body to Inertial)

Rx = @(theta) [sym(1), 0, 0; 
               0, cos(theta), -sin(theta); 
               0, sin(theta), cos(theta)];
Ry = @(theta) [cos(theta), 0, sin(theta); 
               0, sym(1), 0; 
              -sin(theta), 0, cos(theta)];
Rz = @(theta) [cos(theta), -sin(theta), 0;
               sin(theta), cos(theta), 0;
               0, 0, sym(1)]; 
               
skew = @(v) [0, -v(3), v(2);v(3), 0, -v(1);-v(2), v(1), 0];

             
Rab = simplify(Rz(psi)*Rx(phi)*Ry(theta));
Rba = transpose(Rab); % B -> A

% Body rates to/from Euler rates (Inertial)
Wab = [1, 0, -sin(theta);0, cos(phi), cos(theta)*sin(phi);0, -sin(phi), cos(theta)*cos(phi)];
Wba = [1, sin(phi)*tan(theta), cos(phi)*tan(theta);0, cos(phi), -sin(phi);0 sin(phi)/cos(theta), cos(phi)/cos(theta)];

%I = [Ixx, Ixy, Ixz;Ixy, Iyy, Iyz;Ixz, Iyz, Izz];
I = [Ixx, 0, 0;0, Iyy, 0;0, 0, Izz]; % Symmetric body

pos = [x;y;z]; % Position vector
vel = diff(pos, t); % Linear velocity vector
acc = diff(vel, t);  % Linear acceleratoin vector
euler = [phi;theta;psi]; % Euler angles
omega = diff(euler, t); # Angular velocity vector
alpha = diff(omega,t); # Angular acceleratoin vector

syms K_t K_d
syms omega_1 omega_2 omega_3 omega_4
syms tau_phi tau_theta tau_psi

% Translational Dynamics:
f_g = [0;0;m*g]; % Force of gravity in global frame
f_d = eye(3) * K_d * vel; % Drag force in global frame
f_t = Rab * [0;0;u1]; % Thrust force in global frame
simplify( (f_g - f_d - f_t)/m )

% Rotational Dynamics
% Motor torques: tau_m = [tau_phi;tau_theta;tau_psi]
tau_m = [l*K_t*(omega_4^2-omega_2^2);
         l*K_t*(omega_1^2-omega_3^2);
         K_d*(omega_1^2-omega_2^2+omega_3^2-omega_4^2)];

% Gyroscopic torque due to quad rotation
tau_g = simplify(skew(omega)*I*omega);

% Gyroscopic torque due to motors rotation, Ir=Rotor moment of inertia
syms Ir
tau_gm = [omega(2)*Ir*(omega_1-omega_2+omega_3-omega_4);
         -omega(1)*Ir*(omega_1-omega_2+omega_3-omega_4);
         0];

simplify( inv(I) * (tau_m - tau_gm - tau_gm) )

% subs(res,[sin(phi), cos(phi), sin(theta), cos(theta), sin(psi), cos(psi)],[phi, 1, theta, 1, psi, 1])

% Model2:
simplify((-f_g + Rab*[0;0;u1]) / m)
simplify(inv(I) * ([u2;u3;u4] - skew([p;q;r])*I*[p;q;r]) )
Wab2 = [cos(theta), 0, -cos(phi)*sin(theta);0, 1, sin(phi);sin(theta), 0, cos(phi)*cos(theta)];