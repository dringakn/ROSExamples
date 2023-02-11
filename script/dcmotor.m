% Define transfer function of the DC motor
% pkg load control

num = [0.05];
den = [1 0.05];
sys = tf(num, den);

% Define PID controller
Kp = 10;
Ki = 0.5;
Kd = 0.1;
C = pid(Kp, Ki, Kd);

% Combine the DC motor and PID controller
sys_cl = feedback(C * sys, 1);

% Define the input signal (step input)
t = 0:0.01:10;
u = ones(size(t));

% Simulate the closed-loop system
[y,t,x] = lsim(sys_cl, u, t);

% Plot the results
plot(t, y, 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Speed (rad/s)');
