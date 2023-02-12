function dydt = inverted_pendulum(t, y)
  dydt = [y(2); -9.8 * sin(y(1)) - 0.5 * y(2)];
endfunction

%[t, y] = ode45(@inverted_pendulum, [0, 10], [0.2, 0]);
%plot(t, y(:,1), '-o');
%xlabel('Time');
%ylabel('Pendulum Angle');
