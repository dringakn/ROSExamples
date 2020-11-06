function res = testSteer(pt2, pt1, rez)
 % Example:  testSteer([1,1],[5,5],0.1)
 eta = rez * 0.2;
 n = ceil(norm(pt2-pt1)/eta);
 res = zeros(n, 2);
 for i = 1:n 
  res(i,:) = Steer(pt2, pt1, eta);
  pt2 = res(i,:);
 end
   close all;
 hold on;
 plot(pt2(1,1), pt2(1,2), 'kx',"markersize", 20);
 plot(pt1(1,1), pt1(1,2), 'k*',"markersize", 20);
 plot(res(:,1),res(:,2), 'r.',"markersize", 20);
 hold off;
 grid on;
 axis equal;
end