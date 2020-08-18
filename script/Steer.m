function res = Steer(pt2, pt1, eta)
  if(Nearest(pt2,pt2)<=eta)
    res = pt1;
  else
    delta = pt2 - pt1;
    if (delta(1,1) == 0)
      res(1,1) = pt2(1,1);
      res(1,2) = pt2(1,2) + eta;
    else
      m = delta(1,2)/delta(1,1);
      res(1,1) = sign(delta(1,1)) * sqrt(eta^2/(1+m^2)) + pt2(1,1);
      res(1,2) = m * (res(1,1) - pt2(1,1)) + pt2(1,2);
    end
  end
end