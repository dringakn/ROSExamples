function res = Nearest(V, pt)
  min = 1e18;
  for i=1:size(V,1)
    dist = sqrt(sum((V(i,:)-pt).^2));
    if(dist<min)
      min = dist;
      res = V(i,:);
    end
  end
end