# compute error of the parameters, given a guess X

function er = calibrate_error(x, z)
  X = v2t(x);
  iX = inverse(X);
  zo = z(1:6);
  zs = z(7:12);
  iZO = inverse(v2t(zo));
  ZS = v2t(zs);
  E = iZO * X * ZS * iX;
  er = t2v(E);
end
