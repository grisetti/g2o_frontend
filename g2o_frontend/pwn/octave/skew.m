# computes the product between a vector and the three derivatives of a rotation matrix w.r.t a quaternion
# dR/dqx * t + dR/dqy *t + dR/dqz *t
# used in the jacobian calculation
# t: the vector
# S: the result
function S = skew(t)
  tx = t(1);
  ty = t(2);
  tz = t(3);
  S = [ 0      (2*tz)   (-2*ty);
       (-2*tz) 0       (2*tx);
       (2*ty)  (-2*tx)   0];
end;
