# computes the product between a vector and the three derivatives of a rotation matrix w.r.t a quaternion
# dR/dqx * t + dR/dqy *t + dR/dqz *t
# used in the jacobian calculation
# t: the vector
# S: the result
function S = skew(t)
  tx = t(1);
  ty = t(2);
  tz = t(3);
  S = [ 0,      -tz,   ty;
        tz,     0,     -tx;
        -ty     tx,   0];
end;
