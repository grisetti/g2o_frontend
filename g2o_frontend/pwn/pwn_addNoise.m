function Pn = pwn_addNoise(P, Omega)
  L = chol(inverse(Omega));
  Pn = P;
  for i = [1:size(P)(2)]
    p = Pn(:, i);
    n = sampleGaussian(L);
    p += n;
    Pn(4:6,i) = p(4:6) * (1./norm(p(4:6)));
  endfor
endfunction