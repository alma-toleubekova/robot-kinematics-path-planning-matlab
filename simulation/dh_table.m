function T = dh_table(theta, d, a, alpha)
%MODIFIED DH transformation for a single robot link (mm units)

c_t = cos(theta);  s_t = sin(theta);
c_a = cos(alpha);  s_a = sin(alpha);

T = [ c_t,        -s_t,         0,      a;
      s_t*c_a,     c_t*c_a,    -s_a,  -s_a*d;
      s_t*s_a,     c_t*s_a,     c_a,   c_a*d;
      0,           0,           0,      1];
end
