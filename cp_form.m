function V = cp_form(v)
% cross product form of a 3-dimensional vector
V = [0 -v(3) v(2);
     v(3) 0 -v(1);
    -v(2) v(1) 0 ];