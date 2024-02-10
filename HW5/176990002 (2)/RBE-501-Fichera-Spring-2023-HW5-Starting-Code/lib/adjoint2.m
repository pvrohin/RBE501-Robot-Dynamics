function Vtrans = adjoint2(V,T)
    % your code here
    O = [0 0 0;0 0 0;0 0 0];
    p = [T(1,4) T(2,4) T(3,4)];
    R = [T(1,1) T(1,2) T(1,3);T(2,1) T(2,2) T(2,3);T(3,1) T(3,2) T(3,3)];
    bracket_p = [0 -p(3) p(2);p(3) 0 -p(1);-p(2) p(1) 0];
    left_bottom_corner = bracket_p*R;
    Vtrans = [R O;left_bottom_corner R]*V;
end