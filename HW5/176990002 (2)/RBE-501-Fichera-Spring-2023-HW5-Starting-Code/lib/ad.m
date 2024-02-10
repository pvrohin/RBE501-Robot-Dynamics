function adV = ad(V)
    % your code here
    skew_w = skew(V(1:3));
    skew_v = skew(V(4:6));
    O = [0 0 0;0 0 0;0 0 0];
    adV = [skew_w O;skew_v skew_w];
end