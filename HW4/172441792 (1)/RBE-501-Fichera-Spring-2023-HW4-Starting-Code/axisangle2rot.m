function R = axisangle2rot(omega,theta)
    % your code here
    w_brackets = [0 -omega(3) omega(2);omega(3) 0 -omega(1);-omega(2) omega(1) 0];
    R = eye(3)+(sin(theta)*w_brackets)+((1-cos(theta))*w_brackets^2);
end