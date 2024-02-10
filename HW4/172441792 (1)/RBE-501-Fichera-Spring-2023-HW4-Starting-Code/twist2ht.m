function T = twist2ht(S,theta)
    % your code here
    v = [S(4) S(5) S(6)]';
    w = [S(1) S(2) S(3)]';
    w_brackets = [0 -w(3) w(2);w(3) 0 -w(1);-w(2) w(1) 0];
    %w_brackets_square = [0 -w(3)*w(3) w(2)*w(2);w(3)*w(3) 0 -w(1)*w(1);-w(2)*w(2) w(1)*w(1) 0]
    p = ((eye(3)*theta)+((1-cos(theta))*w_brackets)+((theta-sin(theta))*w_brackets^2))*v;
    R = axisangle2rot(w,theta);
%     if w==[0;0;0]
%         R=eye(3);
%     end
    % If needed, you can calculate a rotation matrix with:
    T = [R p;0 0 0 1];
end