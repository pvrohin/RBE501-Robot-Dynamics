function T = fkine_problem2(S,M,q)
    % your code here
    
    % If needed, you can convert twists to homogeneous transformation matrices with:
    T = twist2ht(S(:,1),q(1))*twist2ht(S(:,2),q(2))*twist2ht(S(:,3),q(3))*twist2ht(S(:,4),q(4))*twist2ht(S(:,5),q(5))*twist2ht(S(:,6),q(6))*M;
end