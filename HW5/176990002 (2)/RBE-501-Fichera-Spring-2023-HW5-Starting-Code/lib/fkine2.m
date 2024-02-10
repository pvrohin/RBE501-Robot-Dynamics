function T = fkine2(S,M,q)
   % your code here
    t = eye(4);
    % If needed, you can convert twists to homogeneous transformation matrices with:
    for i = 1:length(q)
         t = t*twist2ht(S(:,i),q(i));
    end  
    T = t*M;
end