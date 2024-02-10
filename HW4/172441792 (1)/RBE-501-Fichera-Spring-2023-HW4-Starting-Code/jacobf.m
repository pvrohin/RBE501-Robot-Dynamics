function J = jacobf(S,q)
    % your code here
    %j = zeros(length(S(:,1)),length(q));
    J = [S(:,1)];
    t = eye(4);
    for i = 2:length(q)
        % If necessary, you calculate the homogeneous transformation associated with a twist V and displacement omega with:
        t = t*twist2ht(S(:,i-1),q(i-1));
    
        % You can also calculate the adjoint transformation of a twist V w.r.t. a homogeneous transformation matrix T with:
        m = adjoint(S(:,i),t);
        
        J = [J,m];
        
    end  
end