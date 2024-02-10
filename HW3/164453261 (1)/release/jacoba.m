function J_a = jacoba(S,M,q)    
    % your code here
    J_a = [];
    J = jacob0(S,q);
    T = fkine(S,M,q);
    p = [T(1,4) T(2,4) T(3,4)];
    bracket_p = [0 -p(3) p(2);p(3) 0 -p(1);-p(2) p(1) 0];
    for i = 1:length(q)
        Jw = J(1:3,i);
        Jv = J(4:6,i);
        j = Jv - (bracket_p*Jw);
        J_a = [J_a j];
    end    
end