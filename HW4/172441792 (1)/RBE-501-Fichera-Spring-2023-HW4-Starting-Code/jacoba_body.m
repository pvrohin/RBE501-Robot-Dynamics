function J_a = jacoba_body(S,M,q)    
    % your code here
    J_a = [];
    J = jacobe_body(S,q);
    T = fkine(S,M,q,"body");
    R=T(1:3,1:3);
    for i = 1:length(q)
        Jw = J(1:3,i);
        Jv = J(4:6,i);
        j = R*Jv;
        J_a = [J_a j];
    end    
end