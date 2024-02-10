function J_b = jacobe(S,M,q)    
    J_s = jacob0(S,q);
    T = fkine(S,M,q,'space');
    Adj = adjointt(T);
    J_b = inv(Adj)*J_s;
end