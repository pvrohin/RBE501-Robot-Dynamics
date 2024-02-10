function J_v = jacobe_body(S,q)    
% Your code here
t=eye(4);
length = size(S,2);
J_v(:,length)=S(:,length);
for i=length-1:-1:1 
        t=twist2ht(S(:,i+1),q(i+1))*t;
        C_inv=pinv(t);
        D=adjoint(S(:,i),C_inv);
        J_v(:,i)=D;
end
end
