function traj = make_trajectory(type, params)
% YOUR CODE HERE
t = params.t;
dt = params.time_step;
q = params.q;
v = params.v;
% a = params.a

t_0 = t(1);
t_f = t(2);
q_0 = q(1);
q_f = q(2);
v_0 = v(1);
v_f = v(2);

traj.t = t_0 : dt : t_f ;

if isequal(type, "cubic")
    
    A = [1 t_0 (t_0)^2 (t_0)^3;
         0 1   2*(t_0)  3*(t_0)^2;
         1 t_f (t_f)^2 (t_f)^3;
         0 1    2*(t_f)  3*(t_f)^2];
    
    C = [q_0; v_0; q_f; v_f];
    
    coefficients = A\C;
    
    traj.q = coefficients(1) + coefficients(2) .* traj.t + coefficients(3) .* traj.t .^2 + coefficients(4) .* traj.t .^3;
    traj.v = coefficients(2) + 2* coefficients(3) .* traj.t + 3* coefficients(4) .* traj.t .^2;
    traj.a = 2*coefficients(3) + 6*coefficients(4) .* traj.t;


else
   a = params.a;
   a_0 = a(1);
   a_f = a(2);
   
   A = [1 t_0 (t_0)^2 (t_0)^3 (t_0)^4 (t_0)^5;
         0 1   2*(t_0)  3*(t_0)^2 4*(t_0)^3 5*(t_0)^4;
         0 0    2       6*t_0   12*t_0^2    20*t_0^3;
         1 t_f (t_f)^2 (t_f)^3 (t_f)^4 (t_f)^5;
         0 1    2*(t_f)  3*(t_f)^2  4*t_f^3 5*t_f^4; 
         0 0    2       6*t_f   12*t_f^2    20*t_f^3;  
         ];
    
   C = [q_0; v_0; a_0; q_f; v_f; a_f];
    
   coefficients = A\C;
   
   traj.q = coefficients(1) + coefficients(2) .* traj.t + coefficients(3) .* traj.t .^2 + coefficients(4) .* traj.t .^3 + coefficients(5) .* traj.t .^4 + coefficients(6) .* traj.t .^5;
   traj.v = coefficients(2) + 2* coefficients(3) .* traj.t + 3* coefficients(4) .* traj.t .^2 + 4* coefficients(5) .* traj.t .^3 + 5* coefficients(6) .* traj.t .^4;
   traj.a = 2*coefficients(3) + 6*coefficients(4) .* traj.t + 12*coefficients(5) .* traj.t .^2 + 20*coefficients(6) .* traj.t .^3;
    
    
    
end    


end