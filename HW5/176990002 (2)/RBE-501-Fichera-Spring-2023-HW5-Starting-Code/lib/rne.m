function [tau,V,Vdot] = rne(params)
%% RNE Implements the Recursive Newton-Euler Inverse Dynamics Algorithm
%
% Inputs: params - a structure containing the following fields:
%           params.g - 3-dimensional column vector describing the acceleration of gravity
%           params.S - 6xn matrix of screw axes (each column is an axis)
%           params.M - 4x4xn home configuration matrix for each link
%           params.G - 6x6xn spatial inertia matrix for each link
%           params.jointPos - n-dimensional column vector of joint coordinates
%           params.jointVel - n-dimensional column vector of joint velocities
%           params.jointAcc - n-dimensional column vector of joint accelerations
%           params.Ftip - 6-dimensional column vector representing the
%           wrench applied at the tip
%
% Output: tau  - n-dimensional column vector of generalized joint forces
%         V    - 6x(n+1) matrix - each column represents the twist of one of the robot's links
%         Vdot - 6x(n+1) matrix - each column represents the acceleration of one of the robot's links
%
% Forward iterations
% YOUR CODE HERE 

g = params.g;
S = params.S;
M = params.M;
G = params.G;
jointPos = params.jointPos;
jointVel = params.jointVel;
jointAcc = params.jointAcc;
Ftip = params.Ftip;

n = size(S,2);

V = zeros(6,n+1);
Vdot = zeros(6,n+1);

A = zeros(6,n);

Vdot(:,1) = [0 0 0 -g]';

%Vdotzero = [0 0 0 g]';

for ii = 1 : n
    Mi = eye(4);
    
    for jj = 1 : ii
        Mi = Mi * M(:,:,jj);
    end
    
    Ai = adjoint(pinv(Mi)) * S(:,ii);
    
    A(:,ii) = Ai;
    
    Ti = twist2ht(Ai,-jointPos(ii)) * pinv(M(:,:,ii));
    
    T(:,:,ii) = Ti;
    
    V(:,ii+1) =  adjoint(Ti)*V(:,ii) + Ai*jointVel(ii);
    Vdot(:,ii+1) = adjoint(Ti)*Vdot(:,ii) + ad(V(:,ii+1))*Ai*jointVel(ii) + Ai*jointAcc(ii);
        
end

T(:,:,ii+1) = pinv(M(:,:,ii+1));

% Backward iterations
% YOUR CODE HERE
tau = zeros(n,1);

%F = zeros(n,1)

Fi = Ftip;

for ii = n : -1 : 1
    Fi = adjoint(T(:,:,ii+1))'*Fi + G(:,:,ii)*Vdot(:,ii+1) - ad(V(:,ii+1))'*(G(:,:,ii)*V(:,ii+1));
    
    tau(ii,1) = Fi' * A(:,ii);
end

end