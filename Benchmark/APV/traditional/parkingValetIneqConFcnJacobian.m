function [G,Gmv,Ge] = parkingIneqConFcnJacobian(X,U,e,data,ref,Qp,Rp,Qt,Rt,distToCenter,safetyDistance)
%#codegen
% Jacobian of inequality constraints for parking.

% Approximate Jacobian based on http://rll.berkeley.edu/~sachin/papers/Schulman-IJRR2014.pdf

% Copyright 2019 The MathWorks, Inc.

%% Ego and obstacles
persistent obstacles ego

if isempty(obstacles)
    %% Ego car
    vdims = vehicleDimensions;
    egoLength = vdims.Length;
    egoWidth = vdims.Width;
    ego = collisionBox(egoLength,egoWidth,0);
    
    %% Obstacles (occupied parking lots)
    
    % parked cars in the bottom of garage
    obs1 = collisionBox(2.9,4.8,0);
    T1 = trvec2tform([23,3.55, 0]);
    obs1.Pose = T1;
    
    obs2 = collisionBox(2.9,4.8,0);
    T2 = trvec2tform([31.5,3.55, 0]);
    obs2.Pose = T2;
    
    obs3 = collisionBox(2.9,4.8,0);
    T3 = trvec2tform([40.5,3.55, 0]);
    obs3.Pose = T3;
    
    % parked cars in the top of garage
    obs4 = collisionBox(2.9,4.8,0);
    T4 = trvec2tform([23,46, 0]);
    obs4.Pose = T4;
    
    obs5 = collisionBox(2.6,4.8,0);
    T5 = trvec2tform([27.3,46.5, 0]);
    obs5.Pose = T5;
    
    obs6 = collisionBox(2.9,4.8,0);
    T6 = trvec2tform([53.5,46, 0]);
    obs6.Pose = T6;
    
    % garage border on the left
    obs7 = collisionBox(0.5,50,0);
    T7 = trvec2tform([0.25,25, 0]);
    obs7.Pose = T7;
    
    % parked area in the middle of garage
    obs8 = collisionBox(48,13,0);
    T8 = trvec2tform([35.7,25, 0]);
    obs8.Pose = T8;
    
    obstacles = {obs1,obs2,obs3,obs4,obs5,obs6,obs7,obs8};
end


%% constraints
p = data.PredictionHorizon;
numObstacles = numel(obstacles);
Nc = numObstacles * p;
Nmv = length(data.MVIndex);
Gmv = zeros(p,Nmv,Nc);
Ge = zeros(Nc,1);
G = zeros(p,data.NumOfStates,Nc);

iter = 1;
for i =1:p
    for ct = 1:numObstacles
        % Update ego pose
        x = X(i,1) + distToCenter*cos(X(i,3));
        y = X(i,2) + distToCenter*sin(X(i,3));
        T = trvec2tform([x,y,0]);
        H = axang2tform([0 0 1 X(i,3)]);
        ego.Pose = T*H;
        % Calculate distances from ego to obstacls
        [~, dist, points] = checkCollision(ego,obstacles{ct});
        normal = [0;0];
        if ~isnan(dist)
            if any((points(1:2,1)-points(1:2,2))~=0)% calculate normal vector
                normal = (points(1:2,1)-points(1:2,2))/norm(points(1:2,1)-points(1:2,2));
            end
        end
        %  Approximate Jacobian using ego center point
        kinemJacob = [1 0 -distToCenter*sin(X(i,3));
            0 1 distToCenter*cos(X(i,3))];
        G(i,:,iter) = -normal'*kinemJacob;
        iter = iter+1;
    end
end

end