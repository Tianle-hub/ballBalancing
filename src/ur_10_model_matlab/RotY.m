function [ Ry ] = RotY( betaY )

Ry = [cos(betaY), 0, sin(betaY); 0, 1, 0; -sin(betaY), 0, cos(betaY)];
end

