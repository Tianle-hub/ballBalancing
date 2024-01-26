function [ Rx ] = RotX( alphaX )

Rx = [1, 0, 0; 0, cos(alphaX), -sin(alphaX); 0, sin(alphaX), cos(alphaX)];

end

