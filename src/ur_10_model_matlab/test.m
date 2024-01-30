% UR10
DH = [0, 0.1273, 0, pi/2;
      -pi/2, 0, -0.612, 0;
      0, 0, -0.5723, 0;
      -pi/2, 0.163941, 0, pi/2;
      0, 0.1157, 0, -pi/2;
      0, 0.0922, 0, 0];

% joints w.r.t base/world
H_stack = forward_kinematics(DH);