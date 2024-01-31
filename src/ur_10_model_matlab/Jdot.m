syms t real
syms q1(t) q2(t) q3(t) q4(t) q5(t) q6(t)
syms qp1 qp2 qp3 qp4 qp5 qp6 real
syms d1 a2 a3 d4 d5 d6 real


q = [q1; q2; q3; q4; q5; q6];
qp = [qp1; qp2; qp3; qp4; qp5; qp6];

Jef = sym(zeros(36,1));

Jef(1) = d6*(cos(q1)*cos(q5) + cos(q2 + q3 + q4)*sin(q1)*sin(q5)) + d4*cos(q1) - a2*cos(q2)*sin(q1) - d5*sin(q2 + q3 + q4)*sin(q1) - a3*cos(q2)*cos(q3)*sin(q1) + a3*sin(q1)*sin(q2)*sin(q3);
Jef(2) = d6*(cos(q5)*sin(q1) - cos(q2 + q3 + q4)*cos(q1)*sin(q5)) + d4*sin(q1) + a2*cos(q1)*cos(q2) + d5*sin(q2 + q3 + q4)*cos(q1) + a3*cos(q1)*cos(q2)*cos(q3) - a3*cos(q1)*sin(q2)*sin(q3);
Jef(3) = 0;
Jef(4) = 0;
Jef(5) = 0;
Jef(6) = 1;
Jef(7) = -cos(q1)*(a3*sin(q2 + q3) + a2*sin(q2) - d5*(cos(q2 + q3)*cos(q4) - sin(q2 + q3)*sin(q4)) - d6*sin(q5)*(cos(q2 + q3)*sin(q4) + sin(q2 + q3)*cos(q4)));
Jef(8) = -sin(q1)*(a3*sin(q2 + q3) + a2*sin(q2) - d5*(cos(q2 + q3)*cos(q4) - sin(q2 + q3)*sin(q4)) - d6*sin(q5)*(cos(q2 + q3)*sin(q4) + sin(q2 + q3)*cos(q4)));
Jef(9) = cos(q1)*(d6*(cos(q5)*sin(q1) - cos(q2 + q3 + q4)*cos(q1)*sin(q5)) + d4*sin(q1) + a2*cos(q1)*cos(q2) + d5*sin(q2 + q3 + q4)*cos(q1) + a3*cos(q1)*cos(q2)*cos(q3) - a3*cos(q1)*sin(q2)*sin(q3)) - sin(q1)*(d6*(cos(q1)*cos(q5) + cos(q2 + q3 + q4)*sin(q1)*sin(q5)) + d4*cos(q1) - a2*cos(q2)*sin(q1) - d5*sin(q2 + q3 + q4)*sin(q1) - a3*cos(q2)*cos(q3)*sin(q1) + a3*sin(q1)*sin(q2)*sin(q3));
Jef(10) = sin(q1);
Jef(11) = -cos(q1);
Jef(12) = 0;
Jef(13) = cos(q1)*(d5*(cos(q2 + q3)*cos(q4) - sin(q2 + q3)*sin(q4)) - a3*sin(q2 + q3) + d6*sin(q5)*(cos(q2 + q3)*sin(q4) + sin(q2 + q3)*cos(q4)));
Jef(14) = sin(q1)*(d5*(cos(q2 + q3)*cos(q4) - sin(q2 + q3)*sin(q4)) - a3*sin(q2 + q3) + d6*sin(q5)*(cos(q2 + q3)*sin(q4) + sin(q2 + q3)*cos(q4)));
Jef(15) = cos(q1)*(d6*(cos(q5)*sin(q1) - cos(q2 + q3 + q4)*cos(q1)*sin(q5)) + d4*sin(q1) + d5*sin(q2 + q3 + q4)*cos(q1) + a3*cos(q1)*cos(q2)*cos(q3) - a3*cos(q1)*sin(q2)*sin(q3)) - sin(q1)*(d6*(cos(q1)*cos(q5) + cos(q2 + q3 + q4)*sin(q1)*sin(q5)) + d4*cos(q1) - d5*sin(q2 + q3 + q4)*sin(q1) - a3*cos(q2)*cos(q3)*sin(q1) + a3*sin(q1)*sin(q2)*sin(q3));
Jef(16) = sin(q1);
Jef(17) = -cos(q1);
Jef(18) = 0;
Jef(19) = cos(q1)*(d5*(cos(q2 + q3)*cos(q4) - sin(q2 + q3)*sin(q4)) + d6*sin(q5)*(cos(q2 + q3)*sin(q4) + sin(q2 + q3)*cos(q4)));
Jef(20) = sin(q1)*(d5*(cos(q2 + q3)*cos(q4) - sin(q2 + q3)*sin(q4)) + d6*sin(q5)*(cos(q2 + q3)*sin(q4) + sin(q2 + q3)*cos(q4)));
Jef(21) = cos(q1)*(d6*(cos(q5)*sin(q1) - cos(q2 + q3 + q4)*cos(q1)*sin(q5)) - cos(q1)*(a3*cos(q2 + q3) + a2*cos(q2)) + d4*sin(q1) + a2*cos(q1)*cos(q2) + d5*sin(q2 + q3 + q4)*cos(q1) + a3*cos(q1)*cos(q2)*cos(q3) - a3*cos(q1)*sin(q2)*sin(q3)) - sin(q1)*(d6*(cos(q1)*cos(q5) + cos(q2 + q3 + q4)*sin(q1)*sin(q5)) + d4*cos(q1) + sin(q1)*(a3*cos(q2 + q3) + a2*cos(q2)) - a2*cos(q2)*sin(q1) - d5*sin(q2 + q3 + q4)*sin(q1) - a3*cos(q2)*cos(q3)*sin(q1) + a3*sin(q1)*sin(q2)*sin(q3));
Jef(22) = sin(q1);
Jef(23) = -cos(q1);
Jef(24) = 0;
Jef(25) = - cos(q2 + q3 + q4)*(d6*(cos(q1)*cos(q5) + cos(q2 + q3 + q4)*sin(q1)*sin(q5)) - d5*sin(q2 + q3 + q4)*sin(q1)) - sin(q2 + q3 + q4)*sin(q1)*(d5*(cos(q2 + q3)*cos(q4) - sin(q2 + q3)*sin(q4)) + d6*sin(q5)*(cos(q2 + q3)*sin(q4) + sin(q2 + q3)*cos(q4)));
Jef(26) = sin(q2 + q3 + q4)*cos(q1)*(d5*(cos(q2 + q3)*cos(q4) - sin(q2 + q3)*sin(q4)) + d6*sin(q5)*(cos(q2 + q3)*sin(q4) + sin(q2 + q3)*cos(q4))) - cos(q2 + q3 + q4)*(d6*(cos(q5)*sin(q1) - cos(q2 + q3 + q4)*cos(q1)*sin(q5)) + d5*sin(q2 + q3 + q4)*cos(q1));
Jef(27) = - sin(q2 + q3 + q4)*cos(q1)*(d6*(cos(q1)*cos(q5) + cos(q2 + q3 + q4)*sin(q1)*sin(q5)) - d5*sin(q2 + q3 + q4)*sin(q1)) - sin(q2 + q3 + q4)*sin(q1)*(d6*(cos(q5)*sin(q1) - cos(q2 + q3 + q4)*cos(q1)*sin(q5)) + d5*sin(q2 + q3 + q4)*cos(q1));
Jef(28) = sin(q2 + q3 + q4)*cos(q1);
Jef(29) = sin(q2 + q3 + q4)*sin(q1);
Jef(30) = -cos(q2 + q3 + q4);
Jef(31) = (cos(q1)*cos(q5) + cos(q2 + q3 + q4)*sin(q1)*sin(q5))*(d5*(cos(q2 + q3)*cos(q4) - sin(q2 + q3)*sin(q4)) - d5*cos(q2 + q3 + q4) + d6*sin(q5)*(cos(q2 + q3)*sin(q4) + sin(q2 + q3)*cos(q4))) - sin(q2 + q3 + q4)*sin(q5)*(d5*(cos(q4)*(cos(q2)*sin(q1)*sin(q3) + cos(q3)*sin(q1)*sin(q2)) - sin(q4)*(sin(q1)*sin(q2)*sin(q3) - cos(q2)*cos(q3)*sin(q1))) + d6*(cos(q1)*cos(q5) + cos(q2 + q3 + q4)*sin(q1)*sin(q5)) - d5*sin(q2 + q3 + q4)*sin(q1));
Jef(32) = (cos(q5)*sin(q1) - cos(q2 + q3 + q4)*cos(q1)*sin(q5))*(d5*(cos(q2 + q3)*cos(q4) - sin(q2 + q3)*sin(q4)) - d5*cos(q2 + q3 + q4) + d6*sin(q5)*(cos(q2 + q3)*sin(q4) + sin(q2 + q3)*cos(q4))) - sin(q2 + q3 + q4)*sin(q5)*(d6*(cos(q5)*sin(q1) - cos(q2 + q3 + q4)*cos(q1)*sin(q5)) - d5*(cos(q4)*(cos(q1)*cos(q2)*sin(q3) + cos(q1)*cos(q3)*sin(q2)) + sin(q4)*(cos(q1)*cos(q2)*cos(q3) - cos(q1)*sin(q2)*sin(q3))) + d5*sin(q2 + q3 + q4)*cos(q1));
Jef(33) = (cos(q1)*cos(q5) + cos(q2 + q3 + q4)*sin(q1)*sin(q5))*(d6*(cos(q5)*sin(q1) - cos(q2 + q3 + q4)*cos(q1)*sin(q5)) - d5*(cos(q4)*(cos(q1)*cos(q2)*sin(q3) + cos(q1)*cos(q3)*sin(q2)) + sin(q4)*(cos(q1)*cos(q2)*cos(q3) - cos(q1)*sin(q2)*sin(q3))) + d5*sin(q2 + q3 + q4)*cos(q1)) - (cos(q5)*sin(q1) - cos(q2 + q3 + q4)*cos(q1)*sin(q5))*(d5*(cos(q4)*(cos(q2)*sin(q1)*sin(q3) + cos(q3)*sin(q1)*sin(q2)) - sin(q4)*(sin(q1)*sin(q2)*sin(q3) - cos(q2)*cos(q3)*sin(q1))) + d6*(cos(q1)*cos(q5) + cos(q2 + q3 + q4)*sin(q1)*sin(q5)) - d5*sin(q2 + q3 + q4)*sin(q1));
Jef(34) = cos(q5)*sin(q1) - cos(q2 + q3 + q4)*cos(q1)*sin(q5);
Jef(35) = - cos(q1)*cos(q5) - cos(q2 + q3 + q4)*sin(q1)*sin(q5);
Jef(36) = -sin(q2 + q3 + q4)*sin(q5);

% Jefp
Jefp = sym(zeros(36,1));
for i=1:36
    Jefp(i) = diff(Jef(i), t);
    Jefp(i) = subs(Jefp(i), [diff(q1(t),t), diff(q2(t),t), diff(q3(t),t),diff(q4(t),t), diff(q5(t),t), diff(q6(t),t)], qp');
    Jefp(i)
end