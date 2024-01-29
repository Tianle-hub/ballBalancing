syms q1 q2 q3 q4 q5 q6 real
syms qp1 qp2 qp3 qp4 qp5 qp6 real
syms qpp1 qpp2 qpp3 qpp4 qpp5 qpp6 real
syms d1 a2 a3 d4 d5 d6 real
syms tcm11 tcm12 tcm13 tcm21 tcm22 tcm23 real
syms tcm31 tcm32 tcm33 tcm41 tcm42 tcm43 real
syms tcm51 tcm52 tcm53 tcm61 tcm62 tcm63 real
syms m1 m2 m3 m4 m5 m6 g real
syms I111 I112 I113 I122 I123 I133 real
syms I211 I212 I213 I222 I223 I233 real
syms I311 I312 I313 I322 I323 I333 real
syms I411 I412 I413 I422 I423 I433 real
syms I511 I512 I513 I522 I523 I533 real
syms I611 I612 I613 I622 I623 I633 real

q = [q1; q2; q3; q4; q5; q6];
qp = [qp1; qp2; qp3; qp4; qp5; qp6];
qpp = [qpp1; qpp2; qpp3; qpp4; qpp5; qpp6];

I(:,:,1) = [I111, I112, I113; I112, I122, I123; I113, I123, I133];
I(:,:,2) = [I211, I212, I213; I212, I222, I223; I213, I223, I233];
I(:,:,3) = [I311, I312, I313; I312, I322, I323; I313, I323, I333];
I(:,:,4) = [I411, I412, I413; I412, I422, I423; I413, I423, I433];
I(:,:,5) = [I511, I512, I513; I512, I522, I523; I513, I523, I533];
I(:,:,6) = [I611, I612, I613; I612, I622, I623; I613, I623, I633];

tcm(:,1) = [tcm11; tcm12; tcm13];
tcm(:,2) = [tcm21; tcm22; tcm23];
tcm(:,3) = [tcm31; tcm32; tcm33];
tcm(:,4) = [tcm41; tcm42; tcm43];
tcm(:,5) = [tcm51; tcm52; tcm53];
tcm(:,6) = [tcm61; tcm62; tcm63];

m = [m1; m2; m3; m4; m5; m6];

n = 6;

tic 
% DH parameters
% % UR10
% DH = [0, 0.1273, 0, pi/2;
%       0, 0, -0.612, 0;
%       0, 0, -0.5723, 0;
%       0, 0.163941, 0, pi/2;
%       0, 0.1157, 0, -pi/2;
%       0, 0.0922, 0, 0];
% 
% % UR5e
% DH = [0, 0.1625, 0, pi/2;
%       0, 0, -0.425, 0;
%       0, 0, -0.3922, 0;
%       0, 0.1333, 0, pi/2;
%       0, 0.0997, 0, -pi/2;
%       0, 0.0996, 0, 0];

% UR10
DH = [q1, d1, 0, pi/2;
      q2, 0, a2, 0;
      q3, 0, a3, 0;
      q4, d4, 0, pi/2;
      q5, d5, 0, -pi/2;
      q6, d6, 0, 0];

% joints w.r.t base/world
H_stack = forward_kinematics(DH);

% com w.r.t base
for i=1:n
    Hcm_stack(:,:,i) = sym(eye(4));
    Hcm_stack(1:3,4,i) = tcm(:,i);
    Hcm_stack(:,:,i) = H_stack(:,:,i+1) * Hcm_stack(:,:,i);
end

% translations
t_0 = sym(zeros(3, n+1));
for i=1:n+1
    t_0(:,i) = simplify(H_stack(1:3,4,i));
end

tcm_0 = sym(zeros(3, n));
for i=1:n
    tcm_0(:,i) = simplify(Hcm_stack(1:3,4,i));
end


% direction of joint
z_0 = sym(zeros(3, n));
for i=1:n
    z_0(:,i) = simplify(H_stack(1:3,1:3,i)*[0; 0; 1]);
end

% Jacobians
J_0 = sym(zeros([6, 6, 6]));
for i=1:n
    for j=1:i
        J_0(:,j,i) = [cross(z_0(:,j), t_0(:,i+1)-t_0(:,j)); z_0(:,j)];
    end
end

Jcm_0 = sym(zeros([6, 6, 6]));
for i=1:n
    for j=1:i
        Jcm_0(:,j,i) = [cross(z_0(:,j), tcm_0(:,i)-t_0(:,j)); z_0(:,j)];
    end
end

toc
fprintf('computing M\n');
tic
% Inertial matrix
M = sym(zeros(n, n));
for i=1:n
    M = M + m(i) * Jcm_0(1:3,:,i).' * Jcm_0(1:3,:,i) + Jcm_0(4:6,:,i).' * Hcm_stack(1:3,1:3,i) * I(i) * Hcm_stack(1:3,1:3,i).' * Jcm_0(4:6,:,i);
end

% parallel computing for speed up 
% M = simplify(M);
M = reshape(M, [1,n*n]);
parfor i=1:n*n
    M(1,i) = simplify(M(1,i));
end
M = reshape(M, [n,n]);

toc
fprintf('computing C\n');
tic
% Coriolis and centripetal matrix
C = sym(zeros(n,n));
parfor k=1:n
    for j=1:n
        for i=1:n
            C(k,j) = C(k,j) + 0.5*(diff(M(k,j),q(i)) + diff(M(k,i),q(j)) - diff(M(i,j),q(k)))*qp(i);
        end
    end
end
% C = simplify(C);
C = reshape(C, [1, n*n]);
parfor i=1:n*n
    C(1,i) = simplify(C(1,i));
end
C = reshape(C, [n,n]);

% Potential energy
P = sym(0);
for i=1:n
    P = P + m(i)*[0;0;g]'*tcm_0(:,i);
end

toc
fprintf('computing G\n');
tic
% Gravitational vector
G = sym(zeros(n,1));
parfor i=1:n
    G(i,1) = simplify(diff(P, q(i)));
end

% G = simplify(G);

% M is symmetric matrix
if isequal(M,M')
    fprintf('M is a symetric matrix.\n');
else
    fprintf('M is not a symetric matrix.\n');
end

toc
fprintf('computing dM\n');
tic
% dM-2C is skew symmetry matrix
dM = sym(zeros(n,n));
parfor i=1:n
    for j=1:n
        % for k=1:n
        %     dM(i,j) = dM(i,j) + diff(M(i,j),q(k))*qp(k);
        % end
        dM(i,j) = diff(M(i,j),q1)*qp1 + diff(M(i,j),q2)*qp2 + diff(M(i,j),q3)*qp3 + diff(M(i,j),q4)*qp4 + diff(M(i,j),q5)*qp5 + diff(M(i,j),q6)*qp6;
    end
end

% N = simplify(dM - 2*C);
N = dM - 2*C;
N = reshape(N, [1, n*n]);
parfor i=1:n*n
    N(1,i) = simplify(N(1,i));
end
N = reshape(N, [n,n]);

if isequal(simplify(N+N'), sym(zeros(n,n)))
    fprintf("N = -N'\n");
else
    fprintf("N ~= -N'\n");
end

x = rand(n,1);
if isequal(simplify(x'*N*x), sym(0))
    fprintf("x'*N*x = 0\n");
else
    fprintf("x'*N*x ~= 0\n");
end

toc
fprintf('computing eqns\n');
tic
% Euler-Lagrange equations
% eqns = simplify(M*qpp + C*qp + G);
eqns = sym(zeros(n,1));
parfor i=1:n
    eqns(i) = simplify(M(i,:)*qpp + C(i,:)*qp + G(i,1));
end
toc
fprintf('computing regressor\n');
tic
% Regression
Iv = [I111 I112 I113 I122 I123 I133 I211 I212 I213 I222 I223 I233 I311 I312 I313 I322 I323 I333 I411 I412 I413 I422 I423 I433 I511 I512 I513 I522 I523 I533 I611 I612 I613 I622 I623 I633];
Ps = [d1 a2 a3 d4 d5 d6 m1 m2 m3 m4 m5 m6 g reshape(tcm, [1,18]) Iv];
Y_tmp = [];
Theta_tmp = [];
y_m_stack = cell(n, 1);
t_v_stack = cell(n, 1);
% to matrix form
parfor i=1:n
    [y_v, t_v] = coeffs(eqns(i), Ps);
    y_m = sym(zeros(n, length(y_v)));
    y_m(i,:) = y_v;
    y_m_stack{i, 1} = y_m;
    t_v_stack{i, 1} = t_v';
end

for i=1:n
    Y_tmp = [Y_tmp y_m_stack{i,1}];
    Theta_tmp = [Theta_tmp; t_v_stack{i,1}];
end
% combine similar terms 
Theta = unique(Theta_tmp);
m = length(Theta);
Y = sym(zeros(n, m));
for i=1:m
    index = ismember(Theta_tmp, Theta(i));
    for j=1:n
        Y(j, i) = Y_tmp(j,:)*index;
    end
end

% verify
err = simplify(eqns - Y*Theta)
toc