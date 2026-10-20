%% Jacobian, Singularity, and Manipulability for 3-DOF RRR Robot
clear; clc; close all;

syms q1 q2 q3 dq1 dq2 dq3 ddq1 ddq2 ddq3 real
syms m1 m2 m3 g I real
syms l1 l2 l3 real

q  = [q1; q2; q3];
dq = [dq1; dq2; dq3];
ddq = [ddq1; ddq2; ddq3];

% --- Link Lengths & masses ---
l1 = 0.5; l2 = 0.5; l3 = 0.5;
m1 = 1; m2 = 1; m3 = 1;


% Visualize the Arm (initial config)
plot_RRR(deg2rad(0), deg2rad(45), deg2rad(60));

% --- HTMs ---
T01 = [cos(q1), -sin(q1), 0, l1*cos(q1);
       sin(q1),  cos(q1), 0, l1*sin(q1);
       0,        0,       1, 0;
       0,        0,       0, 1]

T12 = [cos(q2), -sin(q2), 0, l2*cos(q2);
       sin(q2),  cos(q2), 0, l2*sin(q2);
       0,        0,       1, 0;
       0,        0,       0, 1]

T23 = [cos(q3), -sin(q3), 0, l3*cos(q3);
       sin(q3),  cos(q3), 0, l3*sin(q3);
       0,        0,       1, 0;
       0,        0,       0, 1]

T02 = simplify(T01 * T12);

disp('overall homogeneous transformation for the end effector. ');

T03 = simplify(T02 * T23)

% Position and Orientation
p = T03(1:3, 4);
o = T03(1:3, 3);

disp('Position Vector p:'); disp(p);
disp('Orientation Vector o:'); disp(o);

% Jacobian
Jv = simplify(jacobian(p, q));
Jw = simplify(jacobian(o, q));
J = [Jv; Jw];
disp('Jacobian J:'); disp(J);

% --- Center of Mass Positions ---
p_com1 = [l1/2; 0; 0; 1];      
p_com2 = [l2/2; 0; 0; 1];
p_com3 = [l3/2; 0; 0; 1];

r1 = simplify(T01 * p_com1); r1 = r1(1:3)
r2 = simplify(T02 * p_com2); r2 = r2(1:3)
r3 = simplify(T03 * p_com3); r3 = r3(1:3)

% Linear Velocity Jacobians
Jv1 = jacobian(r1, q)
Jv2 = jacobian(r2, q)
Jv3 = jacobian(r3, q)

% Angular Velocity Jacobians
z0 = [0; 0; 1];
z1 = T01(1:3,3);
z2 = T02(1:3,3);

Jw1 = [z0, zeros(3,1), zeros(3,1)];
Jw2 = [z0, z1, zeros(3,1)];
Jw3 = [z0, z1, z2];

% Kinetic Energy
T1 = 0.5 * m1 * dq.' * (Jv1.' * Jv1) * dq + 0.5 * dq.' * (Jw1.' * I * Jw1) * dq;
T2 = 0.5 * m2 * dq.' * (Jv2.' * Jv2) * dq + 0.5 * dq.' * (Jw2.' * I * Jw2) * dq;
T3 = 0.5 * m3 * dq.' * (Jv3.' * Jv3) * dq + 0.5 * dq.' * (Jw3.' * I * Jw3) * dq;
T_total = simplify(T1 + T2 + T3)

% Potential Energy
gravity = [0; 0; -g];
V1 = m1 * gravity.' * r1;
V2 = m2 * gravity.' * r2;
V3 = m3 * gravity.' * r3;
V_total = simplify(V1 + V2 + V3)

% Lagrangian
L = T_total - V_total;

% Euler-Lagrange Equations
eqn = sym('eqn', [3 1]);
for i = 1:3
    dLdq = diff(L, q(i));
    dLdqdot = diff(L, dq(i));
    ddt_dLdqdot = 0;
    for j = 1:3
        ddt_dLdqdot = ddt_dLdqdot + diff(dLdqdot, q(j)) * dq(j) + diff(dLdqdot, dq(j)) * sym(['ddq' num2str(j)]);
    end
    eqn(i) = simplify(ddt_dLdqdot - dLdq);
end
disp('Euler-Lagrange Equations:'); disp(eqn);


