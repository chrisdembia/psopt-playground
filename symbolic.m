
syms g;
syms L1;
syms L2;
syms m1;
syms m2;
syms theta1;
syms theta2;
syms theta1dot;
syms theta2dot;
syms tau1;
syms tau2;

tau = [tau1; tau2];

c1 = cos(theta1);
c2 = cos(theta2);
s2 = sin(theta2);
z1 = m2 * L1 * L2 * c2;
M12 = m2 * L2^2 + z1;

M = [m1 * L1^2 + m2 * (L1^2 + L2^2) + 2 * z1, M12;
     M12,                                     m2 * L2^2];
V = m2 * L1 * L2 * s2 * [theta2dot * (-2 * theta1dot - theta2dot);
                         theta1dot^2];
G = g * [L1 * c1 * (m1 + m2) + m2 * L2;
         m2 * L2 * cos(theta1 + theta2)];

thetadot = simplify(M\(tau - (V + G)), 100);

thetadot(1)

disp('hi')
thetadot(2)
