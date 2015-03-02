function simtwolink

    [t, future] = ode45(@rhs, [0, 10], [-pi, 0, 0, 0]');

    save('simtwolink_t.dat', '-ascii', 't');
    save('simtwolink_x.dat', '-ascii', 'future');

end

function v = pow(x, y)
    v = x^y;
end

function ydot = rhs(t, state)

    ydot = zeros(4, 1);

    g = 9.81;
    L1 = 1;
    L2 = 1;
    m1 = 1;
    m2 = 1;

    theta1 = state(1);
    theta2 = state(2);
    theta1dot = state(3);
    theta2dot = state(4);
    tau1 = 0; %controls[ CINDEX(1) ];
    tau2 = 0; %controls[ CINDEX(2) ];
    
    ydot(1) = theta1dot;
    ydot(2) = theta2dot;

    ydot(3) = (2*L2*tau1 - 2*L2*tau2 - cos(theta2)*(2*L1*tau2 - 2*pow(L1,2)*L2*m2*pow(theta1dot,2)*sin(theta2)) - 2*pow(L2,2)*g*m2 + 2*pow(L2,2)*g*m2*cos(theta1 + theta2) + 2*L1*pow(L2,2)*m2*pow(theta1dot,2)*sin(theta2) + 2*L1*pow(L2,2)*m2*pow(theta2dot,2)*sin(theta2) - 2*L1*L2*g*m1*cos(theta1) - L1*L2*g*m2*cos(theta1) + L1*L2*g*m2*cos(theta1 + 2*theta2) + 4*L1*pow(L2,2)*m2*theta1dot*theta2dot*sin(theta2))/(L2*(2*pow(L1,2)*m1 + 2*pow(L1,2)*m2 - 2*pow(L1,2)*m2*pow(cos(theta2),2)));
    
    ydot(4) = -(2*pow(L2,2)*m2*tau1 - 2*pow(L1,2)*m1*tau2 - 2*pow(L1,2)*m2*tau2 - 2*pow(L2,3)*g*pow(m2,2) - 2*pow(L2,2)*m2*tau2 + 2*pow(L2,3)*g*pow(m2,2)*cos(theta1 + theta2) + pow(L1,2)*L2*g*pow(m2,2)*cos(theta1 + theta2) - 2*L1*pow(L2,2)*g*pow(m2,2)*cos(theta2) - pow(L1,2)*L2*g*pow(m2,2)*cos(theta1 - theta2) + 2*L1*pow(L2,2)*g*pow(m2,2)*cos(theta1 + 2*theta2) + 2*pow(L1,2)*pow(L2,2)*pow(m2,2)*pow(theta1dot,2)*sin(2*theta2) + pow(L1,2)*pow(L2,2)*pow(m2,2)*pow(theta2dot,2)*sin(2*theta2) + 2*L1*L2*m2*tau1*cos(theta2) - 4*L1*L2*m2*tau2*cos(theta2) + 2*L1*pow(L2,3)*pow(m2,2)*pow(theta1dot,2)*sin(theta2) + 2*pow(L1,3)*L2*pow(m2,2)*pow(theta1dot,2)*sin(theta2) + 2*L1*pow(L2,3)*pow(m2,2)*pow(theta2dot,2)*sin(theta2) + pow(L1,2)*L2*g*m1*m2*cos(theta1 + theta2) - 2*L1*pow(L2,2)*g*m1*m2*cos(theta1) - pow(L1,2)*L2*g*m1*m2*cos(theta1 - theta2) + 2*pow(L1,2)*pow(L2,2)*pow(m2,2)*theta1dot*theta2dot*sin(2*theta2) + 2*pow(L1,3)*L2*m1*m2*pow(theta1dot,2)*sin(theta2) + 4*L1*pow(L2,3)*pow(m2,2)*theta1dot*theta2dot*sin(theta2))/(pow(L2,2)*m2*(2*pow(L1,2)*m1 + pow(L1,2)*m2 - pow(L1,2)*m2*cos(2*theta2)));
end
