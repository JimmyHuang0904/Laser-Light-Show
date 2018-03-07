% Inverse Kinematics & Jacobian of a linkage using 2 linear actuators
% q1 = q2 = 0 corresponds to x = 0
% Both q1 and q2 move to the right as joint value increases
% l = link length;

l = 5;             % Link Length
P = [-1 4.0];      % End-Effector

% Compute Inverse Kinematics
Q = InvKin(P, l);
J = Jacobian(P, l)

% Draw Robot
figure(1)
clf;

% Stators
h    = plot([-10 -5], [0 0], 'b');
hold on;
h(2) = plot([10 5], [0 0], 'b');
set(h, 'LineWidth', 15);

% Rotors
h    = plot([-5 Q(1)], [0 0], 'c');
hold on;
h(2) = plot([Q(2) 5], [0 0], 'c');
set(h, 'LineWidth', 5);

% Links
h    = plot([Q(1) P(1)], [0 P(2)], 'k');
hold on;
h(2) = plot([Q(2) P(1)], [0 P(2)], 'k');
set(h, 'LineWidth', 2);

grid on;
axis equal;
set(gca, 'YLim', [-5 10]);


% ---------------
% Local Functions
% ---------------

% Direct Kinematics
function [P] = DirKin(Q, l)
x = mean(Q);
deltaQ = abs(Q(1)-Q(2))/2;
y = sqrt(l^2 - deltaQ^2);
P = [x y];
end


% Inverse Kinematics
function [Q] = InvKin(P, l)
x = P(1);
y = P(2);
deltaQ = sqrt(l^2 - y^2);
q1 = x - deltaQ;
q2 = x + deltaQ;
Q = [q1 q2];
end


% Finite Diference (Numeric Approximation) Jacobian Matrix
function [J] = Jacobian(P, l)
delta = 1e-6;                  % Small increment for finite dif Jacobian
Q = InvKin(P, l);              % Compute actual joint values

% Increment q1 
QQ = Q + [delta 0];
PP = DirKin(QQ, l);
J11 = (PP(1) - P(1)) / delta;
J21 = (PP(2) - P(2)) / delta;

% Increment q2
QQ = Q + [0 delta];
PP = DirKin(QQ, l);
J12 = (PP(1) - P(1)) / delta;
J22 = (PP(2) - P(2)) / delta;

J = [J11 J12
     J21 J22];
end