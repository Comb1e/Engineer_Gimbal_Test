syms x A;
% 使用二阶泰勒展开式逼近原方程
% eq = 5/2*x^2 -(180/pi+22)*x +(A-10) == 0;
eq = 5/2*x^2 -(180/pi+12)*x +(A-10) == 0;
% 求解多项式方程
sol = solve(eq, x)
% 显示解

% clc
% clear
% 
% A = 60
% 
% x1 = (22*pi + 2^(1/2)*(3960*pi - 5*A*pi^2 + 292*pi^2 + 16200)^(1/2) + 180)/(5*pi)
% x2 = (22*pi - 2^(1/2)*(3960*pi - 5*A*pi^2 + 292*pi^2 + 16200)^(1/2) + 180)/(5*pi)

%  (2^(1/2)*(17*A*pi^2 - 170*pi^2 + 16200)^(1/2) - 180)/(17*pi)
% -(2^(1/2)*(17*A*pi^2 - 170*pi^2 + 16200)^(1/2) + 180)/(17*pi)

% syms x A;
% % 使用泰勒展开近似cos和sin函数
% approx_eq = x + 5*(1 - x^2/2) + 22*(x - x^3/6) == A - 5;
% % 解得近似的二阶多项式方程
% sol = solve(approx_eq, x);
% % 显示结果
% disp(sol);