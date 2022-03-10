%% %% %% ECBF

%% SETUP
clear 
clc 
addpath('./Functions')

%% PARAMETERS

n_cart = 3;
k = 10;
m1 = 10;
m2 = 10;
m3 = 10;
x3_max = 3.15;
x3d = 0;

x0 = [0; 1; 2; 0; 0; 0];
%% LINEAR SYSTEM MATRICES

Atmp = [-k/m1 k/m1 0;
         k/m2 -2*k/m2 k/m2;
         0 k/m3 -k/m3];

A = [zeros(3), eye(3);
     Atmp, zeros(3)];

B = [zeros(3, 1); 1/m1; zeros(2, 1)];

C =  [zeros(1, 2), 1, zeros(1, 3)];

r = relativeDegree(A, B, C);

%% I/O CANONICAL FORM
F = [zeros(r-1, 1), eye(r-1); zeros(1, r)];
G = [zeros(r-1, 1); 1];

%% 