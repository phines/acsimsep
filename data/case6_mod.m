function [baseMVA, bus, gen, branch, area, gencost,load] = case6_mod

C = fnConstants;
baseMVA = 100.0000;

bus = ...
[
  1 3    0.00    0.00    0.00    0.00   1 1.050   0.000  500.0   1 1.10 0.90;
  2 2    0.00    0.00    0.00    0.00   1 1.050  -3.725  500.0   1 1.10 0.90;
  3 2    0.00    0.00    0.00    0.00   2 1.070  -4.334  500.0   1 1.10 0.90;
  4 1   70.00   70.00    0.00    0.00   1 0.986  -4.179  500.0   1 1.10 0.90;
  5 1   70.00   70.00    0.00    0.00   2 0.980  -5.224  500.0   1 1.10 0.90;
  6 1   70.00   70.00    0.00    0.00   2 1.001  -5.971  500.0   1 1.10 0.90;
];

gen = ...
[
  1  108.45   23.12  500.00  -500.00 1.050  100.0 1 1000.00    0.00
  2   50.00   86.86  500.00  -500.00 1.050  100.0 1 1000.00    0.00
  3   60.00   98.83  500.00  -500.00 1.070  100.0 1 1000.00    0.00
];
gen(:,C.RUR) = 0;
gen(:,C.RDR) = 10;
gen(:,C.RUC) = 50;
gen(:,C.RDC) = 30;

branch = ...
[
  1  2 0.1000 0.2000 0.0200  70.0000  85.0000  105.0000 0.0000 0.0000 1;
  1  4 0.0500 0.2000 0.0200  70.0000  85.0000  105.0000 0.0000 0.0000 1;
  1  5 0.0800 0.3000 0.0300  70.0000  85.0000  105.0000 0.0000 0.0000 1;
  2  3 0.0500 0.2500 0.0300  70.0000  85.0000  105.0000 0.0000 0.0000 1;
  2  4 0.0500 0.1000 0.0100  70.0000  85.0000  105.0000 0.0000 0.0000 1;
  2  5 0.1000 0.3000 0.0200  70.0000  85.0000  105.0000 0.0000 0.0000 1;
  2  6 0.0700 0.2000 0.0250  70.0000  85.0000  105.0000 0.0000 0.0000 1;
  3  5 0.1200 0.2600 0.0250  70.0000  85.0000  105.0000 0.0000 0.0000 1;
  3  6 0.0200 0.1000 0.0100  70.0000  85.0000  105.0000 0.0000 0.0000 1;
  4  5 0.2000 0.4000 0.0400  70.0000  85.0000  105.0000 0.0000 0.0000 1;
  5  6 0.1000 0.3000 0.0300  70.0000  85.0000  105.0000 0.0000 0.0000 1;
];

branch(:,C.RATE_A:C.RATE_C) = branch(:,C.RATE_A:C.RATE_C);

%%-----  OPF Data  -----%%
%% area data
area = [
	1	5;
];

%% generator cost data
% gencost = [
% 	2	1500.00	0.00	3	0.11	5	150;
% 	2	2000.00	0.00	3	0.085	1.2	600;
% 	2	3000.00	0.00	3	0.1225	1	335;
% ];

gencost = [
	2	1500.00	0.00	3	0	1	0;
	2	2000.00	0.00	3	0	1	0;
	2	3000.00	0.00	3	0	1	0;
];

%% load
load = [...
           4        1000          80          80           0    0  1;
           5        1000          80          80           0    0  1;
           6        1000          80          80           0    0  1;
        ];
