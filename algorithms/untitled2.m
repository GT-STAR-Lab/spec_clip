load('X.mat');
load('Y.mat');

% [A, m] = learnSOCmodel(X.', Y.', []);

X = X.';
Y = Y.';
Pnew = [X(:,1), Y];
[U_wls,S_wls,V_wls] = svd(Pnew,0);

n = 759;
V_wls = V_wls(:,1:n);
S_wls = S_wls(1:n,1:n);
U_wls = U_wls(:,1:n);

timeWLS = clock;
[A_WLS, ~, ~, ~] = learnWLSmodel(V_wls,S_wls,1,0);