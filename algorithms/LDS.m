function [unclipped] = LDS(X_m_N, Y_m_N)
% A_ls = pinv(X_N_m.' * X_N_m) * (X_N_m.' * Y_N_m);
A_ls = Y_m_N * pinv(X_m_N);
% clipped = eigenclip(A_ls, clip_target);
unclipped = A_ls;
end