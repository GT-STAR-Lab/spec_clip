function [unclipped] = LDS_high_dim(X_m_N, Y_m_N)
    [m, N] = size(X_m_N);
    % for i = 1:N
    %     z_t = X_m_N(:, i);
    %     z_t_1 = Y_m_N(:, i);
    %     A = A + z_t_1 * z_t.';
    %     G = G + z_t * z_t.';
    %     disp(i);
    % end
    % A = A / N;
    % G = G / N;
    A = Y_m_N * X_m_N.';
    G = X_m_N * X_m_N.';
    unclipped = A * pinv(G);

end