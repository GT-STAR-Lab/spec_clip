function newA = eigenclip(A, target)
    [V, D] = eig(A);
    for i = 1:size(A, 1)
        temp = D(i, i);
        n = norm(temp);
        if(n >= target)
            D(i, i) = D(i, i) / n * target;
        end
    end
    newA = V * D * inv(V);
    newA = real(newA);
end