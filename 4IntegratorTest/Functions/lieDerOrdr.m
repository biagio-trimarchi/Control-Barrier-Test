function L = lieDerOrdr(h, f, x, r)
    L = h;
    for i = 1:r
        L = lieDer(L, f, x);
    end
end