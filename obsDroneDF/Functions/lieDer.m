function L = lieDer(h, f, x)
    L = gradient(h, x)'*f;
end