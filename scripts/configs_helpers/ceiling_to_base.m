%% Ceiling to base frame transformer

function [T] = ceiling_to_base(x, y, z, offset, mag, phi)
    Tc = transl(0, 0, z) * trotz(deg2rad(phi)) * troty(deg2rad(mag)) * trotz(-deg2rad(phi)) * transl(0, 0, -offset);
    T = Tc * transl(x, y, 0);
end