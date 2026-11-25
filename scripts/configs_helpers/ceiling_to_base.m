%% Ceiling to base frame transformer

function [T] = ceiling_to_base(x, y, ceil_height, offset, mag, phi)
    Tc = transl(0, 0, ceil_height) * trotz(deg2rad(phi)) * troty(deg2rad(mag)) * trotz(-deg2rad(phi)) * transl(0, 0, -offset);
    T = Tc * transl(x, y, 0);
end




