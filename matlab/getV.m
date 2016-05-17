function V = getV(ps)
% extract the complex voltage from a power system structure
C = psconstants;

j = sqrt(-1);
V = ps.bus(:,C.bu.Vmag) .* exp( j * ps.bus(:,C.bu.Vang) * pi / 180 );
