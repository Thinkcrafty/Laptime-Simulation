function [s, kappa] = generate_example_track(ds)

L = 1200;
s = (0:ds:L)';
N = length(s);

kappa = zeros(N,1);
kappa(s > 200 & s < 350) = 1/25;
kappa(s > 500 & s < 650) = 1/40;
kappa(s > 800 & s < 900) = 1/18;

end
