clear
clc

rho = 1.225;
g = 9.81;
car.m = 240;
car.Cd = 1.2;
car.Cl = 2.4;
car.A = 1.2;      % car frontal area
car.Crr = 0.019;  % Rolling resistance
car.Pmax = 50e3;  % Maximum Enigne Output
car.Fb_max = 1.9*car.m*g;    % Mximum Braking force

car.L = 1.2;      % wheelbase
car.a = 0.6;      % CG to front axle
car.b = 0.8;      % CG to rear axle
car.track = 1.2;  % track width
car.h_cg = 0.30;  % CG height

car.aero_balance = 1.0; % Self-explanatory

car.mu = 0.17;    % Tyre coefficient of friction
car.Fz_ref = 3000; % reference vertical load at which mu is measured
car.mu_sens = 0.1  % Hyper parameter to tune the mu scaling for various loading

ds = 1.0          % Discretize the track at intervals of 1m
[track.s, track.k] = generate_example_track(ds);
N = length(track.s);

v_max_corner = zeros(N,1);  % To store maximum velocity at corners
v_fwd = zeros(N,1);         % To store maximum velocity limited by acc
v_bwd = zeros(N,1);         % To store maximum velocity limited by braking

for i=1:N
    curv = abs(track.k(i));
    if curv<1e-6
        v_max_corner(i) = 100;
    else
        for j=1:10
            v_rand = 60;
            D = 0.5*rho*car.A*car.Cl*v_rand^2;
            Fz = car.m*g + D;
            mu_eff = tyre_mu(Fz, car);
            v_rand = sqrt(mu_eff*Fz/(car.m*curv));
        end
        v_max_corner(i) = v_rand;  % Store the calculated maximum velocity for the current corner
    end
end

v_fwd(1) = 0;    % No acc limit initially

for i=1:N-1
    v = v_fwd(i);
    D = 0.5*rho*car.A*car.Cl*v^2;
    drag = 0.5*rho*car.A*car.Cd*v^2;
    F_eng = min(car.Pmax/max(v,1),5000);
    acc = (F_eng - drag - (car.Crr*car.m*g))/car.m;  % Calculate acceleration
    v_temp = sqrt(max(v^2 + 2*acc*ds,0));
    v_fwd(i+1) = min(v_temp, v_max_corner(i+1));
end

v_bwd(N) = v_fwd(N);

for i = N:-1:2
    v_next = v_bwd(i);
    v_curr_est = v_fwd(i-1);

    a_req = (v_next^2 - v_curr_est^2)/(2*ds);
    a_req = max(a_req, 0);
    ay = v_next^2 * track.k(i);

    D = 0.5*rho*car.Cl*car.A*v_next^2;
    Df = car.aero_balance*D;
    Dr = (1 - car.aero_balance)*D;

    Fzf0 = (car.b/car.L)*car.m*g + Df;
    Fzr0 = (car.a/car.L)*car.m*g + Dr;

    dFz_long = car.m * abs(a_req) * car.h_cg / car.L;

    Fzf = Fzf0 + dFz_long;
    Fzr = Fzr0 - dFz_long;

    dFz_lat = car.m * abs(ay) * car.h_cg / car.track;

    Fz_fl = max(Fzf/2 + dFz_lat/2, 50);
    Fz_fr = max(Fzf/2 - dFz_lat/2, 50);
    Fz_rl = max(Fzr/2 + dFz_lat/2, 50);
    Fz_rr = max(Fzr/2 - dFz_lat/2, 50);

    mu_fl = tyre_mu(Fz_fl, car);
    mu_fr = tyre_mu(Fz_fr, car);
    mu_rl = tyre_mu(Fz_rl, car);
    mu_rr = tyre_mu(Fz_rr, car);

    Fx_max = mu_fl*Fz_fl + mu_fr*Fz_fr + mu_rl*Fz_rl + mu_rr*Fz_rr;
    Fx_max = min(Fx_max, car.Fb_max);

    a_brake_max = Fx_max/car.m;

    a_brake = min(abs(a_req), a_brake_max);

    v_temp = sqrt(max(v_next^2 + 2*a_brake*ds,0));
    v_bwd(i-1) = min(v_temp, v_fwd(i-1));
end

v = min(v_fwd,v_bwd);
dt = ds./max(v,0.1);
lap_time = sum(dt);
fprintf("Lap time is %f", lap_time);

figure;
plot(track.s, v, 'LineWidth', 2);
xlabel('Distance (m)');
ylabel('Speed (m/s)');
title('FSAE Lap Time Simulator');
grid on;

figure;
plot(track.s, v_max_corner, '--', 'LineWidth', 1.5); hold on;
plot(track.s, v_fwd, ':', 'LineWidth', 1.5);
plot(track.s, v_bwd, '-.', 'LineWidth', 1.5);
plot(track.s, v, 'LineWidth', 2);
legend('Corner Limit','Forward','Backward','Final');
xlabel('Distance (m)');
ylabel('Speed (m/s)');
title('Speed Constraints');
grid on;
