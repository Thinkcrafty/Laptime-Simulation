function mu_eff = tyre_mu(Fz, car)

mu_eff = car.mu*(1 - car.mu_sens * log(Fz/car.Fz_ref));
mu_eff = max(mu_eff, 0.7);

end
