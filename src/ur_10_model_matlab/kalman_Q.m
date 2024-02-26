delta_t = 0.002;

Gs = [1/24*delta_t^4; 1/6*delta_t^3; 1/2*delta_t^2; delta_t];

Q = Gs * Gs.'/ delta_t