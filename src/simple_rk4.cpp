#include "simple_rk4.hpp"

void SimpleRK4::integrate(const std::function<void(const state_type&, state_type&, double)>& system,
                          state_type& state,
                          double start_time,
                          double end_time,
                          double dt)
{
    if (dt <= 0) return;
    state_type k1(state.size()), k2(state.size()), k3(state.size()), k4(state.size());
    state_type tmp(state.size());

    for (double t = start_time; t < end_time; t += dt) {
        system(state, k1, t);

        for (size_t i = 0; i < state.size(); ++i)
            tmp[i] = state[i] + dt * 0.5 * k1[i];
        system(tmp, k2, t + dt * 0.5);

        for (size_t i = 0; i < state.size(); ++i)
            tmp[i] = state[i] + dt * 0.5 * k2[i];
        system(tmp, k3, t + dt * 0.5);

        for (size_t i = 0; i < state.size(); ++i)
            tmp[i] = state[i] + dt * k3[i];
        system(tmp, k4, t + dt);

        for (size_t i = 0; i < state.size(); ++i)
            state[i] += dt / 6.0 * (k1[i] + 2.0 * k2[i] + 2.0 * k3[i] + k4[i]);
    }
}
