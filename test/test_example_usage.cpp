#include "../include/simple_rk4.hpp"
#include <iostream>
#include <cmath>
#include <cassert>

int main() {
    SimpleRK4::state_type state = {1.0};
    auto system = [](const SimpleRK4::state_type &x, SimpleRK4::state_type &dxdt, double) {
        dxdt[0] = -x[0];
    };
    SimpleRK4::integrate(system, state, 0.0, 1.0, 0.01);
    double expected = std::exp(-1.0);
    assert(std::abs(state[0] - expected) < 1e-3);
    std::cout << "Final value: " << state[0] << std::endl;
    return 0;
}
