#ifndef SIMPLE_RK4_HPP
#define SIMPLE_RK4_HPP

#include <vector>
#include <functional>
#include <cstddef>

class SimpleRK4 {
public:
    using state_type = std::vector<double>;

    static void integrate(const std::function<void(const state_type&, state_type&, double)>& system,
                          state_type& state,
                          double start_time,
                          double end_time,
                          double dt);
};

#endif // SIMPLE_RK4_HPP
