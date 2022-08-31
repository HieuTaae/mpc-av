#ifndef MPC_H
#define MPC_H

#include <cppad/cppad.hpp>
#include "Eigen-3.3/Eigen/Core"

// Contains model parameters and solves for optimal inputs
class MPC {
public:
    // Prediction horizon
    const size_t N{10};
    // Step duration
    const double dt{0.1};
    // Vehicle front to center of gravity distance
    const double Lf{2.67};
    // Maximum allowable velocity
    const double v_ref{70};
    // Delay between commanding and actuating
    const double latency{0.1};
    // Start positions of states and inputs in cost vector
    const size_t x_start{0};
    const size_t y_start{x_start + N};
    const size_t psi_start{y_start + N};
    const size_t v_start{psi_start + N};
    const size_t cte_start{v_start + N};
    const size_t epsi_start{cte_start + N};
    const size_t delta_start{epsi_start + N};
    const size_t a_start{delta_start + N - 1};

    // Constructor
    MPC();

    // Destructor
    virtual ~MPC();

    std::vector<double> Solve(const Eigen::VectorXd &state, const Eigen::VectorXd &coeffs);
};

// Provides method of updating cost function value to the solver
class Cost : public MPC {
public:
    // Solver-required type
    typedef CPPAD_TESTVECTOR(CppAD::AD<double>) ADvector;

    // Fitted polynomial coefficients
    Eigen::VectorXd coeffs;

    explicit Cost(Eigen::VectorXd coeffs);
    void operator()(ADvector &cost, const ADvector &vars);
};

#endif  // MPC_H
