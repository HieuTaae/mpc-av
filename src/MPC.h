#ifndef MPC_H
#define MPC_H

#include <cppad/cppad.hpp>
#include "Eigen-3.3/Eigen/Core"

class MPC {
public:
    const size_t N{10};

    const double dt{0.1};

    const double Lf{2.67};

    const double v_ref{70};

    const double latency{0.1};

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

class Cost : public MPC {
public:
    typedef CPPAD_TESTVECTOR(CppAD::AD<double>) ADvector;

    Eigen::VectorXd coeffs;

    explicit Cost(Eigen::VectorXd coeffs);

    void operator()(ADvector &cost, const ADvector &vars);
};

#endif  // MPC_H
