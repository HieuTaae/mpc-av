#include "MPC.h"
#include <cppad/ipopt/solve.hpp>
#include <utility>

MPC::MPC() = default;

MPC::~MPC() = default;

std::vector<double> MPC::Solve(const Eigen::VectorXd &state, const Eigen::VectorXd &coeffs) {
    bool ok = true;
    typedef CPPAD_TESTVECTOR(double) Dvector;

    size_t n_vars = state.size() * N + 2 * (N - 1);
    size_t n_constraints = state.size() * N;

    Dvector vars(n_vars);
    std::fill(vars.begin(), vars.end(), 0.0);

    vars[x_start] = state[0];
    vars[y_start] = state[1];
    vars[psi_start] = state[2];
    vars[v_start] = state[3];
    vars[cte_start] = state[4];
    vars[epsi_start] = state[5];

    Dvector vars_lb(n_vars);
    Dvector vars_ub(n_vars);

    for (int i = 0; i < delta_start; ++i) {
        vars_lb[i] = -1.0e19;
        vars_ub[i] = 1.0e19;
    }

    for (int i = delta_start; i < a_start; ++i) {
        vars_lb[i] = -0.436332;
        vars_ub[i] = 0.436332;
    }

    for (int i = a_start; i < n_vars; ++i) {
        vars_lb[i] = -1.0;
        vars_ub[i] = 1.0;
    }

    Dvector constraints_lb(n_constraints);
    Dvector constraints_ub(n_constraints);

    for (int i = 0; i < n_constraints; ++i) {
        constraints_lb[i] = 0.0;
        constraints_ub[i] = 0.0;
    }

    constraints_lb[x_start] = state[0];
    constraints_lb[y_start] = state[1];
    constraints_lb[psi_start] = state[2];
    constraints_lb[v_start] = state[3];
    constraints_lb[cte_start] = state[4];
    constraints_lb[epsi_start] = state[5];

    constraints_ub[x_start] = state[0];
    constraints_ub[y_start] = state[1];
    constraints_ub[psi_start] = state[2];
    constraints_ub[v_start] = state[3];
    constraints_ub[cte_start] = state[4];
    constraints_ub[epsi_start] = state[5];

    Cost cost(coeffs);

    std::string options;
    // Uncomment this if you'd like more print information
    options += "Integer print_level  0\n";
    options += "Sparse  true        forward\n";
    options += "Sparse  true        reverse\n";
    options += "Numeric max_cpu_time          0.5\n";

    // place to return solution
    CppAD::ipopt::solve_result<Dvector> solution;

    // solve the problem
    CppAD::ipopt::solve<Dvector, Cost>(options, vars, vars_lb, vars_ub, constraints_lb,
                                       constraints_ub, cost, solution);

    // Check some of the solution values
    ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

    auto cost_result = solution.obj_value;
    std::cout << "Cost " << cost_result << std::endl;

    std::vector<double> result;
    result.push_back(solution.x[delta_start]);
    result.push_back(solution.x[a_start]);

    for (int i = 0; i < N - 1; ++i) {
        result.push_back(solution.x[x_start + i + 1]);
        result.push_back(solution.x[y_start + i + 1]);
    }

    return result;
}

Cost::Cost(Eigen::VectorXd coeffs) {
    this->coeffs = std::move(coeffs);
}

void Cost::operator()(Cost::ADvector &cost, const Cost::ADvector &vars) {
    cost[0] = 0;

    // State cost
    for (int t = 0; t < N; ++t) {
        cost[0] += CppAD::pow(vars[v_start + t] - v_ref, 2);
        cost[0] += 10 * CppAD::pow(vars[cte_start + t], 2);
        cost[0] += 1500 * CppAD::pow(vars[epsi_start + t], 2);
    }

    // Actuator cost
    for (int t = 0; t < N - 1; ++t) {
        cost[0] += 2000 * CppAD::pow(vars[delta_start + t], 2);
        cost[0] += 40 * CppAD::pow(vars[a_start + t], 2);
    }

    // Smooth actuator
    for (int t = 0; t < N - 2; ++t) {
        cost[0] += 7000 * CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);
        cost[0] += 50 * CppAD::pow(vars[a_start + t + 1] - vars[a_start + t], 2);
    }

    // Initial constraints
    cost[x_start + 1] = vars[x_start];
    cost[y_start + 1] = vars[y_start];
    cost[psi_start + 1] = vars[psi_start];
    cost[v_start + 1] = vars[v_start];
    cost[cte_start + 1] = vars[cte_start];
    cost[epsi_start + 1] = vars[epsi_start];

    // Rest of contraints
    for (int t = 1; t < N; ++t) {
        // States at time t
        CppAD::AD<double> x0 = vars[x_start + t - 1];
        CppAD::AD<double> y0 = vars[y_start + t - 1];
        CppAD::AD<double> psi0 = vars[psi_start + t - 1];
        CppAD::AD<double> v0 = vars[v_start + t - 1];
        CppAD::AD<double> cte0 = vars[cte_start + t - 1];
        CppAD::AD<double> epsi0 = vars[epsi_start + t - 1];

        // Actuators at time t
        CppAD::AD<double> delta0 = vars[delta_start + t - 1];
        CppAD::AD<double> a0 = vars[a_start + t - 1];

        CppAD::AD<double> f0 = coeffs[0] + coeffs[1] * x0 + coeffs[2] * pow(x0, 2) + coeffs[3] * pow(x0, 3);
        CppAD::AD<double> der = coeffs[1] + 2 * coeffs[2] * x0 + 3 * coeffs[3] * pow(x0, 2);
        CppAD::AD<double> psides0 = CppAD::atan(der);

        // States at time t + 1
        CppAD::AD<double> x1 = vars[x_start + t];
        CppAD::AD<double> y1 = vars[y_start + t];
        CppAD::AD<double> psi1 = vars[psi_start + t];
        CppAD::AD<double> v1 = vars[v_start + t];
        CppAD::AD<double> cte1 = vars[cte_start + t];
        CppAD::AD<double> epsi1 = vars[epsi_start + t];

        cost[x_start + t + 1] = x1 - (x0 + v0 * CppAD::cos(psi0) * (dt + latency));
        cost[y_start + t + 1] = y1 - (y0 + v0 * CppAD::sin(psi0) * (dt + latency));
        cost[psi_start + t + 1] = psi1 - (psi0 - v0 * delta0 * (dt + latency) / Lf);
        cost[v_start + t + 1] = v1 - (v0 + a0 * (dt + latency));
        cost[cte_start + t + 1] = cte1 - (f0 - y0 + v0 * CppAD::sin(epsi0) * (dt + latency));
        cost[epsi_start + t + 1] = epsi1 - (psi0 - psides0 + v0 * delta0 * (dt + latency) / Lf);
    }
}


