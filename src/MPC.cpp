#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"

using CppAD::AD;

// Number of timesteps in horizon
size_t N = 25;

// Distance between front of vehicle and center of gravity
const double Lf = 2.67; 

double ref_v = 40;
double dt = 0.05;

size_t x_start = 0;
size_t y_start = x_start + N;
size_t psi_start = y_start + N;
size_t v_start = psi_start + N;
size_t cte_start = v_start + N;
size_t epsi_start = cte_start + N;
size_t delta_start = epsi_start + N;
size_t a_start = delta_start + N - 1;

class FG_eval {
public:
  using ADvector = CPPAD_TESTVECTOR(AD<double>);

  Eigen::VectorXd coeffs;       
  FG_eval(Eigen::VectorXd coeffs) { this->coeffs = coeffs; }

  void operator()(ADvector& fg, const ADvector& vars) {
    // Cost function
    fg[0] = 0;

    for (int t = 0; t < N; ++t) {
      fg[0] += 10 * CppAD::pow(vars[cte_start + t], 2);
      fg[0] += 10 * CppAD::pow(vars[epsi_start + t], 2);
      fg[0] += 2 * CppAD::pow(vars[v_start + t] - ref_v, 2);
    }

    for (int t = 0; t < N - 1; ++t) {
      fg[0] += 8 * CppAD::pow(vars[delta_start + t] * vars[v_start + t], 2);
    }

    for (int t = 0; t < N - 2; ++t) {
      fg[0] += 2 * CppAD::pow(vars[delta_start + t + 1] - 
                              vars[delta_start + t], 2);
      fg[0] += 5 * CppAD::pow(vars[a_start + t + 1] - 
                              vars[a_start + t], 2);
    }

    // Constraints
    fg[1 + x_start] = vars[x_start];
    fg[1 + y_start] = vars[y_start];
    fg[1 + psi_start] = vars[psi_start];
    fg[1 + v_start] = vars[v_start];
    fg[1 + cte_start] = vars[cte_start];
    fg[1 + epsi_start] = vars[epsi_start];

    for (int t = 1; t < N; ++t) {
      // The state at time t + 1
      AD<double> x1 = vars[x_start + t];
      AD<double> y1 = vars[y_start + t];
      AD<double> psi1 = vars[psi_start + t];
      AD<double> v1 = vars[v_start + t];
      AD<double> cte1 = vars[cte_start + t];
      AD<double> epsi1 = vars[epsi_start + t];

      // The state at time t
      AD<double> x0 = vars[x_start + t - 1];
      AD<double> y0 = vars[y_start + t - 1];
      AD<double> psi0 = vars[psi_start + t - 1];
      AD<double> v0 = vars[v_start + t - 1];
      AD<double> cte0 = vars[cte_start + t - 1];
      AD<double> epsi0 = vars[epsi_start + t - 1];

      // Account for latency
      AD<double> delta0 = vars[delta_start + t - 1];
      AD<double> a0 = vars[a_start + t - 1];

      if (t > 1) {
        delta0 = vars[delta_start + t - 2];
        a0 = vars[a_start + t - 2];
      }

      AD<double> f0 = (coeffs[0] + coeffs[1] * x0 + 
                       coeffs[2] * CppAD::pow(x0, 2) + 
                       coeffs[3] * CppAD::pow(x0, 3));
      AD<double> psides0 = (CppAD::atan(coeffs[1] + 2 * coeffs[2] * x0 + 
                            3 * coeffs[3] * CppAD::pow(x0, 2)));

      fg[1 + x_start + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
      fg[1 + y_start + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
      fg[1 + psi_start + t] = psi1 - (psi0 - v0 * delta0 / Lf * dt);
      fg[1 + v_start + t] = v1 - (v0 + a0 * dt);
      fg[1 + cte_start + t] = cte1 - (f0 - y0 + v0 * CppAD::sin(epsi0) * dt);
      fg[1 + epsi_start + t] = epsi1 - (psi0 - psides0 - v0 * delta0 / Lf * dt);
    }
  }
};

// MPC class definition
MPC::MPC() {}

MPC::~MPC() {}

std::vector<double> MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs) {
  using Dvector = CPPAD_TESTVECTOR(double);
  
  bool ok = true;
  size_t n_vars = 6 * N + 2 * (N - 1);
  size_t n_constraints = 6 * N;

  Dvector vars(n_vars);
  Dvector vars_lower(n_vars);
  Dvector vars_upper(n_vars);

  for (int i = 0; i < n_vars; ++i) {
    vars[i] = 0;
  }

  // Lower and upper limits for variables.
  for (int i = 0; i < delta_start; ++i) {
    vars_lower[i] = -1.0e19;
    vars_upper[i] = 1.0e19;
  }

  for (int i = delta_start; i < a_start; ++i) {
    vars_lower[i] = -0.436332;
    vars_upper[i] = 0.436332;
  }

  for (int i = a_start; i < n_vars; ++i) {
    vars_lower[i] = -1.0;
    vars_upper[i] = 1.0;
  }

  Dvector constraints_lower(n_constraints);
  Dvector constraints_upper(n_constraints);

  // Lower and upper limits for constraints
  for (int i = 0; i < n_constraints; i++) {
    constraints_lower[i] = 0;
    constraints_upper[i] = 0;
  }

  // Initial state
  vars[x_start] = (constraints_lower[x_start] = 
                   constraints_upper[x_start] = state[0]);
  vars[y_start] = (constraints_lower[y_start] = 
                   constraints_upper[y_start] = state[1]);
  vars[psi_start] = (constraints_lower[psi_start] = 
                     constraints_upper[psi_start] = state[2]);
  vars[v_start] = (constraints_lower[v_start] = 
                   constraints_upper[v_start] = state[3]);
  vars[cte_start] = (constraints_lower[cte_start] = 
                     constraints_upper[cte_start] = state[4]);
  vars[epsi_start] = (constraints_lower[epsi_start] = 
                      constraints_upper[epsi_start] = state[5]);

  // Object that computes objective and constraints
  FG_eval fg_eval(coeffs);

  // Options for IPOPT solver
  std::string options;

  // Uncomment this if you'd like more print information
  options += "Integer print_level  0\n";

  // NOTE: Setting sparse to true allows the solver to take advantage
  // of sparse routines, this makes the computation MUCH FASTER. You
  // can uncomment 1 of these and see if it makes a difference or not, but
  // if you uncomment both, the computation time should go up in orders of
  // magnitude.
  options += "Sparse  true        forward\n";
  options += "Sparse  true        reverse\n";

  // NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
  // Change this as you see fit.
  options += "Numeric max_cpu_time          0.5\n";

  CppAD::ipopt::solve_result<Dvector> solution;

  CppAD::ipopt::solve<Dvector, FG_eval>(
      options, vars, vars_lower, vars_upper, constraints_lower,
      constraints_upper, fg_eval, solution);

  // Check some of the solution values
  ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

  auto cost = solution.obj_value;
  std::cout << "Cost " << cost << std::endl;

  // First actuations (steering [deg], throttle)
  return {solution.x[delta_start], solution.x[a_start]};
}
