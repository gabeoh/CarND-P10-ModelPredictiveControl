#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"

using CppAD::AD;
using namespace std;

// Set the timestep length and duration
size_t N = 25;
double dt = 0.05;

// Set number of steps to cover the latency
// 100ms = 50ms * 2
size_t N_latency = 2;

// This value assumes the model presented in the classroom is used.
//
// It was obtained by measuring the radius formed by running the vehicle in the
// simulator around in a circle with a constant steering angle and velocity on a
// flat terrain.
//
// Lf was tuned until the the radius formed by the simulating the model
// presented in the classroom matched the previous radius.
//
// This is the length from front to CoG that has a similar radius.
const double Lf = 2.67;

// Referential speed value
double v_ref = 50;

// The solver takes all the state variables and actuator
// variables in a singular vector. Thus, we should to establish
// when one variable starts and another ends to make our lifes easier.
const size_t x_start = 0;
const size_t y_start = x_start + N;
const size_t psi_start = y_start + N;
const size_t v_start = psi_start + N;
const size_t cte_start = v_start + N;
const size_t epsi_start = cte_start + N;
const size_t delta_start = epsi_start + N;
const size_t a_start = delta_start + N - 1;

class FG_eval {
public:
  // Fitted polynomial coefficients
  Eigen::VectorXd coeffs;
  FG_eval(Eigen::VectorXd coeffs) { this->coeffs = coeffs; }

  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
  void operator()(ADvector& fg, const ADvector& vars) {
    // Implement MPC
    // `fg` is a vector of the cost constraints, `vars` is a vector of variable values (state & actuators)

    //////////////////////////
    // Compute cost
    //////////////////////////
    //
    // The cost contributions are adjusted (normalized) so that each factor has meaningful impacts
    // on the overall cost.
    // First, the magnitude of the speed value is much bigger than other factors such as CTE and
    // angular error (ePsi).  Thus, the cost contributions from the speed discrepancy is heavily
    // discounted.
    // Also, the magnitude of angular components such as ePsi and delta is smaller compared to
    // the CTE and the acceleration values.  Thus, the larger multiplication factors are applied
    // to the angular components.
    // Lastly, through the experiments, it was discovered that the model has a strong tendency
    // to optimize the cost by maneuvering the steering wheel than the acceleration.  This caused
    // the vehicle to move in the wobbly manner.  In the real life, it is more common (and safe)
    // to reduce speed than to perform crazy steering maneuver in order to track sharp turns.
    // In order to achieve this, heavy penalties on the large delta and the delta differentials,
    // large reduction on the acceleration contributions, and further reduction on the speed
    // discrepancy are applied.
    //
    // Since the variables in the latency steps are not affected by the current actuator values,
    // they are excluded from the cost computations (ie. skip vars in [0, ..., N_latency) )
    fg[0] = 0.0;
    // Compute cost factors from state
    for (size_t t = N_latency; t < N; ++t) {
      fg[0] += (CppAD::pow(vars[cte_start + t], 2) * 40.0);
      fg[0] += (CppAD::pow(vars[epsi_start + t], 2) * 80.0);
      fg[0] += (CppAD::pow(vars[v_start + t] - v_ref, 2) / 400.0);
    }
    // Compute cost factors from actuators - penalize large actuator values
    for (size_t t = N_latency; t < N - 1; ++t) {
      fg[0] += (CppAD::pow(vars[delta_start + t], 2) * 40.0);
      fg[0] += (CppAD::pow(vars[a_start + t], 2) / 40.0);
    }
    // Compute cost factors from actuator differentials - smoothen the actuator changes
    int actuator_diff_start = 0;
    if (N_latency > 0)
      actuator_diff_start = N_latency - 1;
    for (size_t t = actuator_diff_start; t < N - 2; ++t) {
      fg[0] += (CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2) * 400.0);
      fg[0] += (CppAD::pow(vars[a_start + t + 1] - vars[a_start + t], 2) / 40.0);
    }

    // Set constraints for initial states
    fg[1 + x_start] = vars[x_start];
    fg[1 + y_start] = vars[y_start];
    fg[1 + psi_start] = vars[psi_start];
    fg[1 + v_start] = vars[v_start];
    fg[1 + cte_start] = vars[cte_start];
    fg[1 + epsi_start] = vars[epsi_start];

    // Set constraints for delta and a during actuation latency
    for (size_t i = 0; i < N_latency; ++i) {
      fg[1 + delta_start + i] = vars[delta_start + i];
      // Actuation constraints only exist for delayed periods
      // Thus, the start of acceleration constraint index is 'delta_start + N_latency' instead of 'a_start'
      fg[1 + delta_start + N_latency + i] = vars[a_start + i];
    }

    // Set constraints for subsequent time frames
    for (size_t t = 1; t < N; ++t) {
      AD<double> x0 = vars[x_start + t - 1];
      AD<double> y0 = vars[y_start + t - 1];
      AD<double> psi0 = vars[psi_start + t - 1];
      AD<double> v0 = vars[v_start + t - 1];
      AD<double> cte0 = vars[cte_start + t - 1];
      AD<double> epsi0 = vars[epsi_start + t - 1];
      AD<double> delta0 = vars[delta_start + t - 1];
      AD<double> a0 = vars[a_start + t - 1];

      AD<double> x1 = vars[x_start + t];
      AD<double> y1 = vars[y_start + t];
      AD<double> psi1 = vars[psi_start + t];
      AD<double> v1 = vars[v_start + t];
      AD<double> cte1 = vars[cte_start + t];
      AD<double> epsi1 = vars[epsi_start + t];

      // Set model constraints
      fg[1 + x_start + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
      fg[1 + y_start + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
      fg[1 + psi_start + t] = psi1 - (psi0 + v0 / Lf * delta0 * dt);
      fg[1 + v_start + t] = v1 - (v0 + a0 * dt);

      AD<double> fx0 = coeffs[0] + coeffs[1] * x0 + coeffs[2] * CppAD::pow(x0, 2) + coeffs[3] * CppAD::pow(x0, 3);
      AD<double> fx0_d = coeffs[1] + 2 * coeffs[2] * x0 + 3 * coeffs[3] * CppAD::pow(x0, 2);
      AD<double> psi0_des = CppAD::atan(fx0_d);
      fg[1 + cte_start + t] = cte1 - (fx0 - y0 + v0 * CppAD::sin(epsi0) * dt);
      fg[1 + epsi_start + t] = epsi1 - (psi0 - psi0_des + v0/Lf * delta0 * dt);
    }
  }
};

//
// MPC class definition implementation.
//
MPC::MPC(): delayed_steerings(N_latency, 0.0), delayed_accelerations(N_latency, 0.0) {}
MPC::~MPC() {}

vector<double> MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs) {
  bool ok = true;
  size_t i;
  typedef CPPAD_TESTVECTOR(double) Dvector;

  // Set the number of model variables (includes both states and inputs).
  // 6 state with 2 actuators
  size_t n_vars = 6 * N + 2 * (N - 1);
  // Set the number of constraints
  size_t n_constraints = 6 * N + 2 * N_latency;

  // Initial value of the independent variables.
  // SHOULD BE 0 besides initial state.
  Dvector vars(n_vars);
  for (i = 0; i < n_vars; i++) {
    vars[i] = 0;
  }

  // Set initial vars with the given initial state
  vars[x_start] = state[0];
  vars[y_start] = state[1];
  vars[psi_start] = state[2];
  vars[v_start] = state[3];
  vars[cte_start] = state[4];
  vars[epsi_start] = state[5];

  // cout << "# Cost from the vehicle state (CTE, ePsi, V): "
  //     << vars[cte_start] << "," << vars[epsi_start] << "," << (vars[v_start] - v_ref) << endl;

  // Set lower and upper limits for variables
  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);
  // Set all non-actuators upper and lower limits to the max negative and positive values
  for (i = 0; i < delta_start; i++) {
    vars_lowerbound[i] = -1.0e19;
    vars_upperbound[i] = 1.0e19;
  }
  // The upper and lower limits of delta are set to -25 and 25 degrees (values in radians)
  // 25 * PI / 180 = 0.436332
  for (i = delta_start; i < a_start; i++) {
    vars_lowerbound[i] = -0.436332;
    vars_upperbound[i] = 0.436332;
  }
  // Acceleration/decceleration upper and lower limits.
  for (i = a_start; i < n_vars; i++) {
    vars_lowerbound[i] = -1.0;
    vars_upperbound[i] = 1.0;
  }

  // Lower and upper limits for the constraints
  // Should be 0 besides initial state.
  Dvector constraints_lowerbound(n_constraints);
  Dvector constraints_upperbound(n_constraints);
  for (i = 0; i < n_constraints; i++) {
    constraints_lowerbound[i] = 0;
    constraints_upperbound[i] = 0;
  }
  // Set constraint lower bound for initial states
  constraints_lowerbound[x_start] = state[0];
  constraints_lowerbound[y_start] = state[1];
  constraints_lowerbound[psi_start] = state[2];
  constraints_lowerbound[v_start] = state[3];
  constraints_lowerbound[cte_start] = state[4];
  constraints_lowerbound[epsi_start] = state[5];
  // Set constraint upper bound for initial states
  constraints_upperbound[x_start] = state[0];
  constraints_upperbound[y_start] = state[1];
  constraints_upperbound[psi_start] = state[2];
  constraints_upperbound[v_start] = state[3];
  constraints_upperbound[cte_start] = state[4];
  constraints_upperbound[epsi_start] = state[5];

  // Set constraints for delta and a during actuation latency
  // Actuators (delta and a) for latency steps are already determined and cannot be altered.
  for (i = 0; i < N_latency; ++i) {
    // Set steering values and constraints
    double steering = delayed_steerings[i];
    vars[delta_start + i] = steering;
    constraints_lowerbound[delta_start + i] = steering;
    constraints_upperbound[delta_start + i] = steering;
    // Set acceleration values and constraints
    double acceleration = delayed_accelerations[i];
    vars[a_start + i] = acceleration;
    // Actuation constraints only exist for delayed periods
    // Thus, the start of acceleration constraint index is 'delta_start + N_latency' instead of 'a_start'
    constraints_lowerbound[delta_start + N_latency + i] = acceleration;
    constraints_upperbound[delta_start + N_latency + i] = acceleration;
  }

  // object that computes objective and constraints
  FG_eval fg_eval(coeffs);

  //
  // NOTE: You don't have to worry about these options
  //
  // options for IPOPT solver
  std::string options;
  // Uncomment this if you'd like more print information
  options += "Integer print_level  0\n";
  // NOTE: Setting sparse to true allows the solver to take advantage
  // of sparse routines, this makes the computation MUCH FASTER. If you
  // can uncomment 1 of these and see if it makes a difference or not but
  // if you uncomment both the computation time should go up in orders of
  // magnitude.
  options += "Sparse  true        forward\n";
  options += "Sparse  true        reverse\n";
  // NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
  // Change this as you see fit.
  options += "Numeric max_cpu_time          0.5\n";

  // place to return solution
  CppAD::ipopt::solve_result<Dvector> solution;

  // solve the problem
  CppAD::ipopt::solve<Dvector, FG_eval>(
      options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
      constraints_upperbound, fg_eval, solution);

  // Check some of the solution values
  ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

  // Cost
  auto cost = solution.obj_value;
  std::cout << "Cost " << cost << std::endl;

  // Return the first actuator values and vehicle trajectories
  vector<double> results;
  if (ok) {
    // Update delayed actuation values (left shift)
    double steering_computed = solution.x[delta_start + N_latency];
    double acceleration_computed = solution.x[a_start + N_latency];
    for (i = 0; i < N_latency  - 1; ++i) {
      delayed_steerings[i] = delayed_steerings[i + 1];
      delayed_accelerations[i] = delayed_accelerations[i + 1];
    }
    delayed_steerings[N_latency - 1] = steering_computed;
    delayed_accelerations[N_latency - 1] = acceleration_computed;

    results.push_back(steering_computed);
    results.push_back(acceleration_computed);
    for (i = 0; i < 2 * N; ++i) {
      results.push_back(solution.x[x_start + i]);
    }
  } else {
    results.push_back(0.0);
    results.push_back(0.1);
  }
  return results;
}
