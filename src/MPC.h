#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"

class MPC {
public:
  MPC();
  virtual ~MPC();

  // Returns first actuations given 
  // an initial state and polynomial coefficients
  std::vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);
};

#endif  // MPC_H
