#pragma once
#include "aerial_autonomy/types/constraint.h"
#include <memory>

/**
* @brief Generates obstacles based on sensor data
*/
class AbstractConstraintGenerator {
public:
  /**
  * @brief Generate constraint vector that contains all
  * the dynamics constraints that should be handled
  * for example in an optimization problem.
  *
  * @return vector of constraints
  */
  virtual std::vector<Constraint> generateConstraints() = 0;
  /**
  * @brief Check if constraint generator is fine
  *
  * @return true if constraint status is ok
  */
  virtual bool getStatus() { return true; }
};

using AbstractConstraintGeneratorPtr =
    std::shared_ptr<AbstractConstraintGenerator>;
