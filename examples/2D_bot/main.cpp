#include <iostream>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/special/TorusStateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/prm/SPARS.h>

#include <ompl/config.h>

#include <KOMO/komo.h>
#include <Kin/viewer.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;

struct ValidityCheckWithKOMO {
  KOMO::Conv_KOMO_SparseNonfactored &nlp;
  ValidityCheckWithKOMO(KOMO::Conv_KOMO_SparseNonfactored &nlp) : nlp(nlp) {}
  bool check(const ob::State *state) {

    // rai: uses quaternion with real part first
    // ompl: uses quaternion with real part last
    const auto *TorusState = state->as<ob::TorusStateSpace::StateType>();

    arr x_query = arr
      {
        TorusState->getS1(),
        TorusState->getS2()
      };

    arr phi;
    nlp.evaluate(phi, NoArr, x_query);
    double tol = 1e-2;

    return std::abs(phi(0)) < tol;
  }
};

void visualize_random() {
  auto filename = "2D_bot.g";
  rai::Configuration C;
  std::cout << C.getJointState() << std::endl;
  C.addFile(filename);
  C.watch(true);

  // random configurations
  size_t N = 20;
  for (size_t i = 0; i < N; i++) {
    arr pose = rand(3);
    std::cout << pose << std::endl;
    C.setJointState(pose);
    C.watch(true);
  }
}

int main(int /*argc*/,char** /*argv*/){
  visualize_random();

  return 0;
}