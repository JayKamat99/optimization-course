#include <iostream>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/special/TorusStateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/prm/SPARS.h>

#include <ompl/config.h>

#include <KOMO/komo.h>
#include <Kin/viewer.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;

std::vector<arr> configs;

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

    // std::cout << "Check at: " << x_query << " " << ( std::abs(phi(0)) < tol)
    // nlp->komo.watch(true);

    return std::abs(phi(0)) < tol;
  }
};

void optimizeAndPlayMovie(){
  //This will initialize the waypoints from the SPARS graph and run optimization on it.
  auto filename = "2D.g";
  rai::Configuration C;
  C.addFile(filename);
  KOMO komo;
  komo.setModel(C, true);
  komo.setTiming(1, 10, 1, 2);
  // komo.addObjective({1.}, FS_positionDiff, {"gripper", "ball"}, OT_sos, {1e1});
  komo.addObjective({.98,1.}, FS_qItself, {}, OT_sos, {1e1}, {1,0}, 0);  //small velocity in that time frame
  komo.addObjective({}, FS_accumulatedCollisions, {}, OT_eq, {1});
  komo.run_prepare(0);

  komo.animateOptimization = 1;
  komo.initWithWaypoints({{1,1},{2,2}} , 2, false); //Get these from the previous function
  komo.optimize();
  komo.plotTrajectory();
  komo.checkGradients();
  rai::ConfigurationViewer V;
  V.setPath(C, komo.x, "result", false);
  while(V.playVideo());
}

void planWithSimpleSetupKOMOinT2(){
  //construct the configuration space we will be working in
  auto space(std::make_shared<ob::TorusStateSpace>());

  // define a simple setup class
  og::SimpleSetup ss(space);

  // set state validity checking based on KOMO
  auto filename = "2D.g";
  rai::Configuration C;
  C.addFile(filename);
  KOMO komo;
  komo.setModel(C, true);
  komo.setTiming(1, 1, 1, 1);
  komo.addObjective({}, FS_accumulatedCollisions, {}, OT_eq, {1});
  komo.run_prepare(0);

  auto nlp = std::make_shared<KOMO::Conv_KOMO_SparseNonfactored>(komo, false);
  ValidityCheckWithKOMO checker(*nlp);

  ss.setStateValidityChecker(
      [&checker](const ob::State *state) { return checker.check(state); });

  // create a start state
  ob::ScopedState<> start(space);
  start = std::vector<double>{-1, 0};

  // create a goal state
  ob::ScopedState<> goal(space);
  goal = std::vector<double>{1, 0};

  // set the start and goal states
  ss.setStartAndGoalStates(start, goal);

  //Define planner
  auto planner(std::make_shared<og::SPARS>(ss.getSpaceInformation()));
  ss.setPlanner(planner);

  // attempt to solve the problem within one second of planning time
  ob::PlannerStatus solved = ss.solve(10.0);

  if (solved) {
    std::cout << "Found solution:" << std::endl;
    // print the path to screen
    ss.simplifySolution();   //This uses the shortcutting algo..
    ss.getSolutionPath().print(std::cout); //This prints out the states

    // lets visualize the waypoints of the path (the trajectory is linear
    // interpolation between them)

    auto states = ss.getSolutionPath().getStates();
    for (auto &s : states) {
      const auto *TorusState = s->as<ob::TorusStateSpace::StateType>();

      arr x_query = arr
      {
        TorusState->getS1(),
        TorusState->getS2()
      };
      // std::cout << x_query << std::endl;
      configs.push_back(x_query);
      C.setJointState(x_query);
      C.watch(true);
    }
    for (auto it = configs.begin(); it != configs.end(); it++)
      cout << *it << " ";
  }
  else
    std::cout << "No solution found" << std::endl;
  optimizeAndPlayMovie();
}

int main(int /*argc*/,char** /*argv*/){
  planWithSimpleSetupKOMOinT2();
  optimizeAndPlayMovie();

  return 0;
}