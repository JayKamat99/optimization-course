#include <iostream>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/ProblemDefinition.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/multilevel/planners/multimodal/LocalMinimaSpanners.h>

#include <ompl/geometric/PathOptimizerKOMO.h>
#include <ompl/geometric/PathSimplifier.h>

#include <ompl/config.h>

#include <KOMO/komo.h>
#include <Kin/viewer.h>

#define PI 3.1412

namespace ob = ompl::base;
namespace og = ompl::geometric;
namespace om = ompl::multilevel;

std::string filename;
unsigned int C_Dimension;

struct ValidityCheckWithKOMO {
	KOMO::Conv_KOMO_SparseNonfactored &nlp;
	ValidityCheckWithKOMO(KOMO::Conv_KOMO_SparseNonfactored &nlp) : nlp(nlp){}
	bool check(const ob::State *state)
	{
		const auto *State = state->as<ob::RealVectorStateSpace::StateType>();

		arr x_query;
		for (unsigned int i = 0; i < C_Dimension; i++){
			x_query.append(State->operator[](i));
		}

		// arr x_query =
		// 	arr{ State->operator[](0), State->operator[](1), State->operator[](2) };

		arr phi;
		nlp.evaluate(phi, NoArr, x_query);
		double tol = 1e-2;

		return std::abs(phi(0)) < tol;
	}
};

void VisualizePath(arrA configs){
	// setup KOMO
    rai::Configuration C;
    C.addFile(filename.c_str());
    KOMO komo;
    komo.verbose = 0;
    komo.setModel(C, true);
    
    komo.setTiming(1., configs.N, 5., 2);
	komo.add_qControlObjective({}, 1, 50.);

    //use configs to initialize with waypoints
    komo.initWithWaypoints(configs, configs.N, false);
    komo.run_prepare(0);
    komo.view(true);
    komo.view_play(true);

	// rai::ConfigurationViewer V;
	// // std::cout << komo.x << std::endl;
	// V.setPath(C, komo.x, "result", true);
	// while(V.playVideo(true));
}

void komoOptimize()
{
	// Create a text string, which is used to output the text file
	ifstream MyReadFile("../Models/Configuration.txt");
	getline (MyReadFile, filename);
	MyReadFile.close(); 

	// set state validity checking based on KOMO
	rai::Configuration C;
	C.addFile(filename.c_str());
	KOMO komo;
	komo.setModel(C, true);
	komo.setTiming(1, 30, 5, 1);
	komo.add_qControlObjective({}, 1, 50.);
	komo.run_prepare(0);
	komo.addObjective({}, FS_accumulatedCollisions, {}, OT_eq, { 1 });
	komo.addObjective({1.}, FS_positionDiff, {"tool0_joint", "ball"}, OT_eq, {1e1});
	komo.add_collision(true);

	komo.optimize();
	std::cout << komo.getPath_q() << std::endl;
	komo.plotTrajectory();
	komo.view(true);
	komo.view_play(true);
}

void plan()
{
	// Create a text string, which is used to output the text file
	ifstream MyReadFile("../Models/Configuration.txt");
	getline (MyReadFile, filename);
	MyReadFile.close(); 

	// set state validity checking based on KOMO
	rai::Configuration C;
	C.addFile(filename.c_str());
	KOMO komo;
	komo.setModel(C, true);
	komo.setTiming(1, 1, 1, 1);
	komo.addObjective({}, FS_accumulatedCollisions, {}, OT_eq, { 1 });
	komo.run_prepare(0);

	C_Dimension = C.getJointStateDimension();

	//Construct the state space we are planning in
	auto space(std::make_shared<ob::RealVectorStateSpace>(C.getJointStateDimension()));

	ob::RealVectorBounds bounds(C.getJointStateDimension());
	bounds.setLow(-PI);
	bounds.setHigh(PI);
	space->setBounds(bounds);

	// create instance of space information
    auto si(std::make_shared<ob::SpaceInformation>(space));

    // set state validity checking for this space
	auto nlp = std::make_shared<KOMO::Conv_KOMO_SparseNonfactored>(komo, false);
	ValidityCheckWithKOMO checker(*nlp);

	si->setStateValidityChecker([&checker](const ob::State *state) {
		return checker.check(state);
	});

    // create a start state
    ob::ScopedState<> start(space);
	for (unsigned int i=0; i<C.getJointStateDimension(); i++){
		start[i] = komo.getConfiguration_q(0).elem(i);
	}

	std::cout << start << std::endl;

    // create a goal state
    ob::ScopedState<> goal(space);
	for (unsigned int i=0; i<C.getJointStateDimension(); i++){
		if (i>3)	continue;
		goal[i] = komo.getConfiguration_q(0).elem(i)+1.5;
	}
	// goal = {0.14, 0.70, 0.9, -1.33, 0, 0.4, 0};

	std::cout << goal << std::endl;

    // create an instance of problem definition
    auto pdef(std::make_shared<ob::ProblemDefinition>(si));

    // Set the start and goal states
    pdef->setStartAndGoalStates(start, goal);

	//Define optimizer
	// og::PathOptimizerPtr optimizer = std::make_shared<og::PathSimplifier>(si);
	og::PathOptimizerPtr optimizer = std::make_shared<og::PathOptimizerKOMO>(si);

	// Define planner
    std::cout << "\nUsing Local Minima Spanner:" << std::endl;
    std::vector<ob::SpaceInformationPtr> siVec;
    siVec.push_back(si);
    auto planner = std::make_shared<om::LocalMinimaSpanners>(siVec);
    planner->setProblemDefinition(pdef);
	planner->setOptimizer(optimizer);
    planner->setup();

	// attempt to solve the problem within sixty seconds of planning time
    ob::PlannerStatus solved = planner->ob::Planner::solve(10.0);
    if (solved || !solved)
    {
		std::cout << "Found solution:" << std::endl;
		auto localMinimaTree = planner->getLocalMinimaTree();
		int NumberOfMinima =  (int)localMinimaTree->getNumberOfMinima();
		int NumberOfLevels =  (int)localMinimaTree->getNumberOfLevel();

		for (int i=0; i<NumberOfLevels; i++){
			for (int j=0; j<NumberOfMinima; j++){
				std::cout << "\nNew path[" << i << j+1 << "] \n" << std::endl;
				auto path = std::dynamic_pointer_cast<ompl::geometric::PathGeometric>(localMinimaTree->getPath(i,j)->asPathPtr());
				//convert path to arrA
				arrA configs;
				for (auto state : (*path).getStates())
				{
					arr config;
					std::vector<double> reals;
					space->copyToReals(reals, state);
					for (double r : reals){
						config.append(r);
					}
					configs.append(config);
				}
				//Visualize in KOMO
				VisualizePath(configs);
				std::dynamic_pointer_cast<ompl::geometric::PathGeometric>(localMinimaTree->getPath(i,j)->asPathPtr())->print(std::cout);
			}
		}
    }
    else
        std::cout << "No solution found" << std::endl;
}

void visualize_random()
{
	auto filename = "/home/jay/git/optimization-course/examples/Models/2D_bot.g";
	rai::Configuration C;
	C.addFile(filename);
	KOMO komo;
	komo.setModel(C, true);
	komo.setTiming(1, 1, 1, 1);
	komo.addObjective({}, FS_accumulatedCollisions, {}, OT_eq, { 1 });
	komo.run_prepare(0);

	auto nlp = std::make_shared<KOMO::Conv_KOMO_SparseNonfactored>(komo,
								       false);

	ObjectiveTypeA ot;
	nlp->getFeatureTypes(ot);
	C.watch(true);

	arr phi;
	size_t N = 20;
	for (size_t i = 0; i < N; i++) {
		arr pose = rand(3);
		C.setJointState(pose);
		nlp->evaluate(phi, NoArr, pose);
		std::cout << pose << " " << phi << std::endl;
		std::cout << komo.getConfiguration_q(0) << std::endl;
		C.watch(true);
	}
}

int main(int /*argc*/, char ** /*argv*/)
{
	/// \brief visualize_random samples random orientations and also checks
	/// for collissions.
	// visualize_random();
	// komoOptimize();
	plan();
	return 0;
}