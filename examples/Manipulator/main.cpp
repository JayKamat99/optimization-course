#include <iostream>
#include <functional>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/ProblemDefinition.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/est/EST.h>
#include <ompl/multilevel/planners/multimodal/LocalMinimaSpanners.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/tools/benchmark/Benchmark.h>

#include <ompl/base/samplers/ObstacleBasedValidStateSampler.h>
#include <ompl/base/samplers/GaussianValidStateSampler.h>

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
			x_query.append((*State)[i]);
		}

		arr phi;
		nlp.evaluate(phi, NoArr, x_query);
		double tol = 1e-2;

		return std::abs(phi(0)) < tol;
	}
};

ob::ValidStateSamplerPtr allocOBValidStateSampler(const ob::SpaceInformation *si)
{
    return std::make_shared<ob::ObstacleBasedValidStateSampler>(si);
}

ob::ValidStateSamplerPtr allocGaussianValidStateSampler(const ob::SpaceInformation *si)
{
    return std::make_shared<ob::GaussianValidStateSampler>(si);
}

void VisualizePath(arrA configs){
	static int Trajectory = 1;
	// setup KOMO
    rai::Configuration C;
    C.addFile(filename.c_str());
    KOMO komo;
    komo.verbose = 0;
    komo.setModel(C, true);
    
    komo.setTiming(1., configs.N, 5., 2);
	komo.add_qControlObjective({}, 1, 1.);

    // std::cout << configs << std::endl;
    komo.addObjective({1.}, FS_qItself, {}, OT_eq, {10}, configs(configs.N-1), 0);
    komo.addObjective({}, FS_accumulatedCollisions, {}, OT_eq, {1.});
    komo.add_collision(true);

    //use configs to initialize with waypoints
	komo.initWithWaypoints(configs, configs.N, false);
    komo.run_prepare(0);
	komo.plotTrajectory();
	std::string SaveToPath = std::string("z.vid/DisplayTrajectory_") + std::to_string(Trajectory) + "/";

	rai::ConfigurationViewer V;
	V.setPath(C, komo.x, "result", true);
	V.playVideo(true, 1., SaveToPath.c_str());
	Trajectory ++;
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

// og::SimpleSetup createSimpleSetup()
// {
// 	return ss;
// }

void plan()
{
	// Create a text string, which is used to output the text file
	// ifstream MyReadFile("../Models/Configuration.txt");
	ifstream MyReadFile("/home/jay/mt-multimodal_optimization/Models/Configuration.txt");
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
	auto space(std::make_shared<ob::RealVectorStateSpace>(C_Dimension));

	ob::RealVectorBounds bounds(C_Dimension);
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

	si->setValidStateSamplerAllocator(allocOBValidStateSampler);
	// si->setValidStateSamplerAllocator(allocGaussianValidStateSampler);

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
	goal = {-1,-1};

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
    // planner->setup();
	og::SimpleSetup ss(si);
	ss.setPlanner(planner);
	ss.setStartAndGoalStates(start,goal);
	ss.setup();

	// attempt to solve the problem within sixty seconds of planning time
    // ob::PlannerStatus solved = planner->ob::Planner::solve(20.0);
	ob::PlannerStatus solved = ss.solve(10.0);

    if (solved == ob::PlannerStatus::StatusType::APPROXIMATE_SOLUTION)
		std::cout << "Found solution: APPROXIMATE_SOLUTION" << std::endl;
	else if (solved == ob::PlannerStatus::StatusType::TIMEOUT)
		std::cout << "Found solution: TIMEOUT" << std::endl;
	else{
		std::cout << "No solution found: Invalid " << std::endl;
		return;
 	}
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

ompl::base::PlannerPtr myConfiguredPlanner(const ompl::base::SpaceInformationPtr &si)
{
    og::EST *est = new og::EST(si);
    est->setRange(100.0);
    return ompl::base::PlannerPtr(est);
	// og::PathOptimizerPtr optimizer = std::make_shared<og::PathOptimizerKOMO>(si);
    // std::vector<ob::SpaceInformationPtr> siVec;
    // siVec.push_back(si);
    // auto planner = std::make_shared<om::LocalMinimaSpanners>(siVec);
	// planner->setOptimizer(optimizer);
	// return ompl::base::PlannerPtr(planner);
}

void benchmark()
{
	bool benchmark = false;
	bool PathOptimizer = true; //this does not matter incase of benchmark
	// Create a text string, which is used to output the text file
	// ifstream MyReadFile("../Models/Configuration.txt");
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
	auto space(std::make_shared<ob::RealVectorStateSpace>(C_Dimension));

	ob::RealVectorBounds bounds(C_Dimension);
	bounds.setLow(-PI);
	bounds.setHigh(PI);
	space->setBounds(bounds);

	//create simple setup
	og::SimpleSetup ss(space);

    // set state validity checking for this space
	auto nlp = std::make_shared<KOMO::Conv_KOMO_SparseNonfactored>(komo, false);
	ValidityCheckWithKOMO checker(*nlp);

	ss.setStateValidityChecker([&checker](const ob::State *state) {
		return checker.check(state);
	});

	// si->setValidStateSamplerAllocator(allocOBValidStateSampler);
	// si->setValidStateSamplerAllocator(allocGaussianValidStateSampler);

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
	// goal = {2,0};

	std::cout << goal << std::endl;

    // create an instance of problem definition
    // auto pdef(std::make_shared<ob::ProblemDefinition>(si));

    // Set the start and goal states
    ss.setStartAndGoalStates(start, goal);
	// auto si = ss.getSpaceInformation();

	// //Define optimizer
	// // og::PathOptimizerPtr optimizer = std::make_shared<og::PathSimplifier>(si);
	// og::PathOptimizerPtr optimizer = std::make_shared<og::PathOptimizerKOMO>(si);


	if(benchmark)
	{
		// First we create a benchmark class:
		ompl::tools::Benchmark b(ss, "my experiment");

		auto si = ss.getSpaceInformation();
		std::vector<ob::SpaceInformationPtr> siVec;
		siVec.push_back(si);
		//RRTstar
		auto planner1(std::make_shared<og::RRTstar>(si));
		b.addPlanner(planner1);
		//MyPlanner
		// auto planner(std::make_shared<om::LocalMinimaSpanners>(siVec));
		// og::PathOptimizerPtr optimizer = std::make_shared<og::PathOptimizerKOMO>(si);
		// planner->setOptimizer(optimizer);
		// b.addPlanner(planner);
		
		// For planners that we want to configure in specific ways,
		// the ompl::base::PlannerAllocator should be used:
		// b.addPlannerAllocator(std::bind(&myConfiguredPlanner, std::placeholders::_1));
		// add post run events
		// collect costs over time
		// etc.
		
		// Now we can benchmark: 5 second time limit for each plan computation,
		// 100 MB maximum memory usage per plan computation, 50 runs for each planner
		// and true means that a text-mode progress bar should be displayed while
		// computation is running.
		ompl::tools::Benchmark::Request req;
		req.maxTime = 5.0;
		req.maxMem = 100.0;
		req.runCount = 1;
		req.displayProgress = true;
		b.benchmark(req);
		
		// This will generate a file of the form ompl_host_time.log
		b.saveResultsToFile();
	}

	else{
		auto si = ss.getSpaceInformation();
		std::vector<ob::SpaceInformationPtr> siVec;
		siVec.push_back(si);
		auto planner1 = std::make_shared<om::LocalMinimaSpanners>(siVec);

		if (PathOptimizer){
			og::PathOptimizerPtr optimizer = std::make_shared<og::PathOptimizerKOMO>(si);
			planner1->setOptimizer(optimizer);
			ss.setPlanner(planner1);
		}
		else{
			auto planner(std::make_shared<og::RRTstar>(si));
			ss.setPlanner(planner);
		}
		
		ss.setup();

		// attempt to solve the problem
		ob::PlannerStatus solved = ss.solve(10.0);

		if (solved == ob::PlannerStatus::StatusType::APPROXIMATE_SOLUTION)
			std::cout << "Found solution: APPROXIMATE_SOLUTION" << std::endl;
		else if (solved == ob::PlannerStatus::StatusType::EXACT_SOLUTION)
			std::cout << "Found solution: EXACT_SOLUTION" << std::endl;
		else if (solved == ob::PlannerStatus::StatusType::TIMEOUT)
			std::cout << "Found solution: TIMEOUT" << std::endl;
		else{
			std::cout << "No solution found: Invalid " << std::endl;
			return;
		}

		if(PathOptimizer){ //This code is for visualization of the paths from PathOptimizer
			auto localMinimaTree = planner1->getLocalMinimaTree();
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
		else{// This is for visualization of paths from other planners
			auto path = ss.getSolutionPath();
			arrA configs;
			for (auto state : path.getStates())
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
		}
	}
}

int main(int /*argc*/, char ** /*argv*/)
{
	/// \brief visualize_random samples random orientations and also checks
	/// for collissions.
	// plan();
	std::cout << "OMPL version: " << OMPL_VERSION << std::endl;
	benchmark();
	return 0;
}