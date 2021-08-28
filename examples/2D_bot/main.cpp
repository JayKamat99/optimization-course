#include <iostream>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/ProblemDefinition.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/multilevel/planners/multimodal/LocalMinimaSpanners.h>

#include <ompl/config.h>

#include <KOMO/komo.h>
// #include <Kin/viewer.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;
namespace om = ompl::multilevel;

struct ValidityCheckWithKOMO {
	KOMO::Conv_KOMO_SparseNonfactored &nlp;
	ValidityCheckWithKOMO(KOMO::Conv_KOMO_SparseNonfactored &nlp) : nlp(nlp){}
	bool check(const ob::State *state)
	{
		const auto *State = state->as<ob::SE2StateSpace::StateType>();

		arr x_query =
			arr{ State->getX(), State->getY(), State->getYaw() };

		arr phi;
		nlp.evaluate(phi, NoArr, x_query);
		double tol = 1e-2;

		return std::abs(phi(0)) < tol;
	}
};

void plan()
{
	// construct the state space we are planning in
	auto space(std::make_shared<ob::SE2StateSpace>());

	// set the bounds
	ob::RealVectorBounds bounds(2);
	bounds.setLow(-2);
	bounds.setHigh(2);
	space->setBounds(bounds);

	// Create a text string, which is used to output the text file
	std::string filename;
	ifstream MyReadFile("/home/jay/git/optimization-course/examples/Models/Configuration.txt");
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
	start = {0,0,0};
	// start[0] = komo.getConfiguration_q(0).elem(0);
	// start[1] = komo.getConfiguration_q(0).elem(1);
	// start[2] = komo.getConfiguration_q(0).elem(2);

	std::cout << start << std::endl;

    // create a goal state
    ob::ScopedState<> goal(space);
    goal[0] = 1;
    goal[1] = 1;
	goal[2] = 0;

    // create an instance of problem definition
    auto pdef(std::make_shared<ob::ProblemDefinition>(si));

    // Set the start and goal states
    pdef->setStartAndGoalStates(start, goal);

	// Define planner
    std::cout << "\nUsing Local Minima Spanner:" << std::endl;
    std::vector<ob::SpaceInformationPtr> siVec;
    siVec.push_back(si);
    auto planner = std::make_shared<om::LocalMinimaSpanners>(siVec);
    planner->setProblemDefinition(pdef);
    planner->setup();

	// attempt to solve the problem within ten seconds of planning time
    ob::PlannerStatus solved = planner->ob::Planner::solve(20.0);
    if (true)
    {
        std::cout << "Found solution:" << std::endl;
        auto localMinimaTree = planner->getLocalMinimaTree();
        int NumberOfMinima =  (int)localMinimaTree->getNumberOfMinima();
        int NumberOfLevels =  (int)localMinimaTree->getNumberOfLevel();
        // std::ofstream out;
        // out.open("Paths_out.txt", std::ios_base::app);
        // out << NumberOfMinima*NumberOfLevels << "\n";
        for (int i=0; i<NumberOfLevels; i++){
            for (int j=0; j<NumberOfMinima; j++){
                std::cout << "\n New path[" << i << j << "] \n" << std::endl;
                // std::dynamic_pointer_cast<ompl::geometric::PathGeometric>(localMinimaTree->getPath(i,j)->asPathPtr())->print(std::cout);
                // std::dynamic_pointer_cast<ompl::geometric::PathGeometric>(localMinimaTree->getPath(i,j)->asPathPtr())->printAsMatrix(out);
            }
        }
    }
    else
        std::cout << "No solution found" << std::endl;
	std::cout << "I reached the end!" << std::endl;
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

	plan();
	std::cout << "I came out of Plan!" << std::endl;
	return 0;
}