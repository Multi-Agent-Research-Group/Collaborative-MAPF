#include <iostream>
#include <Eigen/Dense>

#include <aikido/perception/AssetDatabase.hpp>
#include <aikido/perception/PoseEstimatorModule.hpp>
#include <aikido/statespace/GeodesicInterpolator.hpp>
#include <aikido/planner/World.hpp>
#include <aikido/rviz/WorldInteractiveMarkerViewer.hpp>
#include <aikido/statespace/dart/MetaSkeletonStateSpace.hpp>
#include <boost/program_options.hpp>
#include <dart/collision/CollisionDetector.hpp>
#include <dart/collision/CollisionGroup.hpp>
#include <dart/collision/CollisionOption.hpp>
#include <dart/dart.hpp>
#include <dart/utils/urdf/DartLoader.hpp>
#include <magi/action/Action.hpp>
#include <magi/action/SequenceAction.hpp>
#include <magi/action/WaitForInputAction.hpp>
#include <pr_tsr/can.hpp>
#include <libherb/Herb.hpp>

namespace po = boost::program_options;

using RobotPtr = std::shared_ptr<herb::Herb>;


using dart::collision::CollisionDetectorPtr;
using dart::collision::CollisionGroup;
using dart::dynamics::SkeletonPtr;

using dart::dynamics::MetaSkeleton;
using dart::dynamics::MetaSkeletonPtr;

using aikido::constraint::TestablePtr;


using aikido::constraint::dart::CollisionFree;
using aikido::constraint::dart::CollisionFreePtr;
using aikido::statespace::dart::MetaSkeletonStateSpace;
using aikido::statespace::dart::MetaSkeletonStateSpacePtr;

using aikido::statespace::GeodesicInterpolator;
using aikido::trajectory::Interpolated;
using aikido::trajectory::InterpolatedPtr;

using magi::action::ActionPtr;

static const std::string dartTopicName("dart_markers");
static const std::string baseFrameName("map");

class HERBUtil
{
public:

	MetaSkeletonStateSpacePtr armSpace;

	aikido::planner::WorldPtr env;

	MetaSkeletonPtr dynamicArm;
	MetaSkeletonPtr staticArm;

	CollisionFreePtr mEnvTestable;
	TestablePtr mFullTestable;

	std::shared_ptr<GeodesicInterpolator> mInterpolator;

	std::vector<SkeletonPtr> obstacles;

	HERBUtil(const std::string& arm_name)
	{
		// waitForUser("Press [ENTER] to exit: ");
		std::cout<<"K"<<std::endl;
		char *argv[] = { "long", "live", "HERB" };
		int argc = 3;
		std::cout << "Starting ROS node." << std::endl;
		ros::init(argc, argv, "HERBUtil");
		ros::NodeHandle nh("~");

		const std::string baseFrameName("map");
		const std::string topicName("dart_markers");
	  
		env = aikido::planner::WorldPtr(new aikido::planner::World("world"));

		const auto resourceRetriever
		 = std::make_shared<aikido::io::CatkinResourceRetriever>();
		ROS_INFO("Loading HERB.");
		herb::Herb robot(env,true);
		
		//adding obstacles in world env

		SkeletonPtr table;
		Eigen::Isometry3d tablePose;
		perceiveObject(nh,resourceRetriever,robot,table,tablePose,"table");

		// SkeletonPtr pitcher;
		// Eigen::Isometry3d pitcherPose;
		// perceiveObject(nh,resourceRetriever,robot,pitcher,pitcherPose,"pitcher");

		SkeletonPtr roof;
		Eigen::Isometry3d roofPose;
		perceiveObject(nh,resourceRetriever,robot,roof,roofPose,"roof");

		// SkeletonPtr shelf;
		// Eigen::Isometry3d shelfPose;
		// perceiveObject(nh,resourceRetriever,robot,shelf,shelfPose,"shelf");

		SkeletonPtr can;
		Eigen::Isometry3d canPose;
		perceiveObject(nh,resourceRetriever,robot,can,canPose,"can");

		SkeletonPtr can2;
		Eigen::Isometry3d can2Pose;
		perceiveObject(nh,resourceRetriever,robot,can2,can2Pose,"can2");

		obstacles.push_back(table);
		obstacles.push_back(roof);
		// obstacles.push_back(pitcher);
		// obstacles.push_back(shelf);
		obstacles.push_back(can);
		obstacles.push_back(can2);


		if(arm_name=="right")
		{
			armSpace = robot.getRightArm()->getStateSpace();

			staticArm = std::const_pointer_cast<MetaSkeleton>(robot.getLeftArm()->getMetaSkeleton());

			dynamicArm = std::const_pointer_cast<MetaSkeleton>(robot.getRightArm()->getMetaSkeleton());

			CollisionDetectorPtr envDetector = dart::collision::FCLCollisionDetector::create();
			mEnvTestable = std::make_shared<CollisionFree>(armSpace, dynamicArm, envDetector);
			std::shared_ptr<CollisionGroup> dynamicArmGroup = envDetector->createCollisionGroup( robot.getRightArm()->getMetaSkeleton().get(),
				robot.getRightHand()->getMetaSkeleton().get());

			std::shared_ptr<CollisionGroup> staticArmGroup = envDetector->createCollisionGroup( robot.getLeftArm()->getMetaSkeleton().get(),
				robot.getLeftHand()->getMetaSkeleton().get());
			
			mEnvTestable->addPairwiseCheck(dynamicArmGroup,staticArmGroup);

			for (SkeletonPtr curObstacle : obstacles)
			{
				std::shared_ptr<CollisionGroup> curObstacleGroup = envDetector->createCollisionGroup(curObstacle.get());
				mEnvTestable->addPairwiseCheck(dynamicArmGroup, curObstacleGroup);
			}

			mFullTestable = robot.getFullCollisionConstraint( armSpace, dynamicArm, mEnvTestable);

			mInterpolator = std::make_shared<GeodesicInterpolator>(armSpace);
		}
		else if(arm_name=="left")
		{
			armSpace = robot.getLeftArm()->getStateSpace();

			dynamicArm = std::const_pointer_cast<MetaSkeleton>(robot.getLeftArm()->getMetaSkeleton());

			staticArm = std::const_pointer_cast<MetaSkeleton>(robot.getRightArm()->getMetaSkeleton());
			
			CollisionDetectorPtr envDetector = dart::collision::FCLCollisionDetector::create();
			mEnvTestable = std::make_shared<CollisionFree>(armSpace, dynamicArm, envDetector);
			std::shared_ptr<CollisionGroup> dynamicArmGroup = envDetector->createCollisionGroup( robot.getLeftArm()->getMetaSkeleton().get(),
				robot.getLeftHand()->getMetaSkeleton().get());
			std::shared_ptr<CollisionGroup> staticArmGroup = envDetector->createCollisionGroup( robot.getRightArm()->getMetaSkeleton().get(),
				robot.getRightHand()->getMetaSkeleton().get());


			mEnvTestable->addPairwiseCheck(dynamicArmGroup,staticArmGroup );

			for (SkeletonPtr curObstacle : obstacles)
			{
				std::shared_ptr<CollisionGroup> curObstacleGroup = envDetector->createCollisionGroup(curObstacle.get());
				mEnvTestable->addPairwiseCheck(dynamicArmGroup, curObstacleGroup);
			}


			mFullTestable = robot.getFullCollisionConstraint( armSpace, dynamicArm, mEnvTestable);

			mInterpolator = std::make_shared<GeodesicInterpolator>(armSpace);
		}
		else
			std::cout<<" [INFO:] ERROR! NO ARM (left/right) SPECIFIED IN CONSTRUCTOR "<<std::endl;

		Eigen::VectorXd relaxedHome(7);
  		relaxedHome << 3.14, -1.57, 0.00, 0.00, 0.00, 0.00, 0.00;
		dynamicArm->setPositions(relaxedHome);
		staticArm->setPositions(relaxedHome);

		// Eigen::VectorXd rRelaxedHome(7);
  		// rRelaxedHome << -0.64, -1.50, -0.26, 1.96, -1.16, 0.87, -1.43;
		// dynamicArm->setPositions(relaxedHome);
		// staticArm->setPositions(rRelaxedHome);

		ros::shutdown();
	}

	void waitForUser(const std::string& msg)
	{
	  std::cout << msg;
	  std::cin.get();
	}

	bool getCollisionStatusVertex(Eigen::VectorXd &config)
	{
		auto state = armSpace->createState();
		armSpace->convertPositionsToState(config, state);	
		if (!mFullTestable->isSatisfied(state))
			return true;
		return false;
	}

	bool getCollisionStatus(Eigen::VectorXd &start, Eigen::VectorXd &end)
	{
		// waitForUser("13Press [ENTER] to exit: ");
		bool col =false;
		auto startState = armSpace->createState();
		armSpace->convertPositionsToState(start, startState);
		// waitForUser("13Press [ENTER] to exit: ");
		auto endState = armSpace->createState();
		armSpace->convertPositionsToState(end, endState);

		double col_resolution = 0.05;
		int qCount = (start-end).norm()/col_resolution;
		if(qCount<2)
			qCount=2;
		double step = 1.0/qCount;
		auto testState = armSpace->createState();
		// waitForUser("13Press [ENTER] to exit: ");
		for (double alpha = 0.0; alpha <= 1.0; alpha += step)
		{
			mInterpolator->interpolate(startState, endState, alpha, testState);
			if (!mFullTestable->isSatisfied(testState))
			{
				col=true;
			}
		}
		// waitForUser("13Press [ENTER] to exit: ");

		return col;
	}

	void executePath(std::vector<Eigen::VectorXd> & configs)
	{
			// Convert the C-Space path to an AIKIDO trajectory and execute it in sim.

		InterpolatedPtr traj = genTrajFromConfigs(configs, armSpace, mInterpolator);

	  // Doing this more than one is frowny face because AIKIDO worlds.
		RobotPtr k = std::make_shared<herb::Herb>(env,true);
		executeSim(k,dynamicArm,env,traj,obstacles);
		std::cout << "[INFO]: Done Executing!" << std::endl;
	}


	const SkeletonPtr makeBodyFromURDF(const std::shared_ptr<aikido::io::CatkinResourceRetriever> resourceRetriever,
		const std::string& uri,const Eigen::Isometry3d& transform)
	{
		dart::utils::DartLoader urdfLoader;
		const SkeletonPtr skeleton = urdfLoader.parseSkeleton(uri, resourceRetriever);

		if (!skeleton)
			throw std::runtime_error("unable to load '" + uri + "'");

		dynamic_cast<dart::dynamics::FreeJoint*>(skeleton->getJoint(0))->setTransform(transform);
		return skeleton;
	}

	bool perceiveObject(ros::NodeHandle nh, const std::shared_ptr<aikido::io::CatkinResourceRetriever> resourceRetriever,
		herb::Herb& robot, dart::dynamics::SkeletonPtr& object, Eigen::Isometry3d& objectPose, std::string name)
	{

		if(name=="table")
		{
			const std::string objectURDFUri("package://pr_assets/data/furniture/uw_demo_table.urdf");
			// Poses for table
			objectPose = Eigen::Isometry3d::Identity();
			// objectPose.translation() = Eigen::Vector3d(1.0, 0.4, 0);
			objectPose.translation() = Eigen::Vector3d(1.0, 0.0, 0);
			Eigen::Matrix3d rot;
			rot = Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitZ())
					* Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY())
					* Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX());
			objectPose.linear() = rot;
			// Load table
			object = makeBodyFromURDF(resourceRetriever, objectURDFUri, objectPose);
			// Add all objects to World
			robot.getWorld()->addSkeleton(object);
			return true;
		}
		else if(name=="shelf")
		{
			const std::string objectURDFUri("package://pr_assets/data/furniture/bookcase.urdf");
			objectPose = Eigen::Isometry3d::Identity();
			objectPose.translation() = Eigen::Vector3d(-0.0, -1.25, 0);
			Eigen::Matrix3d rot;
			rot = Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ())
						* Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY())
						* Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX());
			objectPose.linear() = rot;
			// Load table
			object = makeBodyFromURDF(resourceRetriever, objectURDFUri, objectPose);
			// Add all objects to World
			robot.getWorld()->addSkeleton(object);
			return true;
		}
		else if(name=="can")
		{
			const std::string objectURDFUri("package://pr_assets/data/objects/can.urdf");
			objectPose = Eigen::Isometry3d::Identity();
			objectPose.translation() = Eigen::Vector3d(1.0, 0.3, 0.7);
			Eigen::Matrix3d rot;
			rot = Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ())
						* Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY())
						* Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX());
			objectPose.linear() = rot;
			// Load can
			object = makeBodyFromURDF(resourceRetriever, objectURDFUri, objectPose);
			// Add all objects to World
			robot.getWorld()->addSkeleton(object);
			return true;
		}
		else if(name=="can2")
		{
			const std::string objectURDFUri("package://pr_assets/data/objects/can.urdf");
			objectPose = Eigen::Isometry3d::Identity();
			objectPose.translation() = Eigen::Vector3d(1.0, -0.3, 0.7);
			Eigen::Matrix3d rot;
			rot = Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ())
						* Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY())
						* Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX());
			objectPose.linear() = rot;
			// Load can
			object = makeBodyFromURDF(resourceRetriever, objectURDFUri, objectPose);
			// Add all objects to World
			robot.getWorld()->addSkeleton(object);
			return true;
		}
		else if (name=="pitcher")
		{
			const std::string objectURDFUri("package://pr_assets/data/objects/rubbermaid_ice_guard_pitcher.urdf");
			objectPose = Eigen::Isometry3d::Identity();
			objectPose.translation() = Eigen::Vector3d(1.0, 0.0, 0)+Eigen::Vector3d(0.0, 0.0, 0.73);
			object = makeBodyFromURDF(resourceRetriever, objectURDFUri, objectPose);
			// Add all objects to World
			robot.getWorld()->addSkeleton(object);
			return true;
		}
		else if (name=="roof")
		{
			const std::string objectURDFUri("package://pr_assets/data/furniture/uw_demo_table.urdf");
			// Poses for table
			objectPose = Eigen::Isometry3d::Identity();
			// objectPose.translation() = Eigen::Vector3d(1.0, 0.4, 0);
			objectPose.translation() = Eigen::Vector3d(1.0, 0.0, 0) + Eigen::Vector3d(0.0, 0.0, 2.3);;
			Eigen::Matrix3d rot;
			rot = Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitZ())
        			* Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitY())
        			* Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX());
			objectPose.linear() = rot;
			// Load table
			object = makeBodyFromURDF(resourceRetriever, objectURDFUri, objectPose);
			// Add all objects to World
			robot.getWorld()->addSkeleton(object);
			return true;
		}
		else
			return false;
	}
	InterpolatedPtr genTrajFromConfigs(std::vector<Eigen::VectorXd>& trajConfigs,
		MetaSkeletonStateSpacePtr armStateSpace, std::shared_ptr<GeodesicInterpolator> interpolator)
	{
		if (trajConfigs.size() == 0)
			return nullptr;

		InterpolatedPtr traj = std::make_shared<Interpolated>(
			armStateSpace,
			interpolator);

		for (int i = 0; i < trajConfigs.size(); i++)
		{
			auto& config = trajConfigs.at(i);
			auto state = armStateSpace->createState();
			armStateSpace->convertPositionsToState(config, state);

			traj->addWaypoint(i, state);
		}

		return traj;
	}

	void executeSim(RobotPtr robot, MetaSkeletonPtr arm, aikido::planner::WorldPtr world, 
		const InterpolatedPtr& untimedTraj,std::vector<SkeletonPtr> obstacles)
	{
		// Make sure obstacles show up.
		for (auto obj : obstacles)
		{
			world->addSkeleton(obj);
		}

		auto timedTraj = robot->retimePath(arm, untimedTraj.get());

		// ROS/RViz set-up.
		char *argv[] = { "long", "live", "HERB" };
		int argc = 3;
		std::cout << "Starting ROS node." << std::endl;
		ros::init(argc, argv, "setup_opt_viz");
		ros::NodeHandle nh("~");

		const std::string baseFrameName("map");
		const std::string topicName("dart_markers");

		// Start the RViz viewer.
		std::cout << "Starting viewer. Please subscribe to the '" << topicName
							<< "' InteractiveMarker topic in RViz." << std::endl;
		aikido::rviz::WorldInteractiveMarkerViewer viewer(world, topicName, baseFrameName);

		viewer.setAutoUpdate(true);


		Eigen::VectorXd relaxedHome(7);
  		relaxedHome << 3.14, -1.57, 0.00, 0.00, 0.00, 0.00, 0.00;
		dynamicArm->setPositions(relaxedHome);
		staticArm->setPositions(relaxedHome);

		// Let's go!
		int userIn;
		std::cout << "[soda_handoff] Enter any number to execute: ";
		std::cin >> userIn;

		robot->executeTrajectory(std::move(timedTraj)).wait();

		std::cout << "[soda_handoff] Enter any number to exit: ";
		std::cin >> userIn;
	}

};