#ifndef COMPOSITE_HERB_UTIL_
#define COMPOSITE_HERB_UTIL_

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

#include <aikido/constraint/Satisfied.hpp>
using dart::dynamics::BodyNodePtr;

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

class CompositeHERBUtil
{
public:

	MetaSkeletonPtr mRightArm;
	MetaSkeletonPtr mLeftArm;

	MetaSkeletonStateSpacePtr mRightArmSpace;
	MetaSkeletonStateSpacePtr mLeftArmSpace;

	aikido::planner::WorldPtr mEnv;
	RobotPtr mRobot;

	CollisionFreePtr mLeftEnvTestable;
	TestablePtr mLeftFullTestable;

	CollisionFreePtr mRightEnvTestable;
	TestablePtr mRightFullTestable;

	std::shared_ptr<GeodesicInterpolator> mLeftInterpolator;
	std::shared_ptr<GeodesicInterpolator> mRightInterpolator;

	std::vector<SkeletonPtr> mObstacles;

	CompositeHERBUtil()
	{
		// waitForUser("Press [ENTER] to start constructor: ");
		char *argv[] = { "long", "live", "HERB" };
		int argc = 3;
		std::cout << "Starting ROS node." << std::endl;
		ros::init(argc, argv, "HERBUtil");
		ros::NodeHandle nh("~");

		const std::string baseFrameName("map");
		const std::string topicName("dart_markers");
	  
		mEnv = aikido::planner::WorldPtr(new aikido::planner::World("world"));

		const auto resourceRetriever = std::make_shared<aikido::io::CatkinResourceRetriever>();
		
		ROS_INFO("Loading HERB in Simulation.");
		mRobot = std::move(std::make_shared<herb::Herb>(mEnv, true));
		
		//adding obstacles in world env

		SkeletonPtr table;
		Eigen::Isometry3d tablePose;
		perceiveObject(nh,resourceRetriever,table,tablePose,"table");

		// SkeletonPtr pitcher;
		// Eigen::Isometry3d pitcherPose;
		// perceiveObject(nh,resourceRetriever,pitcher,pitcherPose,"pitcher");

		SkeletonPtr roof;
		Eigen::Isometry3d roofPose;
		perceiveObject(nh,resourceRetriever,roof,roofPose,"roof");

		// SkeletonPtr shelf;
		// Eigen::Isometry3d shelfPose;
		// perceiveObject(nh,resourceRetriever,shelf,shelfPose,"shelf");

		SkeletonPtr can;
		Eigen::Isometry3d canPose;
		perceiveObject(nh,resourceRetriever,can,canPose,"can");

		SkeletonPtr can2;
		Eigen::Isometry3d can2Pose;
		perceiveObject(nh,resourceRetriever,can2,can2Pose,"can2");

		mObstacles.push_back(table);
		
		mObstacles.push_back(roof);
		// mObstacles.push_back(pitcher);
		// mObstacles.push_back(shelf);
		mObstacles.push_back(can);
		mObstacles.push_back(can2);

		// ROS_INFO_STREAM("Starting viewer. Please subscribe to the '"<<dartTopicName<<"'InteractiveMarker topic in rviz.");
		// aikido::rviz::WorldInteractiveMarkerViewer viewer(env, dartTopicName, baseFrameName);
		// viewer.setAutoUpdate(true);
		// waitForUser("Press [ENTER] to exit: ");

		mRightArmSpace = mRobot->getRightArm()->getStateSpace();
		mRightArm = std::const_pointer_cast<MetaSkeleton>(mRobot->getRightArm()->getMetaSkeleton());

		mLeftArmSpace = mRobot->getLeftArm()->getStateSpace();
		mLeftArm = std::const_pointer_cast<MetaSkeleton>(mRobot->getLeftArm()->getMetaSkeleton());


		CollisionDetectorPtr envDetector = dart::collision::FCLCollisionDetector::create();
		
		mLeftEnvTestable = std::make_shared<CollisionFree>(mLeftArmSpace, mLeftArm, envDetector);
		mRightEnvTestable = std::make_shared<CollisionFree>(mRightArmSpace, mRightArm, envDetector);


		//Collision group for left arm
		std::shared_ptr<CollisionGroup> leftArmGroup = envDetector->createCollisionGroup( mRobot->getLeftArm()->getMetaSkeleton().get(),
				mRobot->getLeftHand()->getMetaSkeleton().get());

		//Collision group for right arm
		std::shared_ptr<CollisionGroup> rightArmGroup = envDetector->createCollisionGroup( mRobot->getRightArm()->getMetaSkeleton().get(),
				mRobot->getRightHand()->getMetaSkeleton().get());

		//adding arms collision checking
		mLeftEnvTestable->addPairwiseCheck(rightArmGroup,leftArmGroup);

		//adding arms collision checking
		mRightEnvTestable->addPairwiseCheck(leftArmGroup,rightArmGroup);

		for (SkeletonPtr curObstacle : mObstacles)
		{
			std::shared_ptr<CollisionGroup> curObstacleGroup = envDetector->createCollisionGroup(curObstacle.get());
				
			mLeftEnvTestable->addPairwiseCheck(leftArmGroup, curObstacleGroup);
			mRightEnvTestable->addPairwiseCheck(rightArmGroup, curObstacleGroup);
		}

		mLeftFullTestable = mRobot->getFullCollisionConstraint( mLeftArmSpace, mLeftArm, mLeftEnvTestable);
		mLeftInterpolator = std::make_shared<GeodesicInterpolator>(mLeftArmSpace);

		mRightFullTestable = mRobot->getFullCollisionConstraint( mRightArmSpace, mRightArm, mRightEnvTestable);
		mRightInterpolator = std::make_shared<GeodesicInterpolator>(mRightArmSpace);
		ros::shutdown();
	}

	void waitForUser(const std::string& msg)
	{
	  std::cout << msg;
	  std::cin.get();
	}

	BodyNodePtr getBodyNodeOrThrow(const SkeletonPtr& skeleton, const std::string& bodyNodeName)
	{
		auto bodyNode = skeleton->getBodyNode(bodyNodeName);
		if (!bodyNode)
		{
			std::stringstream message;
	    	message << "Bodynode [" << bodyNodeName << "] does not exist in skeleton.";
	    	throw std::runtime_error(message.str());
		}
		return bodyNode;
	}
	
	void moveArmTo(RobotPtr robot,const MetaSkeletonStateSpacePtr& armSpace,const MetaSkeletonPtr& armSkeleton,const Eigen::VectorXd& goalPos)
	{
		double planningTimeout{5.};
		// No collision checking
		auto testable = std::make_shared<aikido::constraint::Satisfied>(armSpace);
		auto trajectory = robot->planToConfiguration(armSpace, armSkeleton, goalPos, nullptr, planningTimeout);
		if (!trajectory)
		{
			throw std::runtime_error("Failed to find a solution");
		}
		// auto smoothTrajectory = robot.retimePath(armSkeleton, trajectory.get());
		auto future = robot->executeTrajectory(std::move(trajectory));
		future.wait();
	}

	const SkeletonPtr makeBodyFromURDF(const std::shared_ptr<aikido::io::CatkinResourceRetriever> resourceRetriever,
		const std::string& uri, const Eigen::Isometry3d& transform)
	{
		dart::utils::DartLoader urdfLoader;
		const SkeletonPtr skeleton = urdfLoader.parseSkeleton(uri, resourceRetriever);

		if (!skeleton)
			throw std::runtime_error("unable to load '" + uri + "'");

		dynamic_cast<dart::dynamics::FreeJoint*>(skeleton->getJoint(0))
				->setTransform(transform);
		return skeleton;
	}


	bool perceiveObject(ros::NodeHandle nh, const std::shared_ptr<aikido::io::CatkinResourceRetriever> resourceRetriever,
		dart::dynamics::SkeletonPtr& object, Eigen::Isometry3d& objectPose, std::string name)
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
			mRobot->getWorld()->addSkeleton(object);
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
			mRobot->getWorld()->addSkeleton(object);
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
			mRobot->getWorld()->addSkeleton(object);
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
			mRobot->getWorld()->addSkeleton(object);
			return true;
		}
		else if (name=="pitcher")
		{
			const std::string objectURDFUri("package://pr_assets/data/objects/rubbermaid_ice_guard_pitcher.urdf");
			objectPose = Eigen::Isometry3d::Identity();
			objectPose.translation() = Eigen::Vector3d(1.0, 0.0, 0)+Eigen::Vector3d(0.0, 0.0, 0.73);
			object = makeBodyFromURDF(resourceRetriever, objectURDFUri, objectPose);
			// Add all objects to World
			mRobot->getWorld()->addSkeleton(object);
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
			mRobot->getWorld()->addSkeleton(object);
			return true;
		}
		else
			return false;
	}
	bool getCollisionStatusEdge(Eigen::VectorXd &source_config, Eigen::VectorXd &target_config)
	{
		// std::cout<<"Inside getCollisionStatus!"<<std::endl;
		int dim = (source_config.size())/2;
		Eigen::VectorXd left_source(dim);
		Eigen::VectorXd left_target(dim);
				
		Eigen::VectorXd right_source(dim);
		Eigen::VectorXd right_target(dim);

		left_source << source_config.segment(0,dim);
		right_source << source_config.segment(dim,dim);

		left_target << target_config.segment(0,dim);
		right_target << target_config.segment(dim,dim);

		// std::cout<<"Source config: ";
		// for(int l=0;l<dim+dim;l++)
			// std::cout<<source_config(l)<<" ";
		// std::cout<<std::endl;

		// std::cout<<"Target config: ";
		// for(int l=0;l<dim+dim;l++)
			// std::cout<<target_config(l)<<" ";
		// std::cout<<std::endl;	

		auto left_source_state = mLeftArmSpace->createState();
		mLeftArmSpace->convertPositionsToState(left_source, left_source_state);
		auto left_target_state = mLeftArmSpace->createState();
		mLeftArmSpace->convertPositionsToState(left_target, left_target_state);	

		auto right_source_state = mRightArmSpace->createState();
		mRightArmSpace->convertPositionsToState(right_source, right_source_state);
		auto right_target_state = mRightArmSpace->createState();
		mRightArmSpace->convertPositionsToState(right_target, right_target_state);

		double col_resolution = 0.05;

		int left_qCount = (left_source-left_target).norm()/col_resolution;
		if(left_qCount<1)
			left_qCount=1;

		int right_qCount = (right_source-right_target).norm()/col_resolution;
		if(right_qCount<1)
			right_qCount=1;

		int qCount=std::max(left_qCount,right_qCount);
		double step = 1.0/qCount;
		double right_step = 1.0/right_qCount;
		double left_step = 1.0/left_qCount;

		auto left_test_state = mLeftArmSpace->createState();
		auto right_test_state = mRightArmSpace->createState();		

		bool col =false;
		double alpha;
		double beta;

		for (alpha = 0.0, beta=0.0; (alpha <= 1.0 || beta <=1.0) ; alpha += left_step, beta+=right_step)
		{
			if(alpha>1.0)
				alpha=1.0;
			if(beta>1.0)
				beta=1.0;
			mLeftInterpolator->interpolate(left_source_state, left_target_state, alpha, left_test_state);
			mLeftArmSpace->setState((&(*mLeftArm)),left_test_state);
			mRightInterpolator->interpolate(right_source_state, right_target_state, beta, right_test_state);
			mRightArmSpace->setState((&(*mRightArm)),right_test_state);
			if (!mRightFullTestable->isSatisfied(right_test_state))
				col=true;
			if (!mLeftFullTestable->isSatisfied(left_test_state))
				col=true;
			if(col==true)
				break;
		}


		//do the same with right {left {}} as checking of left testable for environment


		// for (double alpha = 0.0; alpha <= 1.0; alpha += step)
		// {
		// 	mRightInterpolator->interpolate(right_source_state, right_target_state, alpha, right_test_state);
		// 	mRightArmSpace->setState((&(*mRightArm)),right_test_state);
		// 	mLeftInterpolator->interpolate(left_source_state, left_target_state, alpha, left_test_state);
		// 	if (!mLeftFullTestable->isSatisfied(left_test_state))
		// 		col=true;
		// }

		// for (double alpha = 0.0; alpha <= 1.0; alpha += step)
		// {
		// 	mLeftInterpolator->interpolate(left_source_state, left_target_state, alpha, left_test_state);
		// 	mLeftArmSpace->setState((&(*mLeftArm)),left_test_state);
		// 	mRightInterpolator->interpolate(right_source_state, right_target_state, alpha, right_test_state);
		// 	if (!mRightFullTestable->isSatisfied(right_test_state))
		// 			col=true;
		// }


		// //do the same with right {left {}} as checking of left testable for environment


		// for (double alpha = 0.0; alpha <= 1.0; alpha += step)
		// {
		// 	mRightInterpolator->interpolate(right_source_state, right_target_state, alpha, right_test_state);
		// 	mLeftArmSpace->setState((&(*mRightArm)),right_test_state);
		// 	mLeftInterpolator->interpolate(left_source_state, left_target_state, alpha, left_test_state);
		// 	if (!mLeftFullTestable->isSatisfied(left_test_state))
		// 		col=true;
		// }

		return col;
	}

	bool getCollisionStatusVertex(Eigen::VectorXd &config)
	{
		// std::cout<<"Inside getCollisionStatus!"<<std::endl;
		int dim = (config.size())/2;
		
		Eigen::VectorXd left_config(dim);
		Eigen::VectorXd right_config(dim);

		left_config << config.segment(0,dim);
		right_config << config.segment(dim,dim);

		// std::cout<<"Source config: ";
		// for(int l=0;l<dim+dim;l++)
			// std::cout<<source_config(l)<<" ";
		// std::cout<<std::endl;

		// std::cout<<"Target config: ";
		// for(int l=0;l<dim+dim;l++)
			// std::cout<<target_config(l)<<" ";
		// std::cout<<std::endl;	

		auto left_state = mLeftArmSpace->createState();
		mLeftArmSpace->convertPositionsToState(left_config, left_state);

		auto right_state = mRightArmSpace->createState();
		mRightArmSpace->convertPositionsToState(right_config, right_state);

		bool col =false;
		mLeftArmSpace->setState((&(*mLeftArm)),left_state);
		mRightArmSpace->setState((&(*mRightArm)),right_state);
		if (!mRightFullTestable->isSatisfied(right_state))
			col=true;
		if (!mLeftFullTestable->isSatisfied(left_state))
			col=true;
		return col;
	}

	void executeEdgeRight(ros::NodeHandle nh, Eigen::VectorXd & source_config, Eigen::VectorXd & target_config)
	{
		// waitForUser("Press [ENTER] to execute right hand: ");
		InterpolatedPtr traj = std::make_shared<Interpolated>(mRightArmSpace,mRightInterpolator);

		auto source_state = mRightArmSpace->createState();
		mRightArmSpace->convertPositionsToState(source_config, source_state);
		
		traj->addWaypoint(0, source_state);

		auto target_state = mRightArmSpace->createState();
		mRightArmSpace->convertPositionsToState(target_config, target_state);
		
		traj->addWaypoint(1, target_state);

		auto timedTraj = mRobot->retimePath(mRightArm, traj.get());

		mRobot->executeTrajectory(std::move(timedTraj)).wait();

		// std::cout << "[INFO]: Done Executing!" << std::endl;
	}

	void executeEdgeLeft(ros::NodeHandle nh, Eigen::VectorXd & source_config, Eigen::VectorXd & target_config)
	{
		// waitForUser("Press [ENTER] to execute left hand: ");
		InterpolatedPtr traj = std::make_shared<Interpolated>(mLeftArmSpace,mLeftInterpolator);

		auto source_state = mLeftArmSpace->createState();
		mLeftArmSpace->convertPositionsToState(source_config, source_state);
		
		traj->addWaypoint(0, source_state);

		auto target_state = mLeftArmSpace->createState();
		mLeftArmSpace->convertPositionsToState(target_config, target_state);
		
		traj->addWaypoint(1, target_state);

		auto timedTraj = mRobot->retimePath(mLeftArm, traj.get());

		mRobot->executeTrajectory(std::move(timedTraj)).wait();

		// std::cout << "[INFO]: Done Executing!" << std::endl;
	}
	void executeTogether( std::vector<Eigen::VectorXd> & configs)
	{
		bool herbReal = false;

		int target = 1;
		herbReal = false;
  		char *argv[] = { "long", "live", "HERB" };
		int argc = 3;
		std::cout << "Starting ROS node." << std::endl;
		ros::init(argc, argv, "execute_viz");
		ros::NodeHandle nh("execute");

		const std::string baseFrameName("map");
		const std::string topicName("dart_markers");


		// aikido::planner::WorldPtr env(new aikido::planner::World("simple_trajectories"));
		// // Load HERB either in simulation or real based on arguments
		// ROS_INFO("Loading HERB.");
		// herb::Herb robot(env, !herbReal);
		// auto robotSkeleton = robot.getMetaSkeleton();

		std::cout << "Starting viewer. Please subscribe to the '" << topicName
					<< "' InteractiveMarker topic in RViz." << std::endl;
		aikido::rviz::WorldInteractiveMarkerViewer viewer(mEnv, topicName, baseFrameName);

		viewer.setAutoUpdate(true);

		size_t dim = (configs.at(0).size())/2;

		Eigen::VectorXd lStart(dim);
		lStart << configs.at(0).segment(0,dim);
		mLeftArm->setPositions(lStart);

		Eigen::VectorXd rStart(dim);
	  	rStart << configs.at(0).segment(dim,dim);
		mRightArm->setPositions(rStart);

		waitForUser("Press [ENTER] to start: ");

		// Create the space.
		using dart::dynamics::Linkage;

		// Get the names right.
		std::stringstream wamBaseName;
		wamBaseName << "herb_frame";

		// Same as hand-base for HERB [offset from wam7 by FT sensor dimension]

		std::stringstream righthandname;
		righthandname << "/right/hand_base";
		std::stringstream lefthandname;
		lefthandname << "/left/hand_base";
		auto mRobotSkeleton = mRobot->getRobotSkeleton();
		auto armBase = getBodyNodeOrThrow(mRobotSkeleton, wamBaseName.str());
		auto righthand = getBodyNodeOrThrow(mRobotSkeleton, righthandname.str());
		auto lefthand = getBodyNodeOrThrow(mRobotSkeleton, lefthandname.str());
		dart::dynamics::Linkage::Criteria::Target startTarget(armBase);
		dart::dynamics::Linkage::Criteria::Target leftTarget(lefthand);
		dart::dynamics::Linkage::Criteria::Target rightTarget(righthand);
		std::vector<dart::dynamics::Linkage::Criteria::Target> targets;
		targets.emplace_back(leftTarget);
		targets.emplace_back(rightTarget);
		dart::dynamics::Linkage::Criteria criteria;
		criteria.mTargets = targets;
		criteria.mStart = startTarget;
		// Create the criteria to end.
		auto arms = Linkage::create(criteria, "bimanual");
		auto bimanualSpace = std::make_shared<MetaSkeletonStateSpace>(arms.get());
		std::cout << bimanualSpace->getDimension() << std::endl;
		auto names = bimanualSpace->getProperties().getDofNames();
		for (int i = 0; i < names.size(); ++i)
		std::cout << names[i] << std::endl;
		waitForUser("Press [ENTER] to start execution: ");

		Eigen::VectorXd bimanualGoal(14);

		bimanualGoal << 3.14, -1.57, 0.00, 0.00, 0.00, 0.00, 0.00, 3.14, -1.57, 0.00, 0.00, 0.00, 0.00, 0.00;

		for (int i = 1; i < configs.size(); i++)
		{
			Eigen::VectorXd bimanualGoal(14);
			bimanualGoal << configs.at(i);
			moveArmTo(mRobot, bimanualSpace, arms, bimanualGoal);
		}

		std::cout<<"FINISHED EXECUTION SUCCESSFULLY";
		ros::shutdown();

	}

	void executePathTPG( std::vector<Eigen::VectorXd> & configs) 
	{
		
		char *argv[] = { "long", "live", "HERB" };
		int argc = 3;
		std::cout << "Starting ROS node." << std::endl;
		ros::init(argc, argv, "execute_viz");
		ros::NodeHandle nh("execute");

		const std::string baseFrameName("map");
		const std::string topicName("dart_markers");

		// Start the RViz viewer.
		std::cout << "Starting viewer. Please subscribe to the '" << topicName
							<< "' InteractiveMarker topic in RViz." << std::endl;
		aikido::rviz::WorldInteractiveMarkerViewer viewer(mEnv, topicName, baseFrameName);

		viewer.setAutoUpdate(true);

		size_t dim = (configs.at(0).size())/2;

		Eigen::VectorXd lStart(dim);
		lStart << configs.at(0).segment(0,dim);
		mLeftArm->setPositions(lStart);

		Eigen::VectorXd rStart(dim);
	  	rStart << configs.at(0).segment(dim,dim);
		mRightArm->setPositions(rStart);

		waitForUser("Press [ENTER] to start Execution: ");


		for(size_t i=0;i<configs.size()-1;i++)
		{
			Eigen::VectorXd source_config(dim+dim);
			Eigen::VectorXd target_config(dim+dim);

			source_config = configs.at(i);
			target_config = configs.at(i+1);

			Eigen::VectorXd left_source(dim);
			Eigen::VectorXd left_target(dim);
					
			Eigen::VectorXd right_source(dim);
			Eigen::VectorXd right_target(dim);

			Eigen::VectorXd left_temp(dim);
			Eigen::VectorXd right_temp(dim);

			Eigen::VectorXd left_temp_previous(dim);
			Eigen::VectorXd right_temp_previous(dim);

			left_source << source_config.segment(0,dim);
			right_source << source_config.segment(dim,dim);

			left_target << target_config.segment(0,dim);
			right_target << target_config.segment(dim,dim);

			left_temp_previous << left_source;
			right_temp_previous << right_source;

			int qCount = 10;
			double step = 1.0/qCount;

			for (double alpha = 0.0 + step; alpha <= 1.0 - step; alpha += step)
			{
				left_temp = left_source*(1-alpha)+left_target*alpha;
				right_temp = right_source*(1-alpha)+right_target*alpha;

				executeEdgeLeft(nh,left_temp_previous,left_temp);
				executeEdgeRight(nh,right_temp_previous,right_temp);

				left_temp_previous << left_temp;
				right_temp_previous << right_temp;

			}
		}

		std::cout << "[INFO]: Done Executing!" << std::endl;
		ros::shutdown();
	}

};

#endif