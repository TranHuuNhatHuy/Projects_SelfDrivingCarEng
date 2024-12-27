
#include <carla/client/Client.h>
#include <carla/client/ActorBlueprint.h>
#include <carla/client/BlueprintLibrary.h>
#include <carla/client/Map.h>
#include <carla/geom/Location.h>
#include <carla/geom/Transform.h>
#include <carla/client/Sensor.h>
#include <carla/sensor/data/LidarMeasurement.h>
#include <thread>

#include <carla/client/Vehicle.h>

//pcl code
//#include "render/render.h"

namespace cc = carla::client;
namespace cg = carla::geom;
namespace csd = carla::sensor::data;

using namespace std::chrono_literals;
using namespace std::string_literals;

using namespace std;

#include <string>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/voxel_grid.h>
#include "helper.h"
#include <sstream>
#include <chrono> 
#include <ctime> 
#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h>
#include <pcl/console/time.h>   // TicToc

// ============================== Define global params

// General params
const static bool USE_ICP = true;				// Use ICP or NDT
const static std::string BASE_PATH = "../";		// Base path for pcl data (relative to build directory)
const static double MIN_LIDAR_DIST = 8.0;		// Min lidar distance to consider
const static int MAX_MAP_POINTS = 5000;			// Max points in map
const static double VOXEL_LEAF_RES = 0.5;		// Voxel leaf resolution

// ICP params
const double ICP_MAX_DIST = 5;					// Max distance for ICP
const int ICP_MAX_ITER = 120;					// Max iterations for ICP
const double ICP_TRANS_EPS = 1e-4;				// Transformation epsilon for ICP
const double ICP_FIT_EPS = 2;					// Fit epsilon for ICP
const double ICP_RANSAC_THRESH = 0.2;			// RANSAC threshold for ICP

// NDT params
const int NDT_MAX_ITER = 95;					// Max iterations for NDT
const double NDT_STEP_SIZE = 0.1;				// Step size for NDT
const double NDT_TRANS_EPS = 1e-4;				// Transformation epsilon for NDT
const double NDT_RES = 1.0;						// NDT resolution

PointCloudT pclCloud;
cc::Vehicle::Control control;
std::chrono::time_point<std::chrono::system_clock> currentTime;
vector<ControlState> cs;

bool refresh_view = false;
void keyboardEventOccurred(const pcl::visualization::KeyboardEvent &event, void* viewer)
{

  	//boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = *static_cast<boost::shared_ptr<pcl::visualization::PCLVisualizer> *>(viewer_void);
	if (event.getKeySym() == "Right" && event.keyDown()){
		cs.push_back(ControlState(0, -0.02, 0));
  	}
	else if (event.getKeySym() == "Left" && event.keyDown()){
		cs.push_back(ControlState(0, 0.02, 0)); 
  	}
  	if (event.getKeySym() == "Up" && event.keyDown()){
		cs.push_back(ControlState(0.1, 0, 0));
  	}
	else if (event.getKeySym() == "Down" && event.keyDown()){
		cs.push_back(ControlState(-0.1, 0, 0)); 
  	}
	if(event.getKeySym() == "a" && event.keyDown()){
		refresh_view = true;
	}
}

void Accuate(ControlState response, cc::Vehicle::Control& state){

	if(response.t > 0){
		if(!state.reverse){
			state.throttle = min(state.throttle+response.t, 1.0f);
		}
		else{
			state.reverse = false;
			state.throttle = min(response.t, 1.0f);
		}
	}
	else if(response.t < 0){
		response.t = -response.t;
		if(state.reverse){
			state.throttle = min(state.throttle+response.t, 1.0f);
		}
		else{
			state.reverse = true;
			state.throttle = min(response.t, 1.0f);

		}
	}
	state.steer = min( max(state.steer+response.s, -1.0f), 1.0f);
	state.brake = response.b;
}

void drawCar(Pose pose, int num, Color color, double alpha, pcl::visualization::PCLVisualizer::Ptr& viewer){

	BoxQ box;
	box.bboxTransform = Eigen::Vector3f(pose.position.x, pose.position.y, 0);
    box.bboxQuaternion = getQuaternion(pose.rotation.yaw);
    box.cube_length = 4;
    box.cube_width = 2;
    box.cube_height = 2;
	renderBox(viewer, box, num, color, alpha);
}

bool pointInEgoRange(auto &point) {
    double distSquared = point.x * point.x + point.y * point.y + point.z * point.z;
    return distSquared <= MIN_LIDAR_DIST;
}

void SetupLidar(auto &lidar) {
    lidar.SetAttribute("upper_fov", "15");
    lidar.SetAttribute("lower_fov", "-25");
    lidar.SetAttribute("channels", "32");
    lidar.SetAttribute("range", "30");
    lidar.SetAttribute("rotation_frequency", "30");
    lidar.SetAttribute("points_per_second", "500000");
}

void InitNDT(pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> &ndt, PointCloudT &map) {
    ndt.setTransformationEpsilon(NDT_TRANS_EPS);
    ndt.setStepSize(NDT_STEP_SIZE);
    ndt.setResolution(NDT_RES);
    ndt.setInputTarget(map);
}

Eigen::Matrix4d RunICP(PointCloudT::Ptr target, PointCloudT::Ptr src, Pose3D pose, int iter) {
    Eigen::Matrix4d transform = transform3D(pose.rotation.yaw, pose.rotation.pitch, pose.rotation.roll,
                                            pose.position.x, pose.position.y, pose.position.z);
    PointCloudT::Ptr srcTransformed(new PointCloudT);
    pcl::transformPointCloud(*src, *srcTransformed, transform);

    pcl::IterativeClosestPoint<PointT, PointT> icp;
    icp.setInputSource(srcTransformed);
    icp.setInputTarget(target);
    icp.setMaxCorrespondenceDistance(ICP_MAX_DIST);
    icp.setMaximumIterations(iter);
    icp.setTransformationEpsilon(ICP_TRANS_EPS);
    icp.setEuclideanFitnessEpsilon(ICP_FIT_EPS);
    icp.setRANSACOutlierRejectionThreshold(ICP_RANSAC_THRESH);

    PointCloudT::Ptr aligned(new PointCloudT);
    icp.align(*aligned);

    return icp.hasConverged() ? icp.getFinalTransformation().cast<double>() * transform : Eigen::Matrix4d::Identity();
}

Eigen::Matrix4d RunNDT(pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> &ndt, PointCloudT::Ptr src, Pose3D pose, int iter) {
    ndt.setMaximumIterations(iter);
    Eigen::Matrix4f guess = transform3D(pose.rotation.yaw, pose.rotation.pitch, pose.rotation.roll,
                                        pose.position.x, pose.position.y, pose.position.z).cast<float>();

    PointCloudT::Ptr aligned(new PointCloudT);
    ndt.align(*aligned, guess);

    return ndt.hasConverged() ? ndt.getFinalTransformation().cast<double>() : Eigen::Matrix4d::Identity();
}

int main() {
    carla::client::Client client("localhost", 2000);
    client.SetTimeout(std::chrono::seconds(2));

    auto world = client.GetWorld();
    auto blueprint = world.GetBlueprintLibrary();
    auto vehicleBp = blueprint->Filter("vehicle")[12];
    auto spawn = world.GetMap()->GetRecommendedSpawnPoints()[1];
    auto vehicle = world.SpawnActor(vehicleBp, spawn);

    auto lidarBp = blueprint->Find("sensor.lidar.ray_cast");
    SetupLidar(lidarBp);
    auto lidarTransform = carla::geom::Transform(carla::geom::Location(-0.5, 0, 1.8));
    auto lidar = boost::static_pointer_cast<carla::client::Sensor>(world.SpawnActor(lidarBp, lidarTransform, vehicle.get()));

    Pose3D pose({0, 0, 0}, {0, 0, 0});
    Pose3D refPose({vehicle->GetTransform().location.x, vehicle->GetTransform().location.y, vehicle->GetTransform().location.z},
                   {vehicle->GetTransform().rotation.yaw * M_PI / 180, 0, 0});

    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->setBackgroundColor(0, 0, 0);
    viewer->registerKeyboardCallback(HandleKeyEvent, (void *)&viewer);

    PointCloudT::Ptr mapCloud(new PointCloudT);
    pcl::io::loadPCDFile(BASE_PATH + "map.pcd", *mapCloud);
    renderPointCloud(viewer, mapCloud, "map", {0, 0, 1});

    pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;
    if (!USE_ICP) InitNDT(ndt, *mapCloud);

    lidar->Listen([&](auto data) {
        if (lidarCloud.points.size() > MAX_MAP_POINTS) return;
        auto scan = boost::static_pointer_cast<carla::sensor::data::LidarMeasurement>(data);
        for (auto pt : *scan) if (!IsPointInEgoRange(pt)) lidarCloud.push_back({pt.point.x, pt.point.y, pt.point.z});
    });

    while (!viewer->wasStopped()) {
        if (refreshView) {
            viewer->setCameraPosition(pose.position.x, pose.position.y, 60, pose.position.x + 1, pose.position.y + 1, 0);
            refreshView = false;
        }

        if (lidarCloud.points.size() > 0) {
            PointCloudT::Ptr filteredCloud(new PointCloudT);
            pcl::VoxelGrid<PointT> voxel;
            voxel.setInputCloud(PointCloudT::Ptr(new PointCloudT(lidarCloud)));
            voxel.setLeafSize(VOXEL_LEAF_RES, VOXEL_LEAF_RES, VOXEL_LEAF_RES);
            voxel.filter(*filteredCloud);

            Eigen::Matrix4d transform = USE_ICP ? RunICP(mapCloud, filteredCloud, pose, ICP_MAX_ITER) : RunNDT(ndt, filteredCloud, pose, NDT_MAX_ITER);
            pose = getPose3D::getPose(transform);

            PointCloudT::Ptr aligned(new PointCloudT);
            pcl::transformPointCloud(*filteredCloud, *aligned, transform);
            renderPointCloud(viewer, aligned, "scan", {1, 0, 0});
            lidarCloud.clear();
        }
        viewer->spinOnce();
    }
    return 0;
}
