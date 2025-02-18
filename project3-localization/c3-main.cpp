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

// PCL includes for point cloud processing
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h>
#include <pcl/console/time.h>										// "Tick tock, m*th*rf*ck*r!" - Samuel Jackson
#include "helper.h"

// Namespace aliases
namespace cc = carla::client;
namespace cg = carla::geom;
namespace csd = carla::sensor::data;

using namespace std::chrono_literals;
using namespace std;

// Global params
PointCloudT pclCloud;												// Point cloud for lidar scans
cc::Vehicle::Control control; 										// Vehicle control commands
std::chrono::time_point<std::chrono::system_clock> currentTime; 	// Timer for scans
vector<ControlState> cs; 											// Vector to store control states
const double leafSize = 1.0; 										// Voxel grid filter leaf size
Pose pose(															// Initial pose
	Point(0,0,0), 
	Rotate(0,0,0)
);

bool refresh_view = false; 											// Flag to refresh viewer

// Callback function for keyboard events
void keyboardEventOccurred(const pcl::visualization::KeyboardEvent &event, void* viewer) {
    if (event.getKeySym() == "Right" && event.keyDown()) {
        cs.push_back(ControlState(0, -0.02, 0));
    } else if (event.getKeySym() == "Left" && event.keyDown()) {
        cs.push_back(ControlState(0, 0.02, 0));
    }
    if (event.getKeySym() == "Up" && event.keyDown()) {
        cs.push_back(ControlState(0.1, 0, 0));
    } else if (event.getKeySym() == "Down" && event.keyDown()) {
        cs.push_back(ControlState(-0.1, 0, 0));
    }
    if (event.getKeySym() == "a" && event.keyDown()) {
        refresh_view = true;
    }
}

// Update vehicle control based on keyboard events
void Accuate(ControlState response, cc::Vehicle::Control& state) {
    if (response.t > 0) {
        if (!state.reverse) {
            state.throttle = min(state.throttle + response.t, 1.0f);
        } else {
            state.reverse = false;
            state.throttle = min(response.t, 1.0f);
        }
    } else if (response.t < 0) {
        response.t = -response.t;
        if (state.reverse) {
            state.throttle = min(state.throttle + response.t, 1.0f);
        } else {
            state.reverse = true;
            state.throttle = min(response.t, 1.0f);
        }
    }
    state.steer = min(max(state.steer + response.s, -1.0f), 1.0f);
    state.brake = response.b;
}

// Render vehicle as a box
void drawCar(Pose pose, int num, Color color, double alpha, pcl::visualization::PCLVisualizer::Ptr& viewer) {
    BoxQ box;
    box.bboxTransform = Eigen::Vector3f(pose.position.x, pose.position.y, 0);
    box.bboxQuaternion = getQuaternion(pose.rotation.yaw);
    box.cube_length = 4;
    box.cube_width = 2;
    box.cube_height = 2;
    renderBox(viewer, box, num, color, alpha);
}

// Align 2 point clouds using ICP (Iterative Closest Point)
Eigen::Matrix4d ICP(PointCloudT::Ptr target, PointCloudT::Ptr source, Pose startingPose, int iterations) {
    Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity();

    // Align source point cloud to starting pose
    Eigen::Matrix4d starting_pose_matrix = transform3D(
		startingPose.rotation.yaw, startingPose.rotation.pitch, startingPose.rotation.roll, 
		startingPose.position.x, startingPose.position.y, startingPose.position.z
	);
    PointCloudT::Ptr transformSource(new PointCloudT);
    pcl::transformPointCloud(*source, *transformSource, starting_pose_matrix);

    // Configure & run ICP
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(transformSource);
    icp.setInputTarget(target);
    icp.setMaximumIterations(iterations);
    icp.setMaxCorrespondenceDistance(2);
    pcl::PointCloud<pcl::PointXYZ> Final;
    icp.align(Final);

    std::cout << "Converged: " << icp.hasConverged() << " score: " << icp.getFitnessScore() << std::endl;

    if (icp.hasConverged()) {
        transformation_matrix = icp.getFinalTransformation().cast<double>() * starting_pose_matrix;
    } else {
        std::cout << "WARNING: ICP is not converging!" << std::endl;
    }

    return transformation_matrix;
}

int main() {

    // Connect to CARLA simulator
    auto client = cc::Client("localhost", 2000);
    client.SetTimeout(10s);
    auto world = client.GetWorld();

    // Get vehicle from blueprint library then spawn it
    auto blueprint_library = world.GetBlueprintLibrary();
    auto vehicles = blueprint_library->Filter("vehicle");
    auto map = world.GetMap();
    auto transform = map->GetRecommendedSpawnPoints()[1];
    auto ego_actor = world.SpawnActor((*vehicles)[12], transform);

    // Config lidar
    auto lidar_bp = *(blueprint_library->Find("sensor.lidar.ray_cast"));
    lidar_bp.SetAttribute("upper_fov", "15");
    lidar_bp.SetAttribute("lower_fov", "-25");
    lidar_bp.SetAttribute("channels", "32");
    lidar_bp.SetAttribute("range", "30");
    lidar_bp.SetAttribute("rotation_frequency", "60");
    lidar_bp.SetAttribute("points_per_second", "500000");

	// Set lidar position
    auto user_offset = cg::Location(0, 0, 0);
    auto lidar_transform = cg::Transform(cg::Location(-0.5, 0, 1.8) + user_offset);
    auto lidar_actor = world.SpawnActor(lidar_bp, lidar_transform, ego_actor.get());
    auto lidar = boost::static_pointer_cast<cc::Sensor>(lidar_actor);
	bool new_scan = true;
	std::chrono::time_point<std::chrono::system_clock> lastScanTime, startTime;

    // Visualization config & init
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->setBackgroundColor(0, 0, 0);
    viewer->registerKeyboardCallback(keyboardEventOccurred, (void*)&viewer);

    auto vehicle = boost::static_pointer_cast<cc::Vehicle>(ego_actor);
    Pose pose(Point(0,0,0), Rotate(0,0,0));

    // Load map point cloud
    PointCloudT::Ptr mapCloud(new PointCloudT);
    pcl::io::loadPCDFile("map.pcd", *mapCloud);
    renderPointCloud(viewer, mapCloud, "map", Color(0,0,1));

    typename pcl::PointCloud<PointT>::Ptr cloudFiltered(new pcl::PointCloud<PointT>);
    typename pcl::PointCloud<PointT>::Ptr scanCloud(new pcl::PointCloud<PointT>);

    lidar->Listen([&new_scan, &lastScanTime, &scanCloud](auto data) {
        if (new_scan) {
            auto scan = boost::static_pointer_cast<csd::LidarMeasurement>(data);
            for (auto detection : *scan) {
                if ((detection.x * detection.x + detection.y * detection.y + detection.z * detection.z) > 8.0) {
                    pclCloud.points.push_back(PointT(detection.x, detection.y, detection.z));
                }
            }
            if (pclCloud.points.size() > 5000) {
                lastScanTime = std::chrono::system_clock::now();
                *scanCloud = pclCloud;
                new_scan = false;
            }
        }
    });

    Pose poseRef(
		Point(
			vehicle->GetTransform().location.x, 
			vehicle->GetTransform().location.y, 
			vehicle->GetTransform().location.z
		), 
		Rotate(
			vehicle->GetTransform().rotation.yaw * pi / 180, 
			vehicle->GetTransform().rotation.pitch * pi / 180, 
			vehicle->GetTransform().rotation.roll * pi / 180
		)
	);
    double maxError = 0;

    while (!viewer->wasStopped()) {
        while (new_scan) {
            std::this_thread::sleep_for(0.1s);
            world.Tick(1s);
        }

        if (refresh_view) {
            viewer->setCameraPosition(
				pose.position.x, 
				pose.position.y, 
				60, 
				pose.position.x + 1, 
				pose.position.y + 1, 
				0, 0, 0, 1
			);
            refresh_view = false;
        }

        viewer->removeShape("box0");
        viewer->removeShape("boxFill0");
        Pose truePose = Pose(
			Point(
				vehicle->GetTransform().location.x, 
				vehicle->GetTransform().location.y, 
				vehicle->GetTransform().location.z
			), 
			Rotate(
				vehicle->GetTransform().rotation.yaw * pi / 180, 
				vehicle->GetTransform().rotation.pitch * pi / 180, 
				vehicle->GetTransform().rotation.roll * pi / 180
			)
		) - poseRef;
        drawCar(truePose, 0, Color(1,0,0), 0.7, viewer);

        double theta = truePose.rotation.yaw;
        double stheta = control.steer * pi/4 + theta;
        viewer->removeShape("steer");
        renderRay(
			viewer, 
			Point(
				truePose.position.x + 2 * cos(theta), 
				truePose.position.y + 2 * sin(theta),
				truePose.position.z
			), 
			Point(
				truePose.position.x + 4 * cos(stheta), 
				truePose.position.y + 4 * sin(stheta),
				truePose.position.z
			), 
			"steer",
			Color(0,1,0)
		);

        ControlState accuate(0, 0, 1);
        if (cs.size() > 0) {
            accuate = cs.back();
            cs.clear();
            Accuate(accuate, control);
            vehicle->ApplyControl(control);
        }

        viewer->spinOnce();

        if (!new_scan) {
            new_scan = true;

            // Filter scan with voxel grid filter
            pcl::VoxelGrid<PointT> vg;
            vg.setInputCloud(scanCloud);
            vg.setLeafSize(leafSize, leafSize, leafSize);
            vg.filter(*cloudFiltered);

            // Update pose with ICP alignment result
            pose = Pose(
				Point(
					vehicle->GetTransform().location.x, 
					vehicle->GetTransform().location.y, 
					vehicle->GetTransform().location.z
				), 
				Rotate(
					vehicle->GetTransform().rotation.yaw * pi / 180, 
					vehicle->GetTransform().rotation.pitch * pi / 180, 
					vehicle->GetTransform().rotation.roll * pi / 180
				)
			) - poseRef;

            Eigen::Matrix4d transformation_matrix = ICP(mapCloud, cloudFiltered, pose, 3);
            pose = getPose(transformation_matrix);

            // Transform scan to align with pose
            PointCloudT::Ptr transform_pose(new PointCloudT);
            pcl::transformPointCloud(*scanCloud, *transform_pose, transformation_matrix);

            viewer->removePointCloud("scan");
            renderPointCloud(viewer, transform_pose, "scan", Color(1,0,0));

            viewer->removeAllShapes();
            drawCar(pose, 1, Color(0,1,0), 0.35, viewer);

            // Errors
			double poseErr_x = truePose.position.x - pose.position.x;
			double poseErr_y = truePose.position.y - pose.position.y;
            double poseError = sqrt(poseErr_x * poseErr_x + poseErr_y * poseErr_y);
            if (poseError > maxError) maxError = poseError;
            double distDriven = sqrt(truePose.position.x * truePose.position.x + truePose.position.y * truePose.position.y);

            viewer->removeShape("maxE");
            viewer->addText("Max Error: " + to_string(maxError) + " m", 200, 100, 32, 1.0, 1.0, 1.0, "maxE",0);
            viewer->removeShape("derror");
            viewer->addText("Pose error: " + to_string(poseError) + " m", 200, 150, 32, 1.0, 1.0, 1.0, "derror",0);
            viewer->removeShape("dist");
            viewer->addText("Distance: " + to_string(distDriven) + " m", 200, 200, 32, 1.0, 1.0, 1.0, "dist",0);

            if (maxError > 1.2 || distDriven >= 170.0) {
                viewer->removeShape("eval");
                if (maxError > 1.2) {
                    viewer->addText("Try Again", 200, 50, 32, 1.0, 0.0, 0.0, "eval",0);
                } else {
                    viewer->addText("Passed!", 200, 50, 32, 0.0, 1.0, 0.0, "eval",0);
                }
            }

            pclCloud.points.clear();
        }
    }

    return 0;
}
