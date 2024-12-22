#include "helpers.h"
#include <carla/client/Client.h>
#include <carla/client/BlueprintLibrary.h>
#include <carla/client/Vehicle.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h>
#include <pcl/visualization/pcl_visualizer.h>

constexpr bool USE_ICP = true;
constexpr double kLeafSize = 0.5;
constexpr int kMaxIterICP = 120;
constexpr int kMaxIterNDT = 95;
constexpr double kTransformEps = 1e-4;
constexpr double kVoxelResolution = 1.0;

void SetLidar(carla::client::Sensor &lidar) {
    lidar.SetAttribute("upper_fov", "15");
    lidar.SetAttribute("lower_fov", "-25");
    lidar.SetAttribute("channels", "32");
    lidar.SetAttribute("range", "30");
    lidar.SetAttribute("rotation_frequency", "30");
    lidar.SetAttribute("points_per_second", "500000");
}

void InitNDT(pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> &ndt, pcl::PointCloud<pcl::PointXYZ>::Ptr map) {
    ndt.setTransformationEpsilon(kTransformEps);
    ndt.setResolution(kVoxelResolution);
    ndt.setInputTarget(map);
}

Eigen::Matrix4d ComputeICP(pcl::PointCloud<pcl::PointXYZ>::Ptr target, pcl::PointCloud<pcl::PointXYZ>::Ptr source, Pose3D initPose) {
    Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d initTransform = transform3D(initPose.rotation.yaw, initPose.rotation.pitch, initPose.rotation.roll, initPose.position.x, initPose.position.y, initPose.position.z);
    pcl::PointCloud<pcl::PointXYZ>::Ptr aligned(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(source);
    icp.setInputTarget(target);
    icp.setMaximumIterations(kMaxIterICP);
    icp.align(*aligned);
    if (icp.hasConverged()) {
        transform = icp.getFinalTransformation().cast<double>() * initTransform;
    }
    return transform;
}

Eigen::Matrix4d ComputeNDT(pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> &ndt, pcl::PointCloud<pcl::PointXYZ>::Ptr source, Pose3D initPose) {
    Eigen::Matrix4f initGuess = transform3D(initPose.rotation.yaw, initPose.rotation.pitch, initPose.rotation.roll, initPose.position.x, initPose.position.y, initPose.position.z).cast<float>();
    pcl::PointCloud<pcl::PointXYZ>::Ptr aligned(new pcl::PointCloud<pcl::PointXYZ>());
    ndt.align(*aligned, initGuess);
    return ndt.hasConverged() ? ndt.getFinalTransformation().cast<double>() : Eigen::Matrix4d::Identity();
}

int main() {
    carla::client::Client client("localhost", 2000);
    auto world = client.GetWorld();
    auto map = world.GetMap();
    auto vehicles = world.GetBlueprintLibrary()->Filter("vehicle");
    auto vehicle = world.SpawnActor((*vehicles)[0], map->GetRecommendedSpawnPoints()[0]);
    auto lidarBlueprint = *(world.GetBlueprintLibrary()->Find("sensor.lidar.ray_cast"));
    SetLidar(lidarBlueprint);
    auto lidar = world.SpawnActor(lidarBlueprint, carla::geom::Transform(), vehicle.get());

    pcl::PointCloud<pcl::PointXYZ>::Ptr mapCloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::io::loadPCDFile("../map.pcd", *mapCloud);
    pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;
    if (!USE_ICP) InitNDT(ndt, mapCloud);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudFiltered(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr scanCloud(new pcl::PointCloud<pcl::PointXYZ>());
    Pose3D pose;

    lidar->Listen([&](auto data) {
        auto scan = boost::static_pointer_cast<carla::sensor::data::LidarMeasurement>(data);
        pcl::VoxelGrid<pcl::PointXYZ> voxelGrid;
        voxelGrid.setInputCloud(scanCloud);
        voxelGrid.setLeafSize(kLeafSize, kLeafSize, kLeafSize);
        voxelGrid.filter(*cloudFiltered);

        Eigen::Matrix4d transform = USE_ICP ? ComputeICP(mapCloud, cloudFiltered, pose) : ComputeNDT(ndt, cloudFiltered, pose);
        pose = getPose(transform);
    });

    return 0;
}

