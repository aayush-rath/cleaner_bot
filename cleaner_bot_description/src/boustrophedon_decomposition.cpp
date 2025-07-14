// #include "file_parser.h"

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/PolygonMesh.h>
#include <pcl/io/obj_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/conversions.h>

#include <thread>
#include <chrono>
#include <limits>

using namespace std::chrono_literals;


inline bool checkAllZ(const pcl::Indices data, pcl::PointCloud<pcl::PointXYZ>& point_cloud) {
    int reference = point_cloud[data[0]].z;
    for (const auto& idx : data) {
        if (point_cloud[idx].z != reference)
            return false;
    }
    return true;
}

int main() {
    // Load the mesh model
    std::string model_path = "/home/aayush/Projects/cleaner_bot_ws/src/cleaner_bot_description/models/BedsideTable2/meshes/BedsideTable2.obj";
    pcl::PolygonMesh::Ptr mesh(new pcl::PolygonMesh);
    pcl::io::loadOBJFile(model_path, *mesh);
    pcl::PointCloud<pcl::PointXYZ> point_cloud;
    pcl::fromPCLPointCloud2(mesh->cloud, point_cloud);

    // Cropping plane height
    float z = 0.2;

    // List of vertices and the new cropped mesh
    pcl::PolygonMesh::Ptr mesh2(new pcl::PolygonMesh);
    pcl::PointCloud<pcl::PointXYZ> points;
    points.width = point_cloud.width;
    points.height = point_cloud.height;
    points.is_dense = false;
    points.points.resize(point_cloud.size());

    pcl::PCLPointCloud2 points2;
    points2.width = points.width;
    points2.height = points.height;

    for (const auto& polygon : mesh->polygons) {

        checkAllZ(polygon.vertices, point_cloud);

        for (std::size_t i = 0; i < polygon.vertices.size(); i++) {
            float del_x = point_cloud.points[polygon.vertices[(i+1)%polygon.vertices.size()]].x - point_cloud.points[polygon.vertices[i%polygon.vertices.size()]].x;
            float del_y = point_cloud.points[polygon.vertices[(i+1)%polygon.vertices.size()]].y - point_cloud.points[polygon.vertices[i%polygon.vertices.size()]].y;
            float del_z = point_cloud.points[polygon.vertices[(i+1)%polygon.vertices.size()]].z - point_cloud.points[polygon.vertices[i%polygon.vertices.size()]].z;

            float x, y;
            std::size_t idx;

            if ((point_cloud.points[polygon.vertices[i%polygon.vertices.size()]].z - z) * (point_cloud.points[polygon.vertices[(i+1)%polygon.vertices.size()]].z - z) < 0 
            && point_cloud.points[polygon.vertices[i%polygon.vertices.size()]].z > z) {
                x = point_cloud.points[polygon.vertices[i%polygon.vertices.size()]].x + z * del_x / del_z;
                y = point_cloud.points[polygon.vertices[i%polygon.vertices.size()]].y + z * del_y / del_z;
                idx = polygon.vertices[i%polygon.vertices.size()];
                points[idx] = pcl::PointXYZ(x, y, z);
            } else if ((point_cloud.points[polygon.vertices[i%polygon.vertices.size()]].z - z) * (point_cloud.points[polygon.vertices[(i+1)%polygon.vertices.size()]].z - z) < 0
            && point_cloud.points[polygon.vertices[i%polygon.vertices.size()]].z <= z) {
                x = point_cloud.points[polygon.vertices[(i+1)%polygon.vertices.size()]].x + z * del_x / del_z;
                y = point_cloud.points[polygon.vertices[(i+1)%polygon.vertices.size()]].y + z * del_y / del_z;
                idx = polygon.vertices[(i+1)%polygon.vertices.size()];
                points[idx] = pcl::PointXYZ(x, y, z);
            } else if (point_cloud.points[polygon.vertices[i%polygon.vertices.size()]].z <= z){
                points[polygon.vertices[i%polygon.vertices.size()]] = point_cloud.points[polygon.vertices[i%polygon.vertices.size()]];
                points[polygon.vertices[(i+1)%polygon.vertices.size()]] = point_cloud.points[polygon.vertices[(i+1)%polygon.vertices.size()]];
            } else {
                std::numeric_limits<float>::quiet_NaN();
                points[polygon.vertices[i%polygon.vertices.size()]] = pcl::PointXYZ(std::numeric_limits<float>::quiet_NaN(), 
                                                                                    std::numeric_limits<float>::quiet_NaN(),
                                                                                    std::numeric_limits<float>::quiet_NaN());
                points[polygon.vertices[(i+1)%polygon.vertices.size()]] = pcl::PointXYZ(std::numeric_limits<float>::quiet_NaN(), 
                                                                                    std::numeric_limits<float>::quiet_NaN(),
                                                                                    std::numeric_limits<float>::quiet_NaN());
                continue;
            }
        }
    }

    pcl::toPCLPointCloud2(points, points2);
    point_cloud.header = mesh->cloud.header;
    mesh2->cloud = points2;
    mesh2->polygons = mesh->polygons;

    // Display the mesh
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer);
    viewer->setBackgroundColor(0.0, 0.0, 0.0);
    viewer->addPolygonMesh(*mesh2, "Mesh");

    while(!viewer->wasStopped()) {
        viewer->spinOnce();
        std::this_thread::sleep_for(100ms);
    }

    return 0;
}