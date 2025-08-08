#include "mesh_cropper.h"

pcl::PolygonMesh::Ptr cropMeshAtHeight(const std::string& model_path, float z)
{
    // Load the mesh model
    pcl::PolygonMesh::Ptr mesh(new pcl::PolygonMesh);
    pcl::io::loadOBJFile(model_path, *mesh);

    // Convert mesh->cloud to point cloud
    pcl::PointCloud<pcl::PointXYZ> point_cloud;
    pcl::fromPCLPointCloud2(mesh->cloud, point_cloud);

    // Prepare output mesh and point cloud
    pcl::PolygonMesh::Ptr cropped_mesh(new pcl::PolygonMesh);
    pcl::PointCloud<pcl::PointXYZ> points;
    points.width = point_cloud.width;
    points.height = point_cloud.height;
    points.is_dense = false;
    points.points.resize(point_cloud.size(),
        pcl::PointXYZ(std::numeric_limits<float>::quiet_NaN(),
                      std::numeric_limits<float>::quiet_NaN(),
                      std::numeric_limits<float>::quiet_NaN())); // Initialize with NaN

    // Iterate over polygons and crop
    for (const auto& polygon : mesh->polygons) {
        checkAllZ(polygon.vertices, point_cloud);

        for (std::size_t i = 0; i < polygon.vertices.size(); i++) {
            auto v1 = polygon.vertices[i % polygon.vertices.size()];
            auto v2 = polygon.vertices[(i + 1) % polygon.vertices.size()];

            float del_x = point_cloud.points[v2].x - point_cloud.points[v1].x;
            float del_y = point_cloud.points[v2].y - point_cloud.points[v1].y;
            float del_z = point_cloud.points[v2].z - point_cloud.points[v1].z;

            float x, y;
            std::size_t idx;

            if ((point_cloud.points[v1].z - z) * (point_cloud.points[v2].z - z) < 0 
                && point_cloud.points[v1].z > z) {
                x = point_cloud.points[v1].x + z * del_x / del_z;
                y = point_cloud.points[v1].y + z * del_y / del_z;
                idx = v1;
                points[idx] = pcl::PointXYZ(x, y, z);

            } else if ((point_cloud.points[v1].z - z) * (point_cloud.points[v2].z - z) < 0
                       && point_cloud.points[v1].z <= z) {
                x = point_cloud.points[v2].x + z * del_x / del_z;
                y = point_cloud.points[v2].y + z * del_y / del_z;
                idx = v2;
                points[idx] = pcl::PointXYZ(x, y, z);

            } else if (point_cloud.points[v1].z <= z) {
                points[v1] = point_cloud.points[v1];
                points[v2] = point_cloud.points[v2];
            }
        }
    }

    // Convert back to PCLPointCloud2
    pcl::PCLPointCloud2 points2;
    pcl::toPCLPointCloud2(points, points2);
    points2.header = mesh->cloud.header;

    // Build final cropped mesh
    cropped_mesh->cloud = points2;
    cropped_mesh->polygons = mesh->polygons;

    return cropped_mesh;
}