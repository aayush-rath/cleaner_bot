#ifndef MESH_CROPPER_H
#define MESH_CROPPER_H

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/PolygonMesh.h>
#include <pcl/io/obj_io.h>
#include <pcl/conversions.h>


inline bool checkAllZ(const pcl::Indices data, pcl::PointCloud<pcl::PointXYZ>& point_cloud) {
    int reference = point_cloud[data[0]].z;
    for (const auto& idx : data) {
        if (point_cloud[idx].z != reference)
            return false;
    }
    return true;
}

pcl::PolygonMesh::Ptr cropMeshAtHeight(const std::string& model_path, float z=0.2);

#endif