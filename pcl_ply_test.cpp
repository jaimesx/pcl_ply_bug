#include <random>
#include <cmath>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>

typedef pcl::PointXYZRGBNormal PointType;
typedef pcl::PointCloud<PointType> PCD;

int main(int argc, char** argv) {
    size_t point_size = sizeof(PointType);
    std::cout << "size of PointType: " << point_size << std::endl;
    std::cout << "size of unsigned int: " << sizeof(unsigned int) << std::endl;

    #if __x86_64__ || __ppc64__ || _WIN64 
    std::cout << "64bits platform" << std::endl;
    #endif 

    std::random_device rd;
    std::mt19937 mt(rd());

    std::uniform_real_distribution<float> dist(5.0, 10.0); 
    std::uniform_real_distribution<float> dist_n(0.0, 1.0);
    std::uniform_int_distribution<int> dist_c(50,100);
    
    size_t max_points = static_cast<size_t>(static_cast<double>(std::numeric_limits<uint32_t>::max()) / point_size);
    size_t n_count = max_points + 1;

    std::cout << "creating cloud of " << n_count << " points" << std::endl;
    PCD::Ptr cloud = std::make_shared<PCD>();
    cloud->resize(n_count);
    for (size_t i = 0; i < n_count; i++)
    {
        cloud->points[i].x = dist(mt);
        cloud->points[i].y = dist(mt);
        cloud->points[i].z = dist(mt);
        cloud->points[i].normal_x = dist_n(mt);
        cloud->points[i].normal_y = dist_n(mt);
        cloud->points[i].normal_z = dist_n(mt);
        cloud->points[i].r = static_cast<uint8_t>(dist_c(mt));
        cloud->points[i].g = static_cast<uint8_t>(dist_c(mt));
        cloud->points[i].b = static_cast<uint8_t>(dist_c(mt));
    }
    
    pcl::PCLPointCloud2 blob;
    pcl::toPCLPointCloud2(*cloud, blob);
    
    std::cout << "データを保存している" << std::endl;
    pcl::io::savePLYFileBinary("test.ply", *cloud);

    std::cout << "データを読み込んでいる" << std::endl;
    cloud = std::make_shared<PCD>();
    pcl::io::loadPLYFile("test.ply", *cloud);

    for (size_t i = 0; i < n_count; i++)
    {
        if (cloud->points[i].x >= 5.0 && cloud->points[i].x <= 10.0 &&
            cloud->points[i].y >= 5.0 && cloud->points[i].y <= 10.0 &&
            cloud->points[i].z >= 5.0 && cloud->points[i].z <= 10.0 &&
            cloud->points[i].r >= 50 && cloud->points[i].r <= 100 &&
            cloud->points[i].g >= 50 && cloud->points[i].g <= 100 &&
            cloud->points[i].b >= 50 && cloud->points[i].b <= 100 &&
            cloud->points[i].normal_x >= 0.0 && cloud->points[i].normal_x <= 1.0 &&
            cloud->points[i].normal_y >= 0.0 && cloud->points[i].normal_y <= 1.0 &&
            cloud->points[i].normal_z >= 0.0 && cloud->points[i].normal_z <= 1.0) {
        }
        else {
            std::cout << i << "," << cloud->points[i].x << "," << cloud->points[i].y << "," << cloud->points[i].z << ",";
            std::cout << +cloud->points[i].r << "," << +cloud->points[i].g << "," << +cloud->points[i].b << ",";
            std::cout << cloud->points[i].normal_x << "," << cloud->points[i].normal_y << "," << cloud->points[i].normal_z << std::endl;
        }
    }
}