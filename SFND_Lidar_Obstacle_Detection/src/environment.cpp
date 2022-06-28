/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"

std::vector<Car> initHighway(bool renderScene, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    Car egoCar( Vect3(0,0,0), Vect3(4,2,2), Color(0,1,0), "egoCar");
    Car car1( Vect3(15,0,0), Vect3(4,2,2), Color(0,0,1), "car1");
    Car car2( Vect3(8,-4,0), Vect3(4,2,2), Color(0,0,1), "car2");	
    Car car3( Vect3(-12,4,0), Vect3(4,2,2), Color(0,0,1), "car3");
  
    std::vector<Car> cars;
    cars.push_back(egoCar);
    cars.push_back(car1);
    cars.push_back(car2);
    cars.push_back(car3);

    if(renderScene)
    {
        renderHighway(viewer);
        egoCar.render(viewer);
        car1.render(viewer);
        car2.render(viewer);
        car3.render(viewer);
    }

    return cars;
}

// void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer)
// {   
//     // RENDER OPTIONS
//     bool renderScene = false;
//     bool render_clusters = true;
//     bool render_box = true;

//     ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI> ();
//     pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud = pointProcessorI->loadPcd("../src/sensors/data/pcd/data_1/0000000000.pcd");
//     pcl::PointXYZI minPt, maxPt;
//     pcl::getMinMax3D(*inputCloud, minPt, maxPt);
//     float filterRes = 0.2f;
//     // Eigen::Vector4f minPoint = {minPt.x, minPt.y, minPt.z, 1.0};
//     // Eigen::Vector4f maxPoint = {maxPt.x, maxPt.y, maxPt.z, 1.0};
//     // cout << "minpoint:" << minPoint << endl;
//     // cout << "maxpoint:" << maxPoint << endl;
//     Eigen::Vector4f minPoint = {-15.0, -6.0, -3.0, 1.0};
//     Eigen::Vector4f maxPoint = {30.0, 6.0, 10.0, 1.0};
//     pcl::PointCloud<pcl::PointXYZI>::Ptr filteredCloud = pointProcessorI->FilterCloud(inputCloud, filterRes, minPoint, maxPoint);

//      // TODO:: Create point processor
//     // ProcessPointClouds<pcl::PointXYZI> pointProcessor;
//     std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud = pointProcessorI->SegmentPlane(filteredCloud, 100, 0.2); //tried 0.1, 0.2, 0.5,
//     renderPointCloud(viewer,segmentCloud.first,"obstCloud",Color(1,0,0));
//     renderPointCloud(viewer,segmentCloud.second,"planeCloud",Color(0,1,0));

//     std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessorI->Clustering(segmentCloud.first, 0.5, 10, 600);

//     int clusterId = 0;
//     std::vector<Color> colors = {Color(1,0,0), Color(0,1,1), Color(1,1,0)};

//     for(pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters)
//     {
//         if(render_clusters)
//         {
//             // std::cout << "cluster size ";
//             pointProcessorI->numPoints(cluster);
//             renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(clusterId),colors[clusterId % colors.size()]);
//         }
//         if(render_box)
//         {
//             Box box =  pointProcessorI->BoundingBox(cluster);
//             renderBox(viewer, box, clusterId);
//         }
//         ++clusterId;    
//     }


//     // renderPointCloud(viewer, inputCloud, "inputCloud");
//     // renderPointCloud(viewer, filteredCloud, "filteredCloud");
// }


void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI>* pointProcessorI, const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud){
    //RENDER OPTIONS
    bool render_pointCloud = false;
    bool render_pointCloud_filtered = false;
    bool render_obst = false;
    bool render_plane = true;
    bool render_clusters = true;
    bool render_box = true;
    
    // HYPER-PARAMETERS
    float filterRes = 0.2;
    Eigen::Vector4f minPoint = {-10, -6, -5, 1};
    Eigen::Vector4f maxPoint = {35, 7, 5, 1};
    int maxIterations = 50;
    float distanceTol = 0.2;
    float clusterTol = 0.5;
    int clusterMinSize = 10;
    int clusterMaxSize = 1000;

    if(render_pointCloud)
        renderPointCloud(viewer, inputCloud, "inputCloud");

    //FILTERING OF INPUT POINT CLOUD
    pcl::PointCloud<pcl::PointXYZI>::Ptr filteredCloud = pointProcessorI->FilterCloud(inputCloud, filterRes, minPoint, maxPoint);
    if(render_pointCloud_filtered)
        renderPointCloud(viewer, filteredCloud, "filteredCloud");

    //PLANE SEGMENTATION
    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr,pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud = pointProcessorI->SegmentPlaneOwnImplementation(filteredCloud, maxIterations, distanceTol);
    // std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud = pointProcessorI->SegmentPlane(filteredCloud, 100, 0.2);
    
    if(render_obst)
    {
        renderPointCloud(viewer, segmentCloud.first, "obstCloud", Color(1, 0, 0));
    }
    if(render_plane)
    {
        renderPointCloud(viewer, segmentCloud.second, "planeCloud", Color(0, 1, 0));
    }

    //CLUSTERING
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessorI->ClusteringOwnImplementation(segmentCloud.first, clusterTol, clusterMinSize, clusterMaxSize);
    // std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessorI->Clustering(segmentCloud.first, 0.5, 10, 600);

    int clusterId = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(1,1,0), Color(0,0,1)};

    for(pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters)
    {
        if(render_clusters)
        {
            pointProcessorI->numPoints(cluster);
            renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(clusterId),colors[clusterId % colors.size()]);
        }
        if(render_box)
        {
            Box box =  pointProcessorI->BoundingBox(cluster);
            renderBox(viewer, box, clusterId);
        }
        ++clusterId;    
    }

}


void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------
    
    // RENDER OPTIONS
    bool renderScene = false;
    bool render_clusters = true;
    bool render_box = true;
    std::vector<Car> cars = initHighway(renderScene, viewer);
    
    // TODO:: Create lidar sensor 
    Lidar* lidar = new Lidar(cars, 0);
    pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud = lidar->scan();
    // renderRays(viewer, lidar->position, inputCloud);
    // renderPointCloud(viewer, inputCloud, "inputcloud");
  

    // TODO:: Create point processor
    ProcessPointClouds<pcl::PointXYZ> pointProcessor;
    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud = pointProcessor.SegmentPlane(inputCloud, 10, 0.2);
    renderPointCloud(viewer,segmentCloud.first,"obstCloud",Color(1,0,0));
    // renderPointCloud(viewer,segmentCloud.second,"planeCloud",Color(0,1,0));

    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = pointProcessor.Clustering(segmentCloud.first, 1.0, 3, 30);

    int clusterId = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(1,1,0), Color(0,0,1)};

    for(pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters)
    {
        if(render_clusters)
        {
            std::cout << "cluster size ";
            pointProcessor.numPoints(cluster);
            renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(clusterId),colors[clusterId]);
        }
        if(render_box)
        {
            Box box =  pointProcessor.BoundingBox(cluster);
            renderBox(viewer, box, clusterId);
        }
        ++clusterId;    
    }
}


//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    viewer->setBackgroundColor (0, 0, 0);
    
    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 16;
    
    switch(setAngle)
    {
        case XY : viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0); break;
        case TopDown : viewer->setCameraPosition(0, 0, distance, 1, 0, 1); break;
        case Side : viewer->setCameraPosition(0, -distance, 0, 0, 0, 1); break;
        case FPS : viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if(setAngle!=FPS)
        viewer->addCoordinateSystem (1.0);
}


int main (int argc, char** argv)
{
    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = FPS;
    initCamera(setAngle, viewer);

    ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
    std::vector<boost::filesystem::path> stream = pointProcessorI->streamPcd("../src/sensors/data/pcd/data_1");
    auto streamIterator = stream.begin();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;
    // simpleHighway(viewer);
    // cityBlock(viewer);

    // while (!viewer->wasStopped ())
    // {
    //     viewer->spinOnce ();
    // } 

    while (!viewer->wasStopped ())
    {
    // Clear viewer
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        // Load pcd and run obstacle detection process
        inputCloudI = pointProcessorI->loadPcd((*streamIterator).string());
        cityBlock(viewer, pointProcessorI, inputCloudI);

        streamIterator++;
        if(streamIterator == stream.end())
            streamIterator = stream.begin();

        viewer->spinOnce ();
    }
}