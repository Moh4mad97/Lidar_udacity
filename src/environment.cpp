/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"
//#include "no_pcl_processPointClouds.cpp"

Box BoundingBox(pcl::PointCloud<pcl::PointXYZ>::Ptr clusterCloud) {
    std::vector<float> x_points;
    std::vector<float> y_points;
    std::vector<float> z_points;
    for (int i = 0; i < clusterCloud->points.size(); i++) {
        x_points.push_back(clusterCloud->points[i].x);
        y_points.push_back(clusterCloud->points[i].y);
        z_points.push_back(clusterCloud->points[i].z);
    }
    Box box;
    box.x_min = *min_element(x_points.begin(), x_points.end());
    box.x_max = *max_element(x_points.begin(), x_points.end());
    box.y_min = *min_element(y_points.begin(), y_points.end());
    box.y_max = *max_element(y_points.begin(), y_points.end());
    box.z_min = *min_element(z_points.begin(), z_points.end());
    box.z_max = *max_element(z_points.begin(), z_points.end());
    return box;
}

std::pair<typename pcl::PointCloud<pcl::PointXYZI>::Ptr, typename pcl::PointCloud<pcl::PointXYZI>::Ptr> Ransac_3d(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, int maxIterations, float distanceTol)
{
    std::unordered_set<int> inliersResult;
    srand(time(NULL));
    int cloud_points = cloud->size();
    while (maxIterations--) {
        std::unordered_set<int> inliers;
        while (inliers.size() < 3)
            inliers.insert(rand() % cloud->points.size());

        float x1, x2, x3, y1, y2, y3, z1, z2, z3;
        auto itr = inliers.begin();
        x1 = cloud->points[*itr].x;
        y1 = cloud->points[*itr].y;
        z1 = cloud->points[*itr].z;
        itr++;
        x2 = cloud->points[*itr].x;
        y2 = cloud->points[*itr].y;
        z2 = cloud->points[*itr].z;
        itr++;
        x3 = cloud->points[*itr].x;
        y3 = cloud->points[*itr].y;
        z3 = cloud->points[*itr].z;

        std::vector<float> v1;
        std::vector<float> v2;
        std::vector<float> v_normal;
        v1 = { x2 - x1,y2 - y1,z2 - z1 };
        v2 = { x3 - x1,y3 - y1,z3 - z1 };
        v_normal = { (y2 - y1) * (z3 - z1) - (z2 - z1) * (y3 - y1),(z2 - z1) * (x3 - x1) - (x2 - x1) * (z3 - z1),(x2 - x1) * (y3 - y1) - (y2 - y1) * (x3 - x1) };


        float a, b, c, d;
        a = v_normal[0];
        b = v_normal[1];
        c = v_normal[2];
        d = -(a * x1 + b * y1 + c * z1);

        float sqrt_abc = sqrt(a * a + b * b + c * c);

        for (int index = 0; index < cloud->points.size(); index++) {
            if (inliers.count(index) > 0)
                continue;

            pcl::PointXYZI point = cloud->points[index];
            float x3, y3, z3, D;
            x3 = point.x;
            y3 = point.y;
            z3 = point.z;

            D = fabs(a * x3 + b * y3 + c * z3 + d) / sqrt_abc;
            if (D <= distanceTol)
                inliers.insert(index);
        }

        if (inliers.size() > inliersResult.size())
            inliersResult = inliers;

    }
    pcl::PointCloud<pcl::PointXYZI>::Ptr  cloudInliers(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZI>());

    for (int index = 0; index < cloud->points.size(); index++)
    {
        pcl::PointXYZI point = cloud->points[index];
        if (inliersResult.count(index))
            cloudInliers->points.push_back(point);
        else
            cloudOutliers->points.push_back(point);
    }
    std::pair<typename pcl::PointCloud<pcl::PointXYZI>::Ptr, typename pcl::PointCloud<pcl::PointXYZI>::Ptr> segResult = std::make_pair(cloudInliers, cloudOutliers);
    return segResult;

}


// Structure to represent node of kd tree
struct Node
{
    std::vector<float> point;
    int id;
    Node* left;
    Node* right;

    Node(std::vector<float> arr, int setId)
        : point(arr), id(setId), left(NULL), right(NULL)
    {}

    ~Node()
    {
        delete left;
        delete right;
    }
};

struct KdTree
{
    Node* root;

    KdTree()
        : root(NULL)
    {}

    ~KdTree()
    {
        delete root;
    }

    void insert(std::vector<float> point, int id)
    {
        // TODO: Fill in this function to insert a new point into the tree
        // the function should create a new node and place correctly with in the root 
        insertHelper(&root, 0, point, id);

    }

    void insertHelper(Node** node, unsigned int depth, std::vector<float> point, int id) {

        if (*node == NULL)
            *node = new Node(point, id);
        else
        {
            //calculate current dim:
            unsigned int cd = depth % 3;

            if (point[cd] < ((*node)->point[cd]))
                insertHelper(&((*node)->left), depth + 1, point, id);
            else
                insertHelper(&((*node)->right), depth + 1, point, id);
        }
    }

    // return a list of point ids in the tree that are within distance of target
    std::vector<int> search(std::vector<float> target, float distanceTol)
    {

        std::vector<int> ids;
        searchHelper(target, root, 0, distanceTol, ids);
        return ids;
    }

    void searchHelper(std::vector<float> target, Node* node, int depth, float distanceTol, std::vector<int>& ids) {
        if (node != NULL) {
            if ((node->point[0] >= (target[0] - distanceTol) && node->point[0] <= (target[0] + distanceTol)) && ((node->point[1] >= (target[1] - distanceTol)) && node->point[1] <= (target[1] + distanceTol)) && ((node->point[2] >= (target[2] - distanceTol)) && node->point[2] <= (target[2] + distanceTol)))
            {
                float distance = sqrt((node->point[0] - target[0]) * (node->point[0] - target[0]) + (node->point[1] - target[1]) * (node->point[1] - target[1]) + (node->point[2] - target[2]) * (node->point[2] - target[2]));
                if (distance <= distanceTol)
                    ids.push_back(node->id);
            }
            if ((target[depth % 3] - distanceTol) < node->point[depth % 3])
                searchHelper(target, node->left, depth + 1, distanceTol, ids);
            if ((target[depth % 3] + distanceTol) > node->point[depth % 3])
                searchHelper(target, node->right, depth + 1, distanceTol, ids);

        }
    }
};

void clusterHelper(int i, const std::vector<std::vector<float>> points, std::vector<int>& cluster, std::vector<bool>& processed, KdTree* tree, float  distanceTol) {

    processed[i] = true;
    cluster.push_back(i);

    std::vector<int> nearest = tree->search(points[i], distanceTol);
    for (int id : nearest) {
        if (!processed[id]) {
            clusterHelper(id, points, cluster, processed, tree, distanceTol);
        }
    }
}


std::vector<std::vector<int>> euclideanCluster(const std::vector<std::vector<float>>& points, KdTree* tree, float distanceTol)
{

    std::vector<std::vector<int>> clusters;

    std::vector<bool> processed(points.size(), false);

    int i = 0;
    while (i < points.size()) {
        if (processed[i]) {
            i++;
            continue;
        }
        std::vector<int> cluster;
        clusterHelper(i, points, cluster, processed, tree, distanceTol);
        clusters.push_back(cluster);
        i++;
    }
    return clusters;

}



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

//void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer)
void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI>* pointProcessorI, const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display City Block     -----
    // ----------------------------------------------------

    //renderPointCloud(viewer, inputCloud, "inputCloud");

    // Filtering
    pcl::PointCloud<pcl::PointXYZI>::Ptr filterCloud = pointProcessorI->FilterCloud(inputCloud, 0.8f, Eigen::Vector4f(-5, -5, -2, 1), Eigen::Vector4f(20, 7.5, 10, 1));

    // Segmentation: 
    std::pair<typename pcl::PointCloud<pcl::PointXYZI>::Ptr, typename pcl::PointCloud<pcl::PointXYZI>::Ptr> segmented_cloud = Ransac_3d(filterCloud, 100, 0.2);
    //renderPointCloud(viewer, segmented_cloud.first, "inliers", Color(0, 1, 0));
    //renderPointCloud(viewer, segmented_cloud.second, "outliers", Color(1, 0, 0));

    // Clustering:
    // Extarcting all XYZ of pcl to be sutable for our Kdtree:
    typename pcl::PointCloud<pcl::PointXYZI>::Ptr obsCloud = segmented_cloud.second;
    std::vector < std::vector<float>> points;
    for (int i = 0; i < obsCloud->points.size(); i++) {
        points.push_back(std::vector<float>());
        for (int j = 0; j < 3; j++) {
            if (j == 0)
                points[i].push_back(obsCloud->points[i].x);
            if (j == 1)
                points[i].push_back(obsCloud->points[i].y);
            if (j == 2)
                points[i].push_back(obsCloud->points[i].z);
        }
    }

    KdTree* tree = new KdTree;
    for (int i = 0; i < points.size(); i++)
        tree->insert(points[i], i);
    
    std::vector<std::vector<int>> clusters = euclideanCluster(points, tree, 0.9);
    
    std::cout << "clustering found " << clusters.size() << std::endl;
    // Render clusters
    int clusterId = 0;
    std::vector<Color> colors = { Color(1,0,0), Color(0,1,0), Color(0,0,1) };
    for (std::vector<int> cluster : clusters)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr clusterCloud(new pcl::PointCloud<pcl::PointXYZ>());
        for (int indice : cluster)
            clusterCloud->points.push_back(pcl::PointXYZ(points[indice][0], points[indice][1], points[indice][2]));
        renderPointCloud(viewer, clusterCloud, "cluster" + std::to_string(clusterId), colors[clusterId % 3]);
        renderPointCloud(viewer, segmented_cloud.first, "inliers", Color(0, 1, 0));
        Box box = BoundingBox(clusterCloud);
        renderBox(viewer, box, clusterId);
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

    if(setAngle!=XY)
        viewer->addCoordinateSystem (1.0);
}


int main (int argc, char** argv)
{
    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = FPS;
    initCamera(setAngle, viewer);
    //cityBlock(viewer);
    
    
    
    
    // Stream pcd frames:
    ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
    std::vector<boost::filesystem::path> stream = pointProcessorI->streamPcd("../src/sensors/data/pcd/data_1");
    auto streamIterator = stream.begin();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;
    while (!viewer->wasStopped())
    {

        // Clear viewer
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        // Load pcd and run obstacle detection process
        inputCloudI = pointProcessorI->loadPcd((*streamIterator).string());
        cityBlock(viewer, pointProcessorI, inputCloudI);

        streamIterator++;
        if (streamIterator == stream.end())
            streamIterator = stream.begin();

        viewer->spinOnce();
    }
    
    while (!viewer->wasStopped())
    {
        viewer->spinOnce();
    }
}
