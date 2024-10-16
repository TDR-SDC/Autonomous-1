#include "ros/ros.h"
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/filter.h>
#include <clustering/CoordinateList.h>
#include <clustering/Coordinates.h>
#include <ground_removal/arr.h>
#include <ground_removal/arrofarr.h>
#include<bits/stdc++.h>
#include <pcl_ros/point_cloud.h>

#define heightlidar 0.25 //0.188
#define lidarframe "rslidar" //"rslidar"
#define sectorangle M_PI/768

//ConeCheck
#define MaxHeight 0.4
#define MinHeight 0.05
// #define MinHeight 0.1 //0.2
// #define MaxPoints 2000
#define MinPoints 10
// #define MaxLen 1 //0.7
#define MaxWidth 0.4

//thresholds
float EuclideanThreshold =  0.4;
float CentroidThreshold = 0.4;

std::vector<std::vector<float>> line;
std::vector<float> temp;
pcl::PointXYZI lidarpoint;

class CustomCluster{
    public:
    int clustersize = 0;
    pcl::PointXYZI Avg;
    pcl::PointXYZI Last;
    pcl::PointXYZI Left;
    pcl::PointXYZI Right;
    pcl::PointXYZI minheight;
    pcl::PointXYZI maxheight ;

    CustomCluster(pcl::PointXYZI Point){
        Avg = Point;
        Last = Point;
        Left = Point;
        Right = Point;
        clustersize = 1;
        minheight = maxheight = Point;
    }
};

int check_distance(pcl::PointXYZI Cluster_Point, pcl::PointXYZI point, float Threshold){
    float distance = pow((Cluster_Point.x  - point.x), 2.0) + pow((Cluster_Point.y  - point.y), 2.0)
     + pow((Cluster_Point.z  - point.z), 2.0);
    return (distance < Threshold);
}

class Clustering
{
public:
    Clustering(ros::NodeHandle ClusteringHandle)
    {
        lidarpoint.x = 0.f; lidarpoint.y = 0.f; lidarpoint.z = -heightlidar;
        Clusters = ClusteringHandle.advertise<clustering::CoordinateList>("Clusters", 1000);
        Clusters_pc = ClusteringHandle.advertise<pcl::PointCloud<pcl::PointXYZI>>("Clusters_PointCloud", 1000);
        subline = ClusteringHandle.subscribe ("line", 1, &Clustering::callbackline, this);
        sub = ClusteringHandle.subscribe ("nogroundcloud", 1, &Clustering::callback, this);
    }

    void callbackline(ground_removal::arrofarr inputline){
        line.clear();
        std::cout << "hi" << std::endl;
        for(auto i:inputline.data){
            temp.clear();
            for(auto j:i.data) temp.push_back(j);
            line.push_back(temp);
        }
    }
    void callback(const sensor_msgs::PointCloud2ConstPtr& input)
    { 
        if(line.size()!=0){ 
        clustering::CoordinateList Cluster;
        Cluster.header.stamp = ros::Time::now();
        Cluster.header.frame_id = lidarframe;
        Cluster.size = 0;
        std::vector<CustomCluster> Clusters_Vector;
        CustomCluster StdCluster(pcl::PointXYZI(0.f));
        Clusters_Vector.push_back(StdCluster);

        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::fromROSMsg (*input, *cloud);

        if (cloud->size()!= 0){
            pcl::PointCloud<pcl::PointXYZI>::Ptr cluster_pc (new pcl::PointCloud<pcl::PointXYZI>);
            cluster_pc->header.frame_id = lidarframe;

            for (pcl::PointXYZI point : cloud->points) {
                if (isfinite(point.x) && isfinite(point.y) && isfinite(point.z)){
                    int found_cluster = 0;
                    for (int index = 0; index < Clusters_Vector.size(); index++){
                        CustomCluster Iter_Cluster = Clusters_Vector[index];
                        if (
                            check_distance(Clusters_Vector[index].Last, point, EuclideanThreshold) ||
                            check_distance(Clusters_Vector[index].Left, point, EuclideanThreshold) ||
                            check_distance(Clusters_Vector[index].Right, point, EuclideanThreshold) ||
                            check_distance(Clusters_Vector[index].Avg, point, CentroidThreshold)
                            ){

                            //after increment
                            int current_size = (Clusters_Vector[index].clustersize++);
                            double inv_current_size = (double)1/(current_size+1);

                            Clusters_Vector[index].Last = point;
                            Clusters_Vector[index].Left =  (Iter_Cluster.Left.y >= point.y) ? point: Iter_Cluster.Left ;
                            Clusters_Vector[index].Right = (Iter_Cluster.Right.y <= point.y) ? point: Iter_Cluster.Right;

                            Clusters_Vector[index].Avg.x = (double)(1 - inv_current_size)*(double)Iter_Cluster.Avg.x + (double)(point.x*inv_current_size);
                            Clusters_Vector[index].Avg.y = (double)(1 - inv_current_size)*(double)Iter_Cluster.Avg.y + (double)(point.y*inv_current_size);
                            Clusters_Vector[index].Avg.z = //(Iter_Cluster.Avg.z < point.z) ? Iter_Cluster.Avg.z : point.z;
                            (double)(1 - inv_current_size)*(double)Iter_Cluster.Avg.z + (double)(point.z*inv_current_size);
                            
                            found_cluster++;

                            if (Iter_Cluster.minheight.z > point.z){
                                Clusters_Vector[index].minheight = point;
                            }
                            if (Iter_Cluster.maxheight.z < point.z){
                                Clusters_Vector[index].maxheight = point;
                            }
                            break;
                        }
                    }
                    if (!found_cluster){
                        CustomCluster NewCluster(point);
                        // NewCluster.PointsArr.push_back(point);
                        Clusters_Vector.push_back(NewCluster);
                    
                    }
                }
            }

            // std::cout<<Clusters_Vector.size()<<std::endl;
            cluster_pc->width = Clusters_Vector.size();
            cluster_pc->height = 1;
            // cluster_pc->push_back(pcl::PointXYZI(0.f));
            float denominator; float term1; float term2; float term3;
            for (int index = 0; index < Clusters_Vector.size(); index++){
                CustomCluster Iter_Cluster = Clusters_Vector[index];
                if (Iter_Cluster.Avg.x == 0.0 && Iter_Cluster.Avg.y == 0.0 && Iter_Cluster.Avg.z == 0.0 ) continue;
                int dist_sq = pow(Iter_Cluster.Avg.x, 2.0) + pow(Iter_Cluster.Avg.y, 2.0) + pow(Iter_Cluster.Avg.z, 2.0);
                int expected_points = (5000)/(dist_sq + 1); //remove dist =0
                
                //checking cone distance from amz lines
                float angleclus = atan2(Iter_Cluster.Avg.x, -Iter_Cluster.Avg.y) - M_PI/6;
                int key = int((angleclus)/(sectorangle));
                if(key<0 || key>=line.size()){
                  std::cout << line.size() << std::endl;
                  std::cout << angleclus << std::endl;
                  std::cout << sectorangle << std::endl;
                  std::cout << key << std::endl;
                  std::cout << "bruh "<<Iter_Cluster.Avg.x<<" "<<Iter_Cluster.Avg.y<<" "<<Iter_Cluster.Avg.z<<" "<<std::endl;
                  cluster_pc->push_back(Iter_Cluster.Avg);
                  clustering::Coordinates CurrentCluster;
                    CurrentCluster.x = Iter_Cluster.Avg.x;
                    CurrentCluster.y = Iter_Cluster.Avg.y;
                    CurrentCluster.z = Iter_Cluster.Avg.z;
                  Cluster.ConeCoordinates.push_back(CurrentCluster);

                }
                denominator = pow(pow(lidarpoint.x-line[key][0], 2) + pow(lidarpoint.y-line[key][1], 2) + pow(lidarpoint.z-line[key][2], 2), 0.5);
                term1 = pow((line[key][1] - lidarpoint.y)*(Iter_Cluster.Avg.z - lidarpoint.z) - (line[key][2]-lidarpoint.z)*(Iter_Cluster.Avg.y-lidarpoint.y), 2);
                term2 = pow((line[key][2] - lidarpoint.z)*(Iter_Cluster.Avg.x - lidarpoint.x) - (line[key][0]-lidarpoint.x)*(Iter_Cluster.Avg.z-lidarpoint.z), 2);
                term3 = pow((line[key][0] - lidarpoint.x)*(Iter_Cluster.Avg.y - lidarpoint.y) - (line[key][1]-lidarpoint.y)*(Iter_Cluster.Avg.x-lidarpoint.x), 2);
                float distance = (pow(term1 + term2 + term3, 0.5))/(denominator);

                denominator = pow(pow(lidarpoint.x-line[key][0], 2) + pow(lidarpoint.y-line[key][1], 2) + pow(lidarpoint.z-line[key][2], 2), 0.5);
                term1 = pow((line[key][1] - lidarpoint.y)*(Iter_Cluster.minheight.z - lidarpoint.z) - (line[key][2]-lidarpoint.z)*(Iter_Cluster.minheight.y-lidarpoint.y), 2);
                term2 = pow((line[key][2] - lidarpoint.z)*(Iter_Cluster.minheight.x - lidarpoint.x) - (line[key][0]-lidarpoint.x)*(Iter_Cluster.minheight.z-lidarpoint.z), 2);
                term3 = pow((line[key][0] - lidarpoint.x)*(Iter_Cluster.minheight.y - lidarpoint.y) - (line[key][1]-lidarpoint.y)*(Iter_Cluster.minheight.x-lidarpoint.x), 2);
                float minzfromground = (pow(term1 + term2 + term3, 0.5))/(denominator);

                denominator = pow(pow(lidarpoint.x-line[key][0], 2) + pow(lidarpoint.y-line[key][1], 2) + pow(lidarpoint.z-line[key][2], 2), 0.5);
                term1 = pow((line[key][1] - lidarpoint.y)*(Iter_Cluster.maxheight.z - lidarpoint.z) - (line[key][2]-lidarpoint.z)*(Iter_Cluster.maxheight.y-lidarpoint.y), 2);
                term2 = pow((line[key][2] - lidarpoint.z)*(Iter_Cluster.maxheight.x - lidarpoint.x) - (line[key][0]-lidarpoint.x)*(Iter_Cluster.maxheight.z-lidarpoint.z), 2);
                term3 = pow((line[key][0] - lidarpoint.x)*(Iter_Cluster.maxheight.y - lidarpoint.y) - (line[key][1]-lidarpoint.y)*(Iter_Cluster.maxheight.x-lidarpoint.x), 2);
                float maxzfromground = (pow(term1 + term2 + term3, 0.5))/(denominator);

                if ( //checks
                1
                // (distance >=0.1 && distance < 0.22)
                // && (Iter_Cluster.Avg.z + LidarHeight < MaxHeight)
                // && (Iter_Cluster.Avg.z + LidarHeight > MinHeight)
                // && (maxzfromground >= 0.26 )
                // && (minzfromground <= 0.17 )
                // && (Iter_Cluster.clustersize < expected_points)
                // && (Iter_Cluster.clustersize > 0.13 * expected_points)
                // && ((Iter_Cluster.Right.y - Iter_Cluster.Left.y)*(Iter_Cluster.Right.y - Iter_Cluster.Left.y) + (Iter_Cluster.Right.x - Iter_Cluster.Left.x)*(Iter_Cluster.Right.x - Iter_Cluster.Left.x) < MaxWidth*MaxWidth)
                // && (Iter_Cluster.clustersize > MinPoints)
                // && (Iter_Cluster.Avg.x*Iter_Cluster.Avg.x + Iter_Cluster.Avg.y*Iter_Cluster.Avg.y < 30)
                // && (curr_colour == 0)
                ){
                    Cluster.size++;
                    clustering::Coordinates CurrentCluster;
                    CurrentCluster.x = Iter_Cluster.Avg.x;
                    CurrentCluster.y = Iter_Cluster.Avg.y;
                    CurrentCluster.z = Iter_Cluster.Avg.z;
                    // CurrentCluster.colour = curr_colour;
                    std::cout << "Cone " <<Iter_Cluster.clustersize<<" "//<<curr_colour<<" "
                    <<Iter_Cluster.Avg.x<<" "<<Iter_Cluster.Avg.y<<" "<<Iter_Cluster.Avg.z
                    <<" "<<distance <<" "<<minzfromground<<" "<<maxzfromground
                    <<" "<<std::endl; 
                    Cluster.ConeCoordinates.push_back(CurrentCluster);
                    cluster_pc->push_back(Iter_Cluster.Avg);
                
                }
            }
            cluster_pc->push_back(pcl::PointXYZI(0.f));
            Clusters_pc.publish(*cluster_pc);
        }
        else{
            std::cout<<"No object found"<<std::endl;
        }
        Clusters.publish(Cluster);
        std::cout<<"Publishing Cone Coordinates"<<std::endl;
    }}

private:
    ros::Publisher Clusters;
    ros::Publisher Clusters_pc;
    ros::Subscriber sub;
    ros::Subscriber subline;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "custom");
    
    ros::NodeHandle ClusteringHandle;
    Clustering node(ClusteringHandle);
    ros::spin();
    return 0;
}