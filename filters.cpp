#include "filters.h"
#include <random>
#include<time.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <std_msgs/Int32.h>
#include <chrono>



#define LIOR_CONST  0.066





Filters::Filters()
{
    pcl2_Header_seq=0;
    filter_number = 1;
    parameter1 = 0;
    parameter2 = 0.1;
    parameter3 = 0.1;
    parameter4 = 0.1;
    parameter5 = 0.1;
    use_multi  = false;
    hardware_ready = 0;
        int fd;
        if ((fd = open("/dev/mem", O_RDWR | O_SYNC)) != -1) {
            bram_x_ptr = (u64 *)mmap(NULL, bram_size, PROT_READ|PROT_WRITE, MAP_SHARED, fd, bram_x);
            bram_y_ptr = (u64 *)mmap(NULL, bram_size, PROT_READ|PROT_WRITE, MAP_SHARED, fd, bram_y);
            bram_z_ptr = (u64 *)mmap(NULL, bram_size, PROT_READ|PROT_WRITE, MAP_SHARED, fd, bram_z);
            bram_i_ptr = (u64 *)mmap(NULL, bram_size, PROT_READ|PROT_WRITE, MAP_SHARED, fd, bram_i);

            hardware_ready=1;
        }


}

void Filters::do_voxelfilter()
{
    pcl::VoxelGrid<pcl::PointXYZI> sor;
    sor.setInputCloud (inputCloud);
    sor.setLeafSize (parameter1,parameter2,parameter3);
    sor.filter (*OutputCloud);

}

void Filters::do_sorfilter()
{
    pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor;
    sor.setInputCloud (inputCloud);
    sor.setMeanK (parameter1);
    sor.setStddevMulThresh (parameter2);
    sor.filter (*OutputCloud);
}

void Filters::do_dorfilter(pcl::PointCloud<PointT>::Ptr newInput)
{
    vector<pcl::PointXYZI> outliners;

    inliners.clear();
    kdtree.setInputCloud(newInput);
    if(thread_list.size()>1)
    {
        for (int i =0;i < thread_list.size();i++)
        {
            thread_list[i]->join();
         }

        thread_list.clear();
    }
    if(use_multi == true)
    {
        thread_list.clear();
        cout << "Running multithreading with "<< number_threads<<endl;
        for (int i =0;i <= number_threads;i++)
        {
            thread_list.push_back(new boost::thread(&Filters::run_worker, this,i));
        }
        for (int i =0;i <= number_threads;i++)
        {
            thread_list[i]->join();
        }

    }
    else {
        for (auto &point : *newInput)
        {
            filter_point(point);
        }
    }
    OutputCloud->resize(inliners.size());
    int counter =0;
    for(auto& point: *OutputCloud)
    {
        point = inliners[counter];
        counter++;
    }
}

void Filters::do_rorfilter()
{
    pcl::RadiusOutlierRemoval<pcl::PointXYZI> outrem;
    outrem.setInputCloud(inputCloud);
    outrem.setRadiusSearch(parameter1);
    outrem.setMinNeighborsInRadius (parameter2);
    outrem.filter (*OutputCloud);

}

void Filters::do_fcsorfilter()
{
    pcl::PointXYZI minPt, maxPt;
    pcl::getMinMax3D (*inputCloud, minPt, maxPt);
    double NumberClusters = parameter1;
    double  ClusterLength = abs(maxPt.x -minPt.x)/NumberClusters;
    double ClusterWidth = abs(maxPt.y - minPt.y)/NumberClusters;
    double ClusterHeight = abs(maxPt.z - minPt.z)/NumberClusters;
    pcl::VoxelGrid<pcl::PointXYZI> voxel;
    voxel.setInputCloud (inputCloud);
    voxel.setLeafSize (ClusterLength,ClusterWidth, ClusterHeight);
    pcl::PointCloud<PointT>::Ptr betweenCloud( new pcl::PointCloud<pcl::PointXYZI>);
    voxel.filter (*betweenCloud);
    pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor;
    sor.setInputCloud (betweenCloud);
    sor.setMeanK (parameter2);
    sor.setStddevMulThresh (parameter3);
    sor.filter (*OutputCloud);
}

void Filters::do_VoxDrorfilter()
{
    pcl::PointXYZI minPt, maxPt;
    pcl::getMinMax3D (*inputCloud, minPt, maxPt);
    double NumberClusters = parameter5;
    double  ClusterLength = abs(maxPt.x -minPt.x)/NumberClusters;
    double ClusterWidth = abs(maxPt.y - minPt.y)/NumberClusters;
    double ClusterHeight = abs(maxPt.z - minPt.z)/NumberClusters;
    pcl::VoxelGrid<pcl::PointXYZI> voxel;
    voxel.setInputCloud (inputCloud);
    voxel.setLeafSize (ClusterLength,ClusterWidth, ClusterHeight);
    pcl::PointCloud<PointT>::Ptr betweenCloud( new pcl::PointCloud<pcl::PointXYZI>);
    voxel.filter (*betweenCloud);
    do_dorfilter(betweenCloud);

}

void Filters::do_GDRORfilter()
{
    //vector<pcl::PointXYZI> inliners;
    inliners.clear();
    kdtree.setInputCloud(inputCloud);
    if(thread_list.size()>1)
    {
        for (int i =0;i < thread_list.size();i++)
        {
            thread_list[i]->join();
         }

        thread_list.clear();
    }
    if(use_multi)
    {
        thread_list.clear();
        cout << "Running multithreading with "<< number_threads<<endl;
        for (int i =0;i <= number_threads;i++)
        {
            thread_list.push_back(new boost::thread(&Filters::run_worker, this,i));
        }
        for (int i =0;i <= number_threads;i++)
        {
            thread_list[i]->join();
        }
    }
    else
    {
        for(auto& point: *inputCloud)
        {
            if(point.z >0- parameter5)
            {
                filter_point(point);
            }

        }
    }

    OutputCloud->resize(inliners.size());
    int counter =0;
    for(auto& point: *OutputCloud)
    {
        point = inliners[counter];
        counter++;
    }

}


void Filters::do_LIORfilter()
{
    inliners.clear();
    kdtree.setInputCloud(inputCloud);
    number_threads = parameter4;
    if(thread_list.size()>1)
    {
        for (int i =0;i < thread_list.size();i++)
        {
            thread_list[i]->join();
         }

        thread_list.clear();
    }
    if (number_threads >1)
    {
        thread_list.clear();

        for (int i =0;i <= number_threads;i++)
        {
            thread_list.push_back(new boost::thread(&Filters::run_lior_worker, this,i));
        }
        for (int i =0;i <= number_threads;i++)
        {
            thread_list[i]->join();
         }
    }
    else {
        for (auto &point : *inputCloud)
        {
            double intensity_trehshold=parameter2;
            if(point._PointXYZI::intensity > intensity_trehshold)
            {
                mutex.lock();
                inliners.push_back(point);
                mutex.unlock();
            }
            else
            {
                filter_pointROR(point);
            }
        }

    }

    OutputCloud->resize(inliners.size());
    int counter = 0;
    for (auto &point : *OutputCloud)
    {
        point = inliners[counter];
        counter++;
    }

}

void Filters::do_DLIORfilter()
{
    inliners.clear();
    kdtree.setInputCloud(inputCloud);
    number_threads = parameter5;
    if(thread_list.size()>1)
    {
        for (int i =0;i < thread_list.size();i++)
        {
            thread_list[i]->join();
         }

        thread_list.clear();
    }

    if (number_threads >1)
    {
        thread_list.clear();
        for (int i =0;i <= number_threads;i++)
        {
            thread_list.push_back(new boost::thread(&Filters::run_dlior_worker, this,i));
        }
        for (int i =0;i <= number_threads;i++)
        {
            thread_list[i]->join();
         }
    }
    else {
        for (auto &point : *inputCloud)
        {
            double intensity_trehshold=parameter4;
            if(point._PointXYZI::intensity > intensity_trehshold)
            {
                mutex.lock();
                inliners.push_back(point);
                mutex.unlock();
            }
            else
            {
                filter_point(point,1);
            }
        }

    }

    OutputCloud->resize(inliners.size());
    int counter = 0;
    for (auto &point : *OutputCloud)
    {
        point = inliners[counter];
        counter++;
    }

}

using namespace std::chrono;

void Filters::do_hardwarefilter()
{
    long long int a_32points_x=0;
    long long int a_32points_y=0;
    long long int a_32points_z=0;
    long long int a_32points_i=0;
    int32_t a_64points_x[2];
    int32_t a_64points_y[2];
    int32_t a_64points_z[2];
    int32_t a_64points_i[2];

    auto start = high_resolution_clock::now();

    bram_x_ptr[0]= inputCloud->size();
    bram_i_ptr[0]= parameter1;
    int i =0;
    int pos_aux=0;
    int bram_aux=1;
    for (auto &point : *inputCloud)
    {
        if ( i == 0)
        {
            a_32points_x = (int16_t)(point.x*100);
            a_32points_y = (int16_t)(point.y*100);
            a_32points_z = (int16_t)(point.z*100);
            a_32points_i = (int16_t)(point.intensity);
            i++;

        }
        else {
            a_32points_x = a_32points_x +((int16_t)(point.x*100)<<(16*i));
            a_32points_y = a_32points_y +((int16_t)(point.y*100)<<(16*i));
            a_32points_z = a_32points_z +((int16_t)(point.z*100)<<(16*i));
            a_32points_i = a_32points_i +((int16_t)(point.intensity)<<(16*i));

            i=0;
            a_64points_x[pos_aux]=a_32points_x;
            a_64points_y[pos_aux]=a_32points_y;
            a_64points_z[pos_aux]=a_32points_z;
            a_64points_i[pos_aux]=a_32points_i;


            pos_aux++;

            if(pos_aux==2)
            {
                memcpy((void*)(bram_x_ptr+bram_aux),a_64points_x,sizeof(int32_t)*2);
                memcpy((void*)(bram_y_ptr+bram_aux),a_64points_y,sizeof(int32_t)*2);
                memcpy((void*)(bram_z_ptr+bram_aux),a_64points_z,sizeof(int32_t)*2);
                memcpy((void*)(bram_i_ptr+bram_aux),a_64points_i,sizeof(int32_t)*2);


                //cout << "sended to mem"<<bram_aux<<endl;
                bram_aux++;
                pos_aux=0;

            }
        }
    }
    //cout << "points saved"<<endl;
    bram_y_ptr[0]=0xffff;
    auto stop = high_resolution_clock::now();
    auto duration = duration_cast<milliseconds>(stop - start);
    frameTime = duration.count();
    //cout <<"sended start signal"<<endl;
    int hardware_finish =1;
    while (hardware_finish) {
        int value = bram_z_ptr[0];
        if(value >=1)
            hardware_finish=0;
        else
             usleep(1);
    }
    bram_z_ptr[0]=0;
    //cout<<"received finish signal"<<endl;
    auto start2 = high_resolution_clock::now();

    decode_pointcloud();


    auto stop2 = high_resolution_clock::now();
    auto duration2 = duration_cast<milliseconds>(stop - start);
    frameTime += duration2.count();



}



void Filters::apply_filters()
{

    switch (filter_number) {
    case 1:
        do_voxelfilter();
        break;
    case 2:
        do_rorfilter();
        break;
    case 3:
        do_sorfilter();
        break;
    case 4:
        do_dorfilter(inputCloud);
        break;
    case 5:
        do_fcsorfilter();
        break;
//    case 6:
//        do_hardwarefilter();
//        break;
    case 7:
        do_GDRORfilter();
        break;
    case 8:
        do_LIORfilter();
        break;
    case 9:
        do_DLIORfilter();
        break;
    case 10:
        do_hardwarefilter();
        break;
    }
    //auto stop = high_resolution_clock::now();
    //auto duration = duration_cast<milliseconds>(stop - start);
    //frameTime = duration.count();
    emit_frametime();
    emit_exitpointcloud();

}


void Filters::decode_pointcloud()
{
    OutputCloud->clear();
    int16_t point_x[4];
    int16_t point_y[4];
    int16_t point_z[4];
    int16_t point_i[4];

    for (int i = 1; i < inputCloud->size()/4; ++i) {


        memcpy((void*)point_x,(bram_x_ptr+i),sizeof(int32_t)*2);
        memcpy((void*)point_y,(bram_y_ptr+i),sizeof(int32_t)*2);
        memcpy((void*)point_z,(bram_z_ptr+i),sizeof(int32_t)*2);
        memcpy((void*)point_i,(bram_i_ptr+i),sizeof(int32_t)*2);



        for (int j = 0; j < 4; j++) {
            //cout<<"entrei aqui"<<endl;

            pcl::PointXYZI point;
            point.x = point_x[j]/100.0;
            point.y= point_y[j]/100.0;
            point.z= point_z[j]/100.0;
            point.intensity = point_i[j];
            //cout << "x: "<< point.x<<"y: "<<point.y<<"z: "<<point.z<<endl;
            if(point.x!=0 )
                 OutputCloud->push_back(point);
        }


    }
    //cout <<"end filter with point cloud size: "<<OutputCloud->size()<<endl;
    //pcl::io::savePCDFileASCII("plswork.pcd", *OutputCloud);


}



void Filters::filter_pointROR(pcl::PointXYZI point)
{
    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;

    int neighbors = kdtree.radiusSearch(point,parameter1, pointIdxRadiusSearch, pointRadiusSquaredDistance);
    if (neighbors >parameter3)
    {
        inliners.push_back(point);
    }
}



void Filters::filter_point(pcl::PointXYZI point,bool isDLIOR)
{
    float distance = sqrt(pow(point.x,2)+pow(point.y,2));
    float search_radius;
    //float anlge = atan2((double)point.y,(double)point.x);
    //if(anlge <0)
    //{
    //anlge = 2*M_PI + anlge;
    //}
    //float anlge = parameter4;
    float anlge;
    if(isDLIOR)
    {
       anlge = 0.3;
    }
    else {
       anlge = parameter4;
    }

    if(distance<parameter1)
    {
        search_radius = parameter1;
    }else
    {
        //float anlge = atan((double)point.y/(double)point.x);


        //anlge = anlge * 180.0 / M_PI;

        search_radius = parameter2 * (distance*anlge);
    }
    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;

    int neighbors = kdtree.radiusSearch (point, search_radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) ;
    if(neighbors>=parameter3)
    {
        mutex.lock();

        inliners.push_back(point);
        mutex.unlock();
        //OutputCloud->push_back(point);

        //}
        //}
    }
}

void Filters::filter_pointGDROR(pcl::PointXYZI point)
{
    if(point.z > parameter5)
    {


        float distance = sqrt(pow(point.x,2)+pow(point.y,2));
        float search_radius;
        //float anlge = atan2((double)point.y,(double)point.x);
        //if(anlge <0)
        //{
        //anlge = 2*M_PI + anlge;
        //}
        float anlge = parameter4;

        if(distance<parameter1)
        {
            search_radius = parameter1;
        }else
        {
            //float anlge = atan((double)point.y/(double)point.x);


            //anlge = anlge * 180.0 / M_PI;

            search_radius = parameter2 * (distance*anlge);
        }
        std::vector<int> pointIdxRadiusSearch;
        std::vector<float> pointRadiusSquaredDistance;

        int neighbors = kdtree.radiusSearch (point, search_radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) ;
        if(neighbors>=parameter3)
        {
            mutex.lock();

            inliners.push_back(point);
            mutex.unlock();
            //OutputCloud->push_back(point);

            //}
            //}
        }
    }
}

void Filters::update_filterSettings(const alfa_dvc::FilterSettings &msg)
{
    cout<<"updating filter Settings to "<<msg.filterNumber<<endl;
    mutex.lock();
    filter_number = msg.filterNumber;
    parameter1 = msg.parameter1;
    parameter2 = msg.parameter2;
    parameter3 = msg.parameter3;
    parameter4 = msg.parameter4;
    parameter5 = msg.parameter5;
    mutex.unlock();

    switch (filter_number) {
    case 1:
        cout << "Updated to voxel Filter";
        break;
    case 2:
        cout << "Updated to ROR Filter";
        break;
    case 3:
        cout << "Updated to SOR Filter";
        break;
    case 4:
        cout << "Updated to DROR Filter";
        break;
    case 5:
        cout << "Updated to FCSOR Filter";
        break;
    case 6:
        cout << "Updated to VDROR Filter";
        break;
    case 7:
        cout << "Updated to GDROR Filter";
        break;
    case 8:
        cout << "Updated to LIOR Filter";
        break;
    case 9:
        cout << "Updated to DLIOR Filter";
        break;
    }
    cout<< " with parameters: 1:"<<msg.parameter1<<"; 2:"<<msg.parameter2<<"; 3:"<<msg.parameter3<<"; 4:"<<msg.parameter4<<"; 5:"<<msg.parameter5<<endl;
    if (thread_list.size()>0)
    {
        for (int i =0;i <= number_threads;i++)
        {
            thread_list[i]->join();
        }
    }

    //thread_list.clear();
    mutex.lock();
    if(filter_number == 4)
    {
        if (parameter5 > 0)
        {
            cout << "Enabling Multihreading"<<endl;
            use_multi = true;
            number_threads = int(parameter5);
        }
        else
        {
            cout << "Disabling Multihreading"<<endl;
            use_multi = false;
        }
    }
        mutex.unlock();

}

void Filters::run_worker(int thread_number)
{

    for(int i =(inputCloud->size()/number_threads)*thread_number; i<= (inputCloud->size()/number_threads)*(thread_number+1);i++)
    {
        pcl::PointXYZI point = (*inputCloud)[i];
        if (filter_number == 4)
        filter_point(point);
        else filter_pointGDROR(point);

    }
}




void Filters::run_lior_worker(int thread_number)
{
    for(int i =(inputCloud->size()/number_threads)*thread_number; i<= (inputCloud->size()/number_threads)*(thread_number+1);i++)
    {
        pcl::PointXYZI point = (*inputCloud)[i];
        double intensity_trehshold=parameter2;
        float distance = sqrt(pow(point.x, 2) + pow(point.y, 2)+pow(point.z, 2));
        if(point._PointXYZI::intensity > intensity_trehshold)
        {
            mutex.lock();
            inliners.push_back(point);
            mutex.unlock();
        }
        else
        {
            filter_pointROR(point);
        }
    }
}



void Filters::run_dlior_worker(int thread_number)
{
    for(int i =(inputCloud->size()/number_threads)*thread_number; i<= (inputCloud->size()/number_threads)*(thread_number+1);i++)
    {
        pcl::PointXYZI point = (*inputCloud)[i];
        double intensity_trehshold=parameter4;
        if(point._PointXYZI::intensity > intensity_trehshold)
        {
            mutex.lock();
            inliners.push_back(point);
            mutex.unlock();
        }
        else
        {
            filter_point(point,1);
        }
    }

}


void Filters::emit_frametime()
{
    ros::NodeHandle n;
    std_msgs::Int32 msg;
    filter_metrics = n.advertise<std_msgs::Int32>("alfa_filter_metrics", 1);
    msg.data = frameTime;
    filter_metrics.publish(msg);

}

void Filters::emit_exitpointcloud()
{
    ros::NodeHandle n;
    filter_pcloud = n.advertise<sensor_msgs::PointCloud2>("alfa_output_pcloud", 2);
    sensor_msgs::PointCloud2 pcl2_frame;
    pcl::toROSMsg(*OutputCloud,pcl2_frame);
    pcl2_frame.header.frame_id = "PclNoise";
    pcl2_frame.header.seq = pcl2_Header_seq;
    pcl2_frame.header.stamp = ros::Time::now();
    pcl2_Header_seq++;
    filter_pcloud.publish(pcl2_frame);
    //cout<<"emitted pointcloud";
}
