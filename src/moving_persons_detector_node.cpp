// moving persons detector using lidar data
// written by O. Aycard

#include "ros/ros.h"
#include "ros/time.h"
#include "sensor_msgs/LaserScan.h"
#include "visualization_msgs/Marker.h"
#include "geometry_msgs/Point.h"
#include "std_msgs/ColorRGBA.h"
#include <cmath>
#include "std_msgs/Bool.h"

//used for clustering
#define cluster_threshold 0.2 //threshold for clustering

//used for detection of motion
#define detection_threshold 0.2 //threshold for motion detection
#define dynamic_threshold 70 //to decide if a cluster is static or dynamic

//used for detection of moving legs
#define leg_size_min 0.05
#define leg_size_max 0.25

//used for detection of moving persons
#define legs_distance_max 0.7

#define nbscans 100

using namespace std;

class moving_persons_detector {

private:
    ros::NodeHandle n;

    ros::Subscriber sub_scan;
//    ros::Subscriber sub_robot_moving;
    ros::Subscriber sub_goal_reached;

    ros::Publisher pub_moving_persons_detector;
    ros::Publisher pub_moving_persons_detector_marker;

    // to store, process and display laserdata
    int nb_beams;
    float range_min, range_max;
    float angle_min, angle_max, angle_inc;
    float range[1000];
    geometry_msgs::Point current_scan[1000];

    //to perform detection of motion
    float background[1000];//to store the background
    bool dynamic[1000];//to store if the current is dynamic or not

    //to perform clustering
    int nb_cluster;// number of cluster
    int cluster[1000]; //to store for each hit, the cluster it belongs to
    float cluster_size[1000];// to store the size of each cluster
    geometry_msgs::Point cluster_middle[1000];// to store the middle of each cluster
    int cluster_dynamic[1000];// to store the percentage of the cluster that is dynamic
    int cluster_start[1000], cluster_end[1000];

    //to perform detection of moving legs and to store them
    int nb_moving_legs_detected;
    geometry_msgs::Point moving_leg_detected[1000];// to store the middle of each moving leg

    //to perform detection of moving person and store them
    int nb_moving_persons_detected;
    geometry_msgs::Point moving_persons_detected[1000];// to store the middle of each moving person

    //to store the goal to reach that we will be published
    geometry_msgs::Point goal_to_reach;
    geometry_msgs::Point old_goal_to_reach;


    // GRAPHICAL DISPLAY
    int nb_pts;
    geometry_msgs::Point display[2000];
    std_msgs::ColorRGBA colors[2000];

    //to check if the robot is moving or not
    bool previous_robot_moving;
    bool current_robot_moving;

    bool new_laser;//to check if new data of laser is available or not
    bool new_robot;//to check if new data of robot_moving is available or not
    bool goal_reached;







    //! ======================================================================
    bool firstTime = true;
    static const constexpr float LOCK_RADIUS = 1.0f;
    static const constexpr float LOST_TIMEOUT_SEC = 5.5f;
    static const int UPDATE_FREQ_HZ = 10;

    int lostTimeoutCycles = 0;

    int cyclesSinceLastDetection = 0;

public:

moving_persons_detector() {

    sub_scan = n.subscribe("scan", 1, &moving_persons_detector::scanCallback, this);
//    sub_robot_moving = n.subscribe("robot_moving", 1, &moving_persons_detector::robot_movingCallback, this);
    sub_goal_reached = n.subscribe("goal_reached", 1, &moving_persons_detector::goal_reachedCallback, this);

    pub_moving_persons_detector_marker = n.advertise<visualization_msgs::Marker>("moving_persons_detector", 1); // Preparing a topic to publish our results. This will be used by the visualization tool rviz
    pub_moving_persons_detector = n.advertise<geometry_msgs::Point>("goal_to_reach", 1);     // Preparing a topic to publish the goal to reach.

    current_robot_moving = true;
    new_laser = false;
    new_robot = false;
    goal_reached = true;
    
    old_goal_to_reach.x = 1000;
    old_goal_to_reach.y = 1000;


    lostTimeoutCycles = UPDATE_FREQ_HZ * LOST_TIMEOUT_SEC;
    //INFINTE LOOP TO COLLECT LASER DATA AND PROCESS THEM
    ros::Rate r(UPDATE_FREQ_HZ);// this node will run at 10hz
    while (ros::ok()) {
        ros::spinOnce();//each callback is called once to collect new data: laser + robot_moving
        update();//processing of data
        r.sleep();//we wait if the processing (ie, callback+update) has taken less than 0.1s (ie, 10 hz)
    }

}

//UPDATE: main processing of laser data and robot_moving
/*//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////*/
void update() {

    goal_reached = false;
    // we wait for new data of the laser and of the robot_moving_node to perform laser processing
    if ( new_laser ) {
        new_laser = false;
        new_robot = false;
        nb_pts = 0;

        //if the robot is not moving then we can perform moving persons detection
        if ( 1 ) {

            ROS_INFO("robot is not moving");
            // if the robot was moving previously and now it is not moving now then we store the background
//            if ( previous_robot_moving && !current_robot_moving )
//                store_background();

            //we search for moving persons in 4 steps
            detect_motion();//to classify each hit of the laser as dynamic or not
            perform_clustering();//to perform clustering
            detect_moving_legs();//to detect moving legs using cluster
            detect_moving_persons();//to detect moving_persons using moving legs detected

            //graphical display of the results

            if (fabs(goal_to_reach.x) > 0.001 && fabs(goal_to_reach.y) > 0.001) {
                //to publish the goal_to_reach
                if (std::hypot(goal_to_reach.x - old_goal_to_reach.x, goal_to_reach.y - old_goal_to_reach.y) > 0.0001) {
                    ROS_INFO("ACHTUNG");
                    old_goal_to_reach = goal_to_reach;
                    display[nb_pts].x = goal_to_reach.x;
                    display[nb_pts].y = goal_to_reach.y;

                    colors[nb_pts].r = 1.0;
                    colors[nb_pts].g = 0.6;
                    colors[nb_pts].b = 0.0;

                    ++nb_pts;

                    pub_moving_persons_detector.publish(goal_to_reach);
                }
            }
            populateMarkerTopic();
        }
//        else
//            ROS_INFO("robot is moving");
//        ROS_INFO("\n");
    }
//    else
//        ROS_INFO("wait for data");

    
            

}// update

// DETECTION OF MOTION
/*//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////*/
void store_background() {
// store all the hits of the laser in the background table

    ROS_INFO("storing background");

    for (int loop=0; loop<nb_beams; loop++)
        background[loop] = range[loop];

}//init_background

void detect_motion() {

    ROS_INFO("detecting motion");

    for (int loop=0; loop<nb_beams; loop++ ) {
        if (std::abs(background[loop] - range[loop]) > detection_threshold) {
            dynamic[loop] = 1;//the current hit is dynamic
        } else {
            dynamic[loop] = 0;//else its static
        }
    }


}//detect_motion

// CLUSTERING
/*//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////*/
void perform_clustering() {
//store in the table cluster, the cluster of each hit of the laser
//if the distance between the previous hit and the current one is lower than "cluster_threshold"
//then the current hit belongs to the current cluster
//else we start a new cluster with the current hit and end the current cluster

    ROS_INFO("performing clustering");

    nb_cluster = 0;//to count the number of cluster

    //initialization of the first cluster
    cluster_start[0] = 0;// the first hit is the start of the first cluster
    cluster[0] = 0;// the first hit belongs to the first cluster
    int nb_dynamic = 0;// to count the number of hits of the current cluster that are dynamic

    for( int loop=1; loop<nb_beams; loop++ ) {//loop over all the hits
        auto curScan = current_scan[loop];
        auto prevScan = current_scan[loop - 1];
        if ((loop != nb_beams - 1) && (std::hypot(curScan.x - prevScan.x, curScan.y - prevScan.y) < cluster_threshold)) {
            cluster[loop] = nb_cluster;
        } else {//the current hit doesnt belong to the same hit
            cluster_end[nb_cluster] = loop - 1;
            //TODO: cluster_dynamic update

            cluster_size[nb_cluster] = std::hypot(current_scan[loop - 1].x - current_scan[cluster_start[nb_cluster]].x,
                                      current_scan[loop - 1].y - current_scan[cluster_start[nb_cluster]].y);

            cluster_dynamic[nb_cluster] = static_cast<int>(
                    100.0 * (static_cast<double>(nb_dynamic) / static_cast<double>(loop - cluster_start[nb_cluster])));
            auto &clMiddle = cluster_middle[nb_cluster];
            clMiddle.x = 0.5 * (current_scan[loop - 1].x + current_scan[cluster_start[nb_cluster]].x);
            clMiddle.y = 0.5 * (current_scan[loop - 1].y + current_scan[cluster_start[nb_cluster]].y);

            /*
              1/ we end the current cluster, so we update:
              - cluster-end to store the last hit of the current cluster
              - cluster_dynamic to store the percentage of hits of the current cluster that are dynamic
              - cluster_size to store the size of the cluster ie, the distance between the first hit of the cluster and the last one
              - cluster_middle to store the middle of the cluster
            */
            //textual display
            ROS_INFO("cluster[%i]: (%f, %f) -> (%f, %f), size: %f, dynamic: %i", nb_cluster,
                     current_scan[cluster_start[nb_cluster]].x, current_scan[cluster_start[nb_cluster]].y,
                     current_scan[cluster_end[nb_cluster]].x, current_scan[cluster_end[nb_cluster]].y,
                     cluster_size[nb_cluster], cluster_dynamic[nb_cluster]);

            //2/ we starta new cluster with the current hit
            nb_dynamic = 0;// to count the number of hits of the current cluster that are dynamic
            nb_cluster++;
            cluster_start[nb_cluster] = loop;
            cluster[loop] = nb_cluster;

        }
        if ( dynamic[loop] ) {
            nb_dynamic++;
        }
    }


}//perfor_clustering

// DETECTION OF MOVING PERSON
/*//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////*/
void detect_moving_legs() {
// a moving leg is a cluster:
// - with a size higher than "leg_size_min";
// - with a size lower than "leg_size_max;
// - more than "dynamic_threshold"% of its hits are dynamic (see, cluster_dynamic table)

//    ROS_INFO("detecting moving legs");
    nb_moving_legs_detected = 0;

    for (int loop=0; loop<nb_cluster; loop++) {
        int currentCluster = loop;
        if ((cluster_size[currentCluster] > leg_size_min)
         && (cluster_size[currentCluster] < leg_size_max)
         && ((cluster_dynamic[currentCluster] > dynamic_threshold) || !firstTime)) {

            // we update the moving_leg_detected table to store the middle of the moving leg
            moving_leg_detected[nb_moving_legs_detected] = cluster_middle[currentCluster];
            ++nb_moving_legs_detected;
            //textual display
            ROS_INFO("moving leg detected[%i]: cluster[%i]", nb_moving_legs_detected, loop);

            //graphical display
            for(int loop2=cluster_start[loop]; loop2<=cluster_end[loop]; loop2++) {
                // moving legs are white
                display[nb_pts].x = current_scan[loop2].x;
                display[nb_pts].y = current_scan[loop2].y;
                display[nb_pts].z = current_scan[loop2].z;

                colors[nb_pts].r = 0.6;
                colors[nb_pts].g = 0;
                colors[nb_pts].b = 0;
                colors[nb_pts].a = 1.0;

                nb_pts++;
            }
        }



    }

    if ( nb_moving_legs_detected ) {
        ROS_INFO("%d moving legs have been detected.\n", nb_moving_legs_detected);
    }

}//detect_moving_legs

void detect_moving_persons() {
    // a moving person has two moving legs located at less than "legs_distance_max" one from the other

//    ROS_INFO("detecting moving persons");
    nb_moving_persons_detected = 0;
    float min_dist = 100000.0;


    for (int loop_leg1=0; loop_leg1<nb_moving_legs_detected; loop_leg1++) {
        for (int loop_leg2=loop_leg1+1; loop_leg2<nb_moving_legs_detected; loop_leg2++) {
            //            if the distance between two moving legs is lower than "legs_distance_max"


            //            then we find a moving person

            if (std::hypot(moving_leg_detected[loop_leg1].x - moving_leg_detected[loop_leg2].x,
                           moving_leg_detected[loop_leg1].y - moving_leg_detected[loop_leg2].y) < legs_distance_max) {
                // we update the moving_persons_detected table to store the middle of the moving person
                auto &curPerson = moving_persons_detected[nb_moving_persons_detected];
                curPerson.x = 0.5 * (moving_leg_detected[loop_leg1].x + moving_leg_detected[loop_leg2].x);
                curPerson.y = 0.5 * (moving_leg_detected[loop_leg1].y + moving_leg_detected[loop_leg2].y);

                // textual display
                ROS_INFO("moving person detected[%i]: leg[%i]+leg[%i] -> (%f, %f)",
                         nb_moving_persons_detected, loop_leg1, loop_leg2,
                         moving_persons_detected[nb_moving_persons_detected].x,
                         moving_persons_detected[nb_moving_persons_detected].y);

                // the moving persons are green
                display[nb_pts].x = moving_persons_detected[nb_moving_persons_detected].x;
                display[nb_pts].y = moving_persons_detected[nb_moving_persons_detected].y;
                display[nb_pts].z = moving_persons_detected[nb_moving_persons_detected].z;

                colors[nb_pts].r = 0;
                colors[nb_pts].g = 0.6;
                colors[nb_pts].b = 0;
                colors[nb_pts].a = 1.0;

                nb_pts++;

                //update of the goal


                if (firstTime) {
                    if (std::hypotf(curPerson.x, curPerson.y) < min_dist) {
                        min_dist = std::hypotf(curPerson.x, curPerson.y);
                        goal_to_reach = curPerson;
                    }
                }
                else if (distancePoints(curPerson, old_goal_to_reach) < min_dist) {
                    min_dist = distancePoints(curPerson, old_goal_to_reach);
                    goal_to_reach = curPerson;

                    if (min_dist > LOCK_RADIUS) {
                        goal_to_reach = old_goal_to_reach;
                    }
                }



            }

        }
    }

                        firstTime = false;

    if (distancePoints(goal_to_reach, old_goal_to_reach) < 0.00001) {
        ++cyclesSinceLastDetection;
    } else {
        cyclesSinceLastDetection = 0;
    }

    if (cyclesSinceLastDetection >= lostTimeoutCycles) {
        cyclesSinceLastDetection = 0;
        firstTime = true;
    }


    if ( nb_moving_persons_detected )
        ROS_INFO("%d moving persons have been detected.\n", nb_moving_persons_detected);

}//detect_moving_persons

//CALLBACKS
/*//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////*/
void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan) {

    new_laser = true;
    // store the important data related to laserscanner
    range_min = scan->range_min;
    range_max = scan->range_max;
    angle_min = scan->angle_min;
    angle_max = scan->angle_max;
    angle_inc = scan->angle_increment;
    nb_beams = ((-1 * angle_min) + angle_max)/angle_inc;

    ROS_INFO("New data of laser received");
    ROS_INFO("range_min, range_max: %f, %f", range_min, range_max);
    ROS_INFO("angle_min: %f", angle_min*180/M_PI);
    ROS_INFO("angle_max: %f", angle_max*180/M_PI);
    ROS_INFO("angle_increment: %f", angle_inc*180/M_PI);
    ROS_INFO("number_of_beams: %d", nb_beams);

    // store the range and the coordinates in cartesian framework of each hit
    float beam_angle = angle_min;
    for ( int loop=0 ; loop < nb_beams; loop++, beam_angle += angle_inc ) {
        if ( ( scan->ranges[loop] < range_max ) && ( scan->ranges[loop] > range_min ) )
            range[loop] = scan->ranges[loop];
        else
            range[loop] = range_max;

        //transform the scan in cartesian framework
        current_scan[loop].x = range[loop] * cos(beam_angle);
        current_scan[loop].y = range[loop] * sin(beam_angle);
        current_scan[loop].z = 0.0;
    }

}//scanCallback

#if 0
void robot_movingCallback(const std_msgs::Bool::ConstPtr& state) {

    new_robot = true;
    ROS_INFO("New data of robot_moving received");
    previous_robot_moving = current_robot_moving;
    current_robot_moving = state->data;

}//robot_movingCallback
#endif

void goal_reachedCallback(const geometry_msgs::Point::ConstPtr& g) {

    goal_reached = true;
    ROS_INFO("___ **  GOAL REACHED CALLBACK ** ____");

}//goal_reachedCallback



// Distance between two points
float distancePoints(geometry_msgs::Point pa, geometry_msgs::Point pb) {

    return sqrt(pow((pa.x-pb.x),2.0) + pow((pa.y-pb.y),2.0));

}

// Draw the field of view and other references
void populateMarkerReference() {

    visualization_msgs::Marker references;

    references.header.frame_id = "laser";
    references.header.stamp = ros::Time::now();
    references.ns = "example";
    references.id = 1;
    references.type = visualization_msgs::Marker::LINE_STRIP;
    references.action = visualization_msgs::Marker::ADD;
    references.pose.orientation.w = 1;

    references.scale.x = 0.02;

    references.color.r = 1.0f;
    references.color.g = 1.0f;
    references.color.b = 1.0f;
    references.color.a = 1.0;
    geometry_msgs::Point v;

    v.x =  0.02 * cos(-2.356194);
    v.y =  0.02 * sin(-2.356194);
    v.z = 0.0;
    references.points.push_back(v);

    v.x =  5.6 * cos(-2.356194);
    v.y =  5.6 * sin(-2.356194);
    v.z = 0.0;
    references.points.push_back(v);

    float beam_angle = -2.356194 + 0.006136;
    // first and last beam are already included
    for (int i=0 ; i< 723; i++, beam_angle += 0.006136){
        v.x =  5.6 * cos(beam_angle);
        v.y =  5.6 * sin(beam_angle);
        v.z = 0.0;
        references.points.push_back(v);
    }

    v.x =  5.6 * cos(2.092350);
    v.y =  5.6 * sin(2.092350);
    v.z = 0.0;
    references.points.push_back(v);

    v.x =  0.02 * cos(2.092350);
    v.y =  0.02 * sin(2.092350);
    v.z = 0.0;
    references.points.push_back(v);

    pub_moving_persons_detector_marker.publish(references);

}

void populateMarkerTopic(){

    visualization_msgs::Marker marker;

    marker.header.frame_id = "laser";
    marker.header.stamp = ros::Time::now();
    marker.ns = "example";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::POINTS;
    marker.action = visualization_msgs::Marker::ADD;

    marker.pose.orientation.w = 1;

    marker.scale.x = 0.05;
    marker.scale.y = 0.05;

    marker.color.a = 1.0;

    //ROS_INFO("%i points to display", nb_pts);
    for (int loop = 0; loop < nb_pts; loop++) {
            geometry_msgs::Point p;
            std_msgs::ColorRGBA c;

            p.x = display[loop].x;
            p.y = display[loop].y;
            p.z = display[loop].z;

            c.r = colors[loop].r;
            c.g = colors[loop].g;
            c.b = colors[loop].b;
            c.a = colors[loop].a;

            //ROS_INFO("(%f, %f, %f) with rgba (%f, %f, %f, %f)", p.x, p.y, p.z, c.r, c.g, c.b, c.a);
            marker.points.push_back(p);
            marker.colors.push_back(c);
        }

    pub_moving_persons_detector_marker.publish(marker);
    populateMarkerReference();

}



};



int main(int argc, char **argv){

    ros::init(argc, argv, "moving_persons_detector");

    moving_persons_detector bsObject;

    ros::spin();

    return 0;
}
