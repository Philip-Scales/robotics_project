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

#define FLOAT_COMP_EPS (0.001f)

//used for clustering
#define CLUSTER_THRESHOLD_BOTTOM 0.2 //bottom laser threshold
#define CLUSTER_THRESHOLD_TOP 0.3 //top laser threshold

//used for detection of motion
#define DETECTION_THRESHOLD 0.2 //threshold for motion detection
#define DYNAMIC_THRESHOLD 70 //to decide if a cluster is static or dynamic

//used for detection of moving legs
#define LEG_SIZE_MIN 0.05
#define LEG_SIZE_MAX 0.25

// used for detection of chests
#define CHEST_SIZE_MIN 0.15
#define CHEST_SIZE_MAX 0.60

// the max offset between the point between the legs and the middle of the chest
#define MAX_DISTANCE_BETWEEN_PERSON_AND_CHEST 0.12

//used for detection of moving persons
#define LEG_DISTANCE_MAX 0.7

using namespace std;

class moving_persons_detector {

private:
    static const int MAX_POINTS_PER_SCAN = 1000;
    ros::NodeHandle n;

    // subscribers for top and bottom lasers
    ros::Subscriber sub_scan_bottom;
    ros::Subscriber sub_scan_top;

    ros::Publisher pub_moving_persons_detector;
    ros::Publisher pub_moving_persons_detector_marker;

    // to store, process and display laserdata

    //! number of beams from the bottom laser
    int nb_beams_bottom;

    //! number of beams from the top laser
    int nb_beams_top;
    float range_min, range_max;
    float angle_min, angle_max, angle_inc;
    float range_bottom[MAX_POINTS_PER_SCAN];
    float range_top[MAX_POINTS_PER_SCAN];
    geometry_msgs::Point current_scan_bottom[MAX_POINTS_PER_SCAN];
    geometry_msgs::Point current_scan_top[MAX_POINTS_PER_SCAN];

    //to perform detection of motion
    float background[MAX_POINTS_PER_SCAN];//to store the background
    bool dynamic[MAX_POINTS_PER_SCAN];//to store if the current is dynamic or not

    //! \struct Cluster_s
    //! \brief Represents the data that corresponds to a single cluster of points
    struct Cluster_s {
        int nb_cluster;// number of cluster
        int cluster[MAX_POINTS_PER_SCAN]; //to store for each hit, the cluster it belongs to
        float cluster_size[MAX_POINTS_PER_SCAN];// to store the size of each cluster
        geometry_msgs::Point cluster_middle[MAX_POINTS_PER_SCAN];// to store the middle of each cluster
        int cluster_dynamic[MAX_POINTS_PER_SCAN];// to store the percentage of the cluster that is dynamic
        int cluster_start[MAX_POINTS_PER_SCAN], cluster_end[MAX_POINTS_PER_SCAN];
    };

    //! Cluster of the points obtained from the bottom laser
    Cluster_s cluster_bottom;

    //! Cluster of the points obtained from the top laser
    Cluster_s cluster_top;

    //to perform detection of moving legs and to store them
    int nb_moving_legs_detected;
    int nb_chests_detected = 0;
    geometry_msgs::Point moving_leg_detected[MAX_POINTS_PER_SCAN];// to store the middle of each moving leg
    geometry_msgs::Point chest_detected[MAX_POINTS_PER_SCAN]; // to store the middle of each detected chest
    geometry_msgs::Point merged_persons[MAX_POINTS_PER_SCAN]; // to store the middle of every detected person

    //! to perform detection of moving person and store them
    //! we detect that the person is moving when their legs are moving
    int nb_moving_persons_detected;

    //! number of true persons - the ones for whom we can detect both legs and the chest
    int nb_merged_persons = 0;

    //! to store the middle of each moving person
    geometry_msgs::Point moving_persons_detected[MAX_POINTS_PER_SCAN];

    //! track the current goal to reach we will be publishing and the previous
    //! goal to reach to be able to compare them
    geometry_msgs::Point goal_to_reach;
    geometry_msgs::Point old_goal_to_reach;


    // GRAPHICAL DISPLAY

    //! number of markers we want to put
    int nb_pts;

    //! positions of the markers
    geometry_msgs::Point display[2000];

    //! colors of the markers
    std_msgs::ColorRGBA colors[2000];

    //! to check if new data of laser is available or not
    bool new_laser;


    /// PERSON TRACKING
    /// Here, number of cycles === number of times the update function has been called

    //! Whether it is the first time we detect a person; this variable will also
    //! become \c true when we lose the previous person and try to find a new goal
    bool firstTimePersonDetected = true;

    //! The time period after which we consider that we lost a person
    static const constexpr float LOST_TIMEOUT_SEC = 5.5f;

    //! ROS node update frequency
    static const int UPDATE_FREQ_HZ = 10;

    //! number of cycles after which we consider that we lost a person
    int lostTimeoutCycles = 0;

    //! number of cycles that elapsed since the last time we detected someone
    int cyclesSinceLastDetection = 0;


    //! the initial value of the radius that we consider when we detect someone
    static const constexpr float DEFAULT_RADIUS = 1.0f;

    //! the default degree of certainity that the person is inside the circle around
    //! their last known position
    static const int DEFAULT_FREQUENCY = 5;

    //! maximum degree of certainity that the person is inside the circle around
    //! their last known position
    static const int MAX_FREQUENCY = 25;

    //! current degree of certainity that the person is inside the circle
    int currentFrequency = DEFAULT_FREQUENCY;

    //! current radius of the circle around the person that we consider
    float radiusRange = DEFAULT_RADIUS;

    //! value by which the radius can change per cycle
    static const constexpr float RADIUS_DELTA = 0.15f;

    //! the minimum radius of the circle around the person
    static const constexpr float MIN_RADIUS_RANGE = 0.50f;


public:

moving_persons_detector() {

    //! Subscription for the callback of the bottom laser scanner
    sub_scan_bottom = n.subscribe("scan2", 1, &moving_persons_detector::scanBottomCallback, this);

    //! Subscription for the callback of the top laser scanner
    sub_scan_top = n.subscribe("scan1", 1, &moving_persons_detector::scanTopCallback, this);

    pub_moving_persons_detector_marker = n.advertise<visualization_msgs::Marker>("moving_persons_detector", 1); // Preparing a topic to publish our results. This will be used by the visualization tool rviz
    pub_moving_persons_detector = n.advertise<geometry_msgs::Point>("goal_to_reach", 1);     // Preparing a topic to publish the goal to reach.

    new_laser = false;

    old_goal_to_reach.x = 1000;
    old_goal_to_reach.y = 1000;


    lostTimeoutCycles = UPDATE_FREQ_HZ * LOST_TIMEOUT_SEC;
    //INFINTE LOOP TO COLLECT LASER DATA AND PROCESS THEM
    ros::Rate r(UPDATE_FREQ_HZ);// this node will run at 10hz
    while (ros::ok()) {
        ros::spinOnce();//each callback is called once to collect new data: laser
        update();//processing of data
        r.sleep();//we wait if the processing (ie, callback+update) has taken less than 0.1s (ie, 10 hz)
    }

}

//UPDATE: main processing of laser data and robot_moving
/*//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////*/
void update() {

    // we wait for new data of the laser to perform laser processing
    if ( new_laser ) {
        new_laser = false;
        nb_pts = 0;

        // TODO: Describe steps here
        detect_motion();//to classify each hit of the laser as dynamic or not

        //! perform clustering for bottom and top laser scanners
        perform_clustering(cluster_bottom, current_scan_bottom, CLUSTER_THRESHOLD_BOTTOM, nb_beams_bottom);
        perform_clustering(cluster_top, current_scan_top, CLUSTER_THRESHOLD_TOP, nb_beams_top);
        detect_moving_legs();//to detect moving legs using cluster
        detect_moving_persons();//to detect moving_persons using moving legs detected
        detect_chests();
        merge_chests_and_persons();
        select_goal_to_reach();

        //graphical display of the results

        if (fabs(goal_to_reach.x) > FLOAT_COMP_EPS && fabs(goal_to_reach.y) > FLOAT_COMP_EPS) {
            //to publish the goal_to_reach
            if (std::hypot(goal_to_reach.x - old_goal_to_reach.x, goal_to_reach.y - old_goal_to_reach.y) > 0.0001) {
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

}// update

// DETECTION OF MOTION
/*//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////*/
void store_background() {
// store all the hits of the laser in the background table

    for (int loop = 0; loop < nb_beams_bottom; ++loop)
        background[loop] = range_bottom[loop];

}//init_background

void detect_motion() {

    for (int loop = 0; loop < nb_beams_bottom; ++loop ) {
        if (std::abs(background[loop] - range_bottom[loop]) > DETECTION_THRESHOLD) {
            dynamic[loop] = 1;//the current hit is dynamic
        } else {
            dynamic[loop] = 0;//else its static
        }
    }


}//detect_motion

// CLUSTERING
/*//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////*/
void perform_clustering(Cluster_s &cluster_ref, geometry_msgs::Point *current_scan, float threshold, int nb_beams) {
//store in the table cluster, the cluster of each hit of the laser
//if the distance between the previous hit and the current one is lower than "cluster_threshold"
//then the current hit belongs to the current cluster
//else we start a new cluster with the current hit and end the current cluster

    cluster_ref.nb_cluster = 0;//to count the number of cluster

    //initialization of the first cluster
    cluster_ref.cluster_start[0] = 0;// the first hit is the start of the first cluster
    cluster_ref.cluster[0] = 0;// the first hit belongs to the first cluster
    int nb_dynamic = 0;// to count the number of hits of the current cluster that are dynamic

    for( int loop = 1; loop < nb_beams; ++loop ) {//loop over all the hits
        auto curScan = current_scan[loop];
        auto prevScan = current_scan[loop - 1];
        if ((loop != nb_beams - 1) && (std::hypot(curScan.x - prevScan.x, curScan.y - prevScan.y) < threshold)) {
            cluster_ref.cluster[loop] = cluster_ref.nb_cluster;
        } else {//the current hit doesnt belong to the same hit
            cluster_ref.cluster_end[cluster_ref.nb_cluster] = loop - 1;
            //TODO: cluster_dynamic update

            cluster_ref.cluster_size[cluster_ref.nb_cluster] = std::hypot(current_scan[loop - 1].x - current_scan[cluster_ref.cluster_start[cluster_ref.nb_cluster]].x,
                                      current_scan[loop - 1].y - current_scan[cluster_ref.cluster_start[cluster_ref.nb_cluster]].y);

            cluster_ref.cluster_dynamic[cluster_ref.nb_cluster] = static_cast<int>(
                    100.0 * (static_cast<double>(nb_dynamic) / static_cast<double>(loop - cluster_ref.cluster_start[cluster_ref.nb_cluster])));
            auto &clMiddle = cluster_ref.cluster_middle[cluster_ref.nb_cluster];
            clMiddle.x = 0.5 * (current_scan[loop - 1].x + current_scan[cluster_ref.cluster_start[cluster_ref.nb_cluster]].x);
            clMiddle.y = 0.5 * (current_scan[loop - 1].y + current_scan[cluster_ref.cluster_start[cluster_ref.nb_cluster]].y);

            nb_dynamic = 0;// to count the number of hits of the current cluster that are dynamic
            cluster_ref.nb_cluster++;
            cluster_ref.cluster_start[cluster_ref.nb_cluster] = loop;
            cluster_ref.cluster[loop] = cluster_ref.nb_cluster;

        }
        if (dynamic[loop]) {
            ++nb_dynamic;
        }
    }


}//perfor_clustering



void detect_chests() {
    nb_chests_detected = 0;

    for (int loop = 0; loop < cluster_top.nb_cluster; ++loop) {
        int currentCluster = loop;
        if ((cluster_top.cluster_size[currentCluster] > CHEST_SIZE_MIN)
         && (cluster_top.cluster_size[currentCluster] < CHEST_SIZE_MAX)
        ) {

            // we update the moving_leg_detected table to store the middle of the moving leg
            chest_detected[nb_chests_detected] = cluster_top.cluster_middle[currentCluster];
            ++nb_chests_detected;

            //! Displays every point that belongs to the chest cluster in blue
#if 1
            for(int loop2 = cluster_top.cluster_start[loop]; loop2 <= cluster_top.cluster_end[loop]; ++loop2) {
                // moving legs are white
                display[nb_pts].x = current_scan_top[loop2].x;
                display[nb_pts].y = current_scan_top[loop2].y;
                display[nb_pts].z = current_scan_top[loop2].z;

                colors[nb_pts].r = 0;
                colors[nb_pts].g = 0.0;
                colors[nb_pts].b = 1.0;
                colors[nb_pts].a = 1.0;

                ++nb_pts;
            }
#endif
        }



    }
}


// DETECTION OF MOVING PERSON
/*//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////*/
void detect_moving_legs() {

    nb_moving_legs_detected = 0;

    for (int loop = 0; loop < cluster_bottom.nb_cluster; ++loop) {
        int currentCluster = loop;
        if ((cluster_bottom.cluster_size[currentCluster] > LEG_SIZE_MIN)
         && (cluster_bottom.cluster_size[currentCluster] < LEG_SIZE_MAX)
         && ((cluster_bottom.cluster_dynamic[currentCluster] > DYNAMIC_THRESHOLD) || !firstTimePersonDetected)) {

            // we update the moving_leg_detected table to store the middle of the moving leg
            moving_leg_detected[nb_moving_legs_detected] = cluster_bottom.cluster_middle[currentCluster];
            ++nb_moving_legs_detected;

            //! Display a marker for every point that belong to leg cluster
            for (int loop2 = cluster_bottom.cluster_start[loop]; loop2 <= cluster_bottom.cluster_end[loop]; ++loop2) {
                // moving legs are dark red
                display[nb_pts].x = current_scan_bottom[loop2].x;
                display[nb_pts].y = current_scan_bottom[loop2].y;
                display[nb_pts].z = current_scan_bottom[loop2].z;

                colors[nb_pts].r = 0.6;
                colors[nb_pts].g = 0;
                colors[nb_pts].b = 0;
                colors[nb_pts].a = 1.0;

                ++nb_pts;
            }
        }
    }
}

void detect_moving_persons() {
    //! a moving person has two moving legs located at less than "legs_distance_max" one from the other

    nb_moving_persons_detected = 0;

    //! if the distance between two moving legs is lower than "legs_distance_max"
    //! then we find a moving person
    for (int loop_leg1 = 0; loop_leg1 < nb_moving_legs_detected; ++loop_leg1) {
        for (int loop_leg2 = loop_leg1 + 1; loop_leg2 < nb_moving_legs_detected; ++loop_leg2) {

            if (std::hypot(moving_leg_detected[loop_leg1].x - moving_leg_detected[loop_leg2].x,
                           moving_leg_detected[loop_leg1].y - moving_leg_detected[loop_leg2].y) < LEG_DISTANCE_MAX) {
                //! we update the moving_persons_detected table to store the middle of the moving person
                auto &curPerson = moving_persons_detected[nb_moving_persons_detected];
                curPerson.x = 0.5 * (moving_leg_detected[loop_leg1].x + moving_leg_detected[loop_leg2].x);
                curPerson.y = 0.5 * (moving_leg_detected[loop_leg1].y + moving_leg_detected[loop_leg2].y);
                ++nb_moving_persons_detected;
            }
        }
    }
}



void merge_chests_and_persons() {
    int kk = 0;

    for (int ii = 0; ii < nb_moving_persons_detected; ++ii) {
        for (int jj = 0; jj < nb_chests_detected; ++jj) {
            geometry_msgs::Point person = moving_persons_detected[ii];
            geometry_msgs::Point chest  = chest_detected[jj];

            if (distancePoints(person, chest) < MAX_DISTANCE_BETWEEN_PERSON_AND_CHEST) {
                merged_persons[kk] = person;
                ++kk;
            }
        }
    }
    nb_merged_persons = kk;
}


void select_goal_to_reach() {

    float min_dist = 100000.0;
    bool detected = false;

    for (int ii = 0; ii < nb_merged_persons; ++ii) {

        auto curPerson = merged_persons[ii];
        //! First time detecting a person
        if (firstTimePersonDetected) {
            if (std::hypotf(curPerson.x, curPerson.y) < min_dist) {
                min_dist = std::hypotf(curPerson.x, curPerson.y);
                goal_to_reach = curPerson;
                radiusRange = DEFAULT_RADIUS;
                currentFrequency = DEFAULT_FREQUENCY;
            }
        }
        //! choosing a person closest to last detection
        else if (distancePoints(curPerson, old_goal_to_reach) < min_dist) {
            min_dist = distancePoints(curPerson, old_goal_to_reach);
            goal_to_reach = curPerson;
            //! person was detected outside circle
            if (min_dist > radiusRange) {
                goal_to_reach = old_goal_to_reach;
            } else {
                detected = true;
                radiusRange = MIN_RADIUS_RANGE;
            }
        }

        //! the moving persons are green
        display[nb_pts].x = moving_persons_detected[nb_moving_persons_detected].x;
        display[nb_pts].y = moving_persons_detected[nb_moving_persons_detected].y;
        display[nb_pts].z = moving_persons_detected[nb_moving_persons_detected].z;

        colors[nb_pts].r = 0;
        colors[nb_pts].g = 0.6;
        colors[nb_pts].b = 0;
        colors[nb_pts].a = 1.0;

        ++nb_pts;

    }

    firstTimePersonDetected = false;
    //! if no person detected within the circle: increase radius, decrease frequency
    if (!detected) {
        ++cyclesSinceLastDetection;
        if (currentFrequency > 0) {
            --currentFrequency;
            radiusRange += RADIUS_DELTA;
        }
    //! if person found in circle: increase frequency, decrease radius
    } else {
        cyclesSinceLastDetection = 0;

        if (currentFrequency < MAX_FREQUENCY) {
            ++currentFrequency;
        }

        if (radiusRange > MIN_RADIUS_RANGE) {
            radiusRange -= RADIUS_DELTA;
        }
    }
    //! officially lost track of person
    if (0 == currentFrequency) {
        firstTimePersonDetected = true;
    }

}


//! CALLBACKS
/*//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////*/
void scanBottomCallback(const sensor_msgs::LaserScan::ConstPtr& scan) {

    new_laser = true;
    // store the important data related to laserscanner
    range_min = scan->range_min;
    range_max = scan->range_max;
    angle_min = scan->angle_min;
    angle_max = scan->angle_max;
    angle_inc = scan->angle_increment;
    nb_beams_bottom = ((-1 * angle_min) + angle_max)/angle_inc;

    // store the range and the coordinates in cartesian framework of each hit
    float beam_angle = angle_min;
    for (int loop = 0; loop < nb_beams_bottom; ++loop, beam_angle += angle_inc) {
        if ((scan->ranges[loop] < range_max) && (scan->ranges[loop] > range_min))
            range_bottom[loop] = scan->ranges[loop];
        else
            range_bottom[loop] = range_max;

        //transform the scan in cartesian framework
        current_scan_bottom[loop].x = range_bottom[loop] * cos(beam_angle);
        current_scan_bottom[loop].y = range_bottom[loop] * sin(beam_angle);
        current_scan_bottom[loop].z = 0.0;
    }

}


void scanTopCallback(const sensor_msgs::LaserScan::ConstPtr& scan) {

    new_laser = true;
    // store the important data related to laserscanner
    range_min = scan->range_min;
    range_max = scan->range_max;
    angle_min = scan->angle_min;
    angle_max = scan->angle_max;
    angle_inc = scan->angle_increment;
    nb_beams_top = ((-1 * angle_min) + angle_max)/angle_inc;

    float beam_angle = angle_min;
    for (int loop = 0 ; loop < nb_beams_top; ++loop, beam_angle += angle_inc) {
        if ((scan->ranges[loop] < range_max) && (scan->ranges[loop] > range_min)) {
            range_top[loop] = scan->ranges[loop];
        } else {
            range_top[loop] = range_max;
        }

        //transform the scan in cartesian framework
        // Apply correction to compensate for the top laser displacement
        current_scan_top[loop].x = range_top[loop] * cos(beam_angle) - 0.10;
        current_scan_top[loop].y = range_top[loop] * sin(beam_angle) - 0.15;
        current_scan_top[loop].z = 0.0;
    }
}


//! Utilities
/*//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////*/
//! Distance between two points
float distancePoints(geometry_msgs::Point pa, geometry_msgs::Point pb) {

    return sqrt(pow((pa.x - pb.x), 2.0) + pow((pa.y - pb.y), 2.0));

}

//! Display the field of view and other references on RVIZ
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

    v.x = 0.02 * cos(-2.356194);
    v.y = 0.02 * sin(-2.356194);
    v.z = 0.0;
    references.points.push_back(v);

    v.x = 5.6 * cos(-2.356194);
    v.y = 5.6 * sin(-2.356194);
    v.z = 0.0;
    references.points.push_back(v);

    float beam_angle = -2.356194 + 0.006136;
    // first and last beam are already included
    for (int i = 0; i < 723; ++i, beam_angle += 0.006136){
        v.x = 5.6 * cos(beam_angle);
        v.y = 5.6 * sin(beam_angle);
        v.z = 0.0;
        references.points.push_back(v);
    }

    v.x = 5.6 * cos(2.092350);
    v.y = 5.6 * sin(2.092350);
    v.z = 0.0;
    references.points.push_back(v);

    v.x = 0.02 * cos(2.092350);
    v.y = 0.02 * sin(2.092350);
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

    for (int loop = 0; loop < nb_pts; ++loop) {
            geometry_msgs::Point p;
            std_msgs::ColorRGBA c;

            p.x = display[loop].x;
            p.y = display[loop].y;
            p.z = display[loop].z;

            c.r = colors[loop].r;
            c.g = colors[loop].g;
            c.b = colors[loop].b;
            c.a = colors[loop].a;

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
