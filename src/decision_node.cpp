#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Quaternion.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include <cmath>
#include <tf/transform_datatypes.h>

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "visualization_msgs/Marker.h"
#include "geometry_msgs/Point.h"
#include "std_msgs/ColorRGBA.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Bool.h"
#include <cmath>
#include "nav_msgs/Odometry.h"
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include "std_srvs/Empty.h"

#include "tf/transform_listener.h"
#include "tf/transform_broadcaster.h"
#include "message_filters/subscriber.h"
#include "tf/message_filter.h"
#include "globals.h"




//#define BASE5
float robair_size = 0.2;//0.2 for small robair
#define security_distance 0.5
#define avoidance_distance 1.5
#define AV_DIST 0.7
#define AV_CHECK 0.6


class decision {
    private:
        ros::NodeHandle n;
        
        //obs dec
        // communication with laser_scanner
        ros::Subscriber sub_laser;

        // communication with action
        ros::Publisher pub_closest_obstacle;
        ros::Publisher pub_closest_obstacle_marker;

        // to store, process and display laserdata
        int nb_beams;
        float beam_angle;
        float range_min, range_max;
        float angle_min, angle_max, angle_inc;
        float range[1000];
        geometry_msgs::Point current_scan[1000];

        // communication with person_detector or person_tracker
        ros::Publisher pub_goal_reached;
        ros::Subscriber sub_goal_to_reach;

        // communication with translation_action
        ros::Publisher pub_translation_to_do;
        ros::Subscriber sub_movement_done;
        ros::Subscriber sub_arrived;

        bool cond_translation;//boolean to check if there is a /translation_to_do
        bool obstacle_detected;
        
        float avoidance_angle;       
        float rotation_to_do;
        float translation_to_do;
        geometry_msgs::Point movement_done;

        bool new_goal_to_reach;//to check if a new /goal_to_reach is available or not
        bool new_movement_done;//to check if a new /movement_done is available or not
        bool avoidance_active; //true if robot is performing avoidance
        bool arrived;

        geometry_msgs::Point goal_to_reach;
        float goal_angle; // angle to goal in robair's frame
        float goal_distance; // distance between robair and goal
        
        geometry_msgs::Point goal_reached;
        geometry_msgs::Point closest_obstacle;
        geometry_msgs::Point previous_closest_obstacle;

            // GRAPHICAL DISPLAY
        int nb_pts;
        geometry_msgs::Point display[2000];
        std_msgs::ColorRGBA colors[2000];


    public:

    decision() {
        
        pub_closest_obstacle_marker = n.advertise<visualization_msgs::Marker>("closest_obstacle", 1);

        // Communication with laser scanner for obs dec
        sub_laser = n.subscribe("scan2", 1, &decision::scanCallback, this);

        // communication with moving_persons_detector or person_tracker
        pub_goal_reached = n.advertise<geometry_msgs::Point>("goal_reached", 1);
        sub_goal_to_reach = n.subscribe("goal_to_reach", 1, &decision::goal_to_reachCallback, this);

        // communication with translation_action
        pub_translation_to_do = n.advertise<geometry_msgs::Point>("translation_to_do", 0);
        sub_movement_done = n.subscribe("movement_done", 1, &decision::movement_doneCallback, this);
        sub_arrived = n.subscribe("arrived", 1, &decision::arrivedCallback, this);

        cond_translation = false;

        new_goal_to_reach = false;
        avoidance_active = false;
        new_movement_done = false;
        arrived = false;
        avoidance_angle = M_PI_4;
        goal_to_reach.x = 0;
        goal_to_reach.y = 0;
        movement_done.x = 0;
        movement_done.y = 0;

        //INFINTE LOOP TO COLLECT LASER DATA AND PROCESS THEM
        ros::Rate r(10);// this node will work at 10hz
        while (ros::ok()) {
            ros::spinOnce();//each callback is called once
            update();
            r.sleep();//we wait if the processing (ie, callback+update) has taken less than 0.1s (ie, 10 hz)
        }
}

geometry_msgs::Point checkCorridor(float angle, float length, float width) {   
    if (length < 0.0) 
        ROS_WARN("  Pozor   !\n");

    geometry_msgs::Point closest_obstacle;
    closest_obstacle.x = range_max;
    closest_obstacle.y = range_max;

    obstacle_detected = false;
    float offset_closest_obstacle_x = range_max;

    //start with checking whole corridor to the new avoidance_goal. if dynamic obs steps in front, just panic for now.
    for (int loop = 0; loop < nb_beams; ++loop) {
        float offset_scan_x = std::cos(angle) * current_scan[loop].x + std::sin(angle) * current_scan[loop].y;
        float offset_scan_y = -std::sin(angle) * current_scan[loop].x + std::cos(angle) * current_scan[loop].y;
        if ((fabs(offset_scan_y) < width) 
            && (current_scan[loop].x > 0)) {
            //scan hit in corridor, scan is closer than closest,  and not behind us
            
            if (fabs(offset_scan_x) < fabs(offset_closest_obstacle_x)) {
                offset_closest_obstacle_x = offset_scan_x;
                closest_obstacle.x = current_scan[loop].x;
                closest_obstacle.y = current_scan[loop].y;
                closest_obstacle.z = current_scan[loop].z;
                obstacle_detected = true;
            }
            //ROS_INFO("closest obstacle: (%f; %f)", closest_obstacle.x, closest_obstacle.y);
        }
    }
    return closest_obstacle;
}

//UPDATE: main processing
/*//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////*/
void update() {  

    //  calculate goal angle and distance
    goal_distance = sqrt( ( goal_to_reach.x * goal_to_reach.x ) + ( goal_to_reach.y * goal_to_reach.y ) );
    goal_angle = acos(goal_to_reach.x / goal_distance);
    if (std::isnan(goal_angle)) {
        goal_angle = 0;
    }
    if ( goal_to_reach.y < 0 )
        goal_angle *=-1;
    ROS_INFO("'''',,,, goal polar coords: dist = %lf angle %lf\n ,,,,'''''\n", goal_distance, goal_angle);
    
#if 1
    closest_obstacle = checkCorridor(goal_angle, goal_distance, robair_size);
    float distanceToObs = std::hypot(closest_obstacle.x, closest_obstacle.y);

    if (distanceToObs < security_distance) {
        goal_angle = 0.0;
        goal_distance = 0.0;
        new_goal_to_reach = true;
    } else if (!avoidance_active) {


        float distanceObsGoal = std::hypot(fabs(closest_obstacle.x - goal_to_reach.x), fabs(closest_obstacle.y - goal_to_reach.y));

        if (distanceObsGoal < 0.35) {

        } else if (distanceToObs < avoidance_distance) { /* avoid */
            auto target_dist = distanceToObs * 1.5;
            auto left_obs = checkCorridor(goal_angle + avoidance_angle, target_dist, robair_size);
            auto right_obs = checkCorridor(goal_angle - avoidance_angle, target_dist, robair_size);

            if ((std::hypot(left_obs.x, left_obs.y) > target_dist)) {
                goal_angle = avoidance_angle;
                goal_distance = target_dist+1.0;
                avoidance_active = true;
            } else if (std::hypot(right_obs.x, right_obs.y) > target_dist) {
                goal_angle = -avoidance_angle;
                goal_distance = target_dist+1.0;
                avoidance_active = true;
            } else {
                goal_angle = 0;
                goal_distance = 0;
                /* STOP */
            }
            new_goal_to_reach = true;
        }
    }
    
   
#endif
    // we receive a new /goal_to_reach and robair is not doing a translation or a rotation

    //ROS_INFO("(decision_node) /goal_to_reach received: (%f, %f)", goal_to_reach.x, goal_to_reach.y);

    if ( new_goal_to_reach ) {
        new_goal_to_reach = false;
        geometry_msgs::Point movement_to_do;

        ROS_INFO("(decision_node) /rotation_to_do: %f  ", goal_angle*180/M_PI);
        ROS_INFO("(decision_node) /translation_to_do: %f", goal_distance);
        
        movement_to_do.x = goal_angle;
        movement_to_do.y = goal_distance;
        pub_translation_to_do.publish(movement_to_do);
        

    }
}// update

//CALLBACKS
/*//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////*/
// - - - -------Decision callbacks
void goal_to_reachCallback(const geometry_msgs::Point::ConstPtr& g) {
// process the goal received from moving_persons detector

    new_goal_to_reach = true;
    avoidance_active = false;
    goal_to_reach.x = g->x;
    goal_to_reach.y = g->y;

}


void movement_doneCallback(const geometry_msgs::Point::ConstPtr& p) {

    new_movement_done = true;
    movement_done.x = p->x;
    movement_done.y = p->y;

}

void arrivedCallback(const std_msgs::Bool::ConstPtr& a) {
    arrived = a->data;
}



// - - - ------- obstacle callbacks
void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan) {

    //new_laser = true;

    nb_beams = ((-1 * scan->angle_min) + scan->angle_max)/scan->angle_increment;
    range_min = scan->range_min;
    range_max = scan->range_max;
    angle_min = scan->angle_min;
    angle_max = scan->angle_max;
    angle_inc = scan->angle_increment;
    nb_beams = ((-1 * angle_min) + angle_max)/angle_inc;

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

    pub_closest_obstacle_marker.publish(references);

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

    pub_closest_obstacle_marker.publish(marker);
    populateMarkerReference();

}


};

int main(int argc, char **argv){

    ROS_INFO("(decision_node) waiting for a /goal_to_reach");
    ros::init(argc, argv, "decision");

    decision bsObject;

    ros::spin();

    return 0;
}
