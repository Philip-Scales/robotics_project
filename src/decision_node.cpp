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
#include <cmath>
#include "nav_msgs/Odometry.h"
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include "std_srvs/Empty.h"

#include "tf/transform_listener.h"
#include "tf/transform_broadcaster.h"
#include "message_filters/subscriber.h"
#include "tf/message_filter.h"

//#define BASE5
float robair_size = 0.2;//0.2 for small robair
#define security_distance 0.5
#define avoidance_distance 1.5
#define AV_DIST 0.7
#define AV_CHECK 0.5


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
        ros::Subscriber sub_translation_done;

        bool cond_translation;//boolean to check if there is a /translation_to_do
        bool obstacle_detected;
        bool ignore_goal;
        
        float avoidance_angle;       
        float rotation_to_do;
        float translation_to_do;
        geometry_msgs::Point translation_done;

        bool new_goal_to_reach;//to check if a new /goal_to_reach is available or not
        bool new_translation_done;//to check if a new /translation_done is available or not
        bool avoidance_active; //true if goal has been changed by avoidance code

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
        sub_translation_done = n.subscribe("translation_done", 1, &decision::translation_doneCallback, this);
        cond_translation = false;

        new_goal_to_reach = false;
        avoidance_active = false;
        new_translation_done = false;
        avoidance_angle = 30.0*3.14/180.0;

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
    beam_angle = angle_min;
    for (int loop=0; loop < nb_beams; loop++, beam_angle += angle_inc ) {
        float offset_scan_x = std::cos(angle) * current_scan[loop].x - std::sin(angle) * current_scan[loop].y;
        float offset_scan_y = std::sin(angle) * current_scan[loop].x + std::cos(angle) * current_scan[loop].y;
        if ((fabs(offset_scan_y) < width) 
            && (offset_scan_x > 0)) {
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
    
    //sleep(0.3);
    
    //check area immediately in front of robair, as in original
    geometry_msgs::Point closest_obstacle = checkCorridor(0,range_max, robair_size);
    if ((obstacle_detected)) {
        nb_pts=0;
        // closest obstacle is red
        display[nb_pts].x = closest_obstacle.x;
        display[nb_pts].y = closest_obstacle.y;
        display[nb_pts].z = closest_obstacle.z;

        colors[nb_pts].r = 0;
        colors[nb_pts].g = 0;
        colors[nb_pts].b = 1;
        colors[nb_pts].a = 1.0;
        nb_pts++;
        populateMarkerTopic();
    }
    else {
        decltype(closest_obstacle) pt;
        pt.x = 10000;
        pt.y = 10000;
        pt.z = 10000;
        closest_obstacle.x = 10000;
        
        closest_obstacle.y = 10000;
        
        closest_obstacle.z = 10000;
        
        //pub_closest_obstacle.publish(pt);
    }
    if ( distancePoints(closest_obstacle, previous_closest_obstacle) > 0.05 ) {
        ROS_INFO("              GEN closest obs: (%f; %f)", closest_obstacle.x, closest_obstacle.y);
        previous_closest_obstacle.x = closest_obstacle.x;
        previous_closest_obstacle.y = closest_obstacle.y;
    }
    
    
    // PANIC Obstacle detection
    if (std::hypot(closest_obstacle.x, closest_obstacle.y) < security_distance) {
        geometry_msgs::Point movement_to_do;
        
        movement_to_do.x = 0.0; //translation
        movement_to_do.y = 0.0; //rotation
        pub_translation_to_do.publish(movement_to_do);
        ROS_WARN(" ---------------  DECISION PUBLISHED STOP (OBS) -------------------");
        return;
    }
    
    
    //later:
    // increment angle when neither corridor is free
    //maybe don't need to make sure WHOLE corridor is free?  just enough to move in one step?
    
    //CAREFUL Collision avoidance
    //  Now we know we are not in immediate danger of collision.
    //  check if we can go towards our goal (determine if need obstacle AVOIDANCE)
    
    //  calculate goal angle and distance
    goal_distance = sqrt( ( goal_to_reach.x * goal_to_reach.x ) + ( goal_to_reach.y * goal_to_reach.y ) );
    goal_angle = acos(goal_to_reach.x / goal_distance);
    if ( goal_to_reach.y < 0 )
        goal_angle *=-1;
    ROS_INFO("'''',,,, goal polar coords: dist = %lf angle %lf\n ,,,,'''''\n", goal_distance, goal_angle);
    
    //  check the corridor between robair and goal
    //  CLOSEST OBSTACLE IS IN ROBAIR'S COORDINATE FRAME
#if 0
    closest_obstacle = checkCorridor(goal_angle, goal_distance, robair_size);
    
    float distanceObsGoal = std::hypot(fabs(closest_obstacle.x - goal_to_reach.x), fabs(closest_obstacle.y - goal_to_reach.y));

    if (std::hypot(closest_obstacle.x, closest_obstacle.y) < avoidance_distance
        && distanceObsGoal > security_distance+10) {   
        //if (1) {     
        // ## ROBAIR RED POINT IF DOING AVOIDANCE
        display[nb_pts].x = -0.001;
        display[nb_pts].y = 0;
        display[nb_pts].z = 0;

        colors[nb_pts].r = 1;
        colors[nb_pts].g = 0;
        colors[nb_pts].b = 0;
        colors[nb_pts].a = 1.0;
        nb_pts++;
        populateMarkerTopic();
        
        //if we have to avoid obstacle, change goal, then proceed as usual
        geometry_msgs::Point movement_to_do;
        //length up to which we check the corridor (hypothenuse of the triangle: robair - obstacle - avoidance goal
        //float corridor_length = std::hypot(closest_obstacle.x, closest_obstacle.y) / std::cos(avoidance_angle);
        //ROS_WARN("corridor_length = %lf avoidance_angle = %lf\n", corridor_length, avoidance_angle);

        //check corridor to right of goal corridor
        closest_obstacle = checkCorridor(goal_angle + avoidance_angle, AV_CHECK, robair_size);
        float tempA = goal_angle + avoidance_angle; //rotation
        float goalx = AV_DIST * std::cos(tempA);
        float goaly = AV_DIST * std::sin(tempA);
        
        display[nb_pts].x = goalx;
        display[nb_pts].y = goaly;
        display[nb_pts].z = 0;

        colors[nb_pts].r = 0;
        colors[nb_pts].g = 1;
        colors[nb_pts].b = 1;
        colors[nb_pts].a = 1.0;
        nb_pts++;
        populateMarkerTopic();
        
        if (std::hypot(closest_obstacle.x, closest_obstacle.y) > AV_CHECK) {
            //corridor is free, use it
            ROS_WARN("  - - -  USING RIGHT CORRIDOR - - -\n ");
            goal_distance = AV_DIST; //translation
            goal_angle = tempA; //rotation
            avoidance_active = true;
        }
        else {
            //check corridor to right of goal corridor
            checkCorridor(goal_angle - avoidance_angle, AV_CHECK, robair_size);
            tempA = goal_angle - avoidance_angle; //rotation
            goalx = AV_DIST * std::cos(tempA);
            goaly = AV_DIST * std::sin(tempA);
            
            display[nb_pts].x = goalx;
            display[nb_pts].y = goaly;
            display[nb_pts].z = 0;

            colors[nb_pts].r = 1;
            colors[nb_pts].g = 1;
            colors[nb_pts].b = 0;
            colors[nb_pts].a = 1.0;
            nb_pts++;
            populateMarkerTopic();
            if (std::hypot(closest_obstacle.x, closest_obstacle.y) > AV_CHECK) {
                //corridor if free, use it
                ROS_WARN("  - - - USING LEFT CORRIDOR - - -\n ");
                goal_distance = AV_DIST; 
                goal_angle = tempA; //rotation
                avoidance_active = true;
            }
            else {
                ROS_WARN("  - - - NO CORRIDORS ARE FREE - - -\n ");
                goal_distance = 0.0; //translation
                //keep rotation the same.
                pub_translation_to_do.publish(movement_to_do);
                ROS_WARN(" ---------------  DECISION PUBLISHED STOP  (NCF) -------------------");
                return;
                
            }
        }
    }
    
#endif
    // we receive a new /goal_to_reach and robair is not doing a translation or a rotation
    if ( new_goal_to_reach) {

        //ROS_INFO("(decision_node) /goal_to_reach received: (%f, %f)", goal_to_reach.x, goal_to_reach.y);

        if ( goal_distance ) { //if we need to move more than 0 meters
            cond_translation = true; 
            geometry_msgs::Point movement_to_do;

            ROS_INFO("(decision_node) /rotation_to_do: %f  ", goal_angle*180/M_PI);
            ROS_INFO("(decision_node) /translation_to_do: %f", goal_distance);
            
            movement_to_do.x = goal_angle;
            movement_to_do.y = goal_distance;
            pub_translation_to_do.publish(movement_to_do);
            ROS_WARN(" ---------------  DECISION PUBLISHED MOVEMENT_TO_DO -------------------");
            
            //put magenta point at goal
            float goalx = goal_distance * std::cos(goal_angle);
            float goaly = goal_distance * std::sin(goal_angle);
            display[nb_pts].x = goalx+0.1;
            display[nb_pts].y = goaly+0.1;
            display[nb_pts].z = 0;

            colors[nb_pts].r = 1;
            colors[nb_pts].g = 0;
            colors[nb_pts].b = 1;
            colors[nb_pts].a = 1.0;
            nb_pts++;
            populateMarkerTopic();
            
            display[nb_pts].x = goal_to_reach.x-0.1;
            display[nb_pts].y = goal_to_reach.y-0.1;
            display[nb_pts].z = 0;

            colors[nb_pts].r = 1;
            colors[nb_pts].g = 0.5;
            colors[nb_pts].b = 0.5;
            colors[nb_pts].a = 1.0;
            nb_pts++;
            populateMarkerTopic();
        }

    }
    new_goal_to_reach = false;
    avoidance_active = false;
    goal_distance = 0.0;
    goal_angle = 0.0;

    
        

    //we receive an ack from translation_action_node. So, we send an ack to the moving_persons_detector_node
    if ( new_translation_done ) {
        ROS_INFO("(decision_node) /translation_done rot: %f trans: %f\n", translation_done.x, translation_done.y);
        cond_translation = false;
        new_translation_done = false;
        
    }
    ROS_INFO("\n");

}// update

//CALLBACKS
/*//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////*/
// - - - -------Decision callbacks
void goal_to_reachCallback(const geometry_msgs::Point::ConstPtr& g) {
// process the goal received from moving_persons detector

    new_goal_to_reach = true;
    goal_to_reach.x = g->x;
    goal_to_reach.y = g->y;

}


void translation_doneCallback(const geometry_msgs::Point::ConstPtr& p) {
// process the range received from the translation node

    new_translation_done = true;
    translation_done.x = p->x;
    translation_done.y = p->y;

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
