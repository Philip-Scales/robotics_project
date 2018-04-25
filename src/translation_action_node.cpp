#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "visualization_msgs/Marker.h"
#include "geometry_msgs/Point.h"
#include "std_msgs/ColorRGBA.h"
#include "std_msgs/Float32.h"
#include <cmath>
#include "nav_msgs/Odometry.h"
#include <tf/transform_datatypes.h>

#include <geometry_msgs/Twist.h>
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Quaternion.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Bool.h"
#include <cmath>
#include <tf/transform_datatypes.h>
#include "geometry_msgs/Point.h"

using namespace std;

#define rotation_error 0.2//radians

#define security_distance 0.5
#define translation_error 0.1

//rotation coefs
#define rkp 0.9
#define rkd 0.00225
#define rki -0.003 //029

//translation coefs
#define tkp 0.5
#define tki 0.0012
#define tkd 0.005



class translation_action {
private:
    
    // communication with decision
    ros::Publisher pub_rotation_done;
    ros::Subscriber sub_rotation_to_do;
    ros::Publisher pub_arrived;

    float rotation_to_do, rotation_done;

    bool new_rotation_to_do;//to check if a new /rotation_to_do is available or not
    bool cond_rotation;
    float init_orientation;
    float current_orientation;
    float previous_orientation;

    float r_error_integral;
    float r_error_previous;
    float t_error_integral;
    float t_error_previous;
    
    ros::NodeHandle n;

    // communication with odometry
    ros::Subscriber sub_odometry;

    // communication with decision
    ros::Publisher pub_movement_done;
    ros::Subscriber sub_translation_to_do;

    // communication with cmd_vel to send command to the mobile robot
    ros::Publisher pub_cmd_vel;

    // communication with obstacle_detection
    ros::Subscriber sub_obstacle_detection;

    bool new_translation_to_do;//to check if a new /translation_to_do is available or not
    bool new_odom;//to check if new data from odometry are available
    bool init_obstacle;//to check if the first "closest_obstacle" has been published or not
    bool will_publish_arrive = true;

    geometry_msgs::Point start_position;
    geometry_msgs::Point current_position;

    float translation_to_do, translation_done;


    int cond_translation;

    geometry_msgs::Point closest_obstacle;
    geometry_msgs::Point movement_to_do;

public:

translation_action() {

    // communication with cmd_vel to command the mobile robot
    pub_cmd_vel = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);

    // communication with odometry
    sub_odometry = n.subscribe("odom", 1, &translation_action::odomCallback, this);
    cond_rotation = false;
    cond_translation = false;
    new_rotation_to_do = false;
    new_translation_to_do = false;
    new_odom = false;
    init_obstacle = false;

    // communication with decision
    pub_arrived = n.advertise<std_msgs::Bool>("arrived", 1);
    pub_movement_done = n.advertise<geometry_msgs::Point>("movement_done", 1);
    sub_translation_to_do = n.subscribe("translation_to_do", 1, &translation_action::translation_to_doCallback, this);//this is the translation that has to be performed


    t_error_integral = 0;
    t_error_previous = 0;
    r_error_integral = 0;
    r_error_previous = 0;

    //INFINTE LOOP TO COLLECT LASER DATA AND PROCESS THEM
    ros::Rate r(10);// this node will run at 10hz
    while (ros::ok()) {
        ros::spinOnce();//each callback is called once to collect new data: laser + robot_moving
        update();//processing of data
        r.sleep();//we wait if the processing (ie, callback+update) has taken less than 0.1s (ie, 10 hz)
    }
}

//UPDATE: main processing
/*//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////*/
void update() {
    float rotation_speed = 0;
    float translation_speed = 0;
    
    // we receive a new "movement"  (translation) to do
    if ( new_translation_to_do && new_odom ) {
        translation_done = distancePoints( start_position, current_position );
        //reset rotation related stuff
        r_error_integral = 0;
        r_error_previous = 0;
        new_translation_to_do = false;
        cond_rotation = true;

        init_orientation = current_orientation;
        rotation_done = current_orientation;
        
                //determine scaled translation based on how much rotation there is to do
        //translation_to_do = std::min((double)translation_to_do, std::max(0.0f, 45-std::fabs(rotation_to_do))/90.0);
        double normalAngle = (M_PI/2.0);
        double coeff = std::max(0.0, (normalAngle-std::fabs(rotation_to_do))/normalAngle);
        translation_to_do = translation_to_do * coeff;
        
        rotation_to_do += current_orientation;
        

        r_error_previous = rotation_to_do;

        if ( rotation_to_do > M_PI )
            rotation_to_do -= 2*M_PI;
        if ( rotation_to_do < -M_PI )
            rotation_to_do += 2*M_PI;
            
        //reset translation related stuff
        t_error_integral = 0;
        t_error_previous = 0;
        new_translation_to_do = false;
        cond_translation = true;
        init_obstacle = false;

        start_position.x = current_position.x;
        start_position.y = current_position.y;
        ROS_INFO("\n(translation_action_node) processing the /rotation_to_do received from the decision node");
        ROS_WARN("(translation_action_node) rotation_to_do: %f", rotation_to_do*180/M_PI);
        ROS_INFO("\n(translation_action_node) processing the /translation_to_do received from the decision node");
        ROS_WARN("(translation_action_node) translation_to_do: %f", translation_to_do);


    }
    //we are performing a rotation
    if ( new_odom && cond_rotation ) {
        rotation_done = current_orientation;
        float error = ( rotation_to_do - rotation_done );
        if ( error > M_PI ) {
            ROS_WARN("(translation_action node) error > 180 degrees: %f degrees -> %f degrees", error*180/M_PI, (error-2*M_PI)*180/M_PI);
            error -= 2*M_PI;
        }
        else {
            if ( error < -M_PI ) {
                ROS_WARN("(translation_action node) error < -180 degrees: %f degrees -> %f degrees", error*180/M_PI, (error+2*M_PI)*180/M_PI);
                error += 2*M_PI;
            }
        }
        
        cond_rotation = ( fabs(error) > rotation_error );
        if ( cond_rotation ) {
            //Implementation of a PID controller for rotation_to_do;
            float error_derivation;
            error_derivation = abs(error - r_error_previous);
            r_error_previous = error;
            ROS_INFO("error_derivaion: %f", error_derivation);
            r_error_integral += error;
            ROS_INFO("r_error_integral: %f", r_error_integral);
            //control of rotation with a PID controller
            rotation_speed = rkp * error + rki * r_error_integral + rkd * error_derivation;
            ROS_INFO("(translation_action_node) current_orientation: %f, orientation_to_reach: %f -> rotation_speed: %f", rotation_done*180/M_PI, rotation_to_do*180/M_PI, rotation_speed*180/M_PI);
        }
        else { // this will rarely (if ever) run, since now we get new translation to do before finishing current one
            ROS_INFO("(translation_action_node) current_orientation: %f, orientation_to_reach: %f -> rotation_speed: %f", rotation_done*180/M_PI, rotation_to_do*180/M_PI, rotation_speed*180/M_PI);
            rotation_done -= init_orientation;
            if ( rotation_done > M_PI ) {
                rotation_done -= 2*M_PI;
            }
            if ( rotation_done < -M_PI ) {
                rotation_done += 2*M_PI;
            }
            ROS_INFO("(translation_action_node) final rotation_done: %f", rotation_done*180/M_PI);
            ROS_INFO("(translation_action_node) waiting for a /rotation_to_do");
            r_error_integral = 0;
            r_error_previous = 0;
        }
    }
    

    if ( new_odom && cond_translation) {
        translation_done = distancePoints( start_position, current_position );
        float error = translation_to_do - translation_done;
        cond_translation = ( fabs(error) > translation_error );
        
        translation_speed = 0;
        if ( cond_translation ) {
            float error_derivation;
            error_derivation = error - t_error_previous;
            t_error_previous = error; // we didn't have this before...
            ROS_INFO("error_derivaion: %f", error_derivation);
            t_error_integral += error;
            ROS_INFO("t_error_integral: %f", t_error_integral);
            //control of translation with a PID controller
            translation_speed = tkp * error + tki * t_error_integral + tkd * error_derivation;
            ROS_INFO("(translation_action_node) translation_done: %f, translation_to_do: %f -> translation_speed: %f", translation_done, translation_to_do, translation_speed);
        }
        else { // this will rarely (if ever) run, since now we get new translation to do before finishing current one
            ROS_INFO("(translation_action_node) final translation_done: %f", translation_done);
            ROS_INFO("(translation_action_node) waiting for a /translation_to_do");
            init_obstacle = false;
        }
        
        if (will_publish_arrive && !cond_translation && !cond_rotation) {
            //publish i've arrived to decision node
            ROS_WARN(":::::  ARRIVED  ::::\n");
            std_msgs::Bool arrived;
            arrived.data = true;
            pub_arrived.publish(arrived);
            will_publish_arrive = false;
        } 
        else {
        
            //Publishing translation done
            geometry_msgs::Point msg_movement_done;
            msg_movement_done.x = current_orientation - previous_orientation;
            msg_movement_done.y = translation_done;
            pub_movement_done.publish(msg_movement_done);
            
            //Publishing command to cmd_vel
            geometry_msgs::Twist twist;
            twist.linear.x = translation_speed;//we perform a translation on the x-axis
            twist.linear.y = 0;
            twist.linear.z = 0;
            twist.angular.x = 0;
            twist.angular.y = 0;
            twist.angular.z = rotation_speed;
            pub_cmd_vel.publish(twist);
            ROS_WARN("X 6-----------------   PUBLISHED TWIST  ---------------6 X");
        }
    }
    new_odom = false;

}// update

//CALLBACKS
/*//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////*/
void odomCallback(const nav_msgs::Odometry::ConstPtr& o) {

    new_odom = true;
    current_position.x = o->pose.pose.position.x;
    current_position.y = o->pose.pose.position.y;
    current_position.z = o->pose.pose.position.z;
    previous_orientation = current_orientation;
    current_orientation = tf::getYaw(o->pose.pose.orientation);


}

void translation_to_doCallback(const geometry_msgs::Point::ConstPtr & r) {
// process the translation to do received from the decision node

    new_translation_to_do = true;
    rotation_to_do = r->x;
    translation_to_do = r->y;

    if (fabs(rotation_to_do) > 0.001 && fabs(translation_to_do > 0.001)) {
        will_publish_arrive = true;
    }

}

void closest_obstacleCallback(const geometry_msgs::Point::ConstPtr& obs) {

    init_obstacle = true;
    closest_obstacle.x = obs->x;
    closest_obstacle.y = obs->y;
    closest_obstacle.z = obs->z;

}//closest_obstacleCallback

// Distance between two points
float distancePoints(geometry_msgs::Point pa, geometry_msgs::Point pb) {

    return sqrt(pow((pa.x-pb.x),2.0) + pow((pa.y-pb.y),2.0));

}


};




int main(int argc, char **argv){

    ROS_INFO("(rotation_node) waiting for a /translation_to_do");
    ros::init(argc, argv, "translation_action");

    translation_action tbsObject;

    ros::spin();

    return 0;
}


