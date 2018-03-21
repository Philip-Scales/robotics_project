#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Quaternion.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include <cmath>
#include <tf/transform_datatypes.h>

//#define BASE5

class decision {
private:

    ros::NodeHandle n;

    // communication with person_detector or person_tracker
    ros::Publisher pub_goal_reached;
    ros::Subscriber sub_goal_to_reach;

    // communication with translation_action
    ros::Publisher pub_translation_to_do;
    ros::Subscriber sub_translation_done;

    bool cond_translation;//boolean to check if there is a /translation_to_do

    float rotation_to_do;
    float translation_to_do;
    float translation_done;

    bool new_goal_to_reach;//to check if a new /goal_to_reach is available or not
    bool new_translation_done;//to check if a new /translation_done is available or not

    geometry_msgs::Point goal_to_reach;
    geometry_msgs::Point goal_reached;

public:

decision() {

    // communication with moving_persons_detector or person_tracker
    pub_goal_reached = n.advertise<geometry_msgs::Point>("goal_reached", 1);
    sub_goal_to_reach = n.subscribe("goal_to_reach", 1, &decision::goal_to_reachCallback, this);

    // communication with translation_action
    pub_translation_to_do = n.advertise<geometry_msgs::Point>("translation_to_do", 0);
    sub_translation_done = n.subscribe("translation_done", 1, &decision::translation_doneCallback, this);
    cond_translation = false;

    new_goal_to_reach = false;
    new_translation_done = false;

    //INFINTE LOOP TO COLLECT LASER DATA AND PROCESS THEM
    ros::Rate r(10);// this node will work at 10hz
    while (ros::ok()) {
        ros::spinOnce();//each callback is called once
        update();
        r.sleep();//we wait if the processing (ie, callback+update) has taken less than 0.1s (ie, 10 hz)
    }

}

//UPDATE: main processing
/*//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////*/
void update() {

    // we receive a new /goal_to_reach and robair is not doing a translation or a rotation
    if (( new_goal_to_reach )) {

        ROS_INFO("(decision_node) /goal_to_reach received: (%f, %f)", goal_to_reach.x, goal_to_reach.y);

        // we have a rotation and a translation to perform
        // we compute the /translation_to_do
        translation_to_do = sqrt( ( goal_to_reach.x * goal_to_reach.x ) + ( goal_to_reach.y * goal_to_reach.y ) );

        if ( translation_to_do ) {
            cond_translation = true;

            //we compute the /rotation_to_do
            rotation_to_do = acos( goal_to_reach.x / translation_to_do );
            #ifdef BASE5
            rotation_to_do -= M_PI/12.0;
            #endif

            if ( goal_to_reach.y < 0 )
                rotation_to_do *=-1;


            geometry_msgs::Point movement_to_do;

            ROS_INFO("(decision_node) /rotation_to_do: %f  ", rotation_to_do*180/M_PI);
            ROS_INFO("(decision_node) /translation_to_do: %f", translation_to_do);
            
            movement_to_do.x = rotation_to_do;
            movement_to_do.y = translation_to_do;
            pub_translation_to_do.publish(movement_to_do);
            ROS_WARN(" ---------------  DECISION PUBLISHED MOVEMENT_TO_DO -------------------");
        }

    }
    new_goal_to_reach = false;

    
        

    //we receive an ack from translation_action_node. So, we send an ack to the moving_persons_detector_node
    if ( new_translation_done ) {
        ROS_INFO("(decision_node) /translation_done : %f\n", translation_done);
        cond_translation = false;
        new_translation_done = false;

        //the translation_to_do is done so we send the goal_reached to the detector/tracker node
        geometry_msgs::Point msg_goal_reached;

        //FIXME: Probably wrong
        msg_goal_reached.x = translation_to_do * std::cos(rotation_to_do);
        msg_goal_reached.y = translation_to_do * std::sin(rotation_to_do);
        ROS_INFO("(decision_node) /goal_reached (%f, %f)", msg_goal_reached.x, msg_goal_reached.y);
        //to complete
        pub_goal_reached.publish(msg_goal_reached);
        ROS_INFO(" ");
        ROS_INFO("(decision_node) waiting for a /goal_to_reach");
    }

}// update

//CALLBACKS
/*//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////*/
void goal_to_reachCallback(const geometry_msgs::Point::ConstPtr& g) {
// process the goal received from moving_persons detector

    new_goal_to_reach = true;
    goal_to_reach.x = g->x;
    goal_to_reach.y = g->y;

}


void translation_doneCallback(const std_msgs::Float32::ConstPtr& r) {
// process the range received from the translation node

    new_translation_done = true;
    translation_done = r->data;

}

// Distance between two points
float distancePoints(geometry_msgs::Point pa, geometry_msgs::Point pb) {

    return sqrt(pow((pa.x-pb.x),2.0) + pow((pa.y-pb.y),2.0));

}

};

int main(int argc, char **argv){

    ROS_INFO("(decision_node) waiting for a /goal_to_reach");
    ros::init(argc, argv, "decision");

    decision bsObject;

    ros::spin();

    return 0;
}
