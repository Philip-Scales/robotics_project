#ifndef _GLOBALS_H_
#define _GLOBALS_H_

#include <math.h>
#include "geometry_msgs/Point.h"
#include <iostream>


geometry_msgs::Point cart_to_polar(geometry_msgs::Point cartPoint) {
    geometry_msgs::Point polarPoint;
    
    polarPoint.y = sqrt( ( cartPoint.x * cartPoint.x ) + ( cartPoint.y * cartPoint.y ) );
    polarPoint.x = acos(cartPoint.x / polarPoint.y);
    if ( cartPoint.y < 0 )
        polarPoint.x *=-1;
            
    return polarPoint;
}

geometry_msgs::Point polar_to_cart(geometry_msgs::Point polarPoint) {
    geometry_msgs::Point cartPoint;
    
    cartPoint.x = polarPoint.y * std::cos(polarPoint.x);
    cartPoint.y = polarPoint.y * std::sin(polarPoint.x);
    
    return cartPoint;
}

//y to left is + so angles are probably wrong
geometry_msgs::Point get_adjusted_goal(geometry_msgs::Point oldGoal, geometry_msgs::Point movement_done) {
    std::cout << " INPUT PT " << oldGoal.x << " " << oldGoal.y << std::endl;
    std::cout << " MVT DONE " << movement_done.x << " " << movement_done.y << std::endl;
    
    geometry_msgs::Point cartNewGoal;
    geometry_msgs::Point polarNewGoal;
    geometry_msgs::Point polarOldGoal = cart_to_polar(oldGoal);
    
    float g = polarOldGoal.y;
    float t = movement_done.y;
    float r = movement_done.x - polarOldGoal.x; //rotated - orginal_yaw
    float u = sqrt(g*g + t*t - 2*g*t*std::cos(r));
    float theta = std::asin(g*std::sin(r)/u);

#if 1
    if (t < g*std::cos(r)) {
        polarNewGoal.x = -theta;
    }
    else {
        if (r > 0) {
            polarNewGoal.x = theta-M_PI/2;
        }
        else {
            polarNewGoal.x = theta+M_PI/2;
        }
    }
#else    
    if (theta < 0) {
        polarNewGoal.x = theta;
    }
    else {
        polarNewGoal.x = -theta;
    }
#endif
    polarNewGoal.y = u;

    
    cartNewGoal = polar_to_cart(polarNewGoal);
    std::cout << "OUTPUT PT " << cartNewGoal.x << " " << cartNewGoal.y << std::endl;
    std::cout << "    THETA " << theta << std::endl;
    

    
    return cartNewGoal;
}


#endif //_GLOBALS_H_
