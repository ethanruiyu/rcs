#ifndef TYPES_H
#define TYPES_H

//#include "nav_msgs/Odometry.h"
#include <opencv2/opencv.hpp>



struct TGlobalOrd
{
  double x;   /*!< x coordinate within global map (m) */
  double y;   /*!< y coordinate within global map (m) */

  bool operator== (const TGlobalOrd &o1)
  {
    return (this->x == o1.x && this->y == o1.y);
  }
};



#endif // TYPES_H

