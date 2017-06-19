#ifndef __POINTCLOUD_ACCUMURATOR_H__
#define __POINTCLOUD_ACCUMURATOR_H__

#include <ros/ros.h>

template <typename T>
class PointcloudAccumurator
{
public:
  class Points : public T
  {
  public:
    ros::Time stamp_;

    Points(const T &points, const ros::Time &stamp) :
      T(points), stamp_(stamp)
    {
    }
  };

  PointcloudAccumurator()
  {
  }

  PointcloudAccumurator(const ros::Duration &duration)
  {
    reset(duration);
  }

  void reset(const ros::Duration &duration)
  {
    time_to_hold_ = duration;
    clear();
  }

  void clear()
  {
    points_.clear();
  }

  void push(const Points &points)
  {
    for(auto it = points_.begin(); it != points_.end(); ++it)
    {
      if(it->stamp_ + time_to_hold_ < points.stamp_)
      {
        it = points_.erase(it);
        continue;
      }
      break;
    }
    points_.push_back(points);
  }

  typename std::list<Points>::iterator begin()
  {
    return points_.begin();
  }
  typename std::list<Points>::iterator end()
  {
    return points_.end();
  }

protected:
  ros::Duration time_to_hold_;
  std::list<Points> points_;

};




#endif
