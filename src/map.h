#ifndef _MAP_H_
#define _MAP_H_

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <geolib/datatypes.h>

class Map
{

public:

    Map();

    Map(int width, int height);

    ~Map();

    void setResolution(double res) { res_ = res; }

    void setOrigin(const geo::Vec3& origin) { origin_ = origin; }

    void setSizeAndClear(int width, int height);

    bool worldToMap(double wx, double wy, int& mx, int& my) const
    {
        if (wx < origin_.x || wy < origin_.y)
            return false;

        mx = (wx - origin_.x) / res_ ;
        my = (wy - origin_.y) / res_ ;

        if (mx < image.cols && my < image.rows)
            return true;

        return false;
    }

    void mapToWorld(unsigned int mx, unsigned int my, double& wx, double& wy) const
    {
        wx = origin_.x + (mx + 0.5) * res_;
        wy = origin_.y + (my + 0.5) * res_;
    }

    double resolution() const { return res_; }

    const geo::Vec3& origin() const { return origin_; }

    int width_in_cells() const { return image.cols; }

    int height_in_cells() const { return image.rows; }

    cv::Mat image;

private:

    double res_;

    geo::Vec3 origin_;

};

#endif
