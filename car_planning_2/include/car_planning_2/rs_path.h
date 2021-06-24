#ifndef RS_PATH__H
#define RS_PATH__H
#include <boost/math/constants/constants.hpp>
#include <cassert>
#include <typeinfo>
#include <vector>


class ReedsSheppStateSpace
{
public:

    enum ReedsSheppPathSegmentType { RS_NOP=0, RS_LEFT=1, RS_STRAIGHT=2, RS_RIGHT=3 };
    static const ReedsSheppPathSegmentType reedsSheppPathType[18][5];
    class ReedsSheppPath
    {
    public:
        ReedsSheppPath(const ReedsSheppPathSegmentType* type=reedsSheppPathType[0],
            double t=std::numeric_limits<double>::max(), double u=0., double v=0.,
            double w=0., double x=0.);

        double length() const { return totalLength_; }
        const ReedsSheppPathSegmentType* type_;
        double length_[5];
        double totalLength_;
    };
    double rho_=0.6;//TURNNING RADIUS

    double distance(double from[3], double to[3]);

    std::vector<int> CurveType(double from[3], double to[3]);

    std::vector<std::vector<float> > GetCurve(double from[3], double to[3], double step_size);

    ReedsSheppPath reedsShepp(double from[3], double to[3]);

public:
    void interpolate(double from[3], ReedsSheppPath &path, double seg, double p_next[3]);
};

#endif