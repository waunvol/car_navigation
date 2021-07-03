#ifndef STEEP_H_
#define STEEP_H_
#include <vector>
#include <cmath>


class steepest
{
private:
    /* data */
public:
    float step=0.005;
    steepest(/* args */);
    ~steepest();
    std::vector<std::vector<float>> steepest_descent(int count, const std::vector<std::vector<float>> &_path);
};




#endif