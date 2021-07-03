#include "steepest.h"

steepest::steepest(/* args */)
{
}

steepest::~steepest()
{
}

std::vector<std::vector<float>> steepest::steepest_descent(int count, const std::vector<std::vector<float>> &_path)
{
    std::vector<std::vector<float>> result={{_path[0][0], _path[0][1]}};

    float gradient;
    float dx=0, dy=0;
    
    for(int i=1; i<_path.size()-1; i++)
    {
        dx=0, dy=0;
        std::vector<float> tmp={0.0,0.0};
        int cnt = count;

        while(cnt)  //迭代count次
        {
            //取前后两点之前的距离求导作为梯度
            // gradient = 2.0*dx + 2.0*_path[i][0] - 2.0*_path[i+1][0];
            gradient = 4.0*dx + 4.0*_path[i][0] - 2.0*_path[i-1][0] - 2.0*_path[i+1][0];
            dx = dx - gradient*step;
            // gradient = 2.0*dx + 2.0*_path[i][1] - 2.0*_path[i+1][1];
            gradient = 4.0*dx + 4.0*_path[i][1] - 2.0*_path[i-1][1] - 2.0*_path[i+1][1];
            dy = dy - gradient*step;    
            cnt--;
        }
        tmp[0]= _path[i][0] + dx;
        tmp[1]= _path[i][1] + dy;
        result.push_back(tmp);
    }

    result.push_back(_path[_path.size()-1]);

    return result;
}


