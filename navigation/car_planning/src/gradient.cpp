#include "gradient.h"

void gradient::calculatePath(int *potential, float start_x, float start_y, vector<pair<float, float>> goal, vector<vector<pair<float, float>>> &path)
{
    for (int i = 0; i < goal.size(); i++)
    {
        vector<pair<float, float>> tmp;
        path.push_back(tmp);
        pathSearch(potential, start_x, start_y, goal.at(i).first, goal.at(i).second, path.at(i));
    }
}


bool gradient::pathSearch(int *potential, float start_x, float start_y, float end_x, float end_y, vector<pair<float, float>> &path)
{
    gradient_init();    //单元个斜率初始化为0
    //从终点往起点处开始搜索，
    int goal = getIndex(end_x, end_y);
    pair<float, float> now;

#define step 0.05

    float dx = 0, dy = 0;
    bool bound=0;

    int n = goal;
    grady[n] = 0;
    gradx[n] = 0;

    float nx = end_x, ny= end_y;

    while (1)
    {
        nx = (n%wid+dx)*0.05;
        ny = (n/wid+dy)*0.05;

        n = getIndex(nx, ny);

        //将当前点加入到path
        now.first = nx;
        now.second = ny;
        path.push_back(now);
        if(fabs(nx - start_x) <= 0.1 && fabs(ny - start_y) <= 0.1)
            now.second = ny;

        if(fabs(nx - start_x) <= 0.05 && fabs(ny - start_y) <= 0.05)
        {
            ROS_INFO("Gradient completed!!!");
            return true;
        }

        bool oscillation_detected = false;  

        //检测震荡
        int cell_now = path.size();
        if(cell_now>2 && path[cell_now-1].first == path[cell_now-3].first
        && path[cell_now-1].second == path[cell_now-3].second)
        {
            ROS_DEBUG("[PathCalc] oscillation detected, attempting fix.");
            oscillation_detected = true;
        }

        if(potential[n]>=65536 || potential[n+1]>=65536 || potential[n-1]>=65536 ||
        potential[n+wid]>=65536 || potential[n+wid+1]>=65536 || potential[n+wid-1]>=65536 ||
        potential[n-wid]>=65536 || potential[n-wid+1]>=65536 || potential[n-wid-1]>=65536
        || oscillation_detected)
        {
            int n_cur = n-1;
            if(potential[n+1]<potential[n_cur])
            {    
                n_cur = n+1;
                }
            if(potential[n+wid]<potential[n_cur])
            {    
                n_cur = n+wid;
                }
            if(potential[n-wid]<potential[n_cur])
            {    
                n_cur = n-wid;
                }
            if(potential[n+wid+1]<potential[n_cur])
            {    
                n_cur = n+wid+1;
                }
            if(potential[n+wid-1]<potential[n_cur])
            {    
                n_cur = n+wid-1;
                }
            if(potential[n-wid+1]<potential[n_cur])
            {    
                n_cur = n-wid+1;
                }
            if(potential[n-wid-1]<potential[n_cur])
            {    
                n_cur = n-wid-1;
                }
            n = n_cur;
            dx = 0;
            dy = 0;
            if(potential[n]>=65536)
            {
                ROS_INFO("Gradient fialed, please try again");
                return 0;
            }
        }
        else
        {
            //取4个格子，算出其对应相邻4个的斜率并然后再乘上差值得到dxdy，然后再这里的大循环按权重再重新计算斜率进行运算
            addGrad(n, potential);
            addGrad(n + 1, potential);
            addGrad(n + wid, potential);
            addGrad(n + wid + 1, potential);

            //哪个近哪个权重高
            float x1 = (0.05 - dx) * gradx[n] + dx * gradx[n + 1];
            float x2 = (0.05 - dx) * gradx[n + wid] + dx * gradx[n + wid + 1];
            float x = (0.05 - dy) * x1 + dy * x2; // interpolated x， 根据权重取
            float y1 = (0.05 - dx) * grady[n] + dx * grady[n + 1];
            float y2 = (0.05 - dx) * grady[n + wid] + dx * grady[n + wid + 1];
            float y = (0.05 - dy) * y1 + dy * y2; // interpolated y， 根据权重取值

            float ss = step / hypot(x, y);  //斜率
            dx += x * ss;
            dy += y * ss;

            // ROS_INFO("dx %f", dx);
            // ROS_INFO("dy %f", dy);
            // check for overflow
            if (dx > 0.05) {
                n++;
                dx -= 0.05;
            }
            if (dx < -0.05) {
                n--;
                dx += 0.05;
            }
            if (dy > 0.05) {
                n += wid;
                dy -= 0.05;
            }
            if (dy < -0.05) {
                n -= wid;
                dy += 0.05;
            }
        }
    }
    ROS_INFO("done!");
}

void gradient::addGrad(int now, int* potential)
{
    float dx=0, dy=0;
    if (gradx[now] + grady[now] > 0.0)    // check this cell
        return;

    if(potential[now-1] < undefined)
        dx = potential[now - 1] - potential[now];
    if(potential[now+1] < undefined)
        dx += potential[now] - potential[now + 1];
    if(potential[now-wid] < undefined)
        dy = potential[now - wid] - potential[now];
    if(potential[now+wid] < undefined)
        dy += potential[now] - potential[now + wid];


    float norm = hypot(dx, dy);
    if(norm>0)  //若不加此判断，在邻近终点时，可能会出现 norm=0导致计算结果溢出，程序报错
    {
        norm = 1 / norm;    
        gradx[now] = norm * dx;
        grady[now] = norm * dy;
    }
    
}

