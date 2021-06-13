#include "dijkstra.h"



bool dijkstra::calculatePotential(float start_x, float start_y, vector<pair<float, float>> destination, int* potential)
{
    if(!CheckCostMap())
        return false;

    vector<int> goal;
    for(auto it=destination.begin(); it!=destination.end(); it++ )
    {
        goal.push_back(GetIndex(it->first, it->second));
    }

    //初始化数组
    int *curBUFF = new int[wid * hig];
    int *nextBUFF = new int[wid * hig];
    int *swapPTR;

    std::fill(potential, potential + wid*hig, 65536);
    bool *pending = new bool[wid*hig*2];
    memset(pending, 0, sizeof(bool)*wid*hig*2);
    int curPTR = 0, nextPTR=0;

    int start_i = GetIndex(start_x, start_y);
    potential[start_i] = 0;
    pushCur(start_i + 1);
    pushCur(start_i - 1);
    pushCur(start_i + wid);
    pushCur(start_i - wid);

    while (1)
    {
        int *buffptr=curBUFF;
        int i = curPTR;

        // cout << *(++buffptr) << endl;
        while (i)
        {
            buffptr++;            
            pending[*buffptr] = false;
            i--;
        }

        i = curPTR;
        while (i)
        {
            //搜索相邻最小的potential

            int Bound_h = 0, Bound_v=0; //越界标志
            int pot_h, pot_v;
            //检查有无超界
            if((curBUFF[i]+1)%wid < curBUFF[i]%wid)
            {
                pot_h = potential[curBUFF[i] + 1];
                Bound_h++;
                }
            else if((curBUFF[i]-1)%wid>curBUFF[i]%wid)
            { 
                pot_h = potential[curBUFF[i] - 1];
                Bound_h++;
                }
            if(curBUFF[i] - wid<0)
            {
                pot_v = potential[curBUFF[i] + wid];
                Bound_v++;
                }
            else if(curBUFF[i] + wid>wid*hig)
            {
                pot_v = potential[curBUFF[i] - wid];
                Bound_v++;
                }
            
            if(!Bound_h)//若无超界，则正常进行比较
                pot_h = (potential[curBUFF[i] - 1] <= potential[curBUFF[i] + 1]) ? potential[curBUFF[i] - 1] : potential[curBUFF[i] + 1];
            if(!Bound_v)
                pot_v = (potential[curBUFF[i] - wid] <= potential[curBUFF[i] + wid]) ? potential[curBUFF[i] - wid] : potential[curBUFF[i] + wid];
            
            int pot = ((pot_h <= pot_v)?pot_h: pot_v) + (*COST)[curBUFF[i]];    //取得最小值

            

            if(pot < potential[curBUFF[i]])
            {
                potential[curBUFF[i]] = pot;
                //COST值符合才会加入下一轮迭代
                if ((*COST)[curBUFF[i]-1]<60 && (*COST)[curBUFF[i]-1]>0 && potential[curBUFF[i]-1] > pot + (*COST)[curBUFF[i]-1] && pending[curBUFF[i]-1]==false)
                    pushNext(curBUFF[i]-1);
                if ((*COST)[curBUFF[i]+1]<60 && (*COST)[curBUFF[i]+1]>0 && potential[curBUFF[i]+1] > pot + (*COST)[curBUFF[i]+1] && pending[curBUFF[i]+1]==false)
                    pushNext(curBUFF[i]+1);
                if ((*COST)[curBUFF[i]-wid]<60 && (*COST)[curBUFF[i]-wid]>0 && potential[curBUFF[i]-wid] > pot + (*COST)[curBUFF[i]-wid] && pending[curBUFF[i]-wid]==false)
                    pushNext(curBUFF[i]-wid);
                if ((*COST)[curBUFF[i]+wid]<60 && (*COST)[curBUFF[i]+wid]>0 && potential[curBUFF[i]+wid] > pot + (*COST)[curBUFF[i]+wid] && pending[curBUFF[i]+wid]==false)
                    pushNext(curBUFF[i]+wid);
            }

            i--;
        }
        //swap
        buffptr = curBUFF;
        curBUFF = nextBUFF;
        nextBUFF = buffptr;
        curPTR = nextPTR;
        nextPTR = 0;

        if(complete(goal, potential))
        {
            ROS_INFO("Dijikstra done!");
            return true;
        }
    }
    return false;
}

bool dijkstra::complete(vector<int> goal, int* potential)
{
    int succeed = goal.size();
    for (int i = 0; i < goal.size(); i++)
    {
        //当有已搜索点是，succeed减一，若所有点均已搜索，则succeed为0
        if(potential[goal.at(i)] < 65536)
            succeed--;
    }

    if(succeed)
        return false;
    return true;
}
