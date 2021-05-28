#include "PlanningNode.h"


nav_msgs::Path getPathMsg(vector<pair<float, float>> path)
{
    nav_msgs::Path msg;

    ros::Time time = ros::Time::now();
    msg.header.frame_id = "map";
    msg.header.stamp = time;

    for (auto it:path)
    {
        geometry_msgs::PoseStamped pose;
        pose.header.stamp = time;
        pose.header.frame_id = "map";
        pose.pose.position.x = it.first - 1;
        pose.pose.position.y = it.second - 13.8;
        // pose.pose.position.z = 0.0;
        pose.pose.orientation.x = 0.0;
        pose.pose.orientation.y = 0.0;
        pose.pose.orientation.z = 0.0;
        pose.pose.orientation.w = 1.0;
        msg.poses.push_back(pose);
    }

    return msg;
}

void getGoal(const geometry_msgs::PoseStamped::ConstPtr msg, vector<pair<float, float>> *goal)
{
    // goal = msg;
    pair<float, float> target;
    int temp0;
    target.first = (*msg).pose.position.x+1.0;
    target.second = (*msg).pose.position.y+13.8;
    
    temp0 = target.first/0.05;
    target.first = temp0*0.05;
    temp0 = target.second/0.05;
    target.second = temp0*0.05;
    
    goal->push_back(target);
    rec_flag = true;
}

//获取当前位置的
void getPosition(const geometry_msgs::Pose::ConstPtr msg, pair<float, float> *starting)
{
    starting->first = (*msg).position.x+1.0;
    starting->second = (*msg).position.y+13.8;
}



void show(int* array,  int index_, int area_, int width)
{

    index_ = index_ - area_ - area_*width+1;
    area_ = 2*area_+1;

    for(int i=0; i<area_;i++)
    {
        for(int j=0; j<area_; j++)
        {
            cout<<array[index_ + i*width + j] <<" ";
        }

    cout<<endl;
    }
}




int main(int argc, char** argv)
{
    ros::init(argc, argv, "PlanningNode");
    ros::NodeHandle n;
 

    vector<int8_t> *cost;
    vector<pair<float, float>> goals;
    vector<vector<pair<float, float>>> paths_d; //用于存放迪杰斯克拉算法结果
    vector<vector<pair<float, float>>> paths_A;          //存放A*算法结果
    int *potential_A, *potential_D;
    nav_msgs::Path pathMSG_A;
    nav_msgs::Path pathMSG_D;

    pair<float, float> starting;

    astar A;
    A.init();
    A.calculateCOST();
    potential_A = new int[A.GetMapSize()];

    dijkstra D;
    D.init();
    D.calculateCOST();
    potential_D = new int[D.GetMapSize()];

    //get goal
    ros::Subscriber GetGoal = n.subscribe<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1, boost::bind(&getGoal, _1, &goals));
    //get position
    ros::Subscriber GetPosition = n.subscribe<geometry_msgs::Pose>("car_position", 1, boost::bind(&getPosition, _1, &starting));

    ros::Publisher pathpub_A = n.advertise<nav_msgs::Path>("Astar_path",3);
    ros::Publisher pathpub_D = n.advertise<nav_msgs::Path>("Dijkstra_path",3);
    // ros::Subscriber OdemSub = n.subscribe("odem", 1, getPosition);
    ros::Rate r(100);
    ROS_INFO("now start!");
    while (ros::ok())
    {
        r.sleep();
        ros::spinOnce();


        if(rec_flag)
        {
            smooth plan;
            ROS_INFO("from: %f, %f",starting.first, starting.second);
            ROS_INFO("goal: %f, %f",goals[0].first, goals[0].second );
            A.calculatePotential(starting.first, starting.second, goals, potential_A);
            gradient path_A(A.wid, A.hig);
            path_A.calculatePath(potential_A, starting.first, starting.second, goals, paths_A);

            paths_A.at(0) = plan.PathSmooth(paths_A.at(0));
            pathMSG_A = getPathMsg(paths_A.at(0));

             

            D.calculatePotential(starting.first, starting.second, goals, potential_D);
            gradient path_D(D.wid, D.hig);
            path_D.calculatePath(potential_D, starting.first, starting.second, goals, paths_d);
            
            paths_d.at(0) = plan.PathSmooth(paths_d.at(0));
            pathMSG_D = getPathMsg(paths_d.at(0));


/*          直接根据potential列出路径
            ros::Time time0 = ros::Time::now();
            pathMSG.header.frame_id = "map";
            pathMSG.header.stamp = time0;
            // int n = A.GetIndex( A.original_x, A.original_y);
            // int goal = A.GetIndex(goals[0].first, goals[0].second);

            int n = D.GetIndex( D.original_x, D.original_y);
            int goal = D.GetIndex(goals[0].first, goals[0].second);
            // ROS_INFO("goal is %f, %f", goals[0].first, goals[0].second);
            // float result = goal%A.wid;
            // ROS_INFO("%f", result);
            // ROS_INFO("goal is %f, %f", (goal%A.wid)*0.05f, (goal/A.wid)*0.05);
            // ROS_INFO("goal is %f, %f", (goal%A.wid)*0.05-1.0, (goal/A.wid)*0.05-13.8);

            int ptr = goal;
            while (goal!=n)
            {
                ptr = goal -1;
                // if(potential[goal +1]<potential[ptr])
                //     ptr= goal+1;
                // if(potential[goal+A.wid]<potential[ptr])
                //     ptr = goal + A.wid;
                // if(potential[goal-A.wid]<potential[ptr])
                //     ptr = goal-A.wid;

                if(potential[goal +1]<potential[ptr])
                    ptr= goal+1;
                if(potential[goal+D.wid]<potential[ptr])
                    ptr = goal + D.wid;
                if(potential[goal-D.wid]<potential[ptr])
                    ptr = goal-D.wid;
                goal = ptr;

                geometry_msgs::PoseStamped pose;
                pose.header.frame_id = "map";
                pose.header.stamp = time0;


                int tmp;
                pose.pose.position.x = (goal%A.wid)*0.05-1.0;
                pose.pose.position.y = (goal/A.wid)*0.05-13.8;
                pose.pose.orientation.w=1;
                pose.pose.orientation.x=0;
                pose.pose.orientation.y=0;
                pose.pose.orientation.z=0;
                pathMSG.poses.push_back(pose);
            }*/
            
            rec_flag = false;

            pathpub_A.publish(pathMSG_A);
            pathpub_D.publish(pathMSG_D);
            r.sleep();

            
        }
        paths_A.clear();
        paths_d.clear();
        goals.clear();
    }
}


