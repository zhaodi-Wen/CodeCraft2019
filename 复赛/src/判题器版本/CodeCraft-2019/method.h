#include <string>
#include <vector>
#include <queue>
#include <ostream>
#include <fstream>
#include <iostream>
#include <sstream>
#include <unordered_map>//使用unorder map加快查找速度
#include <math.h>
#include <algorithm>
#include <numeric>
#if !defined(METHOD_H_)
#define METHOD_H_

using namespace std;

class Car;
class Road;
class Cross;

class Car {
  public:
    int id;                                                                         // 车辆id
    int from_id;                                                                    // 始发地路口id
    int to_id;                                                                      // 目的地路口id
    int max_speed;                                                                  // 最高时速
    int plan_time;                                                                  // 计划出发时间
    int priority;                                                                   //优先级问题
    int preset ;                                                                    //是否预先出发

    int status = 0;                                                                 // 0-未出发状态 1-等待行驶状态 2-终止状态 3-到达终点

    int speed = 0;                                                                  // 当前速度
    int current_time = plan_time;                                                   //当前车的时间
    unordered_map<int,int> road_weight;
    Road* at_road;                                                                  // 车辆当前所处的道路
    int at_channel_direction = 1;                                                    // 1-正向，2-反向
    int at_channel_id = -1;                                                         // 车辆当前所处的车道
    int at_road_position = 0;
    int could_move_distance=0;                                                       //可以移动的长度
    int current_road_idx = 0;
    // 车辆在道路中的位置，行驶时限制： 闭区间[1, road.length], 未发车时为0

    vector<Road*> path; // 车辆的行驶路径

    Car(int id_, int from_id_, int to_id_, int max_speed_, int plan_time_,int priority_ ,int preset_);
    Car(string s);
    void updateStatus(int);
    void changeSpeed(int);
    void changeRoad(Road*, int, int);
    void changePosition(int);
    void move();

    Car* getFrontCar(); // 获取前车

    /* 车辆在车库中出库的优先级
     * 1.　有空位优先级搞得先走
     * 1. 如果出发时间不同，则先出发者优先级高
     * 2. 如果出发时间相通，则id小者优先级高
     */
    struct cmpAtPort {
      bool operator () (Car*& a, Car*& b) {
        return !(
          (a->priority>b->priority && a->plan_time < b->plan_time)||
          ((a->priority==b->priority)&&(a->plan_time < b->plan_time)) ||
          ((a->priority==b->priority)&&(a->plan_time == b->plan_time) && (a->id < b->id))
        );
      }
    };

    /* 车辆在道路上时的优先级
     * 1.　优先级高的就是大佬,什么都优先
     * 2. 优先级相同,对于不同位置的车辆，靠前的先行
     * 2. 对于相通位置的车辆，车道序号小者先行
     */
    struct cmpAtRoad {
      bool operator () (Car*& a, Car*& b) {
        return !(
          (a->priority>b->priority)||
          (a->priority==b->priority && a->at_road_position > b->at_road_position) ||
          (a->priority==b->priority && a->at_road_position == b->at_road_position &&
            a->at_channel_id < b->at_channel_id)//优先级
        );
      }
    };


};

class Road {
  public:
    int id; // 道路id
    int length; // 道路长度
    int max_speed; // 最高限速
    int from_id; // 起点路口id
    int to_id; // 终点路口id
    int channels_num; // 车道数目
    bool is_duplex; // 是否双向道路

    int car_num = 0;//道路上的车的数目

    vector<queue<Car*>> s2e_channels = *(new vector<queue<Car*>>(channels_num,queue<Car*>())); // 正向车道 from_cross -> to_cross
    vector<queue<Car*>> e2s_channels = *(new vector<queue<Car*>>(channels_num,queue<Car*>()));; // 反向车道 to_cross -> from_cross
    queue<Car*> s2e_priority_queue;    //创建正向车道的优先队列
    queue<Car*> e2s_priority_queue;    //创建反方向的优先队列

    Road(int id_, int length_, int max_speed_, int from_id_, int to_id_,
        int channels_, bool is_duplex_);
    Road(string s);
    int getAvailableChannelIndex(vector<queue<Car*> >);
    void addCar(Car*, int);
};

class Cross {
  public:
    int id; // 路口id
    int top_road_id; // 0点方向路口id
    int right_road_id; // 3点方向路口id
    int bottom_road_id; // 6点方向路口id
    int left_road_id; // 9点方向路口id

    // 神奇车库，使用优先队列来实现，出库的时候按照id以及出发时间进行排序出发
    priority_queue<Car*, vector<Car*>, Car::cmpAtPort> car_port;

    Cross(int id_, int top_road_id_, int right_road_id_, int bottom_road_id_,
        int left_road_id_);
    Cross(string s);

    /* 向车库中添加一辆车 */
    int addCar(Car* c);
    // 重置结点
    void resetNode();
};
class Pre_set_car{
public:
    int id;//车辆的ID
    int plan_time;//车辆出发时间
    vector<int > path; //路线
    Pre_set_car(int id_, int plan_time_,vector<int> path_);
    Pre_set_car(string s);
};

class Traffic {
  public:
    // 为了保证全局状态统一，只允许路、路口、车辆的实例存在一份
    vector<Cross> crosses;
    vector<Road> roads;
    vector<Car> cars;
    vector<Pre_set_car> pre_set_cars;                                               /*预先发车的类*/
    vector<Car*> priority_cars;                                                      /*优先级的车的类*/
    vector<Car*> un_priority_cars;                                                   /*非优先级的车的类*/

    int total_time = 0;//时间问题                                                     /*当前调度时间,并行的*/
    int avg_len;
    int c1 = 100;//调参
    int c2 = 1;//调参
    int epoch=5000;//调参
    bool no_dead_lock = true;
//    int batch_size = ceil(cars.size()/epoch);

    unordered_map<int, int> car_id2index;                                           /*通过车的ID搜到的他的索引*/
    unordered_map<int, int> road_id2index;                                          /*通过路的ID搜到的他的索引*/
    unordered_map<int, int> cross_id2index;                                         /*通过路口的ID搜到的他的索引*/
    unordered_map<int, unordered_map<int, double>> id_time_weights;                 /*每条道路在不同时刻的权重值
 * {key:time ,value(key:roadID,value:weight)}*/

    unordered_map<int ,Road> road_ID_dict;                                          /*建立Road_ID和后面信息的字典*,值是Road的实例,这个会好一点*/
    unordered_map<int ,Car> car_ID_dict;                                            /*建立car_ID和后面信息的字典,值是Car的实例*/
    unordered_map<int , Cross> cross_ID_dict;                                       /*建立cross_ID和后面信息的字典,值是cross的实例*/
    unordered_map<int, int>  timeIndex ;                                            //建立预设的时间片的出发时间的字典,用来计算当前时间片的车的数量,以便于发车的批次选取
    template<typename T>
    unordered_map<int, int> buildTimeIndex(vector<T> &vs);                      /*建立timeIndex*/
    unordered_map<int,Car > num_map;

    void initTraffic(string car_path, string cross_path, string road_path,string pre_set_car);/*初始化文件信息*/
//    void initTraffic(vector<Car> cars, vector<Cross> crosses, vector<Road> roads);
    void buildIndex();                                                              /*建立上面的索引信息*/

    //分割函数
    vector<string> &split(const string &str, const string &delimiters, vector<string> &elems, bool skip_empty = false);

    void get_priority_or_unpriority_car(vector<Car*> &,vector<Car >, bool priority);/*根据优先级将车辆分为两部份,便于后边操作*/

    template<class TrafficInstance>
    vector<TrafficInstance> initInstance(string file_path);                      /**生成实例,就是初始化的时候用的*/
    template <typename T>
    vector<queue<T *>> init_queue(vector<T> vs);
    template<class Pre_set_car_Instance>
    vector<Pre_set_car_Instance> init_Pre_setCar(string file_path);

    template<typename T>
    unordered_map<int, int> buildMapIndex(vector<T>& vs,bool sort);;             /*用来建立上面的索引的函数*/

    template <typename T>
    unordered_map<int,T> build_ID_dict (vector<T>& vs);                          /*用来建立上面的对象map的函数*/

    void update_preset_car(vector<Car> &);

    int direction(Car &);                                                      /*获得在当前道路的方向,返回的是起点*/
    void driveCurrentRoad();                                                     /*调度函数*/
    void driveCarInitList(int t, bool priority);                                 //当前时间片
    void createCartSequence();                                                   //创建优先队列,是用于路口转向的
    bool moveToNextRoad(Car &car,int time);
    void portCarsToPort();
//    void change_car_start_time(pre_set_cars);
    void getPathOfCar(Car*,int time);                                                     /*获得车的路径*/
    void getAllCarPath();                                                        /*主函数*/
    double getWeightOfRange(int from_time, int to_time, int road_id, Car* car);  /*获取某条路在某个时间段的平均权重*/
    double getWeightOf(int, int, Car* car);                                      /* 获取某条路在某个时刻的权重 */
    void setWeightOf(int, int, double, Car*);                                    /*设置权重*/
    void init_preset_road_weight();                                              /*根据预设的车对道路进行权重设置*/
    void updateWeightsByPath(Car* car,int time);
    int getSpeedOf(Car* car, Road* road);
    int getTimeCostOf_pre(Car *car, Road *road);                                 /*获得预设的车对道路的权重设置,这里权重仅仅是长度/速度,后面可以改,这是我写的*/
    int getTimeCostOf(Car* car,Road* road,Cross* cross,int time);                         /*获得某个路口的平均权重,这个权重包括下歌路口的4条路的平均权重*/
    vector<Road*> getAdjRoadOfCross(Cross* cross);                               /*获得每个路口的邻接道路的实例*/
    Road* getRoadById(int);                                                      /* 根据id获取路的对象.这个可能和前面的方法有点相似 */
    Car* getCarById(int);                                                        /* 根据id获取Car的对象.这个可能和前面的方法有点相似 */
    Cross* getCrossById(int);                                                    /* 根据id获取Cross的对象.这个可能和前面的方法有点相似 */

    vector<string> path2string();                                                /*路径转string,用于输出*/
    void checkPath();                                                            /*检查路劲是否合法*/

    void computeAvglen();
    int get_start_road(Car &,int );
};

#endif // METHOD_H_
