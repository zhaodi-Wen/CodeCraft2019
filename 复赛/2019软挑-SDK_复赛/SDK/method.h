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
    int id; // 车辆id
    int from_id; // 始发地路口id
    int to_id; // 目的地路口id
    int max_speed; // 最高时速
    int plan_time; // 计划出发时间
    int priority; //优先级问题
    int preset ;  //是否预先出发

    int status = 0; // 0-未出发状态 1-等待行驶状态 2-终止状态

    int speed = 0; // 当前速度
    int current_time = plan_time;

    Road* at_road; // 车辆当前所处的道路
    int at_channel_direction = 1; // 1-正向，2-反向
    int at_channel_id = -1; // 车辆当前所处的车道
    int at_road_position = 0;
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

    vector<queue<Car*> > s2e_channels; // 正向车道 from_cross -> to_cross
    vector<queue<Car*> > e2s_channels; // 反向车道 to_cross -> from_cross

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
    vector<Pre_set_car> pre_set_cars;


    int avg_len;
    int c1 = 100;//调参
    int c2 = 1;//调参
    int epoch=5000;//调参
//    int batch_size = ceil(cars.size()/epoch);

    unordered_map<int, int> car_id2index;
    unordered_map<int, int> road_id2index;
    unordered_map<int, int> cross_id2index;
    unordered_map<int, unordered_map<int, double>> id_time_weights;/*每条道路在不同时刻的权重值
 * {key:time ,value(key:roadID,value:weight)}*/

    unordered_map<int ,Road> road_ID_dict;/*建立Road_ID和后面信息的字典*/
    unordered_map<int ,Car> car_ID_dict;/*建立car_ID和后面信息的字典*/
    unordered_map<int , Cross> cross_ID_dict;
    unordered_map<int, int>  timeIndex ;  //建立预设的时间片的出发时间的字典

    void initTraffic(string car_path, string cross_path, string road_path,string pre_set_car);
    void initTraffic(vector<Car> cars, vector<Cross> crosses, vector<Road> roads);
    void buildIndex();
    unordered_map<int,int> builTimeIndex(vector<Pre_set_car>);
    //分割函数
    vector<string> &split(const string &str, const string &delimiters, vector<string> &elems, bool skip_empty = false);

    template<class TrafficInstance>
    vector<TrafficInstance> initInstance(string file_path);

    template<class Pre_set_car_Instance>
    vector<Pre_set_car_Instance> init_Pre_setCar(string file_path);

    template<typename T>
    unordered_map<int, int> buildMapIndex(vector<T>& vs);;

    template <typename T>
    unordered_map<int,T> build_ID_dict (vector<T>& vs);

    void update_preset_car(vector<Car> &);

    void portCarsToPort();
//    void change_car_start_time(pre_set_cars);
    void getPathOfCar(Car*);
    void getAllCarPath();
    double getWeightOfRange(int from_time, int to_time, int road_id, Car* car);
    double getWeightOf(int, int, Car* car);
    void setWeightOf(int, int, double, Car*);
    void init_preset_road_weight();
    void updateWeightsByPath(Car* car);
    int getSpeedOf(Car* car, Road* road);
    int getTimeCostOf_pre(Car *car, Road *road);
    int getTimeCostOf(Car* car,Road* road,Cross* cross);


    vector<Road*> getAdjRoadOfCross(Cross* cross);
    Road* getRoadById(int);
    Car* getCarById(int);
    Cross* getCrossById(int);

    vector<string> path2string();
    void checkPath();

    void computeAvglen();
};

#endif // METHOD_H_
