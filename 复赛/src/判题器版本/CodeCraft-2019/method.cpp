#include "method.h"
#include <stdlib.h>

using namespace std;

/*道路信息初始化*/
Road::Road(int id_, int length_, int max_speed_, int from_id_, int to_id_,
           int channels_, bool is_duplex_) {
    id = id_;
    length = length;
    max_speed = max_speed_;
    from_id = from_id_;
    to_id = to_id_;
    channels_num = channels_;
    is_duplex = is_duplex_;

}
/**根据ID*/

/*输入道路的流*/
Road::Road(string s) {
    stringstream ss;
    ss << s;
    ss >> id >> length >> max_speed >> channels_num
       >> from_id >> to_id >> is_duplex;
}

/* 获取可用的车道索引 */
int Road::getAvailableChannelIndex(vector<queue<Car *>> channels) {
    int index = 0;
    for (auto channel = channels.begin(); channel != channels.end(); channel++) {
        if (channel->size() == 0)
            return index;
        if (channel->back()->at_road_position > 0) {//判断队尾的车的位置是否大于0
            /*路的逻辑是 -------------->(尾部是0 逐渐增大至 队头)*/
            return index; //返回车的channel的索引
        }
        index++;
    }
    return -1;
}

/* 车辆驶入道路的逻辑 */
void Road::addCar(Car *car, int last_cross_id) {
// 根据车辆驶来的方向来确认要驶入的道路
    vector<queue<Car *>> *target_channels = &s2e_channels;
    int direction = 1;
    if (last_cross_id != from_id) {
        if (!is_duplex) throw runtime_error("道路不是双向车道！");
        target_channels = &e2s_channels;
        direction = -1;
    }
// TODO: 处理车辆驶入道路的逻辑
// 1. 判断车道空闲情况，注意调用该函数时，目标车道一定是空闲的，否则将会抛出错误
    int availableChannelIndex = this->getAvailableChannelIndex(*target_channels);
// int availableChannelIndex = 0;
    if (availableChannelIndex < 0)
        throw runtime_error("不合法的调度，目标道路无空闲");
// 2. 车辆驶入新道路
    target_channels->at(availableChannelIndex).push(car);
// 3. 更改车辆所在道路以及车道
    car->changeRoad(this, availableChannelIndex, direction);
    target_channels->at(0).push(car);
}

Car::Car(int id_, int from_id_, int to_id_, int max_speed_, int plan_time_, int priority_, int preset_) {
    id = id_;
    from_id = from_id_;
    to_id = to_id_;
    max_speed = max_speed_;
    plan_time = plan_time_;
    priority = priority_;
    preset = preset_;
}

/**
* 从流中生成类实例，字符串流必须包含与类构造函数相匹配的参数数目
*/
Car::Car(string s) {
    stringstream ss;
    ss << s;
    ss >> id >> from_id >> to_id >> max_speed >> plan_time >> priority >> preset;
}

void Car::updateStatus(int new_status) {
    status = new_status;
}

void Car::changeSpeed(int new_s) {
    if (new_s > max_speed)
        throw runtime_error("The new speed cannot larger than max speed!");
    speed = new_s;
}

void Car::changeRoad(Road *road_, int channel_, int direction) {
    if (channel_ >= road_->channels_num) //
        throw runtime_error("Target road channels wrong!");
    at_road = road_;
    at_channel_id = channel_;
    at_channel_direction = direction;
}

void Car::changePosition(int new_p) {
    if (new_p > at_road->length) throw runtime_error("车辆所在位置超出车道限制！");
    at_road_position = new_p;
}

/* 车辆往前行驶一个时间单位 */
void Car::move() {
// 有两种情况，一种是车辆在车道内部行驶，另一种是车辆经过路口，进入下一条路
// 车辆开始行驶时，必须处于等待行驶状态，行驶过后，变为终止状态
//    if (status != 1) throw runtime_error("只有处于等待状态的车辆才能行驶！");
// 开始判断车辆是否需要驶出路口
//    cout<<"状态 "<<status<<endl;
    if (status == 1) {
        Car *front_car = this->getFrontCar();

        int speed = min(max_speed, at_road->max_speed);

        this->current_time += 1;
//        cout << this->id << " " << this->current_time << endl;
        if (front_car != NULL) {
            // 存在前车
            if (front_car->status == 1) {
                // 前车处于等待行驶状态，则本车保持不动
                this->updateStatus(1);
            } else if (front_car->status == 2) {
                // 前车处于终止状态，则本车向前行驶，并将自身状态设置为终止状态
                this->could_move_distance = min(could_move_distance,
                                                front_car->at_road_position - this->at_road_position - 1);//能移动的长度
                this->changePosition(this->at_road_position + could_move_distance);
                this->updateStatus(2); // 车辆移动完毕，处于终止状态
            }
//            else {
//                throw runtime_error("道路存在不明情况的前车");
//            }
        } else {
            // 不存在前车
            if (at_road->length - this->at_road_position <= speed) {
                // 出道路
                // 当该车辆要行驶出路口时，将车辆状态设置为等待行驶状态
                // 因为路口处的规则比较复杂，需要单独进行处理
                this->could_move_distance = at_road->length - this->at_road_position;//说明到路口了
                this->changePosition(this->at_road_position + could_move_distance);
                this->updateStatus(1);

            } else {
                // 不出道路，直接处理车辆状态即可
                this->could_move_distance = this->at_road_position + speed;//能够移动的长度
                this->changePosition(this->at_road_position + could_move_distance);
                this->updateStatus(2);
            }
        }
    }
}

Car *Car::getFrontCar() {
//    cout << at_road->s2e_channels.size() << endl;
//    cout << at_road->e2s_channels.size() << endl;
//    cout << at_channel_direction << endl;
    if (at_road->e2s_channels[0].front() != nullptr) {
//        cout << &at_road->e2s_channels << endl;
        auto *a = &at_road->e2s_channels;
//        cout << a[0].front().front()->id << endl;
    }
    auto *channels = at_channel_direction > 0 ?
                     &at_road->s2e_channels : &at_road->e2s_channels;//查看方向
    auto *channel = &channels->at(at_channel_id);
    Car *result;
// 如果车辆是车道中最靠前的一辆车，则返回NULL，表示没有前车
    if (channel->empty() || this->id == channel->front()->id) return NULL;
// 否则就搜索前车
    for (auto it = channel->back(); it->id != this->id; it--) {
        result = it;
        // 一直搜索到队首，都搜索不到，则报错
//        if (it == channel->front())
//            throw runtime_error("车辆状态与所在道路状态不一致，请检查！");
    }
    return result;
}

Cross::Cross(int id_, int top_road_id_, int right_road_id_, int bottom_road_id_,
             int left_road_id_) {
    id = id_;
    top_road_id = top_road_id_;
    right_road_id = right_road_id_;
    bottom_road_id = bottom_road_id_;
    left_road_id = left_road_id_;
}

Cross::Cross(string s) {
    stringstream ss;
    ss << s;
    ss >> id >> top_road_id >> right_road_id >> bottom_road_id >> left_road_id;
}


Pre_set_car::Pre_set_car(int id_, int plan_time_, vector<int> path_) {
    id = id_;
    plan_time = plan_time_;
    path = path_;
}

Pre_set_car::Pre_set_car(string s) {
    stringstream ss;
    ss << s;
    ss >> id >> plan_time;
    for (int d; ss >> d; path.push_back(d)) {};
}


/* 车辆入库操作使用指针，来保证全局状态统一 */
int Cross::addCar(Car *c) {
    car_port.push(c);
    return car_port.size();
}


vector<string> &Traffic::split(const string &str, const string &delimiters, vector<string> &elems, bool skip_empty) {
    string::size_type pos, prev = 0;
    while ((pos = str.find_first_of(delimiters, prev)) != string::npos) {
        if (pos > prev) {
            if (skip_empty && 1 == pos - prev) break;
            elems.emplace_back(str, prev, pos - prev);
        }
        prev = pos + 1;
    }
    if (prev < str.size()) {
        elems.emplace_back(str, prev, str.size() - prev);
    }
    return elems;
}

/*根据输入流文件初始化road,car,cross,perset的实例*/
/*preset是否会多余后续会考虑*/
void Traffic::initTraffic(string car_path, string road_path, string cross_path, string pre_set_car) {
// 构建路网的基本实例
    cars = initInstance<Car>(car_path);  //将对象塞进vector容器中
    roads = initInstance<Road>(road_path);
    cout<<roads[0].e2s_channels.size()<<endl;
    crosses = initInstance<Cross>(cross_path);
    pre_set_cars = initInstance<Pre_set_car>(pre_set_car);
    Road *r;
    for(int i=0;i<roads.size();i++)
    {
        r = &roads[i];
        if(r->is_duplex)
        {
            r->e2s_channels = *(new vector<queue<Car*>>(r->channels_num,queue<Car*>()));
            r->s2e_channels = *(new vector<queue<Car*>>(r->channels_num,queue<Car*>()));;
        } else
            r->s2e_channels  = *(new vector<queue<Car*>>(r->channels_num,queue<Car*>()));;
    }

    cout<<roads[1].e2s_channels.size()<<endl;

    for (int i = 0; i < cars.size(); i++)
        if (cars[i].priority == 1)
            priority_cars.push_back(&cars[i]);
        else
            un_priority_cars.push_back(&cars[i]);
//    cout << priority_cars.front()->id << endl;
    sort(priority_cars.begin(), priority_cars.end(), [](Car *a, Car *b) {
        return a->plan_time < b->plan_time;
    });
//    un_priority_cars = get_priority_or_unpriority_car(cars, 0);

    buildIndex();
    update_preset_car(cars);
    init_preset_road_weight();
    for(auto p:cars)
        num_map.insert(pair<int,Car>(p.id,p));
}


void Traffic::buildIndex() {
    car_id2index = buildMapIndex<Car>(cars, false);
    road_id2index = buildMapIndex<Road>(roads, true);//按照ID升序,为了调度
    cross_id2index = buildMapIndex<Cross>(crosses, true);//按照ID升序,为了调度
    road_ID_dict = build_ID_dict<Road>(roads);
    car_ID_dict = build_ID_dict<Car>(cars);
    cross_ID_dict = build_ID_dict<Cross>(crosses);
    timeIndex = buildTimeIndex(pre_set_cars);
    portCarsToPort();
    computeAvglen();
}



//template<typename T>
//unordered_map<int, T> Traffic::build_ID_dict(vector<T> &cross) {
//    unordered_map<int, T> ID_dict;
//    for (unsigned int i = 0; i < int(cross.size()); i++)
//        ID_dict.insert(pair<int, T>(cross[i].id, cross[i]));
//    return ID_dict;
//}

template<typename T>
unordered_map<int, T> Traffic::build_ID_dict(vector<T> &cross) {
    unordered_map<int, T> ID_dict;
    for (unsigned int i = 0; i < int(cross.size()); i++)
        ID_dict.insert(pair<int, T>(cross[i].id, cross[i]));
    return ID_dict;
}

/*计算道路长度,没有必要的了,估计*/
void Traffic::computeAvglen() {
    int s = 0;
    for (auto r:roads) {
        s += r.length;
    }
    avg_len = s / roads.size();
}


/* 将车辆停靠在对应的车库中去 */
void Traffic::portCarsToPort() {
    for (auto c = cars.begin(); c != cars.end(); c++) {
        if (c->from_id) {
            auto the_cross_index = cross_id2index.find(c->from_id);
            if (the_cross_index != cross_id2index.end()) {
                crosses[the_cross_index->second].addCar(&*c);
            }
        }
    }
}

/**核心调度算法*/
/* 生成车辆的路径规划 */
void Traffic::getPathOfCar(Car *car, int time) {
    struct CrossD {
        Cross *cross = NULL;  //路口信息
        CrossD *prior_crossd = NULL;//下一条优先的道路
        Road *road = NULL;  //当前的路
        double min_d = __DBL_MAX__;  //路口权重
        int t;
    };
    vector<CrossD> G(int(crosses.size()), CrossD());//初始化所有路口信息,用vector容器存储
// CrossD[crosses.size()] G;
    unordered_map<int, CrossD *> S;//字典,路口ID 和信息
    unordered_map<int, CrossD *> Q;
// 定义比较函数
    auto cmp = [Q](pair<const int, CrossD *> a, pair<const int, CrossD *> b) {
        return a.second->min_d < b.second->min_d;
    };//比较哪个的权重大

// 1. 初始化列表
    for (int i = 0; i < int(crosses.size()); i++) {
        G[i].cross = &crosses[i];//当前路口
        G[i].t = car->plan_time;//车辆的出发时间
        if (crosses[i].id == car->from_id) {
            G[i].min_d = 0;
        }
        Q[crosses[i].id] = &G[i];
    }

// 2. 开始处理
    int last_q_size = Q.size(); // 判断死锁
    while (!Q.empty()) {
        auto min_iter = min_element(Q.begin(), Q.end(), cmp);/*Q的权重最小值*/
        auto u = min_iter->second;
        Q.erase(min_iter->first);//删除该元素
        S[u->cross->id] = &*u;
        vector<Road *> adjs = getAdjRoadOfCross(u->cross);//获得路口的连通道路的ID
        // 遍历u的邻居结点，松弛对应的值
        for (auto it = adjs.begin(); it != adjs.end(); it++) {//四条道路遍历
            // 首先剔除掉已经在S中的结点
            // cout << u->cross->id << ' ' << (*it)->from_id << endl;
            if (S.find((*it)->from_id) != S.end() &&
                S.find((*it)->to_id) != S.end())
                continue;
            // cout << "re: " << u->cross->id << ' ' << (*it)->from_id << endl;
            // 然后剔除不支持逆行的路口，即出发点不是u，并且不是双通路
            if ((*it)->from_id != u->cross->id && !(*it)->is_duplex) continue;
            // 然后对剩下的路口进行计算
            int d_t = getTimeCostOf(car, *it, u->cross, time);/*路口时间长度*/
            double new_d = u->min_d + getWeightOfRange(u->t, u->t + d_t, (*it)->id, car);//路口权重
            // 找到该路口对应的目标节点
            int target_cross_id = (*it)->from_id != u->cross->id ?
                                  (*it)->from_id : (*it)->to_id;
            CrossD *v = Q[target_cross_id];
            // 如果新的权重值小于目标节点权重值，则更新对应权重
            if (new_d < v->min_d) {
                v->min_d = new_d;
                v->prior_crossd = &*u;
                v->road = *it;
                v->t = u->t + d_t;
            }

        }

        if (last_q_size == int(Q.size())) {
            throw runtime_error("路径生成失败！");
        };
        last_q_size = Q.size();
    }
// 3. 此时S中所有点的权重均已更新完毕，生成对应的路径
    vector<Road *> path;
    auto target_cross = S.find(car->to_id);
    if (target_cross == S.end()) throw runtime_error("路径生成错误！");
    CrossD *the_crossd = target_cross->second;
    while (the_crossd->prior_crossd != NULL) {
        path.push_back(the_crossd->road);
        the_crossd = the_crossd->prior_crossd;
    }
    reverse(path.begin(), path.end());
    car->path = path; // 更新车辆的路径

}

void Traffic::getAllCarPath() {
    /*按照车辆的*/
    sort(cars.begin(), cars.end(), [](Car &a, Car &b) {
        return a.max_speed > b.max_speed;
    });

    int i = 0;
    int d_t = 1;
    int index = 0;
    //改变时间
//    line++;
    //时间片
    int j = 1;
    int start = 1;//表示当前可供选择的最小时间起点
    int batch_size = ceil(cars.size() / epoch);
//    vector<Car>::iterator it;
//    for (it = cars.begin(); it != cars.end(); it++) {
//        if (it->preset) {
//            j++;
//            continue;
//        }
//        if (j > epoch) {
//            j = 1;
//            if (timeIndex[start] > batch_size) {
//                start++;
//                j = start;
//            }
//        }
//        if (timeIndex[j] > batch_size)
//            j++;
//        it->plan_time = max(it->plan_time, j);
//        timeIndex[j]++;
//        j++;
//    }
//    //根据出发时间排序
//    /*根据优先级来排序,时间和id来排序*/
//    cout << cars[1].status << endl;
//
    sort(cars.begin(), cars.end(), [](Car &a, Car &b) {
        return (
                ((a.priority < b.priority)) ||
                ((a.priority == b.priority) && (a.plan_time < b.plan_time)) ||
                ((a.priority == b.priority) && (a.plan_time == b.plan_time) && (a.id < b.id))
        );
    });


    //确定出发道路
    vector<int> cost;

//    for(int i=0;i<cars.size();i++)
//    {
//        Car p = cars[i];
//        if(!p.preset) {
//            cost.clear();
////        vector<Road *> adjs = getAdjRoadOfCross(u->cross)
//            int crossId = p.from_id;
//            auto it = cross_ID_dict.find(crossId);
//            if (it != cross_ID_dict.end()) {
//                Cross *cross = &(*it).second;
//                cout << (*it).first << endl;
//                vector<Road *> adjs = getAdjRoadOfCross(cross);
//                for (auto it = adjs.begin(); it != adjs.end(); it++)
//                    cost.push_back((*it)->length / min(p.max_speed, (*it)->max_speed));
//                auto index = std::min_element(std::begin(cost), std::end(cost));
//                Road *road = *(adjs.begin() + *index);
//                p.path.push_back(road);
//                int b = road->channels_num-1;
//                if(p.from_id==road->from_id)
//                    road->s2e_channels[rand()%b].push(&p);
//                else
//                    road->e2s_channels[rand()%b].push(&p);
//
//            }
//        }
//    }

    ///用来判断有多少辆车还没到终点的
    while (num_map.size() != 0)//
    {
        total_time++;
        cout<<"时间片 "<<total_time<<endl;
        for (int i = 0; i < cars.size(); i++) {
//            cout<<"i "<<i<<endl;
            Car &p = cars[i];

            if (p.plan_time == total_time) {
                int a = get_start_road(p, total_time);//获得路径初始的,同时更状态为1
                if (!a) {
//                    cout << "car id " << p.id << endl;
//                    cout << "road id" << p.at_road->id << endl;
//                    cout << "queue size" << p.at_road->e2s_channels.size() << endl;
                }
            }

        }
        driveCurrentRoad();//车道上的车辆开始运行
        driveCarInitList(total_time, true);//优先车辆上路
        driveCarInitList(total_time, false);
//        driveCurrentRoad();//车道上的车辆开始运行
        createCartSequence();//优先队列先走
//        for (auto p:cars) {
//            if (p.status != 0 && !p.priority)
//                moveToNextRoad(p, total_time);
//        }
        for(int i=0;i<roads.size();i++)
        {
            Road &r = roads[i];
            cout<<"ID "<<r.id<<endl;
            if(r.is_duplex){
                while(!r.s2e_priority_queue.empty() ) {//正向的队列
                    Car *car = r.s2e_priority_queue.front();
                    if(car->to_id==car->path[car->path.size()-1]->to_id) {
                        car->status=3;
                        num_map.erase(car->id);
                        continue;
                    }
                    moveToNextRoad(*car,total_time);
                    r.s2e_priority_queue.pop();
//                    cout<<r.s2e_priority_queue.size()<<endl;

                }
                while (!r.e2s_priority_queue.empty() && r.e2s_priority_queue.front()!=NULL) {
                        cout << r.id << endl;
                        cout << r.e2s_priority_queue.size() << endl;
                        Car *car = r.e2s_priority_queue.front();
                        if (car->to_id == car->path[car->path.size() - 1]->to_id) {
                            car->status = 3;
                            num_map.erase(car->id);
                            continue;
                        }
                        moveToNextRoad(*car, total_time);
                        r.e2s_priority_queue.pop();
                    }


            } else{
                while (!r.s2e_priority_queue.empty()) {
                    Car *car = r.s2e_priority_queue.front();
                    if(car->to_id==car->path[car->path.size()-1]->to_id) {
                        car->status=3;
                        num_map.erase(car->id);
                        continue;
                    }
                    moveToNextRoad(*car,total_time);
                    r.s2e_priority_queue.pop();
                }
            }

        }

//        cout << "hello" << endl;
//        checkPath();
    }
}

/*获得车的当前节点*/
int Traffic::direction(Car &car) {
    if (car.preset) {
        if (car.current_road_idx == 0) {
            if (car.from_id == car.path[car.current_road_idx]->from_id)
                return car.path[car.current_road_idx++]->to_id;
            else
                return car.path[car.current_road_idx++]->from_id;
        } else {

            Road *pre = car.path[car.current_road_idx - 1];
            Road *current = car.path[car.current_road_idx];
            if (pre->from_id == current->from_id || pre->to_id == current->from_id)
                return current->to_id;
            else
                return current->from_id;

        }
    } else {
        vector<Road *>::iterator it = car.path.end() - 1;
        if (car.path.size() == 0)
            return car.from_id;
        else if (car.path.size() == 1) {
            if (car.path[0]->from_id == car.from_id)
                return car.path[0]->to_id;
            else
                return car.path[0]->from_id;
        } else {
            Road *current = *(it);//当前路径
            Road *pre = *(it - 1);//上一个车道
            //判断方向,也就是当前道路的终点是下一条道路的起点
            if (!current->is_duplex)
                return current->to_id;//返回的是当前终点
            if (!pre->is_duplex && current->is_duplex) {
                if (pre->to_id == current->from_id)
                    return current->to_id;
                else
                    return current->from_id;
            }
            if (current->is_duplex && pre->is_duplex)//两条路都是双向的
            {
                if (current->from_id == pre->from_id || current->from_id == pre->to_id)
                    return current->to_id;
                else if (current->to_id == pre->from_id || current->to_id == pre->to_id)
                    return current->from_id;

            }
        }
    }
}

/*驱动每辆车终止或者等待状态*/
void Traffic::driveCurrentRoad() {
    queue<Car *> new_queue;
    vector<queue<Car * >>::iterator c;
    for (int i=0;i<roads.size();i++) {
        Road r = roads[i];
//        cout<<r.id<<endl;
        if (r.is_duplex)/*双向的*/
        {
            cout<<r.e2s_channels.size()<<endl;
            for (c = r.e2s_channels.begin(); c != r.e2s_channels.end(); c++)/*进入正向车道 from_cross -> to_cross,c是队列了*/
            {
                while (!(*c).empty()) {
//                    cout<<"id "<<c->front()->id<<endl;
                    int a =(*c).front()->id;
                    cout<<a<<" 之前的位置"<<(*c).front()->at_road_position<<endl;
                    (*c).front()->move();//每辆车进行自己的move
                    cout<<a<<" 之后的位置 "<<(*c).front()->at_road_position<<endl;

                    int channel = (*c).front()->at_road->id;
                    new_queue.push((*c).front());//临时queue作为桥梁进行辅助
                    (*c).pop();
                }
                while (!new_queue.empty()) {
                    (*c).push(new_queue.front());//c重新恢复回来
                    new_queue.pop();
                }
            }
            for (c = r.s2e_channels.begin(); c != r.s2e_channels.end(); c++)/*进入正向车道 to_cross -> from_cross,c是队列了*/
            {
                while (!(*c).empty()) {
                    int a =(*c).front()->id;
                    cout<<a<<" 之前的位置"<<(*c).front()->at_road_position<<endl;
                    (*c).front()->move();//每辆车进行自己的move
                    cout<<a<<" 之后的位置 "<<(*c).front()->at_road_position<<endl;
                    int channel = (*c).front()->at_road->id;
//                    cout<<"队内移动 "<<a<<" "<<channel<<endl;
                    new_queue.push((*c).front());//临时queue作为桥梁进行辅助
                    (*c).pop();
                }
                while (!new_queue.empty()) {

                    (*c).push(new_queue.front());//c重新恢复回来
                    new_queue.pop();
                }

            }
        } else {//单向的车道的话就遍历一个就可以了
            for (c = r.s2e_channels.begin(); c != r.s2e_channels.end(); c++)/*进入正向车道 to_cross -> from_cross,c是队列了*/
            {
                while (!c->empty()) {
                    int a =(*c).front()->id;
                    cout<<a<<" 之前的位置"<<(*c).front()->at_road_position<<" ";
                    (*c).front()->move();//每辆车进行自己的move
                    cout<<a<<" 之后的位置 "<<(*c).front()->at_road_position<<endl;
                    int channel = (*c).front()->at_road->id;
//                    cout<<"队内移动 "<<a<<" "<<channel<<endl;
                    new_queue.push((*c).front());//临时queue作为桥梁进行辅助
                    (*c).pop();

                }
                while (!new_queue.empty()) {
                    (*c).push(new_queue.front());//c重新恢复回来
                    new_queue.pop();
                }
            }
        }
    }
}

/*优先车辆先上路并且在对应的道路上行驶*/
void Traffic::driveCarInitList(int t, bool priority) {/*当前时间片*/

    if (priority) {
        if (!priority_cars.empty()) {
            sort(priority_cars.begin(), priority_cars.end(), [](Car *a, Car *b) {
                return a->plan_time < b->plan_time;
            });
            vector<Car *>::iterator it1 = priority_cars.begin();
            while (it1 != priority_cars.end()) {
                Car *p = *it1;
//                cout << p->id << endl;
                if (p->plan_time == t) {
                    p->updateStatus(1);
                    cout<<"状态 "<<p->status<<endl;
//                    cout << "init list" << endl;
//                    cout << p->id << endl;
//                    cout << p->at_road->s2e_channels.size() << endl;
//                    cout << p->at_road->e2s_channels.size() << endl;
//                    p->move();
                    priority_cars.erase(it1++);
//                    cout<<"priority "<<priority_cars.size()<<endl;

                } else
                    it1++;
            }
        }
    }else {
        if (!un_priority_cars.empty()) {
            sort(un_priority_cars.begin(), un_priority_cars.end(), [](Car *a, Car *b) {
                return a->plan_time < b->plan_time;
            });
            vector<Car *>::iterator it2 = un_priority_cars.begin();
            while (it2 != un_priority_cars.end()) {
                Car *p = *it2;
//                cout << p->id << endl;
                if (p->plan_time == t) {
                    p->updateStatus(1);
                    cout<<"状态 "<<p->status<<endl;

//                    cout << "init list" << endl;
//                    cout << p->id << endl;
//                    cout << p->at_road->s2e_channels.size() << endl;
//                    cout << p->at_road->e2s_channels.size() << endl;
//                    p->move();  //车辆移动
                    un_priority_cars.erase(it2++); //重未出发队列中删除
//                    cout<<"非预制 "<<un_priority_cars.size()<<endl;
                }
                it2++;
            }
        }
    }
}

/*创建优先队列*/
void Traffic::createCartSequence() {
    vector<Car *> priority_car;//创建临时变量
    for (int i = 0; i < roads.size(); i++) {
        Road *r = &roads[i];
        if (r->is_duplex) {//如果是双向的
            for (int j = 0; j < r->s2e_channels.size(); j++) {//对于每个车道的进行遍历,每个车道是一个vector
//            for (auto c:r.s2e_channels) {/*进入正向车道 from_cross -> to_cross,c是队列了*/
                queue<Car *> c = r->s2e_channels[j];//车道里面是queue,取到他
                if (c.size() > 0) {//车道有车
                    cout<<r->id<<" 车道最前面是 "<<c.front()->id<<" "<<c.front()->at_road_position<<" "<<c.front()->max_speed<<" "<<r->length<<endl;
                    if (c.front()->at_road_position == r->length-c.front()->max_speed && c.front()->status == 1)//最前面的车在车道最前面.等待着换车道
                        priority_car.push_back(c.front());//加入队列中
                    sort(priority_car.begin(), priority_car.end(), Car::cmpAtRoad());//排序
                    /*优先车辆入队列*/
                    for (auto p:priority_car) {
                        cout<<"优先队列 "<<p->id<<endl;
                        r->s2e_priority_queue.push(p);
                    }
                }
            }
            for (int j = 0; j < r->e2s_channels.size(); j++)
//            for (auto c:r.e2s_channels) {/*进入正向车道 from_cross -> to_cross,c是队列了*/
            {
                queue<Car *> c = r->e2s_channels[j];
                if (c.size() > 0) {
//                    cout<<"车道最前面是 "<<c.front()->id<<" "<<c.front()->at_road_position<<" "<<r->length<<endl;

                    if (c.front()->at_road_position == r->length-c.front()->speed && c.front()->status == 1)
                        priority_car.push_back(c.front());//加入
                    sort(priority_car.begin(), priority_car.end(), Car::cmpAtRoad());//排序
                    /*优先车辆入队列*/
                    for (auto p:priority_car){
                        cout<<"优先队列 "<<p->id<<endl;

                        r->e2s_priority_queue.push(p);
                }
                }
            }
        } else {
            for (int j = 0; j < r->s2e_channels.size(); j++) {
//            for (auto c:r.s2e_channels) {/*进入正向车道 from_cross -> to_cross,c是队列了*/
                queue<Car *> c = r->s2e_channels[j];
                if (c.size() > 0) {
//                    cout<<"车道最前面是 "<<c.front()->id<<" "<<c.front()->at_road_position<<" "<<r->length<<endl;

                    if (c.front()->at_road_position <r->length-c.front()->speed)
                        priority_car.push_back(c.front());//加入
                    sort(priority_car.begin(), priority_car.end(), Car::cmpAtRoad());//排序
                    /*优先车辆入队列*/
                    for (auto p:priority_car) {
                        cout<<"优先队列 "<<p->id<<endl;

                        r->s2e_priority_queue.push(p);
                    }
                }
            }
        }
    }
}


bool Traffic::moveToNextRoad(Car &car, int time) {


    int current_cross_id = direction(car);//获得当前节点


    //获得下一条路
    Road *current_road = car.path[car.path.size() - 1];
    cout<<"当前道路的 id"<<current_road->id<<endl;
    auto it = cross_ID_dict.find(current_cross_id);
    Cross *current_cross = &(*it).second;//获得这个路口的ID;
    int current_direction;
    if (current_road->from_id == current_cross_id)
        current_direction = -1;
    else
        current_direction = 1;

    Road *nextroad;

    /*判断是不是满了*/

    vector<Road *> adjs = getAdjRoadOfCross(current_cross);//获得路口的连通道路的ID
    sort(adjs.begin(), adjs.end(), [](Road *&a, Road *&b) {
        return ((a->id < b->id));
    });
    //先计算cost
    vector<int> cost;
    unordered_map<int, unordered_map<int, Road>> avaliable_path;
    struct available_choose {
        Road *newRoad;
        Cross *newCross;
        queue<Car *> channels;
        int cost;
        int flag;//表示可不可以走
        int speed;
        int direction;//判断正反向 1,2;
        int new_position;
        int channel_id;
    };


    vector<available_choose> S;
    available_choose P;


    // 遍历u的邻居结点，松弛对应的值
    int num = 0;
    int choose_index = 0;
    for (auto it = adjs.begin(); it != adjs.end(); it++) {//四条道路遍历
        // 首先剔除掉已经在S中的结点
//        cout << (*it)->from_id << endl;
        if ((*it)->id == -1)
            continue;
        if (current_road->from_id == (*it)->from_id &&
            current_road->to_id == (*it)->to_id)
            continue;
        // 然后剔除不支持逆行的路口，即出发点不是u，并且不是双通路
        if ((*it)->from_id != current_cross_id && !(*it)->is_duplex) continue;
        // 然后对剩下的路口进行计算
        int d_t = getTimeCostOf(&car, *it, current_cross, time);/*路口时间长度的权值公式*/

        nextroad = (*it);
        if(current_cross_id==car.to_id) {
            car.status = 3;
            num_map.erase(car.id);
        }
        if (nextroad->from_id == current_cross_id)//判断方向 from -->to
        {
            //遍历每个channel
            int index = 0;
            for (auto c:nextroad->s2e_channels) {
                if (c.size() > 0) {
                    if (c.size() == nextroad->length ||
                        (c.back()->status == 2 && c.back()->at_road_position == 0))//如果满了或者车的状态都是终止状态
                        index++;
                    else if (c.size() < nextroad->length ||
                             (c.back()->status == 2 && c.back()->at_road_position > 0))//道路没有满,并且最后一辆车的位置不止最后面,且处于种植状态
                    {
                        /*车辆进入新的车道*/
                        int speed = min(car.max_speed, nextroad->max_speed);//进入下一车道的速度
                        P.speed = speed;
                        P.channels = c;
                        P.cost = d_t;
                        P.newRoad = nextroad;
                        P.direction = 1;
                        P.channel_id = index;
                        if (speed > car.could_move_distance)
                            P.new_position = speed - car.could_move_distance;
                        else
                            P.new_position = car.at_road_position;
                        S.push_back(P);
                        break;

////                        if (speed > car.could_move_distance) {
////                            if(current_direction==1) {
////                                current_road->s2e_channels[car.at_channel_id].pop();
////                                current_road->s2e_priority_queue.pop();//全部出队
////                            } else
////                            {
////                                current_road->e2s_channels[car.at_channel_id].pop();
////                                current_road->e2s_priority_queue.pop();//全部出队
////                            }
////                            car.changePosition(speed - car.could_move_distance);//更新位置
////                            car.changeRoad(nextroad, index, 1);//进入新的道路,且是正向的
////                            nextroad->s2e_channels[index].push(&car);
////                            car.updateStatus(2);
////                            c.push(&car);
//                        break;
//                        }
                        return true;//退出
                    } else if (c.size() < nextroad->length || c.back()->status == 1) {
                        P.speed = 0;
                        P.channels = c;
                        P.cost = d_t;
                        P.newRoad = nextroad;
                        P.direction = 1;
                        S.push_back(P);
//                        car.updateStatus(2);//结束
//                        return false
//
                        break;
                    }


                    if (index == nextroad->s2e_channels.size() - 1) {
                        continue;
//                    return true;//如果车道整个塞满了
                    }
                } else {
                    int speed = min(car.max_speed, nextroad->max_speed);//进入下一车道的速度
                    P.speed = speed;
                    P.channels = c;
                    P.cost = d_t;
                    P.newRoad = nextroad;
                    P.direction = 1;
                    P.channel_id = 0;
                    if (speed > car.could_move_distance)
                        P.new_position = speed - car.could_move_distance;
                    else
                        P.new_position = car.at_road_position;
                    S.push_back(P);
                    break;
                }
            }
        } else if (nextroad->to_id == current_cross_id) {
            int index = 0;
            if (nextroad->e2s_channels[0].front() != NULL) {
                for (auto c:nextroad->e2s_channels) {
                    if (c.size() > 0) {
                        if (c.size() == nextroad->length ||
                            c.back()->status == 2 && c.back()->at_road_position == 0)//如果满了或者车的状态都是终止状态
                            index++;
                        else if (c.size() < nextroad->length ||
                                 c.back()->status == 2 && c.back()->at_road_position > 0)//道路没有满,并且最后一辆车的位置不止最后面,且处于种植状态
                        {
                            /*车辆进入新的车道*/
                            int speed = min(car.max_speed, nextroad->max_speed);//进入下一车道的速度
                            P.speed = speed;
                            P.channels = c;
                            P.cost = d_t;
                            P.newRoad = nextroad;
                            P.direction = 1;
                            if (speed > car.could_move_distance)
                                P.new_position = speed - car.could_move_distance;
                            else
                                P.new_position = car.at_road_position;
                            S.push_back(P);
                            break;
//                        break;
//                    if (speed > car.could_move_distance) {
//                        current_road->e2s_channels[car.at_channel_id].pop();
//                        current_road->e2s_priority_queue.pop();
//                        car.changePosition(speed - car.could_move_distance);//更新位置
//                        car.changeRoad(nextroad, index, -1);//进入新的道路,且是反向的
////                            nextroad
//                        car.updateStatus(2);
//                    }
//                    return true;
                        } else if (c.size() < nextroad->length || c.back()->status == 1) {//为等待状态
                            P.speed = 0;
                            P.channels = c;
                            P.cost = d_t;
                            P.newRoad = nextroad;
                            P.direction = 1;
                            S.push_back(P);
//                        car.updateStatus(2);//结束
//                        return false;
                            break;
                        }

                    } else {
                        int speed = min(car.max_speed, nextroad->max_speed);//进入下一车道的速度
                        P.speed = speed;
                        P.channels = c;
                        P.cost = d_t;
                        P.newRoad = nextroad;
                        P.channel_id = 0;
                        P.direction = 1;
                        if (speed > car.could_move_distance)
                            P.new_position = speed - car.could_move_distance;
                        else
                            P.new_position = car.at_road_position;
                        S.push_back(P);
                        break;
                    }
                }
            }
        }
    }
    sort(S.begin(), S.end(), [](available_choose &a, available_choose &b) {
        return a.cost < b.cost;
    });
    available_choose K = S[0];//最优解
    current_road->car_num--;
    K.newRoad->car_num++;
    if (K.speed > car.could_move_distance) {
        if (current_direction == 1) {
            current_road->s2e_channels[car.at_channel_id].pop();
            current_road->s2e_priority_queue.pop();//全部出队
        } else {
            current_road->e2s_channels[car.at_channel_id].pop();
            current_road->e2s_priority_queue.pop();//全部出队
        }

        car.changePosition(K.new_position);//更新位置
        cout<<" "<<car.id<<" 车跟新的位置是 "<<car.at_road_position<<" ";
        car.changeRoad(nextroad, K.channel_id, K.direction);//进入新的道路,且是正向的
        car.updateStatus(2);
        car.path.push_back(K.newRoad);
        if (K.direction == 1)
            nextroad->s2e_channels[K.channel_id].push(&car);
        else
            nextroad->e2s_channels[K.channel_id].push(&car);
        for(auto p:car.path)
            cout<<"路径是 "<<p->id<<" ";
        return true;
    }

}


/* 设置某条路在某个时刻的权重 */
void Traffic::setWeightOf(int t, int road_id, double w, Car *car) {
    auto t_weights = id_time_weights.find(t);
    if (t_weights != id_time_weights.end()) {
        auto id_weights = t_weights->second.find(road_id);
        if (id_weights != t_weights->second.end()) {
            id_weights->second += w;
        }
    }
    id_time_weights[t][road_id] = w + getWeightOf(t, road_id, car);
//    cout << t << " " << road_id << " " << id_time_weights[t][road_id] << endl;
}

/*根据预置的车辆来设置道路权重*/
void Traffic::init_preset_road_weight() {
    vector<Pre_set_car>::iterator car = pre_set_cars.begin();
    for (; car != pre_set_cars.end(); car++) {
        if (car->path.empty())throw runtime_error("车辆路径为空,不合法!");
        int t = car->plan_time;

        int car_id = car->id;//确认carID;
        std::unordered_map<int, Car>::iterator car_it;
        car_it = car_ID_dict.find(car->id);//索引到固定的项
        vector<int> value;
        for (auto p:car->path) {
            unordered_map<int, Road>::iterator road_it;
            road_it = road_ID_dict.find(p);
            double time_cost = getTimeCostOf_pre(&(*car_it).second, &(*road_it).second);

            for (int i = 0; i < time_cost; i++) {
                t += 1;
                // 道路权重是时间开销/车道数量
                double w = time_cost / (*road_it).second.channels_num;
                setWeightOf(t, car_id, w, &(*car_it).second);
            }

        }
    }
}

/* 根据车辆路径更新权重 */
void Traffic::updateWeightsByPath(Car *car, int time) {
    if (car->path.empty()) throw runtime_error("车辆的路径为空！");
    int t = car->plan_time; // 开始时间
//  cout<<car->priority<<endl;
    int time_cost;
    int next_cross_id;
    vector<Road *>::iterator p = car->path.begin();
    for (; p != car->path.end() - 1; p++) {/*p是road对象*/
        int next_cross_id;
        int current_road_id = (*p)->id;
//        cout << "road ID " << current_road_id << endl;
        int next_road_id = (*(p + 1))->id;//下一条路的ID
        /**确定路口*/

        if ((*(p + 1))->is_duplex && (*(p + 1))->is_duplex)/*如果两条路都是双向的*/{

            if ((*p)->to_id == (*(p + 1))->from_id || (*p)->from_id == (*(p + 1))->from_id)
                next_cross_id = (*(p + 1))->from_id;
            if ((*p)->to_id == (*(p + 1))->to_id || (*p)->from_id == (*(p + 1))->to_id)
                next_cross_id = (*(p + 1))->to_id;
        } else if ((*p)->is_duplex && (*(p + 1))->is_duplex == 0)

            next_cross_id = (*(p + 1))->from_id;
        else if ((*p)->is_duplex == 0)
            next_cross_id = (*p)->to_id;

//        cout << (*p)->id << endl;
//        cout << "下一条路的Id" << endl;
//        cout << (*(p + 1))->id << endl;
//        cout << "next corss id " << next_cross_id << endl;

        std::unordered_map<int, Cross>::iterator cross_it;
        cross_it = cross_ID_dict.find(next_cross_id);//索引下个节点的对象上

        time_cost = getTimeCostOf(car, *p, &(*cross_it).second, time);//获得这个路口的平均权重
        for (int i = 0; i < time_cost; i++) {
            t += 1;
            // 道路权重是时间开销/车道数量
            double w = time_cost / (*p)->channels_num;
            setWeightOf(t, (*p)->id, w, car);
        }
    }
    /*否则就直接计算最后一条路的权重*/
    time_cost = getTimeCostOf_pre(car, *p);
    for (int i = 0; i < time_cost; i++) {
        t += 1;
        // 道路权重是时间开销/车道数量
        double w = time_cost / (*p)->channels_num;
        setWeightOf(t, (*p)->id, w, car);
    }

}


/* 获取某条路在某个时间段的平均权重 */
double Traffic::getWeightOfRange(int from_time, int to_time, int road_id, Car *car) {
    double w = 0.0;
    int t = to_time - from_time + 1;
    while (from_time <= to_time) {
        w += getWeightOf(from_time, road_id, car);
        from_time++;
    }
    // return w;
    return t > 0 ? w / t : w;
}

/* 获取某条路在某个时刻的权重 */
double Traffic::getWeightOf(int t, int road_id, Car *car) {
    auto t_weights = id_time_weights.find(t);
    if (t_weights != id_time_weights.end()) {
        auto id_weights = t_weights->second.find(road_id);
        if (id_weights != t_weights->second.end()) {
            return id_weights->second;
        }
    }
    Road *r = getRoadById(road_id);
    return getTimeCostOf_pre(car, r);
}


/* 根据id获取路 */
Road *Traffic::getRoadById(int road_id) {
    auto index = road_id2index.find(road_id);
    if (index == road_id2index.end() ||
        index->second >= int(roads.size()))
        throw runtime_error("查询路出错！");
    return &roads.at(index->second);
}

/* 根据id获取车 */
Car *Traffic::getCarById(int car_id) {
    auto index = car_id2index.find(car_id);
    if (index == car_id2index.end() ||
        index->second >= int(cars.size())) {
        throw runtime_error("查询车辆出错！");
    };
    return &cars.at(index->second);
}

/* 根据id获取路口 */
Cross *Traffic::getCrossById(int cross_id) {
    auto index = cross_id2index.find(cross_id);
    if (index == cross_id2index.end() ||
        index->second >= int(crosses.size())) {
        throw runtime_error("查询路口出错！");
    }
    return &crosses.at(index->second);
}

int Traffic::getSpeedOf(Car *car, Road *road) {
    return min(car->max_speed, road->max_speed);
}

int Traffic::getTimeCostOf_pre(Car *car, Road *road) {
    int speed = getSpeedOf(car, road);
    return ceil(double(road->length) / double(speed));
}

int Traffic::getTimeCostOf(Car *car, Road *road, Cross *cross, int t) {
    int speed = getSpeedOf(car, road);
    //对着这个路口的下一个路口的四条路进行预测//
    //判断起点
    int next_cross_id;
    int current_road_id = road->id;

    if (road->from_id == cross->id)
        next_cross_id = road->to_id;//下一个节点,要研究的就是下一个节点
    else
        next_cross_id = road->from_id;

    std::unordered_map<int, Cross>::iterator cross_it;
    cross_it = cross_ID_dict.find(next_cross_id);//索引下歌节点的对象上
    vector<int> next_cross_average_weight;

    if ((*cross_it).second.top_road_id != -1 && (*cross_it).second.top_road_id != current_road_id)
        next_cross_average_weight.push_back(id_time_weights[t][(*cross_it).second.top_road_id]);
    if ((*cross_it).second.right_road_id != -1 && (*cross_it).second.right_road_id != current_road_id)
        next_cross_average_weight.push_back(id_time_weights[t][(*cross_it).second.right_road_id]);
    if ((*cross_it).second.bottom_road_id != -1 && (*cross_it).second.bottom_road_id != current_road_id)
        next_cross_average_weight.push_back(id_time_weights[t][(*cross_it).second.bottom_road_id]);
    if ((*cross_it).second.left_road_id != -1 && (*cross_it).second.left_road_id != current_road_id)
        next_cross_average_weight.push_back(id_time_weights[t][(*cross_it).second.left_road_id]);

    double average;
    if (next_cross_average_weight.size())
        average = std::accumulate(std::begin(next_cross_average_weight),
                                  std::end(next_cross_average_weight), 0.0) / next_cross_average_weight.size();
    double cost = double(average / road->channels_num) * c1 + ((road->length) / double(speed)) * c2;
//        cout<<"更新后的"<<t<<" "<<road->id<<" "<<cost<<endl;
    return ceil(cost);//调参
}

/*通过路口ID获得可以行走的路的ID*/
vector<Road *> Traffic::getAdjRoadOfCross(Cross *cross) {
    vector<Road *> result;
    int ids[] = {
            cross->top_road_id,
            cross->right_road_id,
            cross->bottom_road_id,
            cross->left_road_id
    };
    for (int i = 0; i < 4; ++i) {
        if (ids[i] > -1) {
            result.push_back(getRoadById(ids[i]));
        }
    }
    return result;
}

//void Traffic::get_priority_or_unpriority_car(vector<Car *> A,vector<Car> cars, bool priority) {
//    int i = 0;
//    for (auto p:cars)
//        if (p.priority == priority) {
//            A.push_back(&p);
//        }
//    cout<<A.front()->id<<endl;
//}

vector<string> Traffic::path2string() {
    vector<string> result;
    for (auto it = cars.begin(); it != cars.end(); it++) {
        if (!it->preset) {
//        cout<<it->id<<" no preset"<<endl;
            stringstream tmp;
            tmp << "(" << it->id << ", " << it->plan_time << ", ";
            int N = it->path.size();
            for (auto r:it->path) {
                tmp << r->id;
                if (r->id != it->path[N - 1]->id) {
                    tmp << ", ";
                }
            }
            tmp << ")";
            result.push_back(tmp.str());
        }
    }
    return result;
}


void Traffic::checkPath() {
    for (auto it:cars) {
        auto last_road = it.path[it.path.size() - 1];
        if (last_road->to_id != it.to_id && last_road->from_id != it.to_id) {
            throw runtime_error("路径生成失败！");
        }
    }
}

/*初始化函数,读入文件的出路,不需要管*/
template<class TrafficInstance>
vector<TrafficInstance> Traffic::initInstance(string file_path) {
    vector<TrafficInstance> instances;

    ifstream the_file;
    the_file.open(file_path);

    if (!the_file.is_open()) return instances;

    string temp_string;
    while (getline(the_file, temp_string)) {
        // 跳过注释行
        if (temp_string[0] == '#') continue;

        // 过滤掉不需要的字符
        string filtered_string;
//        for (auto s:temp_string) {
//            if (s != '(' && s != ')' && s != ',') {
//                filtered_string += s;
//            }
//        }
        temp_string.erase(temp_string.length() - 1, 1);
        temp_string.erase(0, 1);

        vector<string> result;

        int i = 0;
        split(temp_string, ",", result);
        for (auto s:result) {
            if (++i == 2 || i == 3) {
                filtered_string += " " + s;
            } else
                filtered_string += s;
        }


        // 生成类实例
        instances.push_back(TrafficInstance(filtered_string));
    }

    the_file.close();
    return instances;
}

template<typename T>
unordered_map<int, int> Traffic::buildMapIndex(vector<T> &vs, bool sorted) {
    if (sorted)
        sort(vs.begin(), vs.end(), [](T &a, T &b) {
            return a.id < b.id;
        });

    unordered_map<int, int> tmp_id2index;
    for (int i = 0; i < int(vs.size()); i++) {
        tmp_id2index[vs[i].id] = i;
    }
    return tmp_id2index;
}

template<typename T>
unordered_map<int, int> Traffic::buildTimeIndex(vector<T> &vs) {
    unordered_map<int, int> tmp_id2index;

    for (int i = 0; i < int(vs.size()); i++) {
        tmp_id2index[vs[i].plan_time] = 0;
    }
    for (int i = 0; i < int(vs.size()); i++) {
        tmp_id2index[vs[i].plan_time]++;
    }

    return tmp_id2index;
}

void Traffic::update_preset_car(vector<Car> &car) {
    std::unordered_map<int, Car>::iterator car_map_it;
    std::unordered_map<int, Road>::iterator road_map_it;
    std::vector<Pre_set_car>::iterator pre_set_car_it;
    std::vector<int>::iterator road_it;
    for (pre_set_car_it = pre_set_cars.begin(); pre_set_car_it != pre_set_cars.end(); pre_set_car_it++) {
        int car_id = (*pre_set_car_it).id;
        int index = car_id2index[car_id];
        car[index].plan_time = (*pre_set_car_it).plan_time;
        for (road_it = (*pre_set_car_it).path.begin(); road_it != (*pre_set_car_it).path.end(); road_it++) {
            road_map_it = road_ID_dict.find(*road_it);
            car[index].path.push_back(&((*road_map_it).second));
        }

    }
}

int Traffic::get_start_road(Car &car, int time) {
    struct crossInfo {
        int cost;
        Road *road;

        bool operator<(const crossInfo &other) const   //升序排序
        {
            return cost > other.cost;
        }

    };
    if (!car.preset) {//非预制车辆
        int crossID = car.from_id;
        auto it = cross_ID_dict.find(crossID);
        Cross cross = (*it).second;
        Road *nextroad;
        vector<Road *> adjs = getAdjRoadOfCross(&cross);//获得路口的连通道路的ID
        vector<crossInfo> roadInfo;
        for (auto it = adjs.begin(); it != adjs.end(); it++) {//四条道路遍历
            // 首先剔除掉已经在S中的结点
            crossInfo s;

            if ((*it)->id == -1)
                continue;

            // 然后剔除不支持逆行的路口，即出发点不是u，并且不是双通路
            // 然后对剩下的路口进行计算
            int d_t = (*it)->car_num / ((*it)->channels_num * (*it)->length);/*路口时间长度的权值公式*/
            s.cost = d_t;
            s.road = *it;
            roadInfo.push_back(s);//把每一条路当前的系数算出来

        }
        sort(roadInfo.begin(), roadInfo.end(), less<crossInfo>());//按照从小到大的顺序排序

        car.path.push_back(roadInfo[0].road);

        Road *road = roadInfo[0].road;
        int channal_id;
        if (car.from_id == roadInfo[0].road->from_id) {//算方向from--to
            channal_id = road->getAvailableChannelIndex(road->s2e_channels);
            if (channal_id == -1) {
                car.plan_time++;//如果没有合适的出发时间++
                return 1;
            }
//            car.at_road->s2e_channels[channal_id].push(&car);

            car.at_channel_id = channal_id;
            car.at_channel_direction = 1;
            road->s2e_channels[channal_id].push(&car);//队列进入
            car.at_road = road;
            car.updateStatus(1);


        } else {
            channal_id = road->getAvailableChannelIndex(road->e2s_channels);//to-->from
            if (channal_id == -1) {
                car.plan_time++;
                return 1;
            }
//            car.at_road->e2s_channels[channal_id].push(&car);

            car.at_channel_id = channal_id;
            car.at_channel_direction = -1;
            road->s2e_channels[channal_id].push(&car);//队列进入
            car.at_road = road;
            car.updateStatus(1);//状态更新,但是没有跟新位置


        }
    } else {
        Road *road = car.path[0];
        if (car.from_id == road->from_id) {
            int channal_id = road->getAvailableChannelIndex(road->s2e_channels);
            if (channal_id == -1) {
                channal_id = 0;
            }
            car.at_channel_id = channal_id;
            car.at_channel_direction = 1;
            road->s2e_channels[channal_id].push(&car);
            car.at_road = road;
            car.updateStatus(1);


        } else {
            int channal_id = road->getAvailableChannelIndex(road->e2s_channels);
            if (channal_id == -1) {
                channal_id = 0;//预制车辆就不更改时间了,硬着头皮上
            }
//            car.at_road->e2s_channels[channal_id].push(&car);

            car.at_channel_id = channal_id;
            car.at_channel_direction = -1;
            road->e2s_channels[channal_id].push(&car);
//            cout<<road->e2s_channels[channal_id].back()->from_id<<endl;
            car.at_road = road;
            car.updateStatus(1);

        }
    }
    return  0;
}