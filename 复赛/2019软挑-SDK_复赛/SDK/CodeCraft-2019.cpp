//
//  main.cpp
//  CodeCraft-2019
//
//  Created by apple on 2019/3/21.
//  Copyright © 2019年 apple. All rights reserved.
//

#include <iostream>
#include <fstream>
#include <vector>
#include <algorithm>
#include <queue>
#include <limits>
#include <time.h>
#include <ext/hash_map>

using namespace __gnu_cxx;
using namespace std;
const int maxnum = 200;
const int maxint = std::numeric_limits<int>::max();
static int carline = 0;
static int roadline = 0;
static int crossline = 0;
static int pre_answer_line = 0;
//static int c[12000][100][100];   // 记录图的两点间路径长度
//static int cardata[100000][8]={0};
//static int roaddata[100000][8]={0};
//static int crossdata[100000][8]={0};
static int CarRoadID[10000][1002] = {0};
static int dist[150000][1500] = {maxint};     // 表示当前点到源点的最短路径长度
static int prev1[150000][1500] = {0};     // 记录当前点的前一个结点
static int outline = 0;                   //表示当前要处理的车次
static int car_Index_ID[800000] = {0};        //因为hash_map是非线性的,所以只能通过carIndex的下标来所以,存储的是carID
static int timeIndex[800000] = {0};       //时间片索引,下标为时间,值为时间片的车数
static int road_Index_ID[10000] = {0};        //道路索引
static int road_ID_Index[10000] = {0};
static int cross_ID_Index[10000] = {0};       //路口ID和文件所在行数的映射
static int cross_Index_ID[10000] = {0};        //文件的下标和ID的映射
static int frequency[10000][1200][4] = {0};//[批次数][道路数][方向搭配frejudge使用]

static int frejudge = -1;//From 1 To 2
static int c1;
static int c2 = 0.0001;
static int c3 = 0.0005;//调参
static int c4 = 4;//每增加一辆车，代价函数就是这么多
static int c5 = 6;
static int c6 = 30;
struct car {
    int from;
    int to;
    int speed;
    int planTime;
    int priority;
    int preset;
};

struct road {
    int length;
    int speed;
    int channel;
    int from;
    int to;
    int isDuplex;
};

struct cross {

    int roadId_1;
    int roadId_2;
    int roadId_3;
    int roadId_4;
};
struct answer {
    int time;
    queue<int> path;
};

typedef pair<int, struct car> PAIR;

static hash_map<int, struct car> cardata;
static hash_map<int, struct road> roaddata;
static hash_map<int, struct cross> crossdata;
static hash_map<int, struct answer> answer_data;

//用于hash_map的比较排序
struct cmp_speed  //自定义比较规则,速度
{

    bool operator()(const PAIR &P1, const PAIR &P2)  //注意是PAIR类型，需要.first和.second。这个和hash_map类似
    {

        return (P1.second.speed > P2.second.speed);
    }
} speed_cmp;

struct cmp_plantime  //自定义比较规则,速度
{
    bool operator()(const PAIR &P1, const PAIR &P2)  //注意是PAIR类型，需要.first和.second。这个和hash_map类似
    {

        return (P1.second.planTime > P2.second.planTime);
    }
} plantime_cmp;


/**--------------分割字符串函数－－－－－－－－－－－－－**/
vector<string> &split(const string &str, const string &delimiters, vector<string> &elems, bool skip_empty = false) {
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


//v是起点ID,outline是车的下标
void Dijkstra(int n, int v, int outline, int graph[1000][1000]) {
    int PointToRoad(int start, int end);   //v是ID
    bool s[maxnum];    // 判断是否已存入该点到S集合中

    //n是节点个数,
    //v是id,需要转化为下标
    //outline是下标,车的下标
    /////////////////////////////////////图上起点到其他所有节点之间的权值初始化start
    for (int i = 1; i <= n; ++i)//下标
    {
        dist[outline][i] = graph[cross_ID_Index[v]][i];   //dist都是下标Index_ID
        //起点权值刷新start
        if ((int(cardata[car_Index_ID[outline]].planTime + c4 - 1) / c4) > 2) {
            int road1, road2, road3, road4;
            int point = 0;
//            cout<<"起点 "<<v<<endl;
            road1 = crossdata[v].roadId_1;//都是ID
            road2 = crossdata[v].roadId_2;//都是ID
            road3 = crossdata[v].roadId_3;//都是ID
            road4 = crossdata[v].roadId_4;//都是ID
            if (road1 != -1) {
                int truejudge = 0;
                if (v == roaddata[road1].from)
                    frejudge = 1;
                else if (v == roaddata[road1].to)
                    frejudge = 2;

                if (roaddata[road1].from != v && roaddata[road1].isDuplex == 1)//双向,但是起点不是在这里
                {
                    truejudge = 1;
                    point = roaddata[road1].from;
                } else if (roaddata[road1].to != v) {
                    truejudge = 1;
                    point = roaddata[road1].to;
                }
                int one1 = 0;
                int x = int((cardata[car_Index_ID[outline]].planTime + 3) / c4);//outline是下标,需要转化为ID
                one1 += frequency[x - 1][road_ID_Index[road1]][frejudge];
                one1 += frequency[x - 2][road_ID_Index[road1]][frejudge];
                double bandwidth2 = roaddata[road1].channel;
                double bandwidth = roaddata[road1].channel * roaddata[road1].length;
                int speedlimit = min(roaddata[road1].speed, cardata[car_Index_ID[outline]].speed);
                //                one1/=(bandwidth*speedlimit);
//                cout<<"point "<<point<<" one1 "<<one1/(bandwidth2*speedlimit)<<" two "<<bandwidth/(bandwidth2*speedlimit)<<endl;
                one1 = (one1 / bandwidth) * c3 + bandwidth / (bandwidth2 * speedlimit) * c2;//weight公式
                if (truejudge == 1)
                    dist[outline][cross_ID_Index[point]] = one1;
//                cout<<dist[outline][cross_ID_Index[point]]<<endl;

                //补充start
                if (point != 0) {
                    int one = 0, two = 0, three = 0, four = 0;
                    int deliver = 0;
                    int j = point; //这里是点cross_ID
                    int y = road1; //这里也是road_ID

                    if (crossdata[j].roadId_1 != -1 && crossdata[j].roadId_1 != y) {
                        deliver++;
                        int current_road_ID = crossdata[j].roadId_1;
                        int road_index = road_ID_Index[crossdata[j].roadId_1];
                        two += frequency[x - 1][road_index][frejudge];
                        two += frequency[x - 2][road_index][frejudge];
                        double bandwidth2 = roaddata[current_road_ID].channel;
                        double bandwidth = roaddata[current_road_ID].channel * roaddata[current_road_ID].length;
                        int speedlimit = min(roaddata[current_road_ID].speed, cardata[car_Index_ID[outline]].speed);
                        //                    two=two/(bandwidth*speedlimit);
//                        cout<<"two1 "<<two<<endl;
//                        cout<<"two2 "<<   two/(bandwidth2*speedlimit*2)<<endl;
                        two = (two / bandwidth) * c3 + two / (bandwidth2 * speedlimit);//weight公式
                    }
                    if (crossdata[j].roadId_2 != -1 && crossdata[j].roadId_2 != y) {
                        deliver++;
                        int current_road_ID = crossdata[j].roadId_2;
                        int road_index = road_ID_Index[crossdata[j].roadId_2];
                        three += frequency[x - 1][road_index][frejudge];
                        three += frequency[x - 2][road_index][frejudge];
                        //                    int bandwidth=roaddata[crossdata[j][3]-5000+1][4];
                        double bandwidth2 = roaddata[current_road_ID].channel;
                        double bandwidth = roaddata[current_road_ID].channel * roaddata[current_road_ID].length;
                        int speedlimit = min(roaddata[current_road_ID].speed, cardata[car_Index_ID[outline]].speed);
                        //                    three/=(bandwidth*speedlimit);
//                        cout<<"three "<<three/(bandwidth2*speedlimit)<<endl;
                        three = (three / bandwidth) * c3 + bandwidth / (bandwidth2 * speedlimit) * c2;//weight公式

                    }
                    if (crossdata[j].roadId_3 != -1 && crossdata[j].roadId_3 != y) {
                        deliver++;
                        int road_index = road_ID_Index[crossdata[j].roadId_3];//当前道路的下标
                        int current_road_ID = crossdata[j].roadId_3;      //当前道路的ID

                        four += frequency[x - 1][road_index][frejudge];
                        four += frequency[x - 2][road_index][frejudge];
                        double bandwidth2 = roaddata[current_road_ID].channel;
                        double bandwidth = roaddata[current_road_ID].channel * roaddata[current_road_ID].length;
                        int speedlimit = min(roaddata[current_road_ID].speed, cardata[car_Index_ID[outline]].speed);
//                        cout<<"four "<<pow(four/(bandwidth2*speedlimit),2)<<endl;

                        four = (four / bandwidth) * c3 + four / (bandwidth2 * speedlimit);//weight公式
                    }
                    if (crossdata[j].roadId_4 != -1 && crossdata[j].roadId_4 != y) {
                        deliver++;
                        int road_index = road_ID_Index[crossdata[j].roadId_4];//当前道路的下标
                        int current_road_ID = crossdata[j].roadId_4;
                        one += frequency[x - 1][road_index][frejudge];
                        one += frequency[x - 2][road_index][frejudge];
                        double bandwidth2 = roaddata[current_road_ID].channel;
                        double bandwidth = roaddata[current_road_ID].channel * roaddata[current_road_ID].length;
                        int speedlimit = min(roaddata[current_road_ID].speed, cardata[car_Index_ID[outline]].speed);
                        //                    one=one/(bandwidth*speedlimit);
//                        cout<<"one "<<one/(bandwidth2*speedlimit)<<endl;

                        one = (one / bandwidth) * c3 + bandwidth / (bandwidth2 * speedlimit) * c2;//weight公式
                    }
                    //                    four/=deliver;
                    int average = 0;
                    average += (one + two + three + four) / deliver;
                    dist[outline][cross_ID_Index[point]] += average;
//                    cout<<dist[outline][cross_ID_Index[point]]<<endl;
                }
                //补充end
            }
            point = 0;
            if (road2 != -1) {
                int truejudge = 0;
                if (v == roaddata[road2].from)
                    frejudge = 1;
                else if (v == roaddata[road2].to)
                    frejudge = 2;
                if (roaddata[road2].from != v && roaddata[road2].isDuplex == 1) {
                    truejudge = 1;
                    point = roaddata[road2].from;
                } else if (roaddata[road2].to != v) {
                    truejudge = 1;
                    point = roaddata[road2].to;
                }
                int two1 = 0;
                int x = int((cardata[car_Index_ID[outline]].planTime + 3) / c4);
                two1 += frequency[x - 1][road_ID_Index[road2]][frejudge];
                two1 += frequency[x - 2][road_ID_Index[road2]][frejudge];
                double bandwidth2 = roaddata[road2].channel;
                double bandwidth = roaddata[road2].channel * roaddata[road2].length;
                int speedlimit = min(roaddata[road2].speed, cardata[car_Index_ID[outline]].speed);
                //                two1/=(bandwidth*speedlimit);
//                cout<<"point "<<point<<" one1 "<<two1/(bandwidth2*speedlimit)<<" two "<<bandwidth/(bandwidth2*speedlimit)<<endl;

                two1 = (two1 / bandwidth) * c3 + bandwidth / (bandwidth2 * speedlimit) * c2;//weight公式
                if (truejudge == 1)
                    dist[outline][cross_ID_Index[point]] = two1;
//                cout<<dist[outline][cross_ID_Index[point]]<<endl;

                //补充start
                if (point != 0) {
                    int one = 0, two = 0, three = 0, four = 0;
                    int deliver = 0;
                    int j = point;
                    int y = road2;
                    if (crossdata[j].roadId_1 != -1 && crossdata[j].roadId_1 != y) {
                        deliver++;
                        int road_index = road_ID_Index[crossdata[j].roadId_1];
                        int current_road_ID = crossdata[j].roadId_1;
                        one += frequency[x - 1][road_index][frejudge];
                        one += frequency[x - 2][road_index][frejudge];
                        double bandwidth2 = roaddata[current_road_ID].channel;  //用ID  //roadID
                        double bandwidth = roaddata[current_road_ID].channel * roaddata[current_road_ID].length;
                        int speedlimit = min(roaddata[current_road_ID].speed, cardata[car_Index_ID[outline]].speed);
                        //                    one/=(bandwidth*speedlimit);
                        one = (one / bandwidth) * c3 + bandwidth / (bandwidth2 * speedlimit) * c2;//weight公式
                    }
                    if (crossdata[j].roadId_2 != -1 && crossdata[j].roadId_2 != y) {
                        deliver++;
                        int road_index = road_ID_Index[crossdata[j].roadId_2];
                        int current_road_ID = crossdata[j].roadId_2;
                        two += frequency[x - 1][road_index][frejudge];
                        two += frequency[x - 2][road_index][frejudge];
                        double bandwidth2 = roaddata[current_road_ID].channel;
                        double bandwidth = roaddata[current_road_ID].channel * roaddata[current_road_ID].length;
                        int speedlimit = min(roaddata[current_road_ID].speed, cardata[car_Index_ID[outline]].speed);
                        //                    two=two/(bandwidth*speedlimit);
                        two = (two / bandwidth) * c3 + bandwidth / (bandwidth2 * speedlimit) * c2;//weight公式
                    }
                    if (crossdata[j].roadId_3 != -1 && crossdata[j].roadId_3 != y) {
                        deliver++;
                        int road_index = road_ID_Index[crossdata[j].roadId_3];
                        int current_road_ID = crossdata[j].roadId_3;
                        three += frequency[x - 1][road_index][frejudge];
                        three += frequency[x - 2][road_index][frejudge];
                        double bandwidth2 = roaddata[current_road_ID].channel;
                        double bandwidth = roaddata[current_road_ID].channel * roaddata[current_road_ID].length;
                        int speedlimit = min(roaddata[current_road_ID].speed, cardata[car_Index_ID[outline]].speed);
                        //                    three/=(bandwidth*speedlimit);
                        three = (three / bandwidth) * c3 + bandwidth / (bandwidth2 * speedlimit) * c2;//weight公式
                    }
                    if (crossdata[j].roadId_4 != -1 && crossdata[j].roadId_4 != y) {
                        deliver++;
                        int road_index = road_ID_Index[crossdata[j].roadId_4];
                        int current_road_ID = crossdata[j].roadId_4;
                        four += frequency[x - 1][road_index][frejudge];
                        four += frequency[x - 2][road_index][frejudge];
                        double bandwidth2 = roaddata[current_road_ID].channel;
                        double bandwidth = roaddata[current_road_ID].channel * roaddata[current_road_ID].length;
                        int speedlimit = min(roaddata[current_road_ID].speed, cardata[car_Index_ID[outline]].speed);
                        //                    four/=(bandwidth*speedlimit);
                        four = (four / bandwidth) * c3 + bandwidth / (bandwidth2 * speedlimit) * c2;//weight公式
                    }
                    //                    four/=deliver;
                    int average = 0;
                    average += (one + two + three + four) / deliver;
                    dist[outline][cross_ID_Index[point]] += average;
//                    cout<<dist[outline][cross_ID_Index[point]]<<endl;
                }
                //补充end

            }
            point = 0;
            if (road3 != -1)//选择的路的ID
            {
                int truejudge = 0;
                if (v == roaddata[road3].from)
                    frejudge = 1;
                else if (v == roaddata[road3].to)
                    frejudge = 2;
                if (roaddata[road3].from != v && roaddata[road3].channel == 1) {
                    truejudge = 1;
                    point = roaddata[road3].from;
                } else if (roaddata[road3].to != v) {
                    truejudge = 1;
                    point = roaddata[road3].to;
                }
                int three1 = 0;
                int x = int((cardata[car_Index_ID[outline]].planTime + 3) / c4);
                three1 += frequency[x - 1][road3][frejudge];
                three1 += frequency[x - 2][road3][frejudge];
                double bandwidth2 = roaddata[road3].channel;
                double bandwidth = roaddata[road3].channel * roaddata[road3].length;
                int speedlimit = min(roaddata[road3].speed, cardata[car_Index_ID[outline]].speed);
                //                three1/=(bandwidth*speedlimit);
//                cout<<"point "<<point<<" one1 "<<three1/(bandwidth2*speedlimit)<<" two "<<bandwidth/(bandwidth2*speedlimit)<<endl;

                three1 = (three1 / bandwidth) * c3 + bandwidth / (bandwidth2 * speedlimit) * c2;//weight公式
                if (truejudge == 1)
                    dist[outline][cross_ID_Index[point]] = three1;
//                cout<<dist[outline][cross_ID_Index[point]]<<endl;

                //补充start
                if (point != 0) {
                    int one = 0, two = 0, three = 0, four = 0;
                    int deliver = 0;
                    int j = point;
                    int y = road3;
                    if (crossdata[j].roadId_1 != -1 && crossdata[j].roadId_1 != y) {
                        deliver++;
                        int road_index = road_ID_Index[crossdata[j].roadId_1];
                        int current_road_ID = crossdata[j].roadId_1;
                        one += frequency[x - 1][road_index][frejudge];
                        one += frequency[x - 2][road_index][frejudge];
                        double bandwidth2 = roaddata[current_road_ID].channel;
                        double bandwidth = roaddata[current_road_ID].channel * roaddata[current_road_ID].length;
                        int speedlimit = min(roaddata[current_road_ID].speed, cardata[car_Index_ID[outline]].speed);
                        //                    one/=(bandwidth*speedlimit);
                        one = (one / bandwidth) * c3 + bandwidth / (bandwidth2 * speedlimit) * c2;//weight公式
                    }
                    if (crossdata[j].roadId_2 != -1 && crossdata[j].roadId_2 != y) {
                        deliver++;
                        int road_index = road_ID_Index[crossdata[j].roadId_2];
                        int current_road_ID = crossdata[j].roadId_2;
                        two += frequency[x - 1][road_index][frejudge];
                        two += frequency[x - 2][road_index][frejudge];
                        double bandwidth2 = roaddata[current_road_ID].channel;
                        double bandwidth = roaddata[current_road_ID].channel * roaddata[current_road_ID].length;
                        int speedlimit = min(roaddata[current_road_ID].speed, cardata[car_Index_ID[outline]].speed);
                        //                    two=two/(bandwidth*speedlimit);
                        two = (two / bandwidth) * c3 + bandwidth / (bandwidth2 * speedlimit) * c2;//weight公式
                    }
                    if (crossdata[j].roadId_3 != -1 && crossdata[j].roadId_3 != y) {
                        deliver++;
                        int road_index = road_ID_Index[crossdata[j].roadId_3];
                        int current_road_ID = crossdata[j].roadId_3;
                        three += frequency[x - 1][road_index][frejudge];
                        three += frequency[x - 2][road_index][frejudge];
                        double bandwidth2 = roaddata[current_road_ID].channel;
                        double bandwidth = roaddata[current_road_ID].channel * roaddata[current_road_ID].length;
                        int speedlimit = min(roaddata[current_road_ID].speed, cardata[car_Index_ID[outline]].speed);
                        //                    three/=(bandwidth*speedlimit);
                        three = (three / bandwidth) * c3 + bandwidth / (bandwidth2 * speedlimit) * c2;//weight公式
                    }
                    if (crossdata[j].roadId_4 != -1 && crossdata[j].roadId_4 != y) {
                        deliver++;
                        int road_index = road_ID_Index[crossdata[j].roadId_4];
                        int current_road_ID = crossdata[j].roadId_4;
                        four += frequency[x - 1][road_index][frejudge];
                        four += frequency[x - 2][road_index][frejudge];
                        double bandwidth2 = roaddata[current_road_ID].channel;
                        double bandwidth = roaddata[current_road_ID].channel * roaddata[current_road_ID].length;
                        int speedlimit = min(roaddata[current_road_ID].speed, cardata[car_Index_ID[outline]].speed);
                        //                    four/=(bandwidth*speedlimit);
                        four = (four / bandwidth) * c3 + bandwidth / (bandwidth2 * speedlimit) * c2;//weight公式
                    }
                    //                    four/=deliver;
                    int average = 0;
                    average += (one + two + three + four) / deliver;
//                    cout<<"average "<<average<<endl;
                    dist[outline][cross_ID_Index[point]] += average;
//                    cout<<dist[outline][cross_ID_Index[point]]<<endl;
                }
                //补充end

            }
            point = 0;
            //            cout<<road4<<endl;
            if (road4 != -1) {
                int truejudge = 0;
                if (v == roaddata[road4].from)
                    frejudge = 1;
                else if (v == roaddata[road4].to)
                    frejudge = 2;
                if (roaddata[road4].from != v && roaddata[road4].isDuplex == 1) {
                    truejudge = 1;
                    point = roaddata[road4].from;
                } else if (roaddata[road4].to != v) {
                    truejudge = 1;
                    point = roaddata[road4].to;
                }
                int four1 = 0;
                int x = int((cardata[car_Index_ID[outline]].planTime + 3) / c4);
                four1 += frequency[x - 1][road_ID_Index[road4]][frejudge];
                four1 += frequency[x - 2][road_ID_Index[road4]][frejudge];
                double bandwidth2 = roaddata[road4].channel;
                double bandwidth = roaddata[road4].channel * roaddata[road4].length;
                int speedlimit = min(roaddata[road4].speed, cardata[car_Index_ID[outline]].speed);
                //                four1/=(bandwidth*speedlimit);
//                cout<<"point "<<point<<" one1 "<<four1/(bandwidth2*speedlimit)<<" two "<<bandwidth/(bandwidth2*speedlimit)<<endl;

                four1 = (four1 / bandwidth) * c3 + bandwidth / (bandwidth2 * speedlimit) * c2;//weight公式
                if (truejudge == 1)
                    dist[outline][cross_ID_Index[point]] = four1;
//                cout<<dist[outline][cross_ID_Index[point]]<<endl;
                //补充start
                if (point != 0) {
                    int one = 0, two = 0, three = 0, four = 0;
                    int deliver = 0;
                    int j = point;
                    int y = road4;
                    if (crossdata[j].roadId_1 != -1 && crossdata[j].roadId_1 != y) {
                        deliver++;
                        int road_index = road_ID_Index[crossdata[j].roadId_1];
                        int current_road_ID = crossdata[j].roadId_1;
                        one += frequency[x - 1][road_index][frejudge];
                        one += frequency[x - 2][road_index][frejudge];
                        double bandwidth2 = roaddata[current_road_ID].channel;
                        double bandwidth = roaddata[current_road_ID].channel * roaddata[current_road_ID].speed;
                        int speedlimit = min(roaddata[current_road_ID].speed, cardata[car_Index_ID[outline]].speed);
                        //                    one/=(bandwidth*speedlimit);
                        one = (one / bandwidth) * c3 + bandwidth / (bandwidth2 * speedlimit) * c2;//weight公式
                    }
                    if (crossdata[j].roadId_2 != -1 && crossdata[j].roadId_2 != y) {
                        deliver++;
                        int road_index = road_ID_Index[crossdata[j].roadId_2];
                        int current_road_ID = crossdata[j].roadId_2;
                        two += frequency[x - 1][road_index][frejudge];
                        two += frequency[x - 2][road_index][frejudge];
                        double bandwidth2 = roaddata[current_road_ID].channel;
                        double bandwidth = roaddata[current_road_ID].channel * roaddata[current_road_ID].length;
                        int speedlimit = min(roaddata[current_road_ID].speed, cardata[car_Index_ID[outline]].speed);
                        //                    two=two/(bandwidth*speedlimit);
                        two = (two / bandwidth) * c3 + bandwidth / (bandwidth2 * speedlimit) * c2;//weight公式
                    }
                    if (crossdata[j].roadId_3 != -1 && crossdata[j].roadId_3 != y) {
                        deliver++;
                        int road_index = road_ID_Index[crossdata[j].roadId_3];
                        int current_road_ID = crossdata[j].roadId_3;
                        three += frequency[x - 1][road_index][frejudge];
                        three += frequency[x - 2][road_index][frejudge];
                        double bandwidth2 = roaddata[current_road_ID].channel;
                        double bandwidth = roaddata[current_road_ID].channel * roaddata[current_road_ID].length;
                        int speedlimit = min(roaddata[current_road_ID].speed, cardata[car_Index_ID[outline]].speed);
                        //                    three/=(bandwidth*speedlimit);
                        three = (three / bandwidth) * c3 + bandwidth / (bandwidth2 * speedlimit) * c2;//weight公式
                    }
                    if (crossdata[j].roadId_4 != -1 && crossdata[j].roadId_4 != y) {
                        deliver++;
                        int road_index = road_ID_Index[crossdata[j].roadId_4];
                        int current_road_ID = crossdata[j].roadId_4;
                        four += frequency[x - 1][road_index][frejudge];
                        four += frequency[x - 2][road_index][frejudge];
                        double bandwidth2 = roaddata[current_road_ID].channel;
                        double bandwidth = roaddata[current_road_ID].channel * roaddata[current_road_ID].speed;
                        int speedlimit = min(roaddata[current_road_ID].speed, cardata[car_Index_ID[outline]].speed);
                        //                    four/=(bandwidth*speedlimit);
                        four = (four / bandwidth) * c3 + bandwidth / (bandwidth2 * speedlimit) * c2;//weight公式
                    }
                    //                    four/=deliver;
                    int average = 0;
                    average += (one + two + three + four) / deliver;
                    dist[outline][cross_ID_Index[point]] += average;
//                    cout<<dist[outline][cross_ID_Index[point]]<<endl;
                }
                //补充end
            }
        }
        //起点权值刷新end
        s[i] = 0;     // 初始都未用过该点
        if (dist[outline][i] == maxint)
            prev1[outline][i] = 0;
        else
            prev1[outline][i] = cross_ID_Index[v];
    }
    dist[outline][cross_ID_Index[v]] = 0;
    s[cross_ID_Index[v]] = 1;   //全部使用下标
    /////////////////////////////////////图上起点到其他所有节点之间的权值初始化end

    ////////////////////////////////////遍历所有点start
    // 依次将未放入S集合的结点中，取dist[]最小值的结点，放入结合S中
    // 一旦S包含了所有V中顶点，dist就记录了从源点到所有其他顶点之间的最短路径长度
    for (int i = 2; i <= n; ++i) {
        int tmp = maxint;
        int u = v;
        // 找出当前未使用的点j的dist[j]最小值
        for (int j = 1; j <= n; ++j)
            if ((!s[j]) && dist[outline][j] < tmp) {
                u = j;              // u保存当前邻接点中距离最小的点的号码
                tmp = dist[outline][j];
            }
        s[u] = 1;    // 表示u点已存入S集合中

        //铺路
        int prepoint = prev1[outline][u];
//        cout<<"prepoint "<<prepoint<<" u "<<u<<endl;
        int chosenroad = PointToRoad(cross_Index_ID[prepoint], cross_Index_ID[u]);
//        cout<<"chosenroad "<<chosenroad<<endl;
        if (prepoint == roaddata[chosenroad].from)
            frejudge = 1;
        else if (prepoint == roaddata[chosenroad].to)
            frejudge = 2;

        int time = int(cardata[car_Index_ID[outline]].planTime + c4 - 1) / c4;
        frequency[time][cross_ID_Index[chosenroad]][frejudge] += 1;
//        cout<<int((cardata[outline][5]+3)/c4)<<","<<chosenroad<<","<<frejudge<<","<<frequency[int((cardata[outline][5]+3)/c4)][chosenroad-5000+1][frejudge]<<endl;
//        cout<<"hello"<<endl;

//        int preprepoint = prev1[outline][prepoint]; //前前一条路的节点
//        int preroad = PointToRoad(preprepoint,prepoint);//前一条路的ID
//        if (preprepoint==roaddata[preroad-5000+1][5])
//            frejudge=1;
//        else if(preprepoint==roaddata[preroad-5000+1][6])
//            frejudge=2;
//        if(frequency[int((cardata[outline][5]+3)/c4)][preroad-5000+1][frejudge])
//            frequency[int((cardata[outline][5]+3)/c4)][preroad-5000+1][frejudge]-=1;

//        if(chosenroad==5049) {
//            cout<<outline<<" "<<frejudge<<endl;
//            cout << frequency[int((cardata[outline][5] + 3) / 4)-1][chosenroad - 5000 + 1][frejudge] << endl;
//        }
        // 更新dist
        for (int j = 1; j <= n; ++j)
            if ((!s[j]) && graph[u][j] < maxint)//u是index,j是index
            {
                int newdist = maxint;
                //修改边权限制条件
                //                cout<<cardata[outline][5]<<endl;

                if ((int(cardata[car_Index_ID[outline]].planTime + 3) / c4) > 2) {
                    int x = int((cardata[car_Index_ID[outline]].planTime + 3) / c4);
//                    cout<<"frequency 5033 1 "<<frequency[x-1][5033-5000+1][1]<<endl;
//                    cout<<"frequency 5033 2 "<<frequency[x-1][5033-5000+1][2]<<endl;
//                    cout<<"frequency 5032 1 "<<frequency[x-1][5032-5000+1][1]<<endl;
//                    cout<<"frequency 5032 2 "<<frequency[x-1][5032-5000+1][2]<<endl;
//                    cout<<"frequency 5026 1 "<<frequency[x-1][5026-5000+1][1]<<endl;
//                    cout<<"frequency 5026 2 "<<frequency[x-1][5026-5000+1][2]<<endl;



                    int y = PointToRoad(cross_Index_ID[u], cross_Index_ID[j]);//roadID
                    if (u == roaddata[y].from)
                        frejudge = 1;
                    else if (u == roaddata[y].to)
                        frejudge = 2;
                    int pretwo = (frequency[x - 1][road_ID_Index[y]][frejudge] +
                                  frequency[x - 2][road_ID_Index[y]][frejudge]);
                    //new调整start
                    int one = 0, two = 0, three = 0, four = 0;
                    int deliver = 0;
                    if (crossdata[cross_Index_ID[j]].roadId_1 != -1 && crossdata[cross_Index_ID[j]].roadId_1 != y) {
                        deliver++;
                        int current_road_ID = crossdata[cross_Index_ID[j]].roadId_1;
                        int road_index = road_ID_Index[crossdata[cross_Index_ID[j]].roadId_1];
                        one += frequency[x - 1][road_index][frejudge];
                        one += frequency[x - 2][road_index][frejudge];
                        double bandwidth2 = roaddata[current_road_ID].channel;   //单条道路的宽度
                        double bandwidth =
                                roaddata[current_road_ID].channel * roaddata[current_road_ID].length;  //长度*宽度
                        int speedlimit = min(roaddata[current_road_ID].speed,
                                             cardata[car_Index_ID[outline]].speed);               //速度
                        //                        one/=(bandwidth*speedlimit);
                        one = (one / bandwidth) * c3 + bandwidth / (bandwidth2 * speedlimit) * c2;//weight公式
                    }
                    if (crossdata[cross_Index_ID[j]].roadId_2 != -1 && crossdata[cross_Index_ID[j]].roadId_2 != y) {
                        deliver++;
                        int current_road_ID = crossdata[cross_Index_ID[j]].roadId_2;
                        int road_index = road_ID_Index[crossdata[cross_Index_ID[j]].roadId_2];
                        two += frequency[x - 1][road_index][frejudge];
                        two += frequency[x - 2][road_index][frejudge];
                        double bandwidth2 = roaddata[current_road_ID].channel;
                        double bandwidth = roaddata[current_road_ID].channel * roaddata[current_road_ID].length;
                        int speedlimit = min(roaddata[current_road_ID].speed, cardata[car_Index_ID[outline]].speed);
                        //                        two=two/(bandwidth*speedlimit);
                        two = (two / bandwidth) * c3 + bandwidth / (bandwidth2 * speedlimit) * c2;//weight公式
                    }
                    if (crossdata[cross_Index_ID[j]].roadId_3 != -1 && crossdata[cross_Index_ID[j]].roadId_3 != y) {
                        deliver++;
                        int current_road_ID = crossdata[cross_Index_ID[j]].roadId_3;
                        int road_index = road_ID_Index[crossdata[cross_Index_ID[j]].roadId_3];
                        three += frequency[x - 1][road_index][frejudge];
                        three += frequency[x - 2][road_index][frejudge];
                        double bandwidth2 = roaddata[current_road_ID].channel;
                        double bandwidth = roaddata[current_road_ID].channel * roaddata[current_road_ID].length;
                        int speedlimit = min(roaddata[current_road_ID].speed, cardata[car_Index_ID[outline]].speed);
                        //                        three/=(bandwidth*speedlimit);
                        three = (three / bandwidth) * c3 + bandwidth / (bandwidth2 * speedlimit) * c2;//weight公式
                    }
                    if (crossdata[cross_Index_ID[j]].roadId_4 != -1 && crossdata[cross_Index_ID[j]].roadId_4 != y) {
                        deliver++;
                        int current_road_ID = crossdata[cross_Index_ID[j]].roadId_4;
//                        cout<<crossdata[cross_Index_ID[j]].roadId_4<<endl;
                        int road_index = road_ID_Index[crossdata[cross_Index_ID[j]].roadId_4];
                        four += frequency[x - 1][road_index][frejudge];
                        four += frequency[x - 2][road_index][frejudge];
                        double bandwidth2 = roaddata[current_road_ID].channel;
                        double bandwidth = roaddata[current_road_ID].channel * roaddata[current_road_ID].length;
                        int speedlimit = min(roaddata[current_road_ID].speed, cardata[car_Index_ID[outline]].speed);
                        //                        four/=(bandwidth*speedlimit);

//                        cout<<"1:  "<<(four/bandwidth)*c3<<" 2  "<<bandwidth/(bandwidth2*speedlimit)<<endl;

                        four = (four / bandwidth) * c3 + bandwidth / (bandwidth2 * speedlimit) * c2;//weight公式
                    }
                    //                    four/=deliver;
                    int average = 0;
                    average += (one + two + three + four) / deliver;
                    //                    cout<<average<<endl;
                    //                    pretwo+=four;
                    //new调整end

                    double bandwidth = (roaddata[y].channel * roaddata[y].length);
                    double bandwidth2 = roaddata[y].channel;
                    int speedlimit = min(roaddata[y].speed, cardata[car_Index_ID[outline]].speed);
                    //                    newdist=dist[outline][u]+(pretwo/(bandwidth*speedlimit))+average;
                    newdist = dist[outline][u] + (pretwo / (bandwidth)) * c3 + average +
                              c2 * (bandwidth / (bandwidth2 * speedlimit));//weight公式
//                    cout<<c2*(pretwo/(bandwidth2*speedlimit))<<"  ";
                    //                    cout<<(pretwo/(bandwidth*speedlimit))<<endl;
                } else
                    newdist = dist[outline][cross_ID_Index[u]] + graph[cross_ID_Index[u]][cross_ID_Index[j]];
//                cout<<"new dist"<<newdist<<endl;
                //
                if (newdist < dist[outline][j]) {
//                    cout<<"if "<<endl;
                    dist[outline][j] = newdist;
//                    cout<<dist[outline][j]<<endl;
                    prev1[outline][j] = u;
                }
            }
    }
    ////////////////////////////////////遍历所有点end

}

//起点 终点//返回道路ID
int PointToRoad(int start, int end) {
    int result = -1;
    int i;
    int yes = 0;
    for (i = 1; i <= roadline; i++) {
        if (start == roaddata[road_Index_ID[i]].from && end == roaddata[road_Index_ID[i]].to) {
            yes = 1;
            break;
        }
        if (start == roaddata[road_Index_ID[i]].to && end == roaddata[road_Index_ID[i]].from &&
            roaddata[road_Index_ID[i]].isDuplex == 1) {
            yes = 1;
            break;
        }
    }
    if (yes == 1) {
        result = road_Index_ID[i];
    }
    return result;
}

//v 起点 u 终点 carnum第几辆车
void searchPath(int v, int u, int carnum) {
    struct answer answer_Info;
    answer_Info.time = cardata[car_Index_ID[carnum]].planTime;

    int que[maxnum];
    int tot = 1;
    que[tot] = u;
    tot++;
    int tmp = prev1[outline][cross_ID_Index[u]];//prev 返回的是index
    while (cross_Index_ID[tmp] != v) {
        que[tot] = cross_Index_ID[tmp];   //que送进去的是ID
        tot++;
        tmp = prev1[outline][tmp];
    }
    que[tot] = v;
    int pa = 1;
    for (int i = tot; i > 0; --i) {
        int result = PointToRoad(que[i], que[i - 1]);
//        CarRoadID[carnum][pa]=result;
        // cout<<"result "<<result<<endl;
        answer_Info.path.push(result);
        pa++;

    }
    answer_data.insert(pair<int, struct answer>(car_Index_ID[carnum], answer_Info));
    //        if(i != 1)
    //            cout << que[i] << " -> ";
    //        else
    //            cout << que[i] << endl;
}
/**---------------------------------读取文件------所有数据从第1行第1列开始存放-------------------------------**/

/**-----------------------读取car文件---------------------------------**/
/*-----(id,from,to,speed,planTime, priority, preset)----------------*/
void carin(const char *nn) {
    string carstr;
    ifstream carfile;
    carfile.open(nn);
    getline(carfile, carstr);
    if (carfile.is_open()) {
        carline = 1;//数据从第1行开始存放
        while (!carfile.eof()) {
            int carID;
            struct car carInfo;
            vector<string> result;
            getline(carfile, carstr);
            //去除前后的()
            carstr.erase(carstr.length() - 1, 1);
            carstr.erase(0, 1);
            //分割字符串
            split(carstr, ",", result);
            vector<string>::iterator iter = result.begin();

            //获取ID
            carID = stoi(*(iter++));
            car_Index_ID[carline] = carID;//当前行的车的ID
            carInfo.from = stoi(*(iter++));
            carInfo.to = stoi(*(iter++));
            carInfo.speed = stoi(*(iter++));
            carInfo.planTime = stoi(*(iter++));
            carInfo.priority = stoi(*(iter++));
            carInfo.preset = stoi(*iter);

            cardata.insert(pair<int, struct car>(carID, carInfo));
            ++carline;
            //提取数字end
        }

    } else {
        // cout << "wrong";
    }
    carfile.close();
    carline--;
}

/**-----------------------读取road文件---------------------------------**/
/*-----------------(id,length,speed,channel,from,to,isDuplex)---------*/
void roadin(const char *nn) {
    string str;
    ifstream roadfile;
    roadfile.open(nn);
    getline(roadfile, str);
    if (roadfile.is_open()) {
        roadline = 1;//数据从第1行开始存放
        while (!roadfile.eof()) {
            int roadID;
            struct road roadInfo;
            vector<string> result;
            getline(roadfile, str);
            //去除前后的()
            str.erase(str.length() - 1, 1);
            str.erase(0, 1);
            //分割字符串
            split(str, ",", result);
            vector<string>::iterator iter = result.begin();

            //获取ID
            roadID = stoi(*(iter++));
            road_Index_ID[roadline] = roadID;    //获取道路iD
            road_ID_Index[roadID] = roadline;
            roadInfo.length = stoi(*(iter++));
            roadInfo.speed = stoi(*(iter++));
            roadInfo.channel = stoi(*(iter++));
            roadInfo.from = stoi((*iter++));
            roadInfo.to = stoi(*(iter++));
            roadInfo.isDuplex = stoi(*iter);

            roaddata.insert(pair<int, struct road>(roadID, roadInfo));
            ++roadline;
            //提取数字end

        }
    } else {
        // cout << "wrong";
    }
    roadfile.close();
    roadline--;
}

/**-----------------------读取cross文件---------------------------------**/
/*-----------------(id,roadId,roadId,roadId,roadId)--------------------*/
void crossin(const char *nn) {
    string str;
    ifstream crossfile;
    crossfile.open(nn);
    getline(crossfile, str);
    if (crossfile.is_open()) {
        crossline = 1;//数据从第1行开始存放
        while (!crossfile.eof()) {
            int crossID;
            struct cross crossInfo;
            vector<string> result;
            getline(crossfile, str);
            //去除前后的()
            str.erase(str.length() - 1, 1);
            str.erase(0, 1);
            //分割字符串
            split(str, ", ", result);
            vector<string>::iterator iter1 = result.begin();

            //获取ID

            crossID = stoi(*(iter1++));
            cross_Index_ID[crossline] = crossID;  //记录函数来获得路口ID
            cross_ID_Index[crossID] = crossline;  //通过索引来获得路口信息
            crossInfo.roadId_1 = stoi(*(iter1++));
            crossInfo.roadId_2 = stoi(*(iter1++));
            crossInfo.roadId_3 = stoi(*(iter1++));
            crossInfo.roadId_4 = stoi(*iter1);

            crossdata.insert(pair<int, struct cross>(crossID, crossInfo));
            ++crossline;
            //提取数字end

        }
    } else {
        // cout << "wrong";
    }
    crossfile.close();
    crossline--;
}

/**-----------------------读取presetAnswer.txt文件---------------------------------**/
/*-----------------(carid, time, roadId1...)--------------------*/

void pre_Answerin(const char *nn) {
    string str;
    ifstream pre_answer_file;
    pre_answer_file.open(nn);
    getline(pre_answer_file, str);
    if (pre_answer_file.is_open()) {
        pre_answer_line = 1;//数据从第1行开始存放
        int crossID;
        while (!pre_answer_file.eof()) {
            int carID;
            struct answer pre_answer_Info;
            vector<string> result;
            getline(pre_answer_file, str);
            //去除前后的()
            str.erase(str.length() - 1, 1);
            str.erase(0, 1);
            //分割字符串
            split(str, ", ", result);
            vector<string>::iterator iter = result.begin();

            //获取ID
            carID = stoi(*iter++);
            pre_answer_Info.time = stoi(*iter++);


            for (iter; iter != result.end(); iter++)
                pre_answer_Info.path.push(stoi(*iter));
            answer_data.insert(pair<int, struct answer>(carID, pre_answer_Info));
            //提取数字end

            ++pre_answer_line;
        }
    } else {
        // cout << "wrong";
    }
    pre_answer_file.close();
    pre_answer_line--;
}

//车辆排序模板函数
template<typename T>
void carsort(const T &cmp) {
    vector<PAIR> scoreVector;
    hash_map<int, struct car>::iterator iter = cardata.begin();
    for (; iter != cardata.end(); iter++)  //这边本来是使用vector直接初始化的，当时由于vc 6.0 编译器问题，只能这样写，而且还有非法内存。。
        scoreVector.push_back(*iter);
    //转化为PAIR的vector
    sort(scoreVector.begin(), scoreVector.end(), cmp);  //需要指定cmp

    vector<PAIR>::iterator it = scoreVector.begin();
    int line = 1;//重新索引
    while (it != scoreVector.end()) {
        car_Index_ID[line++] = it->first;

        it++;
    }
}

//统计预制发车的时间片的时间
void calculate_pre_set_car_time() {
    int line = 1;
    hash_map<int, struct answer>::iterator it = answer_data.begin();
    for (; it != answer_data.end(); it++)
        timeIndex[it->second.time]++;
}


void ChangeStartTime() {
    int j = 1;   //用来统计时间的
    int line = 0;//从0开始
    int epoch = 3000;/**调参调参调参**/
    int start = 1;//表示当前可供选择的最小时间起点
    int car_num = int(carline / epoch);//每个时间片的发车速度
    int time;
    for(int i=1;i<=c5;i++)//5
    {
        int j=1;
        while(j<=c6)//50
        {
            if(cardata[car_Index_ID[line]].preset==1){
                line++;
                continue;
            }
            cardata[car_Index_ID[line]].planTime=max(j,cardata[line].planTime);
            line++;
            j++;
        }
    }
    j = 1;
    while (line < carline) {
        line++;
        if (cardata[car_Index_ID[line]].preset) {//如果是预制车辆跳过
            line;
            continue;
        }

        if (j > epoch)//调参啊调参
        {
            j=1;
            if (timeIndex[start] >= car_num) {//如果时间片用尽,那么往后推
                start++;
                j = start;  //重置时间片
                timeIndex[j]++;  //时间片计数
            }
        }
        //j是时刻记录当前批次的时间点的统计
        cardata[car_Index_ID[line]].planTime = max(j, cardata[car_Index_ID[line]].planTime);
        if (car_Index_ID[time] > car_num)//如果这个时间点满了,那么调整时间
            cardata[car_Index_ID[line]].planTime++;
        j = cardata[car_Index_ID[line]].planTime;//j更新
        timeIndex[j++]++; //时间片车数更新

    }
    carsort(plantime_cmp);


}


int main(int a, char *argv[]) {//car road cross answer
    clock_t first, finish;
    first = clock();
    // cout << "in main" << endl;
    int (*graph)[1000] = new int[1000][1000];

   const char* carargv=argv[1];
   const char* roadargv=argv[2];
   const char* crossargv=argv[3];
   const char *pre_answer_argv =argv[4];
//
    // const char *carargv = "/media/wzd/File2/大学项目/codedraft/c/2/2-map-training-1/car.txt";
    // const char *roadargv = "/media/wzd/File2/大学项目/codedraft/c/2/2-map-training-1/road.txt";
    // const char *crossargv = "/media/wzd/File2/大学项目/codedraft/c/2/2-map-training-1/cross.txt";
    // const char *pre_answer_argv = "/media/wzd/File2/大学项目/codedraft/c/2/2-map-training-1/presetAnswer.txt";

    //读取数据
    crossin(crossargv);
    carin(carargv);
    roadin(roadargv);
    pre_Answerin(pre_answer_argv);

    carsort(speed_cmp);//速度比较
    ChangeStartTime();


    // cout << "hello";

    // 各数组都从下标1开始
    //初始化地图
    for (int i = 1; i <= crossline; ++i)
        for (int j = 1; j <= crossline; ++j)
            if (i != j) {
                graph[i][j] = maxint;
            } else
                graph[i][j] = 0;
    for (int i = 1; i <= roadline; ++i) {
        int start_ID = roaddata[road_Index_ID[i]].from; //起点的ID
        int end_ID = roaddata[road_Index_ID[i]].to;     //终点的ID
        int distance = roaddata[road_Index_ID[i]].length;
        int start = cross_ID_Index[start_ID];
        int end = cross_ID_Index[end_ID];
        graph[start][end] = distance;
        if (roaddata[road_Index_ID[i]].isDuplex == 1)       // 双向
        {
            swap(start, end);
            graph[start][end] = distance;
        }
    }
    // 各数组都从下标1开始

    for (outline = 1; outline <= 10; outline++) {
        int ID = car_Index_ID[outline];
//
        if (!cardata[ID].preset) {
            // cout << outline << endl;
            int start_ID = cardata[ID].from;
            int end_ID = cardata[ID].to;
            Dijkstra(crossline, start_ID, outline, graph);// 这里传进去的是起点的ID和车的行数索引
            searchPath(start_ID, end_ID, outline);
        }

    }
    // cout << "write file" << endl;
    ofstream outfile;
   const char* answer_file=argv[5];
    // const char *answer_file = "/media/wzd/File2/大学项目/codedraft/c/2/2-map-training-1/answer.txt";
    outfile.open(answer_file);

    hash_map<int, struct answer>::iterator it = answer_data.begin();
    struct answer unset;
    queue<int> answer_queue;
    for (; it != answer_data.end(); it++) {
        int carID = it->first;
        if (!cardata[carID].preset)//未预置
        {
            outfile << "(" << carID << ",";
            unset = it->second;
            outfile << unset.time << ",";
            answer_queue = unset.path;
            int size = answer_queue.size();
            int index = 1;
            while (!answer_queue.empty() && ++index<size){
//                cout<<"size "<<answer_queue.size()<<endl;
                outfile << answer_queue.front() << ",";
//                cout<< answer_queue.front()<<endl;
                answer_queue.pop();
            }
            outfile << answer_queue.front();
            answer_queue.pop();
            outfile << ")" << endl;

        }
    }
    finish = clock();
     cout<< double(finish - first) / CLOCKS_PER_SEC << endl;
    //cout 最短路径长度
//    cout << "源点到最后一个顶点的最短路径长度: " << dist[crossline] << endl;//距离升序
//    // 路径
//    cout << "源点到最后一个顶点的路径为: ";
//    searchPath(prev, 2, crossline);
}
