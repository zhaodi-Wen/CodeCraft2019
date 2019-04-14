//
//  main.cpp
//  CodeCraft-2019
//
//  Created by apple on 2019/3/21.
//  Copyright © 2019年 apple. All rights reserved.
//

#include <iostream>
#include <fstream>
#include <cmath>
using namespace std;
const int maxnum = 200;
const int maxint = 999999;
static int carline=0;
static int roadline=0;
static int crossline=0;
//static int c[12000][100][100];   // 记录图的两点间路径长度
static double maxtest1=-10;
//[][][]用来记录数值上限用于限定开辟空间的大小
static int cardata[110000][8]={0};//[车辆数][属性]
static int roaddata[10000][8]={0};//[道路数][属性]
static int crossdata[10000][8]={0};//[路口数][属性]
static int CarRoadID[110000][2200]={0};//[车辆数][道路数]
static int dist[110000][300]={0};     // 表示当前点到源点的最短路径长度[车辆数][路口数]
static int prev1[110000][300]={0};     // 记录当前点的前一个结点[车辆数][路口数]
static int frequency[6000][1200][4]={0};//[批次数][道路数][方向搭配frejudge使用]
static int CarIDToIndex[110000];
static int RoadIDToIndex[110000];
static int CorssIDToIndex[110000];
static int frejudge=-1;//From 1 To 2
static int c1;
static int c2=10;
static double c3=1300;//调参
static int c4 = 4;//每增加一辆车，代价函数就是这么多
static int c5 = 3;
static int c6= 10;
static int pici=1900;
static int outline=0;
//所有数据从第1行第1列开始存放
//cross
//(id,roadId,roadId,roadId,roadId)
//car
//(id,from,to,speed,planTime)
//road
//(id,length,speed,channel,from,to,isDuplex)
//v为起点 n为节点数 dist到不同点去距离 prev不同点的前节点 c表示图

double Congestion(int num)
{
    return (1/2+1/(1+exp(-num)));
}

void Dijkstra(int n,int v,int outline,int c[2500][2500])
{
    int PointToRoad(int start,int end);//return ID
    bool s[maxnum];    // 判断是否已存入该点到S集合中 INDEX
    
    
    /////////////////////////////////////图上起点到其他所有节点之间的权值初始化start
    for(int i=1; i<=n; ++i)
    {
        dist[outline][i] = c[v][i];
        //起点权值刷新start
        if(int((cardata[outline][5]+3)/c4)>2)
        {
            int road1,road2,road3,road4;
            int point=0;
            road1=crossdata[v][2];
            road2=crossdata[v][3];
            road3=crossdata[v][4];
            road4=crossdata[v][5];
            if(road1!=1)
            {
                int truejudge=0;
                if (v==CorssIDToIndex[roaddata[RoadIDToIndex[road1]][5]])
                    frejudge=1;
                else if(v==CorssIDToIndex[roaddata[RoadIDToIndex[road1]][6]])
                    frejudge=2;
                if( CorssIDToIndex[roaddata[RoadIDToIndex[road1]][5]]!=v&&roaddata[RoadIDToIndex[road1]][7]==1)
                {
                    truejudge=1;
                    point=CorssIDToIndex[roaddata[RoadIDToIndex[road1]][5]];//下标
                }
                else if(CorssIDToIndex[roaddata[RoadIDToIndex[road1]][6]]!=v)
                {
                    truejudge=1;
                    point=CorssIDToIndex[roaddata[RoadIDToIndex[road1]][6]];
                }
                int one1=0;
                int x=int((cardata[outline][5]+3)/c4);//
                one1+=frequency[x-1][RoadIDToIndex[road1]][frejudge];
                one1+=frequency[x-2][RoadIDToIndex[road1]][frejudge];
                double bandwidth2=roaddata[RoadIDToIndex[road1]][4];
                double bandwidth=roaddata[RoadIDToIndex[road1]][4]*roaddata[RoadIDToIndex[road1]][2];
                int speedlimit=min(roaddata[RoadIDToIndex[road1]][3],cardata[outline][4]);
                //                one1/=(bandwidth*speedlimit);
//                cout<<"one1 "<<one1/(bandwidth2*speedlimit)<<endl;
                one1=(one1/bandwidth)*c3+bandwidth/(bandwidth2*speedlimit)*c2;//weight公式
                if(truejudge==1)
                    dist[outline][point]=one1;

                //补充start
                if(point!=0)
                {
                    int one=0,two=0,three=0,four=0;
                    int deliver=0;
                    int j=point;
                    int y=road1;
                    if(crossdata[j][5]!=1&&crossdata[j][5]!=y)
                    {
                        deliver++;
                        one+=frequency[x-1][RoadIDToIndex[crossdata[j][5]]][frejudge];
                        one+=frequency[x-2][RoadIDToIndex[crossdata[j][5]]][frejudge];
                        double bandwidth2=roaddata[RoadIDToIndex[crossdata[j][5]]][4];
                        double bandwidth=roaddata[RoadIDToIndex[crossdata[j][5]]][4]*roaddata[RoadIDToIndex[crossdata[j][5]]][2];
                        int speedlimit=min(roaddata[RoadIDToIndex[crossdata[j][5]]][3],cardata[outline][4]);
                        //                    one=one/(bandwidth*speedlimit);
//                        cout<<"one "<<one/(bandwidth2*speedlimit)<<endl;

                        one=(one/bandwidth)*c3+bandwidth/(bandwidth2*speedlimit)*c2;//weight公式
                    }
                    if(crossdata[j][2]!=1&&crossdata[j][2]!=y)
                    {
                        deliver++;
                        two+=frequency[x-1][RoadIDToIndex[crossdata[j][2]]][frejudge];
                        two+=frequency[x-2][RoadIDToIndex[crossdata[j][2]]][frejudge];
                        double bandwidth2=roaddata[RoadIDToIndex[crossdata[j][2]]][4];
                        double bandwidth=roaddata[RoadIDToIndex[crossdata[j][2]]][4]*roaddata[RoadIDToIndex[crossdata[j][2]]][2];
                        int speedlimit=min(roaddata[RoadIDToIndex[crossdata[j][2]]][3],cardata[outline][4]);
                        //                    two=two/(bandwidth*speedlimit);
//                        cout<<"two1 "<<two<<endl;
//                        cout<<"two2 "<<   two/(bandwidth2*speedlimit*2)<<endl;
                        two=(two/bandwidth)*c3+two/(bandwidth2*speedlimit);//weight公式
                    }
                    if(crossdata[j][3]!=1&&crossdata[j][3]!=y)
                    {
                        deliver++;
                        three+=frequency[x-1][RoadIDToIndex[crossdata[j][3]]][frejudge];
                        three+=frequency[x-2][RoadIDToIndex[crossdata[j][3]]][frejudge];
                        //                    int bandwidth=roaddata[RoadIDToIndex[crossdata[j][3]]][4];
                        double bandwidth2=roaddata[RoadIDToIndex[crossdata[j][3]]][4];
                        double bandwidth=roaddata[RoadIDToIndex[crossdata[j][3]]][4]*roaddata[RoadIDToIndex[crossdata[j][3]]][2];
                        int speedlimit=min(roaddata[RoadIDToIndex[crossdata[j][3]]][3],cardata[outline][4]);
                        //                    three/=(bandwidth*speedlimit);
//                        cout<<"three "<<three/(bandwidth2*speedlimit)<<endl;
                        three=(three/bandwidth)*c3+bandwidth/(bandwidth2*speedlimit)*c2;//weight公式

                    }
                    if(crossdata[j][4]!=1&&crossdata[j][4]!=y)
                    {
                        deliver++;
                        four+=frequency[x-1][RoadIDToIndex[crossdata[j][4]]][frejudge];
                        four+=frequency[x-2][RoadIDToIndex[crossdata[j][4]]][frejudge];
                        double bandwidth2=roaddata[RoadIDToIndex[crossdata[j][4]]][4];
                        double bandwidth=roaddata[RoadIDToIndex[crossdata[j][4]]][4]*roaddata[RoadIDToIndex[crossdata[j][4]]][2];
                        int speedlimit=min(roaddata[RoadIDToIndex[crossdata[j][4]]][3],cardata[outline][4]);
//                        cout<<"four "<<pow(four/(bandwidth2*speedlimit),2)<<endl;

                        four=(four/bandwidth)*c3+four/(bandwidth2*speedlimit);//weight公式
                    }
                    //                    four/=deliver;
                    int average=0;
                    average+=(one+two+three+four)/deliver;
                    dist[outline][point]+=average;
                }
                //补充end
            }
            point=0;//下标
            if(road2!=1)
            {
                int truejudge=0;
                if (v==CorssIDToIndex[roaddata[RoadIDToIndex[road2]][5]])
                    frejudge=1;
                else if(v==CorssIDToIndex[roaddata[RoadIDToIndex[road2]][6]])
                    frejudge=2;
                if( CorssIDToIndex[roaddata[RoadIDToIndex[road2]][5]]!=v&&roaddata[RoadIDToIndex[road2]][7]==1)
                {
                    truejudge=1;
                    point=CorssIDToIndex[roaddata[RoadIDToIndex[road2]][5]];
                }
                else if(CorssIDToIndex[roaddata[RoadIDToIndex[road2]][6]]!=v)
                {
                    truejudge=1;
                    point=CorssIDToIndex[roaddata[RoadIDToIndex[road2]][6]];
                }
                int two1=0;
                int x=int((cardata[outline][5]+3)/c4);
                two1+=frequency[x-1][RoadIDToIndex[road2]][frejudge];
                two1+=frequency[x-2][RoadIDToIndex[road2]][frejudge];
                double bandwidth2=roaddata[RoadIDToIndex[road2]][4];
                double bandwidth=roaddata[RoadIDToIndex[road2]][4]*roaddata[RoadIDToIndex[road2]][2];
                int speedlimit=min(roaddata[RoadIDToIndex[road2]][3],cardata[outline][4]);
                //                two1/=(bandwidth*speedlimit);
                two1=(two1/bandwidth)*c3+bandwidth/(bandwidth2*speedlimit)*c2;//weight公式
                if( truejudge==1)
                    dist[outline][point]=two1;

                //补充start
                if(point!=0)
                {
                    int one=0,two=0,three=0,four=0;
                    int deliver=0;
                    int j=point;
                    int y=road2;
                    if(crossdata[j][5]!=1&&crossdata[j][5]!=y)
                    {
                        deliver++;
                        one+=frequency[x-1][RoadIDToIndex[crossdata[j][5]]][frejudge];
                        one+=frequency[x-2][RoadIDToIndex[crossdata[j][5]]][frejudge];
                        double bandwidth2=roaddata[RoadIDToIndex[crossdata[j][5]]][4];
                        double bandwidth=roaddata[RoadIDToIndex[crossdata[j][5]]][4]*roaddata[RoadIDToIndex[crossdata[j][5]]][2];
                        int speedlimit=min(roaddata[RoadIDToIndex[crossdata[j][5]]][3],cardata[outline][4]);
                        //                    one/=(bandwidth*speedlimit);
                        one=(one/bandwidth)*c3+bandwidth/(bandwidth2*speedlimit)*c2;//weight公式
                    }
                    if(crossdata[j][2]!=1&&crossdata[j][2]!=y)
                    {
                        deliver++;
                        two+=frequency[x-1][RoadIDToIndex[crossdata[j][2]]][frejudge];
                        two+=frequency[x-2][RoadIDToIndex[crossdata[j][2]]][frejudge];
                        double bandwidth2=roaddata[RoadIDToIndex[crossdata[j][2]]][4];
                        double bandwidth=roaddata[RoadIDToIndex[crossdata[j][2]]][4]*roaddata[RoadIDToIndex[crossdata[j][2]]][2];
                        int speedlimit=min(roaddata[RoadIDToIndex[crossdata[j][2]]][3],cardata[outline][4]);
                        //                    two=two/(bandwidth*speedlimit);
                        two=(two/bandwidth)*c3+bandwidth/(bandwidth2*speedlimit)*c2;//weight公式
                    }
                    if(crossdata[j][3]!=1&&crossdata[j][3]!=y)
                    {
                        deliver++;
                        three+=frequency[x-1][RoadIDToIndex[crossdata[j][3]]][frejudge];
                        three+=frequency[x-2][RoadIDToIndex[crossdata[j][3]]][frejudge];
                        double bandwidth2=roaddata[RoadIDToIndex[crossdata[j][3]]][4];
                        double bandwidth=roaddata[RoadIDToIndex[crossdata[j][3]]][4]*roaddata[RoadIDToIndex[crossdata[j][3]]][2];
                        int speedlimit=min(roaddata[RoadIDToIndex[crossdata[j][3]]][3],cardata[outline][4]);
                        //                    three/=(bandwidth*speedlimit);
                        three=(three/bandwidth)*c3+bandwidth/(bandwidth2*speedlimit)*c2;//weight公式
                    }
                    if(crossdata[j][4]!=1&&crossdata[j][4]!=y)
                    {
                        deliver++;
                        four+=frequency[x-1][RoadIDToIndex[crossdata[j][4]]][frejudge];
                        four+=frequency[x-2][RoadIDToIndex[crossdata[j][4]]][frejudge];
                        double bandwidth2=roaddata[RoadIDToIndex[crossdata[j][4]]][4];
                        double bandwidth=roaddata[RoadIDToIndex[crossdata[j][4]]][4]*roaddata[RoadIDToIndex[crossdata[j][4]]][2];
                        int speedlimit=min(roaddata[RoadIDToIndex[crossdata[j][4]]][3],cardata[outline][4]);
                        //                    four/=(bandwidth*speedlimit);
                        four=(four/bandwidth)*c3+bandwidth/(bandwidth2*speedlimit)*c2;//weight公式
                    }
                    //                    four/=deliver;
                    int average=0;
                    average+=(one+two+three+four)/deliver;
                    dist[outline][point]+=average;
                }
                //补充end

            }
            point=0;
            if(road3!=1)
            {
                int truejudge=0;
                if (v==CorssIDToIndex[roaddata[RoadIDToIndex[road3]][5]])
                    frejudge=1;
                else if(v==CorssIDToIndex[roaddata[RoadIDToIndex[road3]][6]])
                    frejudge=2;
                if( CorssIDToIndex[roaddata[RoadIDToIndex[road3]][5]]!=v&&roaddata[RoadIDToIndex[road3]][7]==1)
                {
                    truejudge=1;
                    point=CorssIDToIndex[roaddata[RoadIDToIndex[road3]][5]];
                }
                else if(CorssIDToIndex[roaddata[RoadIDToIndex[road3]][6]]!=v)
                {
                    truejudge=1;
                    point=CorssIDToIndex[roaddata[RoadIDToIndex[road3]][6]];
                }
                int three1=0;
                int x=int((cardata[outline][5]+3)/c4);
                three1+=frequency[x-1][RoadIDToIndex[road3]][frejudge];
                three1+=frequency[x-2][RoadIDToIndex[road3]][frejudge];
                double bandwidth2=roaddata[RoadIDToIndex[road3]][4];
                double bandwidth=roaddata[RoadIDToIndex[road3]][4]*roaddata[RoadIDToIndex[road3]][2];
                int speedlimit=min(roaddata[RoadIDToIndex[road3]][3],cardata[outline][4]);
                //                three1/=(bandwidth*speedlimit);
                three1=(three1/bandwidth)*c3+bandwidth/(bandwidth2*speedlimit)*c2;//weight公式
                if( truejudge==1)
                    dist[outline][point]=three1;

                //补充start
                if(point!=0)
                {
                    int one=0,two=0,three=0,four=0;
                    int deliver=0;
                    int j=point;
                    int y=road3;
                    if(crossdata[j][5]!=1&&crossdata[j][5]!=y)
                    {
                        deliver++;
                        one+=frequency[x-1][RoadIDToIndex[crossdata[j][5]]][frejudge];
                        one+=frequency[x-2][RoadIDToIndex[crossdata[j][5]]][frejudge];
                        double bandwidth2=roaddata[RoadIDToIndex[crossdata[j][5]]][4];
                        double bandwidth=roaddata[RoadIDToIndex[crossdata[j][5]]][4]*roaddata[RoadIDToIndex[crossdata[j][5]]][2];
                        int speedlimit=min(roaddata[RoadIDToIndex[crossdata[j][5]]][3],cardata[outline][4]);
                        //                    one/=(bandwidth*speedlimit);
                        one=(one/bandwidth)*c3+bandwidth/(bandwidth2*speedlimit)*c2;//weight公式
                    }
                    if(crossdata[j][2]!=1&&crossdata[j][2]!=y)
                    {
                        deliver++;
                        two+=frequency[x-1][RoadIDToIndex[crossdata[j][2]]][frejudge];
                        two+=frequency[x-2][RoadIDToIndex[crossdata[j][2]]][frejudge];
                        double bandwidth2=roaddata[RoadIDToIndex[crossdata[j][2]]][4];
                        double bandwidth=roaddata[RoadIDToIndex[crossdata[j][2]]][4]*roaddata[RoadIDToIndex[crossdata[j][2]]][2];
                        int speedlimit=min(roaddata[RoadIDToIndex[crossdata[j][2]]][3],cardata[outline][4]);
                        //                    two=two/(bandwidth*speedlimit);
                        two=(two/bandwidth)*c3+bandwidth/(bandwidth2*speedlimit)*c2;//weight公式
                    }
                    if(crossdata[j][3]!=1&&crossdata[j][3]!=y)
                    {
                        deliver++;
                        three+=frequency[x-1][RoadIDToIndex[crossdata[j][3]]][frejudge];
                        three+=frequency[x-2][RoadIDToIndex[crossdata[j][3]]][frejudge];
                        double bandwidth2=roaddata[RoadIDToIndex[crossdata[j][3]]][4];
                        double bandwidth=roaddata[RoadIDToIndex[crossdata[j][3]]][4]*roaddata[RoadIDToIndex[crossdata[j][3]]][2];
                        int speedlimit=min(roaddata[RoadIDToIndex[crossdata[j][3]]][3],cardata[outline][4]);
                        //                    three/=(bandwidth*speedlimit);
                        three=(three/bandwidth)*c3+bandwidth/(bandwidth2*speedlimit)*c2;//weight公式
                    }
                    if(crossdata[j][4]!=1&&crossdata[j][4]!=y)
                    {
                        deliver++;
                        four+=frequency[x-1][RoadIDToIndex[crossdata[j][4]]][frejudge];
                        four+=frequency[x-2][RoadIDToIndex[crossdata[j][4]]][frejudge];
                        double bandwidth2=roaddata[RoadIDToIndex[crossdata[j][4]]][4];
                        double bandwidth=roaddata[RoadIDToIndex[crossdata[j][4]]][4]*roaddata[RoadIDToIndex[crossdata[j][4]]][2];
                        int speedlimit=min(roaddata[RoadIDToIndex[crossdata[j][4]]][3],cardata[outline][4]);
                        //                    four/=(bandwidth*speedlimit);
                        four=(four/bandwidth)*c3+bandwidth/(bandwidth2*speedlimit)*c2;//weight公式
                    }
                    //                    four/=deliver;
                    int average=0;
                    average+=(one+two+three+four)/deliver;
//                    cout<<"average "<<average<<endl;
                    dist[outline][point]+=average;
                }
                //补充end

            }
            point=0;
            //            cout<<road4<<endl;
            if(road4!=1)
            {
                int truejudge=0;
                if (v==CorssIDToIndex[roaddata[RoadIDToIndex[road4]][5]])
                    frejudge=1;
                else if(v==CorssIDToIndex[roaddata[RoadIDToIndex[road4]][6]])
                    frejudge=2;
                if(CorssIDToIndex[roaddata[RoadIDToIndex[road4]][5]]!=v&&roaddata[RoadIDToIndex[road4]][7]==1)
                {
                    truejudge=1;
                    point=CorssIDToIndex[roaddata[RoadIDToIndex[road4]][5]];
                }
                else if(CorssIDToIndex[roaddata[RoadIDToIndex[road4]][6]]!=v)
                {
                    truejudge=1;
                    point=CorssIDToIndex[roaddata[RoadIDToIndex[road4]][6]];
                }
                int four1=0;
                int x=int((cardata[outline][5]+3)/c4);
                four1+=frequency[x-1][RoadIDToIndex[road4]][frejudge];
                four1+=frequency[x-2][RoadIDToIndex[road4]][frejudge];
                double bandwidth2=roaddata[RoadIDToIndex[road4]][4];
                double bandwidth=roaddata[RoadIDToIndex[road4]][4]*roaddata[RoadIDToIndex[road4]][2];
                int speedlimit=min(roaddata[RoadIDToIndex[road4]][3],cardata[outline][4]);
                //                four1/=(bandwidth*speedlimit);
                four1=(four1/bandwidth)*c3+bandwidth/(bandwidth2*speedlimit)*c2;//weight公式
                if( truejudge==1)
                    dist[outline][point]=four1;

                //补充start
                if(point!=0)
                {
                    int one=0,two=0,three=0,four=0;
                    int deliver=0;
                    int j=point;
                    int y=road4;
                    if(crossdata[j][5]!=1&&crossdata[j][5]!=y)
                    {
                        deliver++;
                        one+=frequency[x-1][RoadIDToIndex[crossdata[j][5]]][frejudge];
                        one+=frequency[x-2][RoadIDToIndex[crossdata[j][5]]][frejudge];
                        double bandwidth2=roaddata[RoadIDToIndex[crossdata[j][5]]][4];
                        double bandwidth=roaddata[RoadIDToIndex[crossdata[j][5]]][4]*roaddata[RoadIDToIndex[crossdata[j][5]]][2];
                        int speedlimit=min(roaddata[RoadIDToIndex[crossdata[j][5]]][3],cardata[outline][4]);
                        //                    one/=(bandwidth*speedlimit);
                        one=(one/bandwidth)*c3+bandwidth/(bandwidth2*speedlimit)*c2;//weight公式
                    }
                    if(crossdata[j][2]!=1&&crossdata[j][2]!=y)
                    {
                        deliver++;
                        two+=frequency[x-1][RoadIDToIndex[crossdata[j][2]]][frejudge];
                        two+=frequency[x-2][RoadIDToIndex[crossdata[j][2]]][frejudge];
                        double bandwidth2=roaddata[RoadIDToIndex[crossdata[j][2]]][4];
                        double bandwidth=roaddata[RoadIDToIndex[crossdata[j][2]]][4]*roaddata[RoadIDToIndex[crossdata[j][2]]][2];
                        int speedlimit=min(roaddata[RoadIDToIndex[crossdata[j][2]]][3],cardata[outline][4]);
                        //                    two=two/(bandwidth*speedlimit);
                        two=(two/bandwidth)*c3+bandwidth/(bandwidth2*speedlimit)*c2;//weight公式
                    }
                    if(crossdata[j][3]!=1&&crossdata[j][3]!=y)
                    {
                        deliver++;
                        three+=frequency[x-1][RoadIDToIndex[crossdata[j][3]]][frejudge];
                        three+=frequency[x-2][RoadIDToIndex[crossdata[j][3]]][frejudge];
                        double bandwidth2=roaddata[RoadIDToIndex[crossdata[j][3]]][4];
                        double bandwidth=roaddata[RoadIDToIndex[crossdata[j][3]]][4]*roaddata[RoadIDToIndex[crossdata[j][3]]][2];
                        int speedlimit=min(roaddata[RoadIDToIndex[crossdata[j][3]]][3],cardata[outline][4]);
                        //                    three/=(bandwidth*speedlimit);
                        three=(three/bandwidth)*c3+bandwidth/(bandwidth2*speedlimit)*c2;//weight公式
                    }
                    if(crossdata[j][4]!=1&&crossdata[j][4]!=y)
                    {
                        deliver++;
                        four+=frequency[x-1][RoadIDToIndex[crossdata[j][4]]][frejudge];
                        four+=frequency[x-2][RoadIDToIndex[crossdata[j][4]]][frejudge];
                        double bandwidth2=roaddata[RoadIDToIndex[crossdata[j][4]]][4];
                        double bandwidth=roaddata[RoadIDToIndex[crossdata[j][4]]][4]*roaddata[RoadIDToIndex[crossdata[j][4]]][2];
                        int speedlimit=min(roaddata[RoadIDToIndex[crossdata[j][4]]][3],cardata[outline][4]);
                        //                    four/=(bandwidth*speedlimit);
                        four=(four/bandwidth)*c3+bandwidth/(bandwidth2*speedlimit)*c2;//weight公式
                    }
                    //                    four/=deliver;
                    int average=0;
                    average+=(one+two+three+four)/deliver;
                    dist[outline][point]+=average;
                    //                    cout<<dist[outline][point]<<endl;
                }
                //补充end
            }
        }
        //起点权值刷新end
        s[i] = 0;     // 初始都未用过该点
        if(dist[outline][i] == maxint)
            prev1[outline][i] = 0;
        else
            prev1[outline][i] = v;
    }
    dist[outline][v] = 0;
    s[v] = 1;
    /////////////////////////////////////图上起点到其他所有节点之间的权值初始化end

    ////////////////////////////////////遍历所有点start
    // 依次将未放入S集合的结点中，取dist[]最小值的结点，放入结合S中
    // 一旦S包含了所有V中顶点，dist就记录了从源点到所有其他顶点之间的最短路径长度
    for(int i=2; i<=n; ++i)
    {
        int tmp = maxint;
        int u = v;
        // 找出当前未使用的点j的dist[j]最小值
        for(int j=1; j<=n; ++j)
            if((!s[j]) && dist[outline][j]<tmp)
            {
                u = j;              // u保存当前邻接点中距离最小的点的号码
                tmp = dist[outline][j];
            }
        s[u] = 1;    // 表示u点已存入S集合中

        //铺路
        int prepoint=prev1[outline][u];
//        cout<<"prepoint "<<prepoint<<" u "<<u<<endl;
        int chosenroad=PointToRoad(prepoint, u);
//        cout<<"chosenroad "<<chosenroad<<endl;
        if (prepoint==CorssIDToIndex[roaddata[RoadIDToIndex[chosenroad]][5]])
            frejudge=1;
        else if(prepoint==CorssIDToIndex[roaddata[RoadIDToIndex[chosenroad]][6]])
            frejudge=2;
        frequency[int((cardata[outline][5]+3)/c4)][RoadIDToIndex[chosenroad]][frejudge]+=1;
//        cout<<int((cardata[outline][5]+3)/c4)<<","<<chosenroad<<","<<frejudge<<","<<frequency[int((cardata[outline][5]+3)/c4)][RoadIDToIndex[chosenroad]][frejudge]<<endl;
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
        for(int j=1; j<=n; ++j)
            if((!s[j]) && c[u][j]<maxint)
            {
                int newdist;
                //修改边权限制条件
                //                cout<<cardata[outline][5]<<endl;
                if(int((cardata[outline][5]+3)/c4)>2)
                {
                    int x=int((cardata[outline][5]+3)/c4);
//                    cout<<"frequency 5033 1 "<<frequency[x-1][5033-5000+1][1]<<endl;
//                    cout<<"frequency 5033 2 "<<frequency[x-1][5033-5000+1][2]<<endl;
//                    cout<<"frequency 5032 1 "<<frequency[x-1][5032-5000+1][1]<<endl;
//                    cout<<"frequency 5032 2 "<<frequency[x-1][5032-5000+1][2]<<endl;
//                    cout<<"frequency 5026 1 "<<frequency[x-1][5026-5000+1][1]<<endl;
//                    cout<<"frequency 5026 2 "<<frequency[x-1][5026-5000+1][2]<<endl;



                    int y=PointToRoad(u, j);
                    if (u==CorssIDToIndex[roaddata[RoadIDToIndex[y]][5]])
                        frejudge=1;
                    else if(u==CorssIDToIndex[roaddata[RoadIDToIndex[y]][6]])
                        frejudge=2;
                    int pretwo=(frequency[x-1][RoadIDToIndex[y]][frejudge]+frequency[x-2][RoadIDToIndex[y]][frejudge]);
                    //new调整start
                    int one=0,two=0,three=0,four=0;
                    int deliver=0;
                    if(crossdata[j][5]!=1&&crossdata[j][5]!=y)
                    {
                        deliver++;
                        one+=frequency[x-1][RoadIDToIndex[crossdata[j][5]]][frejudge];
                        one+=frequency[x-2][RoadIDToIndex[crossdata[j][5]]][frejudge];
                        double bandwidth2=roaddata[RoadIDToIndex[crossdata[j][5]]][4];   //单条道路的宽度
                        double bandwidth=roaddata[RoadIDToIndex[crossdata[j][5]]][4]*roaddata[RoadIDToIndex[crossdata[j][5]]][2];  //长度*宽度
                        int speedlimit=min(roaddata[RoadIDToIndex[crossdata[j][5]]][3],cardata[outline][4]);               //速度
                        //                        one/=(bandwidth*speedlimit);
                        one=(one/bandwidth)*c3+bandwidth/(bandwidth2*speedlimit)*c2;//weight公式
                    }
                    if(crossdata[j][2]!=1&&crossdata[j][2]!=y)
                    {
                        deliver++;
                        two+=frequency[x-1][RoadIDToIndex[crossdata[j][2]]][frejudge];
                        two+=frequency[x-2][RoadIDToIndex[crossdata[j][2]]][frejudge];
                        double bandwidth2=roaddata[RoadIDToIndex[crossdata[j][2]]][4];
                        double bandwidth=roaddata[RoadIDToIndex[crossdata[j][2]]][4]*roaddata[RoadIDToIndex[crossdata[j][2]]][2];
                        int speedlimit=min(roaddata[RoadIDToIndex[crossdata[j][2]]][3],cardata[outline][4]);
                        //                        two=two/(bandwidth*speedlimit);
                        two=(two/bandwidth)*c3+bandwidth/(bandwidth2*speedlimit)*c2;//weight公式
                    }
                    if(crossdata[j][3]!=1&&crossdata[j][3]!=y)
                    {
                        deliver++;
                        three+=frequency[x-1][RoadIDToIndex[crossdata[j][3]]][frejudge];
                        three+=frequency[x-2][RoadIDToIndex[crossdata[j][3]]][frejudge];
                        double bandwidth2=roaddata[RoadIDToIndex[crossdata[j][3]]][4];
                        double bandwidth=roaddata[RoadIDToIndex[crossdata[j][3]]][4]*roaddata[RoadIDToIndex[crossdata[j][3]]][2];
                        int speedlimit=min(roaddata[RoadIDToIndex[crossdata[j][3]]][3],cardata[outline][4]);
                        //                        three/=(bandwidth*speedlimit);
                        three=(three/bandwidth)*c3+bandwidth/(bandwidth2*speedlimit)*c2;//weight公式
                    }
                    if(crossdata[j][4]!=1&&crossdata[j][4]!=y)
                    {
                        deliver++;
                        four+=frequency[x-1][RoadIDToIndex[crossdata[j][4]]][frejudge];
                        four+=frequency[x-2][RoadIDToIndex[crossdata[j][4]]][frejudge];
                        double bandwidth2=roaddata[RoadIDToIndex[crossdata[j][4]]][4];
                        double bandwidth=roaddata[RoadIDToIndex[crossdata[j][4]]][4]*roaddata[RoadIDToIndex[crossdata[j][4]]][2];
                        int speedlimit=min(roaddata[RoadIDToIndex[crossdata[j][4]]][3],cardata[outline][4]);
                        //                        four/=(bandwidth*speedlimit);

//                        cout<<"1:  "<<pow((four/bandwidth)*c3,2)<<" 2  "<<bandwidth/(bandwidth2*speedlimit)<<endl;

                        four=(four/bandwidth)*c3+bandwidth/(bandwidth2*speedlimit)*c2;//weight公式
                    }
                    //                    four/=deliver;
                    int average=0;
                    average+=(one+two+three+four)/deliver;
                    //                    cout<<average<<endl;
                    //                    pretwo+=four;
                    //new调整end

                    double bandwidth=(roaddata[RoadIDToIndex[y]][2]*roaddata[RoadIDToIndex[y]][4]);
                    double bandwidth2=roaddata[RoadIDToIndex[y]][4];
                    int speedlimit=min(roaddata[RoadIDToIndex[y]][3],cardata[outline][4]);
                    //                    newdist=dist[outline][u]+(pretwo/(bandwidth*speedlimit))+average;
                    newdist=dist[outline][u]+(pretwo/(bandwidth))*c3+average+c2*(pretwo/(bandwidth2*speedlimit));//weight公式
//                    cout<<c2*(pretwo/(bandwidth2*speedlimit))<<"  ";
                    //                    cout<<(pretwo/(bandwidth*speedlimit))<<endl;
                    if(maxtest1<(pretwo/(bandwidth2*speedlimit)))
                        maxtest1=(pretwo/(bandwidth2*speedlimit));
                }
                else
                    newdist = dist[outline][u] + c[u][j];
                //
                if(newdist < dist[outline][j])
                {
                    dist[outline][j] = newdist;
                    prev1[outline][j] = u;
                }
            }
    }
    ////////////////////////////////////遍历所有点end
    
}

//起点 终点//返回道路ID
int PointToRoad(int start,int end)
{
    int result=-1;
    int i;
    int yes=0;
    for(i=1;i<=roadline;i++)
    {
        if(start==CorssIDToIndex[roaddata[i][5]]&&end==CorssIDToIndex[roaddata[i][6]])
        {
            yes=1;
            break;
        }
        if(start==CorssIDToIndex[roaddata[i][6]]&&end==CorssIDToIndex[roaddata[i][5]]&&roaddata[i][7]==1)
        {
            yes=1;
            break;
        }
    }
    if(yes==1)
    {
        result=roaddata[i][1];
    }
    return result;
}

//v 起点 u 终点 carnum第几辆车
void searchPath(int v, int u,int carnum)
{
    int que[maxnum];
    int tot = 1;
    que[tot] = u;
    tot++;
    int tmp = prev1[outline][u];
    while(tmp != v)
    {
        que[tot] = tmp;
        tot++;
        //        cout<<tot<<endl;
        tmp = prev1[outline][tmp];
    }
    que[tot] = v;
    int pa=1;
    for(int i=tot; i>1; --i)
    {
        int result=PointToRoad(que[i], que[i-1]);
        CarRoadID[carnum][pa]=result;
        pa++;
    }
    //        if(i != 1)
    //            cout << que[i] << " -> ";
    //        else
    //            cout << que[i] << endl;
}

void carin(char* nn)
{
    string str;
    ifstream carfile;
    carfile.open(nn);
    getline(carfile,str);
    if(carfile.is_open())
    {
        carline=1;//数据从第1行开始存放
        while(!carfile.eof())
        {
            getline(carfile,str);
            //        cout<<str<<endl;
            //        cout<<str<<endl;
            int i=0;
            int sum=0;
            
            //提取数字start
            int point=1;//数据从第1列开始存放
            while (i < str.length())
            {
                if ('0' <= str.at(i) && str.at(i) <= '9')
                    sum = sum * 10 + (str.at(i) - '0');
                else if(sum>0)
                {
                    cardata[carline][point]=sum;
                    if(point==1)
                    {
                        CarIDToIndex[cardata[carline][point]]=carline;
                    }
                    point++;
                    sum=0;
                    
                }
                i++;
            }
            ++carline;
            //提取数字end
            
        }
        //        cout<<"line"<<line;
        //        for(int i=1;i<=line-1;i++)
        //        {for(int j=1;j<=5;j++)
        //            {
        //                cout<<cardata[i][j]<<" ";
        //            }
        //            cout<<endl;
        //        }
    }
    else
    {
        cout<<"wrong";
    }
    carfile.close();
    carline--;
}


void roadin(char* nn)
{
    string str;
    ifstream roadfile;
    roadfile.open(nn);
    getline(roadfile,str);
    if(roadfile.is_open())
    {
        roadline=1;//数据从第1行开始存放
        while(!roadfile.eof())
        {
            getline(roadfile,str);
            //            cout<<str<<endl;
            int i=0;
            int sum=0;
            
            //提取数字start
            int point=1;//数据从第1列开始存放
            while (i < str.length())
            {
                if ('0' <= str.at(i) && str.at(i) <= '9')
                    sum = sum * 10 + (str.at(i) - '0');
                else if(sum>0)
                {
                    roaddata[roadline][point]=sum;
                    if(point==1)
                    {
                        RoadIDToIndex[roaddata[roadline][point]]=roadline;
                    }
                    point++;
                    sum=0;
                }
                i++;
            }
            ++roadline;
            //提取数字end
            
        }
        //                cout<<"line"<<roadline;
        //                for(int i=1;i<=roadline-1;i++)
        //                {for(int j=1;j<=7;j++)
        //                    {
        //                        cout<<roaddata[i][j]<<" ";
        //                    }
        //                    cout<<endl;
        //                }
    }
    else
    {
        cout<<"wrong";
    }
    roadfile.close();
    roadline--;
}


void crossin(char* nn)
{
    string str;
    ifstream crossfile;
    crossfile.open(nn);
    getline(crossfile,str);
    if(crossfile.is_open())
    {
        crossline=1;//数据从第1行开始存放
        while(!crossfile.eof())
        {
            getline(crossfile,str);
            //            cout<<str<<endl;
            int i=0;
            int sum=0;
            
            //提取数字start
            int point=1;//数据从第1列开始存放
            while (i < str.length())
            {
                if ('0' <= str.at(i) && str.at(i) <= '9')
                    sum = sum * 10 + (str.at(i) - '0');
                else if(sum>0)
                {
                    crossdata[crossline][point]=sum;
                    if(point==1)
                    {
                        CorssIDToIndex[crossdata[crossline][point]]=crossline;
                    }
                    point++;
                    sum=0;
                }
                i++;
            }
            ++crossline;
            //提取数字end
            
        }
        //                cout<<"line"<<crossline;
        //                for(int i=1;i<=crossline-1;i++)
        //                {for(int j=1;j<=5;j++)
        //                    {
        //                        cout<<crossdata[i][j]<<" ";
        //                    }
        //                    cout<<endl;
        //                }
    }
    else
    {
        cout<<"wrong";
    }
    crossfile.close();
    crossline--;
}


////////////////////速度排序
void carsort()
{
    for (int i=1;i<=carline;i++)
        for(int j=i+1;j<=carline;j++)
        {
            if(cardata[i][4]<cardata[j][4])
            {
//                int k= CarIDToIndex[cardata[i][1]];
                
                swap(cardata[i],cardata[j]);
//                int k;
//                k=CarIDToIndex[cardata[i][1]];
                CarIDToIndex[cardata[i][1]]=i;
                CarIDToIndex[cardata[j][1]]=j;
                
            }
        }
}


/////////////////////////////时间片排序
void carsort1()
{
    for (int i=1;i<=carline;i++)
        for(int j=i+1;j<=carline;j++)
        {
            if(cardata[i][5]>cardata[j][5])
            {
//                int k= CarIDToIndex[cardata[i][1]];
                swap(cardata[i],cardata[j]);
//                int k;
//                k=CarIDToIndex[cardata[i][1]];
                CarIDToIndex[cardata[i][1]]=i;
                CarIDToIndex[cardata[j][1]]=j;
            }
        }
}

void ChangeStartTime()
{
    int i=1;
    int j=1;
    int point=1;
    //    while(point<=carline)//版本1
    //    {
    //        if(i>70)//调参啊调参
    //        {
    //            i=1;
    //            j+=4;
    //        }
    //        cardata[point][5]=max(j,cardata[point][5]);
    //        point++;
    //        i++;
    //    }

        for(int i=1;i<=c5;i++)//5
        {
            int j=1;
            while(j<=c6)//50
            {
                cardata[point][5]=max(j,cardata[point][5]);
                point++;
                j++;
            }
        }
//        point =c5*c6;
        j=1;
    while(point<=carline)
    {

        if(j>pici)//调参啊调参
        {
            j=1;
        }
        
        cardata[point][5]=max(j,cardata[point][5]);
        //        cout<<cardata[point][5]<<endl;
        j++;
        point++;
    }
    carsort1();
    
}

int main(int a,char* argv[])
//int main()
{//car road cross answer
    int (*c)[2500]=new int[2500][2500];
    char* carargv=argv[1];
    char* roadargv=argv[2];
    char* crossargv=argv[3];
//        string carargv="/Users/apple/Desktop/软件精英挑战赛/2019软挑-初赛-SDK/SDK/SDK_C++/config/car.txt";
//        string roadargv="/Users/apple/Desktop/软件精英挑战赛/2019软挑-初赛-SDK/SDK/SDK_C++/config/road.txt";
//        string crossargv="/Users/apple/Desktop/软件精英挑战赛/2019软挑-初赛-SDK/SDK/SDK_C++/config/cross.txt";
    crossin(crossargv);
    roadin(roadargv);
    carin(carargv);
    carsort();
//    for(int i=1;i<=carline;i++)
//        cout<<CarIDToIndex[cardata[i][1]]<<endl;
    ChangeStartTime();
//    for(int i=1;i<=carline;i++)
//                cout<<CarIDToIndex[cardata[i][1]]<<endl;
    // 各数组都从下标1开始
    
    //    cout<<crossline<<endl;
    // 初始化c[][]为maxintt//图的初始化
    for(outline=1;outline<=carline;outline++)
    {
        for(int i=1; i<=crossline; ++i)
            for(int j=1; j<=crossline; ++j)
                c[i][j] = maxint;//store index
        
        //根据道路建图
        for(int i=1; i<=roadline; ++i)
        {
            if(roaddata[i][7] ==1)       // 双向
            {
                c[CorssIDToIndex[roaddata[i][5]]][CorssIDToIndex[roaddata[i][6]]] = roaddata[i][2];
                c[CorssIDToIndex[roaddata[i][6]]][CorssIDToIndex[roaddata[i][5]]] = roaddata[i][2];
            }
            else// 单向
            {
                c[CorssIDToIndex[roaddata[i][5]]][CorssIDToIndex[roaddata[i][6]]] = roaddata[i][2];
            }
        }
        
        for(int i=1; i<=crossline; ++i)
            dist[outline][i] = maxint;//store index
        //    for(int i=1; i<=crossline; ++i)
        //    {
        //        for(int j=1; j<=crossline; ++j)
        //            printf("%8d", c[i][j]);
        //        printf("\n");
        //    }
        int start=CorssIDToIndex[cardata[outline][2]];
        int end=CorssIDToIndex[cardata[outline][3]];
        Dijkstra(crossline, start,outline,c);

        searchPath(start,end,outline);

    }

//    cout<<frequency<<endl;
    ofstream outfile;
    char* answer=argv[4];
//        string answer="/Users/apple/Desktop/软件精英挑战赛/2019软挑-初赛-SDK/SDK/SDK_C++/config/answer.txt";
    outfile.open(answer);
    for(int i=1;i<=carline;i++)
    {
        outfile<<"("<<cardata[i][1]<<","<<cardata[i][5]<<",";
        for(int j=1;;j++)
        {
            if(CarRoadID[i][j]==0)
                break;
            else
            {
                outfile<<CarRoadID[i][j];
            }
            if(CarRoadID[i][j+1]!=0)
                outfile<<",";
        }
        outfile<<")"<<endl;
    }
    // 最短路径长度
    //    cout << "源点到最后一个顶点的最短路径长度: " << dist[crossline] << endl;//距离升序
    //    // 路径
    //    cout << "源点到最后一个顶点的路径为: ";
    //    searchPath(prev, 2, crossline);
    cout<<"maxtestis:   "<<maxtest1;
}

