# CodeCraft2019
### 华为2019年软挑落下帷幕了,最终进入了粤港澳赛区的复赛32强,特意来开源一波我们的源代码,这次真的是把地杰斯特拉给彻底玩坏了(逃)
分享下我们的思路.

### 初赛
- 其实整个思路很简单，我们没有往着写判题器的角度想，全部都是最短路径，很自然想到的DJ这种贪心算法.
和隔壁班同学讨论之后，我们的思路是
- ①每一个节点求最短路径时除了考虑节点的当前路径，还要考虑下一个节点连接的路径的情况，也就是说，我们会预测这条道路的另一个节点的道路分布情况，求取均值，当成是要选择的道路的权值．
- ②在选择的下一个节点邻接道路的权值是，我们考虑了前两个时间片的该条道路经过的车辆数，不过一开始只是简单的对整条道路进行评估，后面我们具体到了车辆所要选择的方向．
贴一段我们的代码分析下
```
int one1=0;//当前道路
int x=int((cardata[outline][5]+3)/c4);//车辆出发批次
one1+=frequency[x-1][RoadIDToIndex[road1]][frejudge];//前一个时间片道路的车辆数
one1+=frequency[x-2][RoadIDToIndex[road1]][frejudge];//前连个时间片道路的车辆数
double bandwidth2=roaddata[RoadIDToIndex[road1]][4]; //道路宽
double bandwidth=roaddata[RoadIDToIndex[road1]][4]*roaddata[RoadIDToIndex[road1]][2];//道路面积
int speedlimit=min(roaddata[RoadIDToIndex[road1]][3],cardata[outline][4]);
one1=(one1/bandwidth)*c3+bandwidth/(bandwidth2*speedlimit)*c2;//weight公式
//weight公式=拥堵数/车道面积*c3　+车道长度/速度*c2,其中c2 ,c3为调参对象，为了突出防止死锁的情况，我们主要考虑公式的前一部份，他起到主要作用．
if(truejudge==1)
    dist[outline][point]=one1;
```
- ③车次出发时间，为了起到并行出发的目的(虽然我们没有判题器)，我们需要确定出发的车次．
--- 队友先是将车辆进行按照速度排序，然后为了避免死锁，有让每一批次里面有速度快的和速度慢的这样相互交叉
--- 接着骚队友相出了想思路，让前几批每次多出发几辆
```
while(point<=carline)
    {
        if(record[j]>shuliang)
        {
            j++;
            continue;
        }
        if(j>pici)//调参啊调参
        {
            j=1;
        }
        
        cardata[point][5]=max(j,cardata[point][5]);
        //        cout<<cardata[point][5]<<endl;
        j++;
        point++;
    }
```
- ④　调参，对批次进行调参，只要批次降下来，说明最后出发的时间越短，这样总调度时间就越短．
- ⑤  结果,初赛图1 2211,8图2 2171

###  复赛
经历了初赛之后，我们很开心,然后开始松懈,几乎是我一个人在弄,有点力不从心
- 修改初赛代码,将数组转为hashmap,但是无奈有点慢,编译和运行
- 写判题器,参考了官网的伪代码,用类重新书写,把分模块写出来了,发现拼接在一起时,错漏百出,最后无奈使用初赛的版本进行改进,最后复赛的结果一般般,可能是太浪了,初赛之后.

### 总结
- 这次比赛让我体会到和一个好的团队进行合作的重要性,我们团队很团结,相互想思路,两个人写代码,一个人负责调参,我们还是很团结的.
- 关于c++,一开始想着使用python的,后来被隔壁班同学劝退,说运行太慢,队友连夜赶出来一份c++,1s跑完,太爽了.整个过程,尤其是后期,对于stl库更加熟悉了,对于指针和地址,引用的使用更加得心应手了.我更加喜欢这门语言,也坚定了深入了解他的决心.

- 教训是,慎重使用auto遍历,这是c++11的新特性,突出的是代码的简洁,编译器会自动判断变量的属性
```
    for(auto p:A)
```
但是注意,他只能遍历,不能修改,不能修改,不能修改.

- 关于熬夜,这次比赛,几乎是半夜3点睡觉,后面更狠,直接通宵了.但是觉得不亦说乎,收获最大是自己的debug能力,我可以不厌其烦的按着clion的F8,一句一句debug.

- 最后其实我想去华为,很想去,尤其是看到进入复赛的那支队伍初赛成绩只是比我们好一点,我酸了.
- 回去修炼了,好好努力

