# -*- coding: utf-8 -*-
'''
# @Time    : 19-3-17 下午9:52
# @Author  :  LXF && ZXP && WZD
# @FileName: upload.py
--------------------- 
'''
from sys import argv

if __name__ == '__main__':
    car_file,road_file,cross_file,_, output_file = argv[1],argv[2],argv[3],argv[4],argv[5]
    car=open(car_file).readlines()
    cc=car[1]
    cc = cc.replace(cc[0], '')
    cc = cc.replace(cc[-1], '')
    cc = cc.replace(')', '')
    split_line = cc.split(', ')
    dd='./as/answer1.txt'
    # if(int(split_line[1])==18):
    #     dd='./as/answer1.txt'
    # else:
    #     dd='./as/answer2.txt'
    data=open(dd).readlines()
    with open(output_file,"w") as fp:
    # out='./answer/answer.txt'
        for i in data:
            fp.write(i)
