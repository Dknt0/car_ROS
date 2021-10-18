#!/usr/bin/env python3
# -*- coding: UTF-8 -*-
from numpy import *
from math import *
from ctypes import *  
import cv2 as cv


steercmd = 0

# Read the image
def lane_detection(image):   
    global steercmd
    
    if image is None: raise ValueError("no image given to mark_lanes")
    
    src = image

    

    gray = cv.cvtColor(src, cv.COLOR_BGR2GRAY)

    # resk = cv.resize(gray,None,fx=0.5,fy=0.5,interpolation=cv.INTER_CUBIC)

    # src_2 = resk
    # src_1 = cv.resize(gray,(640,480))
    # src_2 = src_1[240:480,:]

    src_2 = gray

    # cv.imshow("gray", src_2)

    kernel=array([-1,0,1])
    dst = cv.filter2D(src_2, -1, kernel)# 2D卷积，特定卷积核，意义：一个像素点左右两边的像素点相加，自己的数据忽略

    # cv.imshow("filter", dst)

    ret, binary = cv.threshold(dst, 20, 255, cv.THRESH_BINARY)# 二值化，THRESH_BINARY代表二值化方式

    # cv.imshow('lane',binary)
    # a = cv.waitKey()


    lines=cv.HoughLinesP(binary,1,pi/180,10,None,60,10)# 直线检测，直线最小长度６０，直线最小距离１０

    # print(lines)
    # print("hahah")

    gpoints = []

    m_x0 = 0
    m_y0 = 0
    m_x1 = 0
    m_y1 = 0
    m_length = 0
    length = 0


    # 计算最长的线
    if any(lines):
        for line in lines:
            for x0, y0, x1, y1 in line:
                length = (x1 - x0) * (x1 - x0) + (y1 - y0) * (y1 - y0)
                if length > m_length:
                    m_x0 = x0
                    m_y0 = y0
                    m_x1 = x1
                    m_y1 = y1
                    m_length = length
                    # print(m_length)


    # cv.line(binary, (m_x0, m_y0), (m_x1, m_y1), 255, 10)
    cv.line(src, (m_x0, m_y0), (m_x1, m_y1), (0, 0, 255), 3)
    cv.circle(src, (m_x0, m_y0), 6, (255, 0, 0), -1)
    cv.circle(src, (m_x1, m_y1), 6, (0, 255, 0), -1)

    # cv.imshow("frame", src)
    # cv.imshow("lines", binary)
    # cv.waitKey()

    slope = rad2deg(math.atan2((m_y1-m_y0),(m_x1-m_x0))) # 计算角度
    # print(x1)

    if (slope > 20 and slope < 160) or (slope < -20 and slope > -160): 
        points = (m_x0,m_y0,m_x1,m_y1) 
        
        # 
        def clac_edgepoints(points,ymin,ymax):
            x = [points[0],points[2]]
            y = [points[1],points[3]]
            k = polyfit(y,x,1) # 多项式系数
            func = poly1d(k) # 多项式方程
            xmin = int(func(ymin))# 直线对应的最低点 
            xmax= int(func(ymax))# 直线对应的最高点
            return[(xmin,ymin),(xmax,ymax)]                 

        if any(points):
            linetop = clac_edgepoints(points,0,240)        
            topx1 = linetop[0][0]
            topy1 = linetop[0][1]
            topx2 = linetop[1][0]
            topy2 = linetop[1][1]
            if topx2 < 700 and topx2 > -100:
                gpoints.append((topx1,topy1,topx2,topy2))
                p0 = (topx1,topy1+240)
                p1 = (topx2,topy2+240) #　整体上移２４０像素                  
                cv.line(src, p0, p1, (0, 255, 255), 4)  # 画直线

                cv.imshow("frame", src)
                cv.imshow("lines", binary)

                # 保证不超速
                def saturate(delta, maxSteeringAngle):
                        delta = sign(delta) * min(abs(delta), maxSteeringAngle)
                        return delta

                # 换算为转弯指令
                steercmd = 10.0*rad2deg(-atan2(topx2-320.0,240.0))*100.0/720.0
                steercmd = saturate(steercmd, 100)
                
                return(src,steercmd,True)

            else:                                 
                return(src,steercmd,False)
        else:
            print("No Lines")
            return(src,steercmd,False)

                    

    
    # if any(lines):
    #     for line in lines:    
    #         for x0,y0,x1,y1 in line:

    #             cv.line(binary, (x0, y0), (x1, y1), 255, 10)
    #             # cv.line(binary, (0, 0), (150, 150), 255, 10)

    #             cv.imshow("lines", binary)
    #             cv.waitKey()

    #             slope = rad2deg(math.atan2((y1-y0),(x1-x0))) # 计算角度
                
    #             # print(x1)

    #             if (slope > 20 and slope < 160) or (slope < -20 and slope > -160): 
    #                 points = (x0,y0,x1,y1) 

    #                 # 
    #                 def clac_edgepoints(points,ymin,ymax):
    #                     x = [points[0],points[2]]
    #                     y = [points[1],points[3]]
    #                     k = polyfit(y,x,1) # 多项式系数
    #                     func = poly1d(k) # 多项式方程
    #                     xmin = int(func(ymin))# 直线对应的最低点 
    #                     xmax= int(func(ymax))# 直线对应的最高点
    #                     return[(xmin,ymin),(xmax,ymax)]                 

    #                 if any(points):
    #                     linetop = clac_edgepoints(points,0,240)        
    #                     topx1 = linetop[0][0]
    #                     topy1 = linetop[0][1]
    #                     topx2 = linetop[1][0]
    #                     topy2 = linetop[1][1]
    #                     if topx2 < 700 and topx2 > -100:
    #                         gpoints.append((topx1,topy1,topx2,topy2))
    #                         p0 = (topx1,topy1+240)
    #                         p1 = (topx2,topy2+240) #　整体上移２４０像素                  
    #                         cv.line(src, p0, p1, (0, 0, 255), 4)  # 画直线
                            
    #                         # 保证不超速
    #                         def saturate(delta, maxSteeringAngle):
    #                                 delta = sign(delta) * min(abs(delta), maxSteeringAngle)
    #                                 return delta

    #                         # 换算为转弯指令
    #                         steercmd = 10.0*rad2deg(-atan2(topx2-320.0,240.0))*100.0/720.0
    #                         steercmd = saturate(steercmd, 100)
                            
    #                         return(src,steercmd,True)

    #                     else:                                 
    #                             return(src,steercmd,False)
    #                 else:
    #                     print("No Lines")
    #                     return(src,steercmd,False)    

    # return(src,steercmd,False) 
    
        
        
        
