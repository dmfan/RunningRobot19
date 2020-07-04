#!/usr/bin/env python3
# coding:utf-8
# Opencv 3.4
# head 看右侧调整位置
#640*480



import cv2
import math
import numpy as np
import threading
import time
import datetime

import CMDcontrol


#################################################初始化#########################################################
camera_out = "chest"
# stream_head = "http://192.168.3.49:8082/?action=stream?dummy=param.mjpg"
# cap_head = cv2.VideoCapture(stream_head)
# stream_chest = "http://192.168.3.49:8080/?action=stream?dummy=param.mjpg"
# cap_chest = cv2.VideoCapture(stream_chest)


cap_chest = cv2.VideoCapture(0)
cap_head = cv2.VideoCapture(2)

action_DEBUG = False
box_debug = True
debug = True
img_debug = True


pic_Org_num = 0
pic_Get_num = 0
state = 1
step = 0
state_sel = 'hole'
reset = 0
skip = 0


#初始化头部舵机角度

chest_ret = False     # 读取图像标志位
ret = False     # 读取图像标志位
ChestOrg_img = None  # 原始图像更新
HeadOrg_img = None  # 原始图像更新
ChestOrg_copy = None
HeadOrg_copy = None
r_width = 480
r_height = 640

chest_r_width = 480
chest_r_height = 640
head_r_width = 640
head_r_height = 480


start_time = datetime.datetime.now()    #fftest
start_time2222 = datetime.datetime.now()    #fftest
end_time = datetime.datetime.now()    #fftest

################################################读取图像线程#################################################
def get_img():
    global ChestOrg_img,HeadOrg_img,HeadOrg_img,chest_ret, pic_Org_num, pic_Get_num
    global ret
    global cap_chest
    global start_time,end_time,end_time
    while True:
        if cap_chest.isOpened():
        # if False:
            chest_ret, ChestOrg_img = cap_chest.read()
            ret, HeadOrg_img = cap_head.read()
            if chest_ret:
                pic_Org_num = pic_Org_num+1
            else:
                print("chest_ret faile ------------------")
            if HeadOrg_img is None:
                print("HeadOrg_img error")
            if ChestOrg_img is None:
                print("ChestOrg_img error")
                
        else:
            time.sleep(0.01)
            ret=True
            print("58L pic  error ")

# 读取图像线程
th1 = threading.Thread(target=get_img)
th1.setDaemon(True)  # 设置为后台线程，这里默认是False，设置为True之后则主线程不用等待子线程
th1.start()


################################################动作执行线程#################################################
def move_action():
    global org_img
    global step, level
    global golf_angle_hole
    global golf_angle_ball, golf_angle
    global golf_dis, golf_dis_y
    global golf_angle_flag, golf_dis_flag
    global golf_angle_start, golf_dis_start
    global golf_ok
    global golf_hole, golf_ball

    CMDcontrol.CMD_transfer()

# 动作执行线程
th2 = threading.Thread(target=move_action)
th2.setDaemon(True)  # 设置为后台线程，这里默认是False，设置为True之后则主线程不用等待子线程
th2.start()


acted_name = ""
def action_append(act_name):
    global acted_name

    # print("please enter to continue...")
    # cv2.waitKey(0)

    if action_DEBUG == False:
        if act_name == "forwardSlow0403" and (acted_name == "Forwalk02RL" or acted_name == "Forwalk02L"):
            acted_name = "Forwalk02LR"
        elif act_name == "forwardSlow0403" and (acted_name == "Forwalk02LR" or acted_name == "Forwalk02R"):
            acted_name = "Forwalk02RL"
        elif act_name != "forwardSlow0403" and (acted_name == "Forwalk02LR" or acted_name == "Forwalk02R"):
            # CMDcontrol.action_list.append("Forwalk02RS")
            # acted_name = act_name
            print(act_name,"动作未执行 执行 Stand")
            acted_name = "Forwalk02RS"
        elif act_name != "forwardSlow0403" and (acted_name == "Forwalk02RL" or acted_name == "Forwalk02L"):
            # CMDcontrol.action_list.append("Forwalk02LS")
            # acted_name = act_name
            print(act_name,"动作未执行 执行 Stand")
            acted_name = "Forwalk02LS"
        elif act_name == "forwardSlow0403":
            acted_name = "Forwalk02R"
        else:
            acted_name = act_name

        CMDcontrol.actionComplete = False
        if len(CMDcontrol.action_list) > 0 :
            print("队列超过一个动作")
            CMDcontrol.action_list.append(acted_name)
        else:
            CMDcontrol.action_list.append(acted_name)
        CMDcontrol.action_wait()

    else:
        print("-----------------------执行动作名：",act_name)
        time.sleep(3) # fftest







box_debug = True

###############得到线形的总的轮廓###############
# 这个比值适应调整  handling
# 排除掉肩部舵机
def getLine_SumContour(contours, area=1):
    global handling
    contours_sum = None
    for c in contours:  # 初始化    contours_sum
        area_temp = math.fabs(cv2.contourArea(c))
        rect = cv2.minAreaRect(c)#最小外接矩形
        box = np.int0(cv2.boxPoints(rect))#最小外接矩形的四个顶点
        edge1=math.sqrt(math.pow(box[3, 1] - box[0, 1], 2) + math.pow(box[3, 0] - box[0, 0], 2))
        edge2=math.sqrt(math.pow(box[3, 1] - box[2, 1], 2) + math.pow(box[3, 0] - box[2, 0], 2))
        ratio=edge1/edge2   # 长与宽的比值大于3认为是条线
        center_y = (box[0,1] + box[1,1] + box[2,1] + box[3,1]) / 4
        if (area_temp > area) and (ratio>3 or ratio<0.33) and center_y > 240:   # 
            contours_sum = c
            break
    for c in contours:
        area_temp = math.fabs(cv2.contourArea(c))
        rect = cv2.minAreaRect(c)#最小外接矩形
        box = np.int0(cv2.boxPoints(rect))#最小外接矩形的四个顶点
        edge1=math.sqrt(math.pow(box[3, 1] - box[0, 1], 2) + math.pow(box[3, 0] - box[0, 0], 2))
        edge2=math.sqrt(math.pow(box[3, 1] - box[2, 1], 2) + math.pow(box[3, 0] - box[2, 0], 2))
        ratio=edge1/edge2
        # print("ratio:",ratio,"area_temp:",area_temp)
  

        if (area_temp > area) and (ratio>3 or ratio<0.33):   # 满足面积条件 长宽比条件

            rect = cv2.minAreaRect(c)#最小外接矩形
            box = np.int0(cv2.boxPoints(rect))#最小外接矩形的四个顶点
            center_x = (box[0,0] + box[1,0] + box[2,0] + box[3,0]) / 4
            center_y = (box[0,1] + box[1,1] + box[2,1] + box[3,1]) / 4

            if center_y > 240:# 满足中心点坐标条件
                contours_sum = np.concatenate((contours_sum, c), axis=0)  # 将所有轮廓点拼接到一起  
                if box_debug:
                    cv2.drawContours(handling, [box], -1, (0, 255, 0), 5)
                    cv2.imshow('handling', handling)
                    cv2.waitKey(10) 
            else:
                if box_debug:
                    cv2.drawContours(handling, [box], -1, (0, 0, 255), 5)
                    cv2.imshow('handling', handling)
                    cv2.waitKey(10)
        else:   # 弃
            rect = cv2.minAreaRect(c)#最小外接矩形
            box = np.int0(cv2.boxPoints(rect))#最小外接矩形的四个顶点
            if box_debug:
                cv2.drawContours(handling, [box], -1, (0, 0, 255), 5)
                cv2.imshow('handling', handling)
                cv2.waitKey(10)

    return contours_sum




color_dist = {'red': {'Lower': np.array([0, 160, 100]), 'Upper': np.array([180, 255, 250])},
              'black_dir': {'Lower': np.array([0, 0, 0]), 'Upper': np.array([130, 145, 90])},
            #   'black_line': {'Lower': np.array([50, 30, 20]), 'Upper': np.array([130, 145, 80])},
               'black_line': {'Lower': np.array([50, 30, 20]), 'Upper': np.array([130, 220, 80])},
              }






# 通过两边的黑线，调整左右位置 和 角度
def head_angle_dis():
    global HeadOrg_img,chest_copy, reset, skip
    global handling
    angle_ok_flag = False
    angle = 90
    dis = 0
    bottom_centreX = 0
    bottom_centreY = 0
    see = False
    dis_ok_count = 0
    headTURN = 0

    step = 1
    print("/-/-/-/-/-/-/-/-/-head_angle_dis")
    while True:
        
        OrgFrame = HeadOrg_img.copy()

        x_start = 260
        blobs = OrgFrame[int(0):int(480), int(x_start):int(380)]  # 只对中间部分识别处理  Y , X
        # cv2.rectangle(blobs,(0,460),(120,480),(255,255,255),-1)     # 涂白
        handling = blobs.copy()
        frame_mask = blobs.copy()

        # 获取图像中心点坐标x, y
        center = []


     # 开始处理图像
     
        hsv = cv2.cvtColor(frame_mask, cv2.COLOR_BGR2HSV)
        hsv = cv2.GaussianBlur(hsv, (3, 3), 0)
        Imask = cv2.inRange(hsv, color_dist['black_line']['Lower'], color_dist['black_line']['Upper'])
        # Imask = cv2.erode(Imask, None, iterations=2)
        Imask = cv2.dilate(Imask, np.ones((3, 3), np.uint8), iterations=2)
        _, cnts, hierarchy = cv2.findContours(Imask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_TC89_L1)  # 找出所有轮廓
        # print("len:",len(cnts))
        cnt_sum = getLine_SumContour(cnts, area=500)


        # 初始化
        L_R_angle = 0 
        blackLine_L = [0,0]
        blackLine_R = [0,0]

        if cnt_sum is not None:
            see = True
            rect = cv2.minAreaRect(cnt_sum)#最小外接矩形
            box = np.int0(cv2.boxPoints(rect))#最小外接矩形的四个顶点
            # cv2.drawContours(OrgFrame, [box], 0, (0, 255, 0), 2)  # 将大矩形画在图上
            # contours_result = cnt_sum[0][0]

            if math.sqrt(math.pow(box[3, 1] - box[0, 1], 2) + math.pow(box[3, 0] - box[0, 0], 2)) > math.sqrt(math.pow(box[3, 1] - box[2, 1], 2) + math.pow(box[3, 0] - box[2, 0], 2)):
                if box[3, 0] - box[0, 0]==0:
                    angle=90
                else:
                    angle = - math.atan((box[3, 1] - box[0, 1]) / (box[3, 0] - box[0, 0]))*180.0/math.pi
                if box[3,1]+box[0,1]>box[2,1]+box[1,1]:
                    Ycenter = int((box[2, 1] + box[1, 1]) / 2)
                    Xcenter = int((box[2, 0] + box[1, 0]) / 2)
                    if box[2, 1] > box[1, 1]:
                        blackLine_L = [box[2, 0] , box[2, 1]]
                        blackLine_R = [box[1, 0] , box[1, 1]]
                    else:
                        blackLine_L = [box[1, 0] , box[1, 1]]
                        blackLine_R = [box[2, 0] , box[2, 1]]
                    cv2.circle(OrgFrame, (Xcenter + x_start, Ycenter), 10, (255,255,0), -1)#画出中心点
                else:
                    Ycenter = int((box[3, 1] + box[0, 1]) / 2)
                    Xcenter = int((box[3, 0] + box[0, 0]) / 2)
                    if box[3, 1] > box[0, 1]:
                        blackLine_L = [box[3, 0] , box[3, 1]]
                        blackLine_R = [box[0, 0] , box[0, 1]]
                    else:
                        blackLine_L = [box[0, 0] , box[0, 1]]
                        blackLine_R = [box[3, 0] , box[3, 1]]
                    cv2.circle(OrgFrame, (Xcenter + x_start, Ycenter), 10, (255,255,0), -1)#画出中心点
            else:
                if box[3, 0] - box[2, 0]==0:
                    angle=90
                else:
                    angle = - math.atan((box[3, 1] - box[2, 1]) / (box[3, 0] - box[2, 0]))*180.0/math.pi # 负号是因为坐标原点的问题
                if box[3,1]+box[2,1]>box[0,1]+box[1,1]:
                    Ycenter = int((box[1, 1] + box[0, 1]) / 2)
                    Xcenter = int((box[1, 0] + box[0, 0]) / 2)
                    if box[0, 1] > box[1, 1]:
                        blackLine_L = [box[0, 0] , box[0, 1]]
                        blackLine_R = [box[1, 0] , box[1, 1]]
                    else:
                        blackLine_L = [box[1, 0] , box[1, 1]]
                        blackLine_R = [box[0, 0] , box[0, 1]]
                    cv2.circle(OrgFrame, (Xcenter + x_start, Ycenter), 10, (255,255,0), -1)#画出中心点
                else:
                    Ycenter = int((box[2, 1] + box[3, 1]) / 2)
                    Xcenter = int((box[2, 0] + box[3, 0]) / 2)
                    if box[3, 1] > box[2, 1]:
                        blackLine_L = [box[3, 0] , box[3, 1]]
                        blackLine_R = [box[2, 0] , box[2, 1]]
                    else:
                        blackLine_L = [box[2, 0] , box[2, 1]]
                        blackLine_R = [box[3, 0] , box[3, 1]]
                    cv2.circle(OrgFrame, (Xcenter + x_start, Ycenter), 10, (255,255,0), -1)#画出中心点


            if blackLine_L[0] == blackLine_R[0]:
                L_R_angle = 0
            else:
                L_R_angle =  -math.atan( (blackLine_L[1]-blackLine_R[1]) / (blackLine_L[0]-blackLine_R[0]) ) *180.0/math.pi





            if img_debug:
                
                cv2.circle(OrgFrame, (blackLine_L[0] + x_start, blackLine_L[1]), 5, [0, 255, 255], 2)
                cv2.circle(OrgFrame, (blackLine_R[0] + x_start, blackLine_R[1]), 5, [255, 0, 255], 2)
                cv2.line(OrgFrame, (blackLine_R[0] + x_start,blackLine_R[1]), (blackLine_L[0] + x_start,blackLine_L[1]), (0, 255, 255), thickness=2)
                cv2.putText(OrgFrame, "L_R_angle:" + str(L_R_angle),(10, OrgFrame.shape[0] - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 255), 2)
                cv2.putText(OrgFrame, "Xcenter:" + str(Xcenter + x_start),(10, OrgFrame.shape[0] - 50), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 255), 2)
                cv2.putText(OrgFrame, "Ycenter:" + str(Ycenter),(200, OrgFrame.shape[0] - 50), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 255), 2)

                # cv2.drawContours(frame_mask, cnt_sum, -1, (255, 0, 255), 3)
                cv2.imshow('frame_mask', frame_mask)
                # cv2.imshow('black', Imask)
                cv2.imshow('OrgFrame', OrgFrame)
                cv2.waitKey(10)
        else:
            see = False


     # 决策执行动作
        if step == 1:
            print("157L 向右看 HeadTurn015")
            action_append("HeadTurn015")
            time.sleep(1)   # timefftest
            step = 2
        elif step == 2:
            if not see:  # not see the edge
                # cv2.destroyAllWindows()
                print("276L 右侧看不到黑线 左侧移 Left3move")
                action_append("Left3move")
                headTURN += 1
                if headTURN > 3:
                    headTURN=0
                    # cv2.waitKey(0)
                    print("276L 右侧看不到黑线 转为左看 waitKey")
                    step = 3
            else:   # 0
                headTURN=0
                if L_R_angle > 2:
                    if L_R_angle > 7:
                        print("416L 左da旋转 turn001L ",L_R_angle)
                        action_append("turn001L")
                    # elif L_R_angle > 5:
                    #     print("419L 左da旋转 turn001L ",L_R_angle)
                    #     action_append("turn001L")
                    else:
                        print("422L 左旋转 turn000L ",L_R_angle)
                        action_append("turn000L")
                    
                    # time.sleep(1)   # timefftest
                elif L_R_angle < -2:
                    if L_R_angle < -7:
                        print("434L 右da旋转  turn001R ",L_R_angle)
                        action_append("turn001R")
                    # elif L_R_angle < -5:
                    #     print("437L 右da旋转  turn001R ",L_R_angle)
                    #     action_append("turn001R")
                    else:
                        print("445L 右旋转  turn000R ",L_R_angle)
                        action_append("turn000R")
                    
                    # time.sleep(1)   # timefftest
                elif Ycenter >= 430:
                    if Ycenter > 450:
                        print("451L 左da侧移 Left3move >440 ",Ycenter)
                        action_append("Left3move")
                    else:
                        print("439L 左侧移 Left02move > 365 ",Ycenter)
                        action_append("Left02move")
                elif Ycenter < 390:
                    if Ycenter < 370:
                        print("445L 右da侧移 Right3move <380 ",Ycenter)
                        action_append("Right3move")
                    else:
                        print("448L 右侧移 Right02move <400 ",Ycenter)
                        action_append("Right02move")
                else:
                    dis_ok_count
                    print("444L 右看 X位置ok")
                    cv2.destroyAllWindows()
                    break

        elif step == 3:
            print("157L 向左看 HeadTurn180")
            action_append("HeadTurn180")
            time.sleep(1)   # timefftest
            step = 4
        elif step == 4:
            if not see:  # not see the edge
                print("294L 左侧 看不到黑线  转为右看")
                headTURN += 1
                if headTURN > 5:
                    headTURN=0
                
                    print("error 两侧都看不到  右侧移 Right3move")
                    action_append("Right3move")
            else:   # 0 +-1
                headTURN=0
                if L_R_angle > 3:
                    if L_R_angle > 8:
                        print("304L 左da旋转 turn001L  ",L_R_angle)
                        action_append("turn001L")
                    # elif L_R_angle > 3:   # fftest delete
                    #     print("304L 左旋转 turn001L  ",L_R_angle)
                    #     action_append("turn000L")
                    else:
                        print("304L 左旋转 turn000L  ",L_R_angle)
                        action_append("turn000L")

                    # time.sleep(1)   # timefftest
                elif L_R_angle < -3:
                    if L_R_angle < -8:
                        print("307L 右da旋转  turn001R  ",L_R_angle)
                        action_append("turn001R")
                    # elif L_R_angle < -7:
                    #     print("307L 右旋转  turn000R  ",L_R_angle)
                    #     action_append("turn000R")
                    else:
                        print("307L 右旋转  turn000R  ",L_R_angle)
                        action_append("turn000R")

                    # time.sleep(1)   # timefftest
                elif Ycenter >= 430:
                    if Ycenter > 450:
                        print("498L 右da侧移 Right3move  ",L_R_angle)
                        action_append("Right3move")
                    else:
                        print("501L 右侧移 Right02move  ",L_R_angle)
                        action_append("Right02move")
                elif Ycenter < 390:
                    if Ycenter < 370:
                        print("497L 左da侧移 Left3move  ",L_R_angle)
                        action_append("Left02move")
                    else:
                        print("500L 左侧移 Left02move  ",L_R_angle)
                        action_append("Left02move")
                else:
                    dis_ok_count
                    print("495L 左看 X位置ok")

                    cv2.destroyAllWindows()
                    break



def head_only_angle():
    global head_copy,chest_copy, reset, skip
    global handling,Head_L_R_angle,see_flag,head_flag,Bbox_centerY,head_step,Bbox_center,blue_rail
    angle_ok_flag = False
    angle = 90
    dis = 0
    bottom_centreX = 0
    bottom_centreY = 0
    dis_ok_count = 0
    headTURN = 0

    while True:
        
        OrgFrame = HeadOrg_img.copy()

        x_start = 260
        blobs = OrgFrame[int(0):int(480), int(x_start):int(380)]  # 只对中间部分识别处理  Y , X
        # cv2.rectangle(blobs,(0,460),(120,480),(255,255,255),-1)     # 涂白
        # handling = cv2.resize(blobs, (resize_width, resize_height), interpolation=cv2.INTER_LINEAR)
        # frame_mask = cv2.resize(blobs, (resize_width, resize_height), interpolation=cv2.INTER_LINEAR)
        handling = blobs.copy()
        frame_mask = blobs.copy()

        # 获取图像中心点坐标x, y
        center = []


     # 开始处理图像
     
        hsv = cv2.cvtColor(frame_mask, cv2.COLOR_BGR2HSV)
        hsv = cv2.GaussianBlur(hsv, (3, 3), 0)
        Imask = cv2.inRange(hsv, color_dist['black_line']['Lower'], color_dist['black_line']['Upper'])
        # Imask = cv2.erode(Imask, None, iterations=2)
        Imask = cv2.dilate(Imask, np.ones((3, 3), np.uint8), iterations=2)
        _, cnts, hierarchy = cv2.findContours(Imask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_TC89_L1)  # 找出所有轮廓
        # print("len:",len(cnts))
        cnt_sum = getLine_SumContour(cnts, area=300)
        cv2.drawContours(frame_mask, cnt_sum, -1, (255, 0, 255), 3)
        # cv2.imshow('black', Imask)

        # 初始化
        Head_L_R_angle = 0 
        blackLine_L = [0,0]
        blackLine_R = [0,0]

        if cnt_sum is not None:
            see_flag = True
            rect = cv2.minAreaRect(cnt_sum)#最小外接矩形
            box = np.int0(cv2.boxPoints(rect))#最小外接矩形的四个顶点
            # cv2.drawContours(OrgFrame, [box], 0, (0, 255, 0), 2)  # 将大矩形画在图上
            # contours_result = cnt_sum[0][0]

            if math.sqrt(math.pow(box[3, 1] - box[0, 1], 2) + math.pow(box[3, 0] - box[0, 0], 2)) > math.sqrt(math.pow(box[3, 1] - box[2, 1], 2) + math.pow(box[3, 0] - box[2, 0], 2)):
                if box[3, 0] - box[0, 0]==0:
                    angle=90
                else:
                    angle = - math.atan((box[3, 1] - box[0, 1]) / (box[3, 0] - box[0, 0]))*180.0/math.pi
                if box[3,1]+box[0,1]>box[2,1]+box[1,1]:
                    Ycenter = int((box[2, 1] + box[1, 1]) / 2)
                    Xcenter = int((box[2, 0] + box[1, 0]) / 2)
                    if box[2, 1] > box[1, 1]:
                        blackLine_L = [box[2, 0] , box[2, 1]]
                        blackLine_R = [box[1, 0] , box[1, 1]]
                    else:
                        blackLine_L = [box[1, 0] , box[1, 1]]
                        blackLine_R = [box[2, 0] , box[2, 1]]
                    cv2.circle(OrgFrame, (Xcenter + x_start, Ycenter), 10, (255,255,0), -1)#画出中心点
                else:
                    Ycenter = int((box[3, 1] + box[0, 1]) / 2)
                    Xcenter = int((box[3, 0] + box[0, 0]) / 2)
                    if box[3, 1] > box[0, 1]:
                        blackLine_L = [box[3, 0] , box[3, 1]]
                        blackLine_R = [box[0, 0] , box[0, 1]]
                    else:
                        blackLine_L = [box[0, 0] , box[0, 1]]
                        blackLine_R = [box[3, 0] , box[3, 1]]
                    cv2.circle(OrgFrame, (Xcenter + x_start, Ycenter), 10, (255,255,0), -1)#画出中心点
            else:
                if box[3, 0] - box[2, 0]==0:
                    angle=90
                else:
                    angle = - math.atan((box[3, 1] - box[2, 1]) / (box[3, 0] - box[2, 0]))*180.0/math.pi # 负号是因为坐标原点的问题
                if box[3,1]+box[2,1]>box[0,1]+box[1,1]:
                    Ycenter = int((box[1, 1] + box[0, 1]) / 2)
                    Xcenter = int((box[1, 0] + box[0, 0]) / 2)
                    if box[0, 1] > box[1, 1]:
                        blackLine_L = [box[0, 0] , box[0, 1]]
                        blackLine_R = [box[1, 0] , box[1, 1]]
                    else:
                        blackLine_L = [box[1, 0] , box[1, 1]]
                        blackLine_R = [box[0, 0] , box[0, 1]]
                    cv2.circle(OrgFrame, (Xcenter + x_start, Ycenter), 10, (255,255,0), -1)#画出中心点
                else:
                    Ycenter = int((box[2, 1] + box[3, 1]) / 2)
                    Xcenter = int((box[2, 0] + box[3, 0]) / 2)
                    if box[3, 1] > box[2, 1]:
                        blackLine_L = [box[3, 0] , box[3, 1]]
                        blackLine_R = [box[2, 0] , box[2, 1]]
                    else:
                        blackLine_L = [box[2, 0] , box[2, 1]]
                        blackLine_R = [box[3, 0] , box[3, 1]]
                    cv2.circle(OrgFrame, (Xcenter + x_start, Ycenter), 10, (255,255,0), -1)#画出中心点


            if blackLine_L[0] == blackLine_R[0]:
                Head_L_R_angle = 0
            else:
                Head_L_R_angle =  -math.atan( (blackLine_L[1]-blackLine_R[1]) / (blackLine_L[0]-blackLine_R[0]) ) *180.0/math.pi

            if img_debug:
                cv2.putText(OrgFrame, "Head_L_R_angle:" + str(Head_L_R_angle),(10, OrgFrame.shape[0] - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 255), 2)
                cv2.putText(OrgFrame, "Xcenter:" + str(Xcenter + x_start),(10, OrgFrame.shape[0] - 50), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 255), 2)
                cv2.putText(OrgFrame, "Ycenter:" + str(Ycenter),(200, OrgFrame.shape[0] - 50), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 255), 2)
                cv2.circle(OrgFrame, (blackLine_L[0] + x_start, blackLine_L[1]), 5, [0, 255, 255], 2)
                cv2.circle(OrgFrame, (blackLine_R[0] + x_start, blackLine_R[1]), 5, [255, 0, 255], 2)
                cv2.line(OrgFrame, (blackLine_R[0] + x_start,blackLine_R[1]), (blackLine_L[0] + x_start,blackLine_L[1]), (0, 255, 255), thickness=2)

            # cv2.imshow('frame_mask', frame_mask)
            # cv2.imshow('OrgFrame', OrgFrame)
            # cv2.waitKey(100)
        else:
            see_flag = False


     # 决策执行动作

        if head_step == 1:
            print("317L 向右看 HeadTurn015")
            action_append("HeadTurn015")
            head_flag = 'R'
            head_step = 2
            time.sleep(1)   # timefftest
        elif head_step == 2:
            if not see_flag:  # not see_flag the edge
                # cv2.destroyAllWindows()
                print("276L 右侧看不到黑线 转为左看")
                headTURN += 1
                if headTURN > 5:
                    headTURN=0
                    # k = cv2.waitKey(0)
                    # if k == 'c':
                    #     continue
                    # elif k == ord('f'):
                    head_step = 3
            else:
                headTURN=0
                # Angle
                if Head_L_R_angle > 2:
                    if Head_L_R_angle > 7:
                        print("329L 左da旋转 turn001L > 7 ",Head_L_R_angle)
                        action_append("turn001L")
                        
                    else:
                        print("333L 左旋转 turn000L > 2 ",Head_L_R_angle)
                        action_append("turn000L")
                        
                elif Head_L_R_angle < -2:
                    if Head_L_R_angle < -7:
                        print("338L 右da旋转  turn001R < -7 ",Head_L_R_angle)
                        action_append("turn001R")
                        
                    else:
                        print("342L 右旋转  turn000R < -2 ",Head_L_R_angle)
                        action_append("turn000R")
                        
                # 栏杆标志位 蓝横杆出现时时，允许侧移
                elif Ycenter > 365 and blue_rail:
                    if Ycenter > 400:   # 400
                        print("3942L 左侧移 Left3move > 400 ",Ycenter)
                        action_append("Left3move")
                    else:
                        print("3942L 左侧移 Left02move > 365 ",Ycenter)
                        action_append("Left02move")
                    break
                elif Ycenter < 360 and blue_rail:
                    if Ycenter < 350:
                        print("390L 右侧移 Right3move <350 ",Ycenter)
                        action_append("Right3move")
                    else:
                        print("390L 右侧移 Right02move <360 ",Ycenter)
                        action_append("Right02move")
                else:
                    print("1133L Ycenter:",Ycenter,"  blue_rail:",blue_rail)
                    dis_ok_count
                    print("1092L before blueBOX X位置ok")
                    break

        elif head_step == 3:
            print("359L 向左看 HeadTurn180")
            action_append("HeadTurn180")
            head_flag = 'L'
            head_step = 4
            time.sleep(1)   # timefftest
        elif head_step == 4:
            if not see_flag:  # not see_flag the edge
                print("367L 左侧看不到")
                headTURN += 1
                if headTURN > 5:
                    headTURN=0
                    # print("294L 左侧 看不到黑线  转为右看")
                    # print("error 两侧都看不到")
                    # print("367L 左侧看不到")
                    # k = cv2.waitKey(0)
                    # if k == 'c':
                    #     continue
                    # elif k == ord('f'):
                    head_step = 1
            else:
                headTURN=0
                # Angle
                if Head_L_R_angle > 4:
                    if Head_L_R_angle > 8:
                        print("372L 左da旋转 turn001L > 8 ",Head_L_R_angle)
                        action_append("turn001L")
                        
                    else:
                        print("3764L 左旋转 turn000L > 4 ",Head_L_R_angle)
                        action_append("turn000L")
                    time.sleep(1)   # timefftest
                elif Head_L_R_angle < -4:
                    if Head_L_R_angle < -8:
                        print("381L 右da旋转  turn001R < 4 ",Head_L_R_angle)
                        action_append("turn001R")
                        
                    else:
                        print("385L 右旋转  turn000R < -8 ",Head_L_R_angle)
                        action_append("turn000R")
                    time.sleep(1)   # timefftest
                    
                # 蓝横杆出现时时，允许侧移
                elif Ycenter > 370 and blue_rail:
                    if Ycenter > 400:
                        print("390L 右侧移 Right3move > 400 ",Ycenter)
                        action_append("Right3move")
                    else:
                        print("390L 右侧移 Right02move > 365 ",Ycenter)
                        action_append("Right02move")
                    break
                elif Ycenter < 340 and blue_rail:
                    if Ycenter < 330:
                        print("3942L 左侧移 Left3move < 350 ",Ycenter)
                        action_append("Left3move")
                    else:
                        print("3942L 左侧移 Left02move < 360 ",Ycenter)
                        action_append("Left02move")
                else:
                    print("1133L Ycenter:",Ycenter,"  blue_rail:",blue_rail)
                    dis_ok_count
                    print("1151L before blueBOX X位置ok")
                    break
    # action_append("HeadTurnMM")


# 2020年4月24日21:03:08
if __name__ == '__main__':
    while len(CMDcontrol.action_list) > 0 :
        print("等待启动")
        time.sleep(1)
    action_append("HeadTurnMM")

    
    while True:
        if HeadOrg_img is not None and ret:
            k = cv2.waitKey(100)
            if k == 27:
                cv2.destroyWindow('camera_test')
                break

            state=1
            step = 0
            head_step = 1
            blue_rail = False
            head_only_angle() #调整中央位置
            print("位置调整OK")
            cv2.waitKey(1000)
            
            
            
        else:
            print('image is empty')
            time.sleep(0.01)
            cv2.destroyAllWindows()
