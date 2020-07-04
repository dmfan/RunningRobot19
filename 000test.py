#!/usr/bin/env python3
# coding:utf-8

import cv2
import math
import numpy as np
import threading
import time
import datetime

import CMDcontrol

robot_IP = "192.168.3.52"
camera_out = "chest"
stream_pic = False
action_DEBUG = False
#################################################初始化#########################################################

if stream_pic:
    stream_head = "http://"+ robot_IP +":8082/?action=stream?dummy=param.mjpg"
    cap_head = cv2.VideoCapture(stream_head)
    stream_chest = "http://"+ robot_IP +":8080/?action=stream?dummy=param.mjpg"
    cap_chest = cv2.VideoCapture(stream_chest)
else:
    cap_chest = cv2.VideoCapture(0)
    cap_head = cv2.VideoCapture(2)


box_debug = False
debug = False
img_debug = False


state = 1
step = 0
state_sel = 'hole'
reset = 0
skip = 0



chest_ret = False     # 读取图像标志位
ret = False           # 读取图像标志位
ChestOrg_img = None   # 原始图像更新
HeadOrg_img = None    # 原始图像更新
ChestOrg_copy = None
HeadOrg_copy = None
r_width = 480
r_height = 640

chest_r_width = 480
chest_r_height = 640
head_r_width = 640
head_r_height = 480
################################################读取图像线程#################################################
def get_img():
    global ChestOrg_img,HeadOrg_img,HeadOrg_img,chest_ret
    global ret
    global cap_chest
    while True:
        if cap_chest.isOpened():
            
            chest_ret, ChestOrg_img = cap_chest.read()
            ret, HeadOrg_img = cap_head.read()
            if (chest_ret == False) or (ret == False):
                print("ret faile ------------------")
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
th1.setDaemon(True)
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
th2.setDaemon(True)
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
        time.sleep(2)









color_range = {
               'yellow_door': [(20, 140, 60), (40, 240, 150)],
               'black_door': [(25, 25, 10), (110, 150, 30)],
               'black_gap': [(0, 0, 0), (180, 255, 70)],
               'yellow_hole': [(20, 120, 95), (30, 250, 190)],
               'black_hole': [(5, 80, 20), (40, 255, 100)],
               'chest_red_floor': [(0, 40, 60), (20,200, 190)],
               'chest_red_floor1': [(0, 100, 60), (20,200, 210)],
               'chest_red_floor2': [(110, 100, 60), (180,200, 210)],
                'green_bridge': [(50, 75, 70), (80, 240, 210)],
               }

color_dist = {'red': {'Lower': np.array([0, 160, 100]), 'Upper': np.array([180, 255, 250])},
               'black_dir': {'Lower': np.array([0, 0, 10]), 'Upper': np.array([170, 170, 45])},
               'black_line': {'Lower': np.array([0, 0, 20]), 'Upper': np.array([100, 160, 80])},
              'blue': {'Lower': np.array([100, 80, 46]), 'Upper': np.array([124, 255, 255])},
              'ball_red': {'Lower': np.array([160, 100, 70]), 'Upper': np.array([190, 215, 145])},
              'blue_hole': {'Lower': np.array([100, 130, 80]), 'Upper': np.array([130, 255, 150])},
              }

###############得到线形的总的轮廓###############
# 这个比值适应调整  handling
# 排除掉肩部黑色
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
        if (area_temp > area) and (ratio>3 or ratio<0.33) and center_y > 240:
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


# 得到最大轮廓和对应的最大面积 
def getAreaMaxContour1(contours):    # 返回轮廓 和 轮廓面积
    contour_area_temp = 0
    contour_area_max = 0
    area_max_contour = None
    for c in contours:  # 历遍所有轮廓
        contour_area_temp = math.fabs(cv2.contourArea(c))  # 计算轮廓面积
        if contour_area_temp > contour_area_max:
            contour_area_max = contour_area_temp
            if contour_area_temp > 25:  #只有在面积大于25时，最大面积的轮廓才是有效的，以过滤干扰
                area_max_contour = c
    return area_max_contour, contour_area_max  # 返回最大的轮廓

########得到最大轮廓############
def getAreaMaxContour2(contours, area=1):
    contour_area_max = 0
    area_max_contour = None
    for c in contours:
        contour_area_temp = math.fabs(cv2.contourArea(c))
        if contour_area_temp > contour_area_max:
            contour_area_max = contour_area_temp
            if contour_area_temp > area:  # 面积大于1
                area_max_contour = c
    return area_max_contour



# 将所有面积大于1的轮廓点拼接到一起    
def getSumContour(contours, area=1):
    contours_sum = None
    # print(len(contours))
    for c in contours:  #   初始化contours
        area_temp = math.fabs(cv2.contourArea(c))
        if (area_temp > area):
            contours_sum = c
            break
    for c in contours:
        area_temp = math.fabs(cv2.contourArea(c))
        if (area_temp > area):
            contours_sum = np.concatenate((contours_sum, c), axis=0)  # 将所有面积大于1的轮廓点拼接到一起          
    return contours_sum

######### 得到所有轮廓的面积##########
def getAreaSumContour(contours):
    contour_area_sum = 0
    for c in contours:  # 历遍所有轮廓
        contour_area_sum += math.fabs(cv2.contourArea(c))  # 计算轮廓面积
    return contour_area_sum  # 返回最大的面积



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
    print("/-/-/-/-/-/-/-/-/-head*angle*dis")
    while True:
        
        OrgFrame = HeadOrg_img.copy()

        x_start = 260
        blobs = OrgFrame[int(0):int(480), int(x_start):int(380)]        # 只对中间部分识别处理  Y , X
        # cv2.rectangle(blobs,(0,460),(120,480),(255,255,255),-1)       # 涂白
        handling = blobs.copy()
        frame_mask = blobs.copy()

        # 获取图像中心点坐标x, y
        center = []


     # 开始处理图像
     
        hsv = cv2.cvtColor(frame_mask, cv2.COLOR_BGR2HSV)
        hsv = cv2.GaussianBlur(hsv, (3, 3), 0)
        Imask = cv2.inRange(hsv, color_dist['black_line']['Lower'], color_dist['black_line']['Upper'])
        # Imask = cv2.erode(Imask, None, iterations=1)
        Imask = cv2.dilate(Imask, np.ones((3, 3), np.uint8), iterations=2)
        _, cnts, hierarchy = cv2.findContours(Imask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_TC89_L1)  # 找出所有轮廓
        # print("327L len:",len(cnts))
        cnt_sum = getLine_SumContour(cnts, area=300)


        # 初始化
        L_R_angle = 0 
        blackLine_L = [0,0]
        blackLine_R = [0,0]

        if cnt_sum is not None:
            see = True
            rect = cv2.minAreaRect(cnt_sum)#最小外接矩形
            box = np.int0(cv2.boxPoints(rect))#最小外接矩形的四个顶点
            # cv2.drawContours(OrgFrame, [box], 0, (0, 255, 0), 2)  # 将大矩形画在图上

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
                # cv2.imshow('frame_mask', frame_mask)
                cv2.imshow('black', Imask)
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
                print("276L 右侧看不到黑线 左侧移 Left3move")
                action_append("Left3move")
                headTURN += 1
                if headTURN > 3:
                    headTURN=0
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
                        print("461L 右旋转  turn000R ",L_R_angle)
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
                        print("474L 右da侧移 Right3move <380 ",Ycenter)
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
                    else:
                        print("304L 左旋转 turn000L  ",L_R_angle)
                        action_append("turn000L")
                    # time.sleep(1)   # timefftest
                elif L_R_angle < -3:
                    if L_R_angle < -8:
                        print("307L 右da旋转  turn001R  ",L_R_angle)
                        action_append("turn001R")
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




#################################################第二关：台阶##########################################
def floor():
    global org_img, state, state_sel, step, reset, skip, debug
    global camera_out
    if (state == 2 or state == 6 or state == 8) and state_sel == 'floor':  # 初始化
        print("/-/-/-/-/-/-/-/-/-进入floor")
        step = 0
        
    r_w = chest_r_width
    r_h = chest_r_height

    top_angle = 0
    T_B_angle = 0
    topcenter_x = 0.5 * r_w
    topcenter_y = 0
    bottomcenter_x = 0.5 * r_w
    bottomcenter_y = 0

    state_sel = 'floor'

    while state_sel == 'floor':


     # 分析图像 # chest
     
    
        if True:    # 上下边沿

            Corg_img = ChestOrg_img.copy()
            Corg_img = np.rot90(Corg_img)
            OrgFrame = Corg_img.copy()

            # 初始化 bottom_right  bottom_left
            bottom_right = (480,0)
            bottom_left =  (0,0)
            top_right = (480,0)  # 右上角点坐标
            top_left = (0,0)  # 左上角点坐标

            frame = cv2.resize(OrgFrame, (chest_r_width, chest_r_height), interpolation=cv2.INTER_LINEAR)
            frame_copy = frame.copy()
            # 获取图像中心点坐标x, y
            center = []
            # 开始处理图像
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            hsv = cv2.GaussianBlur(hsv, (3, 3), 0)
            
            Imask = cv2.inRange(hsv, color_range['chest_red_floor1'][0], color_range['chest_red_floor1'][1])  # 对原图像和掩模(颜色的字典)进行位运算
            # opened = cv2.morphologyEx(Imask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))  # 开运算 去噪点
            # Imask = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))  # 闭运算 封闭连接
            # Imask = cv2.erode(Imask, None, iterations=2)
            Imask = cv2.dilate(Imask, np.ones((3, 3), np.uint8), iterations=2)

            _, cnts, hierarchy = cv2.findContours(Imask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_TC89_L1)  # 找出所有轮廓
            
            cnt_sum, area_max = getAreaMaxContour1(cnts)  # 找出最大轮廓
            C_percent = round(area_max * 100 / (r_w * r_h), 2)  # 最大轮廓百分比
            cv2.drawContours(frame, cnt_sum, -1, (255, 0, 255), 3)
            if cnt_sum is not None:
                see = True
                rect = cv2.minAreaRect(cnt_sum)#最小外接矩形
                box = np.int0(cv2.boxPoints(rect))#最小外接矩形的四个顶点
                
                bottom_right = cnt_sum[0][0]  # 右下角点坐标
                bottom_left = cnt_sum[0][0]  # 左下角点坐标
                top_right = cnt_sum[0][0]  # 右上角点坐标
                top_left = cnt_sum[0][0]  # 左上角点坐标
                for c in cnt_sum:

                    if c[0][0] + 1 * (r_h - c[0][1]) < bottom_left[0] + 1 * (r_h - bottom_left[1]):
                        bottom_left = c[0]
                    if c[0][0] + 1 * c[0][1] > bottom_right[0] + 1 * bottom_right[1]:
                        bottom_right = c[0]

                    if c[0][0] + 3 * c[0][1] < top_left[0] + 3 * top_left[1]:
                        top_left = c[0]
                    if (r_w - c[0][0]) + 3 * c[0][1] < (r_w - top_right[0]) + 3 * top_right[1]:
                        top_right = c[0]

                    # if debug:
                    #     handling = ChestOrg_img.copy()
                    #     cv2.circle(handling, (c[0][0], c[0][1]), 5, [0, 255, 0], 2)
                    #     cv2.circle(handling, (bottom_left[0], bottom_left[1]), 5, [255, 255, 0], 2)
                    #     cv2.circle(handling, (bottom_right[0], bottom_right[1]), 5, [255, 0, 255], 2)
                    #     cv2.imshow('handling', handling)  # 显示图像
                    #     cv2.waitKey(2)

                bottomcenter_x = (bottom_left[0] + bottom_right[0]) / 2  # 得到bottom中心坐标
                bottomcenter_y = (bottom_left[1] + bottom_right[1]) / 2

                topcenter_x = (top_right[0] + top_left[0]) / 2  # 得到top中心坐标
                topcenter_y = (top_left[1] + top_right[1]) / 2

                bottom_angle =  -math.atan( (bottom_right[1]-bottom_left[1]) / (bottom_right[0]-bottom_left[0]) ) *180.0/math.pi
                top_angle =  -math.atan( (top_right[1]-top_left[1]) / (top_right[0]-top_left[0]) ) *180.0/math.pi
                if math.fabs(topcenter_x - bottomcenter_x) <= 1:  # 得到连线的角度
                    T_B_angle = 90
                else:
                    T_B_angle = - math.atan((topcenter_y - bottomcenter_y) / (topcenter_x - bottomcenter_x)) * 180.0 / math.pi

                if img_debug:
                    cv2.drawContours(frame_copy, [box], 0, (0, 255, 0), 2)  # 将大矩形画在图上
                    cv2.line(frame_copy, (bottom_left[0],bottom_left[1]), (bottom_right[0],bottom_right[1]), (255, 255, 0), thickness=2)
                    cv2.line(frame_copy, (top_left[0],top_left[1]), (top_right[0],top_right[1]), (255, 255, 0), thickness=2)
                    cv2.line(frame_copy, (int(bottomcenter_x),int(bottomcenter_y)), (int(topcenter_x),int(topcenter_y)), (255, 255, 255), thickness=2)    # T_B_line

                    cv2.putText(frame_copy, "bottom_angle:" + str(bottom_angle), (30, 450), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 0),2)  # (0, 0, 255)BGR
                    cv2.putText(frame_copy, "top_angle:" + str(top_angle),(30, 150), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 0), 2)
                    cv2.putText(frame_copy, "T_B_angle:" + str(T_B_angle),(30, 400), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 255), 2)

                    cv2.putText(frame_copy, "bottomcenter_x:" + str(bottomcenter_x), (30, 480), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 0),2)  # (0, 0, 255)BGR
                    cv2.putText(frame_copy, "y:" + str(int(bottomcenter_y)), (300, 480), cv2.FONT_HERSHEY_SIMPLEX, 0.65,(0, 0, 0), 2)  # (0, 0, 255)BGR

                    cv2.putText(frame_copy, "topcenter_x:" + str(topcenter_x), (30, 180), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 0),2)  # (0, 0, 255)BGR
                    cv2.putText(frame_copy, "topcenter_y:" + str(int(topcenter_y)), (230, 180), cv2.FONT_HERSHEY_SIMPLEX, 0.65,(0, 0, 0), 2)  # (0, 0, 255)BGR

                    cv2.putText(frame_copy, 'C_percent:' + str(C_percent) + '%', (30, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 0), 2)
                    cv2.putText(frame_copy, "step:" + str(step), (30, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 0),2)  # (0, 0, 255)BGR
                    
                    cv2.circle(frame_copy, (int(topcenter_x), int(topcenter_y)), 5, [255, 0, 255], 2)
                    cv2.circle(frame_copy, (int(bottomcenter_x), int(bottomcenter_y)), 5, [255, 0, 255], 2)
                    cv2.circle(frame_copy, (top_right[0], top_right[1]), 5, [0, 255, 255], 2)
                    cv2.circle(frame_copy, (top_left[0], top_left[1]), 5, [0, 255, 255], 2)
                    cv2.circle(frame_copy, (bottom_right[0], bottom_right[1]), 5, [0, 255, 255], 2)
                    cv2.circle(frame_copy, (bottom_left[0], bottom_left[1]), 5, [0, 255, 255], 2)
                    cv2.imshow('Chest_Camera', frame_copy)  # 显示图像
                    cv2.imshow('chest_red_mask', Imask)
                    cv2.waitKey(100)

            else:
                print("chest NONE")



     # 决策执行动作
        angle_ok_flag = False
        if step == 0:   # 前进依据chest 调整大致位置，方向  看底边线调整角度
            if C_percent > 1 and bottomcenter_y < 380:
                print("676L 继续前行 forwardSlow0403",bottomcenter_y)
                action_append("forwardSlow0403")

            elif 380 <= bottomcenter_y <430:
                if bottom_angle > 3:  # 需要左转
                    if bottom_angle > 6:
                        print("725L 大左转一下  turn001L ",bottom_angle)
                        action_append("turn001L")
                    else:
                        print("728L bottom_angle > 3 需要小左转 turn001L ",bottom_angle)
                        action_append("turn001L")
                elif bottom_angle < -3:  # 需要右转
                    if bottom_angle < -6:
                        print("732L 右da旋转  turn001R < -6 ",Head_L_R_angle)
                        action_append("turn001R")
                    else:
                        print("735L bottom_angle < -3 需要小右转 turn001R ",bottom_angle)
                        action_append("turn001R")
                elif -3 <= bottom_angle <= 3:  # 角度正确
                    print("448L 角度合适")
                    angle_ok_flag = True

                if angle_ok_flag:
                    if bottomcenter_x < 200:
                        print("431L 向左侧移 Left02move ",bottomcenter_x)
                        action_append("Left02move")
                    elif bottomcenter_x > 260:
                        print("433L 向右侧移 Right02move ",bottomcenter_x)
                        action_append("Right02move")
                    else:
                        print("483L 变小步继续前行 Forwalk01",bottomcenter_y)
                        action_append("Forwalk01")
            elif 430 <= bottomcenter_y <= 540:
                if bottom_angle > 4:  # 需要左转
                    if bottom_angle > 6:
                        print("746L 大左转一下  turn001L ",bottom_angle)
                        action_append("turn001L")
                    else:
                        print("749L bottom_angle > 4 需要小左转 turn001L ",bottom_angle)
                        action_append("turn001L")
                elif bottom_angle < -4:  # 需要右转
                    if bottom_angle < -6:
                        print("338L 右da旋转  turn001R < -6 ",bottom_angle)
                        action_append("turn001R")
                    else:
                        print("746L bottom_angle < -4 需要小右转 turn001R ",bottom_angle)
                        action_append("turn001R")
                elif -3 <= bottom_angle <= 3:  # 角度正确
                    print("448L 角度合适")
                    angle_ok_flag = True

                if angle_ok_flag:
                    if bottomcenter_x < 200:
                        print("431L 向左侧移 Left1move ",bottomcenter_x)
                        action_append("Left1move")
                    elif bottomcenter_x > 260:
                        print("433L 向右侧移 Right1move ",bottomcenter_x)
                        action_append("Right1move")
                    else:
                        print("486L 到达上台阶边沿，变前挪动 Forwalk00 bottomcenter_y:",bottomcenter_y)
                        action_append("Forwalk00")


            elif bottomcenter_y > 540:
                print("然后开始第二步------")
                step = 1
                angle_ok_flag = False
            else:  # C_percent < 1 and bottomcenter_y < 380 
                print("error769L 前进")
        elif step == 1: # 看中线调整角度
            print("719L 上台阶 上台阶 UpBridge")
            action_append("UpBridge")
            step = 2


                
        elif step == 2: # 已经上台阶  调整方向  快走三步  看上方顶点边线
            if 0 < T_B_angle < 86:  # 右转
                print("730L 右转 turn001R T_B_angle:",T_B_angle)
                action_append("turn001R")
                # time.sleep(1)   # timefftest
            elif -86 < T_B_angle < 0:  # 左转
                print("359L 左转 turn001L T_B_angle:",T_B_angle)
                action_append("turn001L")
                # time.sleep(1)   # timefftest
            elif T_B_angle <= -86 or T_B_angle >= 86:  # 角度正确
                print("738L T_B_angle 角度合适 ")
                if topcenter_x < 200:
                    print("740L <210 向左侧移 Left1move ",topcenter_x)
                    action_append("Left1move")
                elif topcenter_x > 240:
                    print("743L >260 向右侧移 Right1move ",topcenter_x)
                    action_append("Right1move")

                    
                elif topcenter_y < 360:
                    print("516L 上台阶后，快走 fastForward06 topcenter_y:",topcenter_y)
                    action_append("fastForward06")
                    print("step 3333 ,",topcenter_y)
                    step = 3
                        
        elif step == 3: #  快走结束
            if topcenter_y < 510 and C_percent > 6:
                if top_angle > 1.5:  # 需要左转
                    if top_angle <= 3:
                        print("468L 3 < < 1.5 需要小左转 turn000L ",top_angle)
                        action_append("turn000L")
                    else:
                        print("468L > 3 需要小左转 turn001L ",top_angle)
                        action_append("turn001L")
                elif top_angle < -1.5:  # 需要右转
                    if top_angle > -3:
                        print("470L -3 < < -1.5 需要小右转 turn000R ",top_angle)
                        action_append("turn000R")
                    else:
                        print("470L < -3 需要小右转 turn001R ",top_angle)
                        action_append("turn001R")
                elif -1.5 <= top_angle <= 1.5:  # 角度正确
                    print("474L top_angle 角度合适 ")
                    if topcenter_x < 190:
                        print("456L <210 向左侧移 Left1move ",topcenter_x)
                        action_append("Left1move")
                    elif topcenter_x > 260:
                        print("458L >260 向右侧移 Right1move ",topcenter_x)
                        action_append("Right1move")
                    else:
                        print("590L <topcenter_y<510 Forwalk01 ",topcenter_y)
                        action_append("Forwalk01")
            else:   # > 510
                print("step 4444 ,",topcenter_y)
                step = 4

        elif step == 4: #  调整角度
            if topcenter_y > 550 and C_percent < 6:
                if C_percent >= 2 :
                    print("823L 下台阶前前进一点点 Forwalk00")
                    action_append("Forwalk00")
                print("487L 下台阶 下台阶 DownBridge")
                action_append("DownBridge")
                step = 5
            else:
                print("566L 微微前挪 y:",topcenter_y," C_percent:",C_percent)
                action_append("Forwalk00")
        elif step ==5:
            print("899L 完成bridge")
            cv2.destroyAllWindows()
            break
    
       
#################################################第三关：雷阵#############################################


head_flag = "MM"
Head_L_R_angle = 0
see_flag = False
Bbox_centerY = 0
head_step = 1
# 通过两边的黑线，仅仅调整角度 
def head_only_angle(resize_width, resize_height):
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
                print("276L 右侧看不到黑线 转为左看")
                headTURN += 1
                if headTURN > 5:
                    headTURN=0
                    head_step = 3
            else:
                headTURN=0
                # Angle
                if Head_L_R_angle > 3:
                    if Head_L_R_angle > 8:
                        print("329L 左da旋转 turn001L > 7 ",Head_L_R_angle)
                        action_append("turn001L")
                        
                    else:
                        print("333L 左旋转 turn000L > 2 ",Head_L_R_angle)
                        action_append("turn000L")
                        
                elif Head_L_R_angle < -1:
                    if Head_L_R_angle < -6:
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
                    # time.sleep(1)   # timefftest
                elif Head_L_R_angle < -4:
                    if Head_L_R_angle < -8:
                        print("381L 右da旋转  turn001R < 4 ",Head_L_R_angle)
                        action_append("turn001R")
                        
                    else:
                        print("385L 右旋转  turn000R < -8 ",Head_L_R_angle)
                        action_append("turn000R")
                    # time.sleep(1)   # timefftest
                    
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



def obstacle():
    global state, HeadOrg_img, step, reset, skip
    global Head_L_R_angle,Bbox_centerY,blue_rail

    state = 3
    blue_rail = False
    
    if state == 3:
        print("/-/-/-/-/-/-/-/-/-进入obscle")
        count = 0
        step = 1
        k = 1
        isgo = False
        
        straight = False  # 直行信号
        left = False  # 左移信号
        left2 = False  # 遇到右边缘且前方有障碍
        right = False  # 右移
        right2 = False  # 遇到左边缘且前方有障碍

    else:
        return

    # 初始化 delta
    delta = datetime.datetime.now()
    delta = delta - delta

    
    head_only_angle(640,480)
    while state == 3:

        if True:
            
            Corg_img = ChestOrg_img.copy()
            Corg_img = np.rot90(Corg_img)
            Corg_img = Corg_img.copy()
            # cv2.rectangle(Corg_img,(0,630),(480,640),(255,255,255),-1)
            hsv = cv2.cvtColor(Corg_img, cv2.COLOR_BGR2HSV)
            hsv = cv2.GaussianBlur(hsv, (3, 3), 0)


         # blue 分析图像 决策执行
            Bumask = cv2.inRange(hsv, color_dist['blue']['Lower'], color_dist['blue']['Upper'])
            Bumask = cv2.erode(Bumask, None, iterations=2)
            Bumask = cv2.dilate(Bumask, np.ones((3, 3), np.uint8), iterations=2)
            # cv2.imshow('Bluemask', Bumask)
            _, cntsblue, hierarchy = cv2.findContours(Bumask, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)  # 找出轮廓
            
            if cntsblue is not None:
                cnt_large = getAreaMaxContour2(cntsblue)    # 取最大轮廓
            else:
                print("1135L cnt_large is None")
                continue
            if cnt_large is not None:
                rect_blue = cv2.minAreaRect(cnt_large)
                box_blue = np.int0(cv2.boxPoints(rect_blue))  # 点的坐标
                Bbox_centerX = int((box_blue[3,0] + box_blue[2,0] + box_blue[1,0] + box_blue[0,0])/4)
                Bbox_centerY = int((box_blue[3,1] + box_blue[2,1] + box_blue[1,1] + box_blue[0,1])/4)
                Bbox_center = [Bbox_centerX,Bbox_centerY]
                cv2.circle(Corg_img, (Bbox_center[0],Bbox_center[1]), 7, (0, 0, 255), -1) # 圆点标记

                cv2.drawContours(Corg_img, [box_blue], -1, (255,0,0), 3)
                obscle_area_blue = 0
                # 当遇到蓝色门槛时停止
                for c in cntsblue:
                    obscle_area_blue += math.fabs(cv2.contourArea(c))
                if  Bbox_centerY >= 280 and obscle_area_blue > 0.05 * 640 * 480 :   # and go_up: # 320  obscle_area_blue > 0.05 * 640 * 480 and
                    state = 4
                    if img_debug:
                        cv2.imshow('Corg_img', Corg_img)
                        cv2.waitKey(10)
                    print("遇到蓝色门槛-----*-----*-----*-----* Bbox_center Y:",Bbox_centerY)
                    action_append("Stand")
                    blue_rail = True
                    

                    cv2.destroyAllWindows()
                    break


         # black 分析图像 决策执行
            Imask = cv2.inRange(hsv, color_dist['black_dir']['Lower'], color_dist['black_dir']['Upper'])
            Imask = cv2.erode(Imask, None, iterations=3)
            Imask = cv2.dilate(Imask, np.ones((3, 3), np.uint8), iterations=2)
            # cv2.imshow('black', Imask)
            _, contours, hierarchy = cv2.findContours(Imask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_TC89_L1)  # 找出所有轮廓
            # print("contours lens:",len(contours))
            cv2.drawContours(Corg_img, contours, -1, (255, 0, 255), 2)


            left_point = [640,0]
            right_point = [0,0]
            if len(contours) != 0:

                Big_battle = [0,0]

                for c in contours:
                    rect = cv2.minAreaRect(c)  # 最小外接矩形
                    box = cv2.boxPoints(rect)   #我们需要矩形的4个顶点坐标box, 通过函数 cv2.cv.BoxPoints() 获得
                    box = np.intp(box)  # 最小外接矩形的四个顶点
                    box_Ax,box_Ay = box[0,0],box[0,1]
                    box_Bx,box_By = box[1,0],box[1,1]
                    box_Cx,box_Cy = box[2,0],box[2,1]
                    box_Dx,box_Dy = box[3,0],box[3,1]
                    box_centerX = int((box_Ax + box_Bx + box_Cx + box_Dx)/4)
                    box_centerY = int((box_Ay + box_By + box_Cy + box_Dy)/4)
                    box_center = [box_centerX,box_centerY]
                    # cv2.circle(Corg_img, (box_centerX,box_centerY), 7, (0, 255, 0), -1) #距离比较点 绿圆点标记
                    # cv2.drawContours(Corg_img, [box], -1, (255,0,0), 3)

                        

                    # 剔除图像上部分点 和底部点
                    if box_centerY < 250 or box_centerY > 610:
                        continue
                    
                    # 遍历点 画圈
                    if box_debug:
                        cv2.circle(Corg_img, (box_centerX,box_centerY), 8, (0, 0, 255), 2) # 圆点标记识别黑点
                        cv2.imshow('Corg_img', Corg_img)
                        cv2.waitKey(1)
                        
                    # 找出最左点与最右点
                    if  box_centerX < left_point[0]:
                        left_point = box_center
                    if box_centerX > right_point[0]:
                        right_point = box_center

                    if box_centerX <= 80 or box_centerX >= 400 :  # 排除左右边沿点 box_centerXbox_centerX 240
                        continue
                    if math.pow(box_centerX - 240 , 2) + math.pow(box_centerY - 640 , 2) < math.pow(Big_battle[0] - 240 , 2) + math.pow(Big_battle[1] - 640 , 2):
                        Big_battle =  box_center  # 这个是要规避的黑点
                        # print("1272L go_up False ",Big_battle[0],Big_battle[1])

                # 显示图
                if img_debug:
                    cv2.circle(Corg_img, (left_point[0],left_point[1]), 7, (0, 255, 0), -1) # 圆点标记
                    cv2.circle(Corg_img, (right_point[0],right_point[1]), 7, (0, 255, 255), -1) # 圆点标记
                    cv2.circle(Corg_img, (Big_battle[0],Big_battle[1]), 7, (255, 255, 0), -1) # 圆点标记
                    cv2.putText(Corg_img, "Head_L_R_angle:" + str(int(Head_L_R_angle)), (230, 400), cv2.FONT_HERSHEY_SIMPLEX, 0.65,(0, 0, 0), 2)  # (0, 0, 255)BGR
                    cv2.putText(Corg_img, "see_flag:" + str(int(see_flag)), (230, 440), cv2.FONT_HERSHEY_SIMPLEX, 0.65,(0, 0, 0), 2)  # (0, 0, 255)BGR 
                    cv2.putText(Corg_img, "Bbox_centerY:" + str(int(Bbox_centerY)), (230, 460), cv2.FONT_HERSHEY_SIMPLEX, 0.65,(0, 0, 0), 2)  # (0, 0, 255)BGR
                    cv2.putText(Corg_img, "Big_battle x,y:" + str(int(Big_battle[0])) +', ' + str(int(Big_battle[1])) , (230, 480), cv2.FONT_HERSHEY_SIMPLEX, 0.65,(0, 0, 0), 2)  # (0, 0, 255)BGR
                    cv2.line(Corg_img, (Big_battle[0],Big_battle[1]), (240,640), (0, 255, 255), thickness=2)
                    # 500线
                    cv2.line(Corg_img, (0,500), (480,500), (255, 255, 255), thickness=2)
                    
                    # cv2.imshow('handling', handling)
                    cv2.imshow('Corg_img', Corg_img)
                    k = cv2.waitKey(100)
                    if k == 27:
                        cv2.destroyWindow('closed_pic')
                        cv2.destroyWindow('org_img_copy')
                        break
                    elif k == ord('s'):
                        print("save picture123")
                        cv2.imwrite("picture123.jpg",HeadOrg_img) #保存图片

                if Big_battle[1] > 440:
                    head_only_angle(640,480)
                # black 决策执行动作
                if Big_battle[1] <= 370:
                    print("608L 前进靠近 forwardSlow0403 ",Big_battle[1])
                    action_append("forwardSlow0403")
                    # action_append("forwardSlow0403")
                # elif Big_battle[1] <= 430:
                #     print("565L 前进靠近 forwardSlow0403 ",Big_battle[1])
                #     action_append("forwardSlow0403")
                # elif Big_battle[1] < 500  and (Big_battle[0] <= (100+(640-Big_battle[1])*0.15) or Big_battle[0] >= (380-(640-Big_battle[1])*0.15)) :
                #     print("568L 前进靠近 forwardSlow0403  ",Big_battle[1])
                #     action_append("forwardSlow0403")
                elif Big_battle[1] < 460:
                    print("571L 慢慢前进靠近 Forwalk01  ",Big_battle[1])
                    action_append("Forwalk01")
                
                # 80---140---*240*---340---400  要同步修改  box_centerX
                # elif Big_battle == right_point and (Big_battle != left_point):
                #     print("277L 右平移两步 Right3move")
                #     action_append("Right3move")
                #     action_append("Right3move")
                # elif Big_battle == left_point and (Big_battle != right_point):
                #     print("279L 向左平移两步 Left3move")
                #     action_append("Left3move")
                #     action_append("Left3move")
                elif (80<= Big_battle[0] and Big_battle[0] < 140):
                    print("275L 右平移一点点 Right02move")
                    action_append("Right02move")
                    action_append("Right02move")
                elif (140<= Big_battle[0] and Big_battle[0]< 240):
                    print("277L 右平移一步 Right3move")
                    action_append("Right3move")

                elif (240<= Big_battle[0] and Big_battle[0]< 340):
                    print("279L 向左平移一步 Left3move")
                    action_append("Left3move")
                elif (340<= Big_battle[0] < 400):
                    print("281L 向左平移一点点 Left02move")
                    action_append("Left02move")
                    action_append("Left02move")
                else:
                    print("1321L error 不在范围")





            else:
                print("287L 无障碍，可前进")
                action_append("forwardSlow0403")
                Big_battle = [0,0]

                if img_debug:
                    cv2.circle(Corg_img, (left_point[0],left_point[1]), 7, (0, 255, 0), -1) # 圆点标记
                    cv2.circle(Corg_img, (right_point[0],right_point[1]), 7, (0, 255, 255), -1) # 圆点标记
                    cv2.circle(Corg_img, (Big_battle[0],Big_battle[1]), 7, (255, 255, 0), -1) # 圆点标记
                    cv2.putText(Corg_img, "Head_L_R_angle:" + str(int(Head_L_R_angle)), (230, 400), cv2.FONT_HERSHEY_SIMPLEX, 0.65,(0, 0, 0), 2)  # (0, 0, 255)BGR
                    cv2.putText(Corg_img, "see_flag:" + str(int(see_flag)), (230, 440), cv2.FONT_HERSHEY_SIMPLEX, 0.65,(0, 0, 0), 2)  # (0, 0, 255)BGR 
                    cv2.putText(Corg_img, "Bbox_centerY:" + str(int(Bbox_centerY)), (230, 460), cv2.FONT_HERSHEY_SIMPLEX, 0.65,(0, 0, 0), 2)  # (0, 0, 255)BGR
                    cv2.line(Corg_img, (Big_battle[0],Big_battle[1]), (240,640), (0, 255, 255), thickness=2)
                    # 500线
                    cv2.line(Corg_img, (0,500), (480,500), (255, 255, 255), thickness=2)
                    
                    cv2.imshow('handling', handling)
                    cv2.imshow('Corg_img', Corg_img)
                    k = cv2.waitKey(100)
                    if k == 27:
                        cv2.destroyWindow('closed_pic')
                        cv2.destroyWindow('org_img_copy')
                        break
                    elif k == ord('s'):
                        print("save picture123")
                        cv2.imwrite("picture123.jpg",HeadOrg_img) #保存图片



#################################################第四关：挡板###########################################
def baffle():
    global state, org_img, step,  reset, skip
    global handling

    state = 4

    if state == 4:
       
        print("/-/-/-/-/-/-/-/-/-进入baffle")
        step = 0
        baffle_dis_Y_flag = False
        baffle_angle = 0
        notok = True
        see = False
        finish = False
        angle = 45
        dis = 0
        dis_flag = False
        angle_flag = False
    else:
        return
    while state == 4:
        if True:

            Corg_img = ChestOrg_img.copy()
            Corg_img = np.rot90(Corg_img)
            
            OrgFrame = Corg_img.copy()
            handling = Corg_img.copy()
            frame = Corg_img.copy()
            center = []



    # 开始处理图像
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            hsv = cv2.GaussianBlur(hsv, (3, 3), 0)
            Imask = cv2.inRange(hsv, color_dist['blue']['Lower'], color_dist['blue']['Upper'])
            Imask = cv2.erode(Imask, None, iterations=2)
            Imask = cv2.dilate(Imask, np.ones((3, 3), np.uint8), iterations=2)
            # cv2.imshow('BLcolor', Imask)
            _, cnts, hieracy = cv2.findContours(Imask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_TC89_L1)  # 找出所有轮廓
            
            # print("cnts len:",len(cnts))
            if cnts is not None:
                cnt_large = getAreaMaxContour2(cnts, area=1000)
            else:
                print("1135L cnt_large is None")
                continue

            blue_bottom_Y = 0
            if cnt_large is not None:
                rect = cv2.minAreaRect(cnt_large)  # 最小外接矩形
                box = np.int0(cv2.boxPoints(rect))  # 最小外接矩形的四个顶点
                
                Ax = box[0, 0]
                Ay = box[0, 1]
                Bx = box[1, 0]
                By = box[1, 1]
                Cx = box[2, 0]
                Cy = box[2, 1]
                Dx = box[3, 0]
                Dy = box[3, 1]
                pt1_x, pt1_y = box[0, 0], box[0, 1]
                pt3_x, pt3_y = box[2, 0], box[2, 1]
                center_x = int((pt1_x + pt3_x) / 2)
                center_y = int((pt1_y + pt3_y) / 2)
                center.append([center_x, center_y])
                cv2.drawContours(OrgFrame, [box], -1, [0, 0, 255, 255], 3)
                cv2.circle(OrgFrame, (center_x, center_y), 10, (0, 0, 255), -1)  # 画出中心点
                # 求得大矩形的旋转角度，if条件是为了判断长的一条边的旋转角度，因为box存储的点的顺序不确定\
                if math.sqrt(math.pow(box[3, 1] - box[0, 1], 2) + math.pow(box[3, 0] - box[0, 0], 2)) > math.sqrt(math.pow(box[3, 1] - box[2, 1], 2) + math.pow(box[3, 0] - box[2, 0], 2)):
                    baffle_angle = - math.atan((box[3, 1] - box[0, 1]) / (box[3, 0] - box[0, 0])) * 180.0 / math.pi
                else:
                    baffle_angle = - math.atan( (box[3, 1] - box[2, 1]) / (box[3, 0] - box[2, 0]) ) * 180.0 / math.pi  # 负号是因为坐标原点的问题
                if center_y > blue_bottom_Y:
                    blue_bottom_Y = center_y
            baffle_dis_Y = blue_bottom_Y
            if baffle_dis_Y > 240:
                baffle_dis_Y_flag = True


            if img_debug:
                cv2.putText(OrgFrame, "baffle_dis_Y:" + str(baffle_dis_Y),
                            (10, OrgFrame.shape[0] - 35), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 255), 2)
            
                cv2.putText(OrgFrame, "baffle_dis_Y_flag:" + str(baffle_dis_Y_flag),
                            (10, OrgFrame.shape[0] - 55), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 255), 2)
            
                cv2.putText(OrgFrame, "baffle_angle:" + str(baffle_angle),
                            (10, OrgFrame.shape[0] - 75), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 255), 2)
                cv2.putText(OrgFrame, "step:" + str(step), (30, OrgFrame.shape[0] - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 0),2)  # (0, 0, 255)BGR
                            
                cv2.imshow('OrgFrame', OrgFrame)
                k = cv2.waitKey(10)
                if k == 27:
                    cv2.destroyWindow('closed_pic')
                    cv2.destroyWindow('org_img_copy')
                    break
                elif k == ord('s'):
                    print("save picture123")
                    cv2.imwrite("picture123.jpg",org_img) #保存图片


            
    # 决策执行动作
            if step == 0:
                if baffle_dis_Y <= 250:
                    print("294L 大步前进 Forwalk02")
                    action_append("Forwalk02")
                elif baffle_dis_Y > 250:
                    step=1
            elif step==1:   # 调整角度 -5 ~ 5
                if baffle_angle > 5:
                    if baffle_angle > 8:
                        print("1471L 大左转一下  turn001L  baffle_angle:",baffle_angle)
                        action_append("turn001L")
                    else:
                        print("1474L 左转 turn000L  baffle_angle:",baffle_angle)
                        action_append("turn000L")
                elif baffle_angle < -5:
                    if baffle_angle < -8:
                        print("1478L 大右转一下  turn001R  baffle_angle:",baffle_angle)
                        action_append("turn001R")
                    else:
                        print("1481L 右转 turn000R  baffle_angle:",baffle_angle)
                        action_append("turn000R")
                else:
                    step=2
                
            elif step == 2:     # 调整前进位置  调整左右位置
                if baffle_dis_Y < 390:
                    print("318L 大一步前进 forwardSlow0403")
                    action_append("forwardSlow0403")
                elif 390 < baffle_dis_Y < 460:
                    print("320L 向前挪动 Forwalk00")
                    action_append("Forwalk00")
                elif 460 < baffle_dis_Y:
                    step = 3
            elif step == 3: # 调整角度
                if baffle_angle > 2:
                    if baffle_angle > 5:
                        print("316L 大左转一下  turn001L ",baffle_angle)
                        action_append("turn001L")
                    else:
                        print("318L 左转 turn001L")
                        action_append("turn001L")
                elif baffle_angle < -2:
                    if baffle_angle < -5:
                        print("321L 大右转一下  turn001R ",baffle_angle)
                        action_append("turn001R")
                    else:
                        print("323L 右转 turn001R ",baffle_angle)
                        action_append("turn001R")
                elif baffle_dis_Y_flag:
                    step = 4
            elif step == 4: # 跨栏后调整方向

                print("342L 前挪一点点")


                print("326L 翻栏杆 翻栏杆 RollRail")
                action_append("Stand")
                action_append("RollRail")
                print("step step step 444 ")
                
                
                action_append("turn004L")
                action_append("turn004L")
                action_append("turn004L")
                # action_append("turn004L")
                # action_append("turn004L")
                
                cv2.destroyAllWindows()
                step = 5
                break
 

################################################第六关：独木桥##########################################
def Greenbridge():
    global state_sel, org_img, step, reset, skip, debug, chest_ret
    

    
    r_w = chest_r_width
    r_h = chest_r_height

    step = 0
    state = 6
    
    print("/-/-/-/-/-/-/-/-/-进入Greenbridge")
    while (state == 6):  # 初始化

    # 开始处理图像

        chest_copy=np.rot90(ChestOrg_img)
        chest_copy=chest_copy.copy()
     # chest
        cv2.rectangle(chest_copy,(0,0),(480,150),(255,255,255),-1)
        border = cv2.copyMakeBorder(chest_copy, 12, 12, 16, 16, borderType=cv2.BORDER_CONSTANT,value=(255, 255, 255))  # 扩展白边，防止边界无法识别
        Chest_img_copy = cv2.resize(border, (r_w, r_h), interpolation=cv2.INTER_CUBIC)  # 将图片缩放
        
        Chest_frame_gauss = cv2.GaussianBlur(Chest_img_copy, (3, 3), 0)  # 高斯模糊
        Chest_frame_hsv = cv2.cvtColor(Chest_frame_gauss, cv2.COLOR_BGR2HSV)  # 将图片转换到HSV空间
        Chest_frame_green = cv2.inRange(Chest_frame_hsv, color_range['green_bridge'][0],color_range['green_bridge'][1])  # 对原图像和掩模(颜色的字典)进行位运算
        Chest_opened = cv2.morphologyEx(Chest_frame_green, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))  # 开运算 去噪点
        Chest_closed = cv2.morphologyEx(Chest_opened, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))  # 闭运算 封闭连接


        (_, Chest_contours, hierarchy) = cv2.findContours(Chest_closed, cv2.RETR_LIST,cv2.CHAIN_APPROX_NONE)  # 找出轮廓cv2.CHAIN_APPROX_NONE
        # print("Chest_contours len:",len(Chest_contours))
        Chest_areaMaxContour, Chest_area_max = getAreaMaxContour1(Chest_contours)  # 找出最大轮廓
        Chest_percent = round(Chest_area_max * 100 / (r_w * r_h), 2)
            
        if Chest_areaMaxContour is not None:
            Chest_rect = cv2.minAreaRect(Chest_areaMaxContour)
            # center, w_h, Head_angle = rect  # 中心点 宽高 旋转角度
            Chest_box = np.int0(cv2.boxPoints(Chest_rect))  # 点的坐标

            # 初始化四个顶点坐标
            Chest_top_left = Chest_areaMaxContour[0][0]
            Chest_top_right = Chest_areaMaxContour[0][0]
            Chest_bottom_left = Chest_areaMaxContour[0][0]
            Chest_bottom_right = Chest_areaMaxContour[0][0]
            for c in Chest_areaMaxContour:  # 遍历找到四个顶点
                if c[0][0] + 1.5 * c[0][1] < Chest_top_left[0] + 1.5 * Chest_top_left[1]:
                    Chest_top_left = c[0]
                if (r_w - c[0][0]) + 1.5 * c[0][1] < (r_w - Chest_top_right[0]) + 1.5 * Chest_top_right[1]:
                    Chest_top_right = c[0]
                if c[0][0] + 1.5 * (r_h - c[0][1]) < Chest_bottom_left[0] + 1.5 * (r_h - Chest_bottom_left[1]):
                    Chest_bottom_left = c[0]
                if c[0][0] + 1.5 * c[0][1] > Chest_bottom_right[0] + 1.5 * Chest_bottom_right[1]:
                    Chest_bottom_right = c[0]
            angle_top = - math.atan((Chest_top_right[1] - Chest_top_left[1]) / (Chest_top_right[0] - Chest_top_left[0])) * 180.0 / math.pi
            angle_bottom = - math.atan((Chest_bottom_right[1] - Chest_bottom_left[1]) / (Chest_bottom_right[0] - Chest_bottom_left[0])) * 180.0 / math.pi
            Chest_top_center_x = int((Chest_top_right[0] + Chest_top_left[0]) / 2)
            Chest_top_center_y = int((Chest_top_right[1] + Chest_top_left[1]) / 2)
            Chest_bottom_center_x = int((Chest_bottom_right[0] + Chest_bottom_left[0]) / 2)
            Chest_bottom_center_y = int((Chest_bottom_right[1] + Chest_bottom_left[1]) / 2)
            Chest_center_x = int((Chest_top_center_x + Chest_bottom_center_x) / 2)
            Chest_center_y = int((Chest_top_center_y + Chest_bottom_center_y) / 2)
            if img_debug:
                cv2.drawContours(Chest_img_copy, [Chest_box], 0, (0, 0, 255), 2)  # 将大矩形画在图上
                cv2.circle(Chest_img_copy, (Chest_top_right[0], Chest_top_right[1]), 5, [0, 255, 255], 2)
                cv2.circle(Chest_img_copy, (Chest_top_left[0], Chest_top_left[1]), 5, [0, 255, 255], 2)
                cv2.circle(Chest_img_copy, (Chest_bottom_right[0], Chest_bottom_right[1]), 5, [0, 255, 255], 2)
                cv2.circle(Chest_img_copy, (Chest_bottom_left[0], Chest_bottom_left[1]), 5, [0, 255, 255], 2)
                cv2.circle(Chest_img_copy, (Chest_top_center_x, Chest_top_center_y), 5, [0, 255, 255], 2)
                cv2.circle(Chest_img_copy, (Chest_bottom_center_x, Chest_bottom_center_y), 5, [0, 255, 255], 2)
                cv2.circle(Chest_img_copy, (Chest_center_x, Chest_center_y), 7, [255, 255, 255], 2)
                cv2.line(Chest_img_copy, (Chest_top_center_x, Chest_top_center_y), (Chest_bottom_center_x, Chest_bottom_center_y), [0, 255, 255],2)  # 画出上下中点连线
            if math.fabs(Chest_top_center_x - Chest_bottom_center_x) <= 1:  # 得到连线的角度
                Chest_angle = 90
            else:
                Chest_angle = - math.atan((Chest_top_center_y - Chest_bottom_center_y) / (Chest_top_center_x - Chest_bottom_center_x)) * 180.0 / math.pi
        else:
            Chest_angle = 90
            # center_x = 0.5*r_w
            Chest_center_x = 0
            Chest_bottom_center_x = 0
            Chest_bottom_center_y = 0
            Chest_top_center_x = 0
            Chest_top_center_y = 0

            angle_top = 90
            angle_bottom = 90

        if img_debug:
            
            cv2.drawContours(Chest_img_copy, Chest_contours, -1, (255, 0, 255), 1)
            cv2.putText(Chest_img_copy, 'Chest_percent:' + str(Chest_percent) + '%', (30, 140), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 0),2)  # (0, 0, 255)BGR
            cv2.putText(Chest_img_copy, "Chest_angle:" + str(int(Chest_angle)), (30, 170), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 0),2)  # (0, 0, 255)BGR
            cv2.putText(Chest_img_copy, "Chest_bottom_center(x,y): " + str(int(Chest_bottom_center_x)) + " , " + str(int(Chest_bottom_center_y)), (30, 240), cv2.FONT_HERSHEY_SIMPLEX, 0.65,(0, 0, 0), 2)  # (0, 0, 255)BGR
            cv2.putText(Chest_img_copy, "Chest_top_center(x,y): " + str(int(Chest_top_center_x)) + " , " + str(int(Chest_top_center_y)), (30, 220), cv2.FONT_HERSHEY_SIMPLEX, 0.65,(0, 0, 0), 2)  # (0, 0, 255)BGR
            cv2.putText(Chest_img_copy, "angle_top:" + str(int(angle_top)), (30, 260), cv2.FONT_HERSHEY_SIMPLEX, 0.65,(0, 0, 0), 2)  # (0, 0, 255)BGR
            cv2.putText(Chest_img_copy, "angle_bottom:" + str(int(angle_bottom)), (30, 280), cv2.FONT_HERSHEY_SIMPLEX, 0.65,(0, 0, 0), 2)  # (0, 0, 255)BGR
            cv2.putText(Chest_img_copy, "step :" + str(int(step)), (30, 300), cv2.FONT_HERSHEY_SIMPLEX, 0.65,(0, 0, 0), 2)  # (0, 0, 255)BGR
            cv2.imshow('Chest_Camera', Chest_img_copy)  # 显示图像
            # cv2.imshow('chest_green_mask', Chest_closed)  # 显示图像
            cv2.waitKey(100)



    # 决策执行动作
        if step == 0: # 接近 看下边沿  角度  Chest_percent > 5
            if Chest_bottom_center_y < 260 :
                # print("296L y<360 大步前进 两步 forwardSlow0403")
                # action_append("forwardSlow0403")
                # action_append("forwardSlow0403")
                print("1694L 快速前进 ", Chest_bottom_center_y)
                action_append("fastForward04")
            elif Chest_bottom_center_y > 460:   # 450
                step = 1
            # 260< Chest_bottom_center_y <460
            elif angle_bottom > 5:
                if angle_bottom > 8:
                    print("1658L 大左转一下 > 8  turn001L ",angle_bottom)
                    action_append("turn001L")
                else:
                    print("1661L 小左转 turn001L ",angle_bottom)
                    action_append("turn001L")
                # time.sleep(1)
            elif angle_bottom < -5:
                if angle_bottom < -8:
                    print("1666L 大右转一下 < -8  turn001R ",angle_bottom)
                    action_append("turn001R")
                else:
                    print("1669L 小右转 turn001R ",angle_bottom)
                    action_append("turn001R")
                # time.sleep(1)
            elif Chest_bottom_center_x > 260:  # 右移    center_x
                    print("161 向右移 Right02move  x>250")
                    action_append("Right02move")
            elif Chest_bottom_center_x < 220:  # 左移  center_x
                print("160 向左移 Left02move x<220")
                action_append("Left02move")
            elif 220 <= Chest_bottom_center_x <= 260:   # Chest_bottom_center_y < 450
                    # print("239 前进两步 forwardSlow0403")
                    # action_append("forwardSlow0403")
                    print("1705L 快走333 fastForward03")
                    action_append("fastForward03")
                    

        elif step == 1:     # 到绿桥边沿，对准绿桥阶段
            if Chest_bottom_center_y > 580:
                step = 2
            elif angle_bottom > 3:
                if angle_bottom > 6:
                    print("1678L 大左转一下 > 6  turn001L ",angle_bottom)
                    action_append("turn001L")
                else:
                    print("1690L 小左转 turn000L ",angle_bottom)
                    action_append("turn000L")
                # time.sleep(1)
            elif angle_bottom < -3:
                if angle_bottom < -6:
                    print("1695L 大右转一下 < -6  turn001R ",angle_bottom)
                    action_append("turn001R")
                else:
                    print("1698L 小右转 turn000R ",angle_bottom)
                    action_append("turn000R")
                # time.sleep(1)
            elif Chest_bottom_center_x > 260:  # 右移    center_x
                    print("1702L 向右移 Right02move  x>250")
                    action_append("Right02move")
            elif Chest_bottom_center_x < 220:  # 左移  center_x
                print("1705L 向左移 Left02move x<220")
                action_append("Left02move")
            elif 220 <= Chest_bottom_center_x <= 260:
                    print("1708L 对准 快走 fastForward03")
                    action_append("fastForward03")

        elif step == 2:   # 已经在独木桥阶段  行走独木桥 调整角度 位置  看中线 角度
            if Chest_percent > 2 and Chest_top_center_y > 360:
                    step = 3
            elif Chest_percent > 2 and Chest_top_center_y > 100:
                # 调整角度位置
                
                if Chest_bottom_center_x >= 250:  # 右移    center_x
                    print("1767L 向右移 Right02move  >250 ,",Chest_bottom_center_x)
                    action_append("Right02move")
                elif Chest_bottom_center_x <= 230:  # 左移  center_x
                    print("1770L 向左移 Left02move <230 ,",Chest_bottom_center_x)
                    action_append("Left02move")
                elif 230 < Chest_bottom_center_x < 250:

                    if 0 < Chest_angle < 86:  # 右转
                        print("1775L 右转 turn001R Chest_angle:",Chest_angle)
                        action_append("turn001R")
                        # time.sleep(1)   # timefftest
                    elif -86 < Chest_angle < 0:  # 左转
                        print("1779L 左转 turn001L Chest_angle:",Chest_angle)
                        action_append("turn001L")
                        # time.sleep(1)   # timefftest
                    else :#走三步
                        # print("337L 前进一步 forwardSlow0403")
                        # action_append("forwardSlow0403")
                        print("1753L 上桥后，快走 fastForward03 Ccenter_y:",Chest_center_x)
                        action_append("fastForward03")
                        time.sleep(1)   # timefftest 快走停顿
            
            else: 
                # print("341L 没有看到绿桥向前直行 forwardSlow0403")
                # action_append("forwardSlow0403")
                print("1741L 已经下桥")
                step = 3
        elif step == 3: # 接近 看上边沿  调整角度  Chest_percent > 5
            if Chest_percent < 1 or Chest_top_center_y > 500:
                # print("297L 接近桥终点 直行两步离开桥 forwardSlow0403")
                # action_append("forwardSlow0403")
                # action_append("forwardSlow0403")
                # action_append("forwardSlow0403")
                # action_append("Stand")
                
                print("1778LL 接近桥终点 快走离开桥 fastForward03")
                action_append("fastForward03")
                step = 4
            elif angle_top > 3:
                if angle_top > 6:
                    print("298L 大左转一下  turn001L")
                    action_append("turn001L")
                else:
                    print("1727L 左转 turn001L")
                    action_append("turn001L")
            elif angle_top < -3:
                if angle_top < -6:
                    print("303L 大右转一下  turn001R")
                    action_append("turn001R")
                else:
                    print("305L 右转 turn001R")
                    action_append("turn001R")
            elif Chest_top_center_x > 250:  # 右移    center_x
                print("363L 向右移  >250")
                action_append("Right1move")
            elif Chest_top_center_x < 220:  # 左移  center_x
                print("366L 向左移 <220")
                action_append("Left1move")
            elif 220<= Chest_top_center_x <= 250:
                # print("1802L 前进一步 forwardSlow0403")
                # action_append("forwardSlow0403")
                print("1804L 快走 fastForward03")
                action_append("fastForward03")

        elif step == 4:# 离开独木桥阶段   chest 出现bridge  依据chest调整角度位置
            print("623L 离开桥")
            print("过桥结束，step = -1  下一关 踢球")
            cv2.destroyAllWindows()
            step = 100

            print("--continue---")
            break
  


################################################第七关：踢球进洞########################################

golf_angle_ball = 90
Chest_ball_angle = 90
hole_Angle = 45
golf_angle = 0
ball_x = 0
ball_y = 0
golf_dis_flag = False   #未使用
golf_angle_flag = False
golf_dis_start = True
golf_angle_start = False
golf_ok = False
hole_flag = False
Chest_ball_flag = False
Chest_golf_angle = 0

ball_dis_start = True
hole_angle_start = False

head_state = 0      # 90 ~ -90      左+90   右-90   

hole_x = 0
hole_y = 0

angle_dis_count = 0
picnum = 0
fast_run = True

def act_move():
    global step, state, reset, skip
    global hole_Angle,ball_hole
    global golf_angle_ball, golf_angle ,Chest_ball_angle ,Chest_golf_angle
    global ball_x, ball_y,Chest_ball_x, Chest_ball_y 
    global golf_angle_flag, golf_dis_flag   #golf_dis_flag未使用
    global golf_angle_start, golf_dis_start
    global golf_ok
    global hole_flag,Chest_ball_flag
    global ball_dis_start,hole_angle_start
    global head_state, angle_dis_count ,fast_run
    ball_hole_angle_ok = False


        # 由脚底到红球延伸出一条射线，依据球洞与该射线的关系，调整机器人位置
        # ball_hole_local()

    if True:
        if step == 0:   # 发现球，发现球洞，记录球与球洞的相对位置
            # print("看黑线调整居中")
            if Chest_ball_flag == True:   # 前进到球跟前
                if fast_run:
                    if Chest_ball_y <= 270:  # 340
                        print("1870L 快走前进 fastForward04 ",Chest_ball_y)
                        # action_append("forwardSlow0403")
                        # action_append("forwardSlow0403")
                        action_append("fastForward04")
                        head_angle_dis()    # headfftest
                    elif Chest_ball_y <= 290:  # 340
                        print("1902L 快走前进 fastForward03 ",Chest_ball_y)
                        action_append("fastForward03")
                        # head_angle_dis()    # headfftest
                    else:
                        print("1902L 快走完成",Chest_ball_y)
                        fast_run = False

                else:
                    if Chest_ball_y <360:   # 390
                        # X
                        if Chest_ball_x < 140:  # 240 - 100
                            print("159L Chest_ball_x < 180 左侧移 ",Chest_ball_x)
                            action_append("Left3move")
                        elif Chest_ball_x > 340:    # 240 + 100
                            print("161L Chest_ball_x > 300 右侧移 ",Chest_ball_x)
                            action_append("Right3move")
                        else:
                            print("168L 前挪一点点 1111111 ",Chest_ball_y)
                            action_append("forwardSlow0403")
                    else: # Chest_ball_y>360
                        print("goto step1  ",Chest_ball_y)
                        step = 1
            else:
                print("183L 未发现红球  左右旋转头部摄像头 寻找红球")
                print("238L 前进 fastForward03")
                action_append("fastForward03") # ffetst
                # 目前假设红球在正前方，能看到
                
                # if head_state == 0:
                #     print("头右转(-60)寻找球")
                #     head_state = -60
                # elif head_state == -60:
                #     print("头由右转变为左转(+60)寻找球")
                #     head_state = 60
                # elif head_state == 60:
                #     print("头部 恢复0 向前迈进")
                
        elif step == 1:     # 看球调整位置   逐步前进调整至看球洞
            if Chest_ball_y <= 350:
                print("174L 前挪一点点 Forwalk00 < 380 ",Chest_ball_y)
                action_append("Forwalk00")
            elif Chest_ball_y > 480:
                print("1903L 后一步 Back2Run > 480",Chest_ball_y)
                action_append("Back2Run")
            elif 350< Chest_ball_y <= 480:
                
                if hole_flag == True:
                    if head_state == -60:
                        print("头右看，看到球洞")
                        step = 2
                        # print("172L 头恢复0 向右平移")
                        # head_state = 0
                    elif head_state == 60:
                        print("头左看，看到球洞")
                        step = 3
                        # print("172L 头恢复0 向左平移")
                        # head_state = 0
                    elif head_state == 0:     # 头前看 看到球洞
                        print("270L step4")
                        step = 4
                else:
                    print("273error 左右旋转头 寻找球洞 ")
                    # 目前假设球洞在前方，head能看到
                
                    # if head_state == 0:
                    #     print("头右转(-60)寻找球")
                    #     head_state = -60
                    # elif head_state == -60:
                    #     print("头由右转变为左转(+60)寻找球")
                    #     head_state = 60
                    # elif head_state == 60:
                    #     print("头部 恢复0 向前迈进")
      


        elif step == 2:
            # 头右看，看到球洞
            print("22222222222找红球与球洞")
            if Chest_ball_y < 160:
                print("174L 一大步前进")

            elif Chest_ball_y < 360:  
                print("177L 后挪一点点")
            elif 160< Chest_ball_y < 320:
                print("找到了在左边跳第4步，找到了在右边跳第3步")

                if hole_flag == True:
                    if head_state == -60:
                        print("头右看，看到球")
                        step = 3
                        # print("172L 头恢复0 向右平移")
                        # head_state = 0
                    elif head_state == 60:
                        print("头左看，看到球")
                        step = 4
                        # print("172L 头恢复0 向左平移")
                        # head_state = 0
                    elif head_state == 0:     # 头前看 看到球洞
                        step = 1
                else:
                    print("左右旋转头 寻找球洞")
                    # 目前假设球洞在前方，head能看到
                
                    if head_state == 0:
                        print("头右转(-60)寻找球")
                        head_state = -60
                    elif head_state == -60:
                        print("头由右转变为左转(+60)寻找球")
                        head_state = 60
                    elif head_state == 60:
                        print("头部 恢复0 向前迈进")

        elif step == 3:
            # 头左看，看到球洞
            print("33333333333左侧移")
            if Chest_ball_y > 280:
                print("后挪一点点")
            elif Chest_ball_y < 150:
                print("前挪一点点")
            elif Chest_ball_x < 450:
                print("左侧移")
            
            if hole_flag == False:
                print("右转")
            else:
                step = 1
                ball_dis_start = True
                hole_angle_start = False
            # 完成左侧移后 右转
            # 找球洞
        
        



        elif step == 4:  # 粗略调整朝向   球与球洞大致在一条线
            # print("调整红球在左脚正前方不远处，看球洞的位置调整")
            if ball_dis_start:
                if Chest_ball_x <= 200:
                    if 240 - Chest_ball_x > 40:
                        print("373L4 需要左侧移 Left3move", Chest_ball_x)
                        action_append("Left3move")
                    else:
                        print("376L4 需要左侧移 Left02move", Chest_ball_x)
                        action_append("Left02move")
                    angle_dis_count = 0
                elif Chest_ball_x > 280:
                    if Chest_ball_x - 240 > 40:
                        print("359L4 需要右侧移 Right3move", Chest_ball_x)
                        action_append("Right3move")
                    else:
                        print("384L4 需要右侧移 Right02move", Chest_ball_x)
                        action_append("Right02move")
                    angle_dis_count = 0
                else:
                    print("388L4 Chest_ball_y---位置ok")
                    ball_dis_start = False
                    hole_angle_start = True
            if hole_angle_start:
                if hole_Angle <=0:
                    # angle
                    if hole_Angle > -86:
                        if hole_Angle >= -82:
                            if Chest_ball_y > 480:
                                print("392L4 需要后挪一点 Back2Run ",Chest_ball_y)
                                action_append("Back2Run")
                                angle_dis_count = 0
                            elif Chest_ball_y < 350:
                                print("395L4 需要前挪一点 Forwalk00",Chest_ball_y)
                                action_append("Forwalk00")
                                angle_dis_count = 0

                            print("381L4 大左转一下  turn004L ", hole_Angle)
                            action_append("turn004L")
                        else:
                            if Chest_ball_y > 485:
                                print("386L4 需要后挪一点 Back1Run ",Chest_ball_y)
                                action_append("Back1Run")
                                angle_dis_count = 0
                            elif Chest_ball_y < 350:
                                print("427L4 需要前挪一点 Forwalk00 ",Chest_ball_y)
                                action_append("Forwalk00")
                                angle_dis_count = 0
                                
                            print("397L4 左转一下  turn001L ", hole_Angle)
                            action_append("turn001L")
                    else:
                        print("401L4 hole_Angle---角度ok")
                        angle_dis_count = angle_dis_count + 1
                        ball_dis_start = True
                        hole_angle_start = False

                    # ball_dis_start = True
                    # hole_angle_start = False
                if hole_Angle >0:
                    # angle
                    if hole_Angle < 86:
                        if hole_Angle <= 82:
                            if Chest_ball_y > 480:
                                print("409L4 需要后挪一点 Back2Run ",Chest_ball_y)
                                action_append("Back2Run")
                                angle_dis_count = 0
                            elif Chest_ball_y < 350:
                                print("427L4 需要前挪一点 Forwalk00 ",Chest_ball_y)
                                action_append("Forwalk00")
                                angle_dis_count = 0

                            print("250L4 大右转一下 turn004R ", hole_Angle)
                            action_append("turn004R")
                        else:
                            if Chest_ball_y > 485:
                                print("421L4 需要后挪一点 Back1Run ",Chest_ball_y)
                                action_append("Back1Run")
                                angle_dis_count = 0
                            elif Chest_ball_y < 350:
                                print("427L4 需要前挪一点 Forwalk00 ",Chest_ball_y)
                                action_append("Forwalk00")
                                angle_dis_count = 0

                            print("352L4 右转一下 turn001R ", hole_Angle)
                            action_append("turn001R")
                    else:
                        print("417L4 hole_Angle---角度OK")         
                        angle_dis_count = angle_dis_count + 1
                        ball_dis_start = True
                        hole_angle_start = False

                    # ball_dis_start = True
                    # hole_angle_start = False

                if angle_dis_count > 3:
                    angle_dis_count = 0
                    print("step step 5555")
                    step = 5
                    

        elif step == 5:  # 调整 球与球洞在一条直线    球范围  230<Chest_ball_y<250
            # print("55555 球与球洞都在")
            # print("调整红球在左脚正前方不远处，看球洞的位置调整")
            if ball_dis_start:  # 390<y<450  230<x<250
                if Chest_ball_x < 220:
                    # if 240 - Chest_ball_x > 40:
                    #     print("443L 需要左侧移 Left02move")
                    #     action_append("Left02move")
                    # else:
                    print("446L 需要左侧移 Left1move", Chest_ball_x)
                    action_append("Left1move")
                    angle_dis_count = 0
                elif Chest_ball_x > 260:
                    # if Chest_ball_x - 240 > 40:
                    #     print("451L 需要右侧移 Right02move")
                    #     action_append("Right02move")
                    # else:
                    print("454L 需要右侧移 Right1move", Chest_ball_x)
                    action_append("Right1move")
                    angle_dis_count = 0
                else:
                    print("340L Chest_ball_y---位置ok")
                    ball_dis_start = False
                    hole_angle_start = True
            if hole_angle_start:
                if hole_Angle <0:
                    # angle
                    if hole_Angle > -87:
                        # y
                        if Chest_ball_y > 485:
                            print("475L 需要后挪一点 Back1Run ",Chest_ball_y)
                            action_append("Back1Run")
                            angle_dis_count = 0
                        elif Chest_ball_y < 390:
                            print("368L 需要前挪一点 Forwalk00",Chest_ball_y)
                            action_append("Forwalk00")
                            angle_dis_count = 0

                        if hole_Angle >= -82:
                            print("465L 大左转一下  turn001L ", hole_Angle)
                            action_append("turn001L")
                        else:
                            print("468L 左转一下  turn001L ", hole_Angle)
                            action_append("turn001L")
                    else:
                        print("471L hole_Angle---角度ok")
                        angle_dis_count = angle_dis_count + 1

                    ball_dis_start = True
                    hole_angle_start = False
                if hole_Angle >0:
                    # angle
                    if hole_Angle < 87:
                        # y
                        if Chest_ball_y > 485:
                            print("475L 需要后挪一点 Back1Run ",Chest_ball_y)
                            action_append("Back1Run")
                            angle_dis_count = 0
                        elif Chest_ball_y < 390:
                            print("368L 需要前挪一点 Forwalk00 ",Chest_ball_y)
                            action_append("Forwalk00")
                            angle_dis_count = 0

                        if hole_Angle <= 82:
                            print("479L 大右转一下 turn001R ", hole_Angle)
                            action_append("turn001R")
                        else:
                            print("482L 右转一下 turn001R ", hole_Angle)
                            action_append("turn001R")
                    else:
                        print("485L hole_Angle---角度OK")               
                        angle_dis_count = angle_dis_count + 1

                    ball_dis_start = True
                    hole_angle_start = False

                if angle_dis_count > 2:
                    angle_dis_count = 0
                    step = 6


        elif step == 6:
            # print("666")
            if Chest_ball_angle > 88 and hole_Angle > 88:
                ball_hole_angle_ok = True
            if Chest_ball_angle < -88 and hole_Angle > 88:
                ball_hole_angle_ok = True
            if Chest_ball_angle < -88 and hole_Angle < -88:
                ball_hole_angle_ok = True
            if Chest_ball_angle > 88 and hole_Angle < -88:
                ball_hole_angle_ok = True

            if Chest_ball_angle > 86 and hole_Angle > 86 and ball_hole_angle_ok == False:
                print("391L 右转一点点 turn001R")
                action_append("turn001R")
            elif Chest_ball_angle < -86 and hole_Angle < -86 and ball_hole_angle_ok == False:
                print("393L 左转一点点 turn001L")
                action_append("turn001L")
            elif Chest_ball_y <= 470:
                print("289L 向前挪动一点点 Forwalk00")
                action_append("Forwalk00")
            else:
                print("next step")
                step = 7

        elif step == 7:
            if Chest_ball_x > 200:  # 210
                print("410L 向右移动 Right1move")
                action_append("Right1move")
            elif Chest_ball_x < 180:
                print("412L 向左移动 Left1move")
                action_append("Left1move")
            elif Chest_ball_y < 490:
                print("289L 向前挪动一点点 Forwalk00")
                action_append("Forwalk00")
            elif Chest_ball_y > 530:
                print("2244L 向后挪动一点点 Back0Run")
                action_append("Back0Run")
            else:
                print("414L 踢球踢球 LfootShot")
                action_append("LfootShot")
                step = 8
                print("完成 77777")
                
                action_append("forwardSlow0403")
                action_append("forwardSlow0403")
                action_append("forwardSlow0403")
                action_append("Stand")

                # action_append("turn004L")
                # action_append("turn004L")
                # action_append("turn004L")
                # action_append("turn004L")
                # action_append("turn004L")
                # action_append("turn004L")
                # action_append("turn004L")
                # action_append("turn004L")
                # action_append("turn004L")
                # action_append("turn004L")
                
                action_append("turn005L")
                action_append("turn005L")
                action_append("turn005L")
                action_append("turn005L")
                

                cv2.destroyAllWindows()
                

 
            


def kick_ball():
    global state, state_sel, step, reset, skip
    global hole_Angle
    global golf_angle_ball, golf_angle ,Chest_ball_angle, Chest_golf_angle
    global ball_x, ball_y ,Chest_ball_x, Chest_ball_y 
    global hole_flag ,Chest_ball_flag
    global ChestOrg_img
    global picnum,img_debug

    # 初始化
    sum_contours = np.array([[[0, 0]], [[0, 1]], [[1, 1]], [[1, 0]]])
    step = 0
    state = 7

    while state == 7:
        if 0<=step < 8: #踢球的七步
            
            ChestOrg = ChestOrg_img.copy()
            ChestOrg=np.rot90(ChestOrg)
            
            Hole_OrgFrame = ChestOrg.copy()
            Ball_OrgFrame = ChestOrg.copy()

            img_h, img_w = Hole_OrgFrame.shape[:2]

            # 把上中心点和下中心点200改为640/2  fftest
            bottom_center = (int(240), int(img_h))  #图像底中点
            top_center = (int(240), int(0))     #图像顶中点
            # bottom_center = (int(640/2), int(img_h))  #图像底中点
            # top_center = (int(640/2), int(0))     #图像顶中点

         # 开始处理图像
            Hole_hsv = cv2.cvtColor(Hole_OrgFrame, cv2.COLOR_BGR2HSV)
            Hole_hsv= cv2.GaussianBlur(Hole_hsv, (3, 3), 0)

            Hole_Imask = cv2.inRange(Hole_hsv, color_dist['blue_hole']['Lower'], color_dist['blue_hole']['Upper'])
            Hole_Imask = cv2.erode(Hole_Imask, None, iterations=1)
            Hole_Imask = cv2.dilate(Hole_Imask, np.ones((3, 3), np.uint8), iterations=3)

            
            # cv2.imshow('hole_mask', Hole_Imask)      # hole mask
            # print('Press a key to continue:')
            # cv2.waitKey(0)


            # 初始化
            hole_center = [0, 0]
            Chest_ball_center = [0, 0]


          # chest 球洞处理
            hole_x = 0
            hole_y = 0

            _, cnts, hierachy = cv2.findContours(Hole_Imask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)    #**获得图片轮廓值  #遍历图像层级关系
            # *取得一个球洞的轮廓*
            for i in range(0, len(cnts)):               # 初始化sum_contours，使其等于其中一个c，便于之后拼接的格式统一
                area = cv2.contourArea(cnts[i])         #计算轮廓面积
                # print("area : ",area)
                if img_debug:
                    cv2.putText(Hole_OrgFrame, "area:" + str(area),(10, Hole_OrgFrame.shape[0] - 55), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 255), 2)
                    cv2.waitKey(1)
                if 640 * 480 * 0.0005 < area < 640 * 480 * 0.45:  # 去掉很小的干扰轮廓以及最大的图像边界
                    # cv2.drawContours(Hole_OrgFrame, cnts, -1, (0, 255, 0), 3)
                    sum_contours = cnts[i]
                    break
                else:
                    # cv2.drawContours(Hole_OrgFrame, cnts, -1, (0, 0, 255), 3)
                    continue
            for c in cnts:
                area = cv2.contourArea(c)                                       #计算轮廓面积
                if 640 * 480 * 0.0005 < area < 640 * 480 * 0.45:
                    sum_contours = np.concatenate((sum_contours, c), axis=0)    #数组拼接
                    cv2.drawContours(Hole_OrgFrame, c, -1, (0, 255, 0), 3)
                else:
                    cv2.drawContours(Hole_OrgFrame, c, -1, (0, 0, 255), 3)
                    continue
            sum_area = cv2.contourArea(sum_contours)    #计算轮廓面积
            if sum_area > 3:
                cnt_large = sum_contours
            else:
                cnt_large = None

            if cnt_large is not None:
                hole_flag = True
                (hole_x, hole_y), radius = cv2.minEnclosingCircle(cnt_large)    #最小内接圆形
                hole_center = (int(hole_x), int(hole_y))
                radius = int(radius)
                cv2.circle(Hole_OrgFrame, hole_center, radius, (100, 200, 30), 2)
                # ellipse = cv2.fitEllipse(cnt_large)
                # cv2.ellipse(OrgFrame,ellipse,(255,255,0),2)
                cv2.line(Hole_OrgFrame, hole_center, bottom_center, (0, 0, 100), 2)
                if (hole_center[0] - bottom_center[0]) == 0:
                    hole_Angle = 90
                else:
                    # hole_Angle  (y1-y0)/(x1-x0)
                    hole_Angle = - math.atan((hole_center[1] - bottom_center[1]) / (hole_center[0] - bottom_center[0])) * 180.0 / math.pi
            else:
                hole_flag = False

            if img_debug:
                cv2.putText(Hole_OrgFrame, "step:" + str(step),
                            (10, Hole_OrgFrame.shape[0] - 35), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 255), 2)
                cv2.putText(Hole_OrgFrame, "hole_angle:" + str(hole_Angle),
                            (10, Hole_OrgFrame.shape[0] - 115), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 255), 2)
                cv2.putText(Hole_OrgFrame, "hole_x:" + str(hole_x),
                            (10, Hole_OrgFrame.shape[0] - 75), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 255), 2)
                cv2.putText(Hole_OrgFrame, "hole_y:" + str(hole_y),
                            (220, Hole_OrgFrame.shape[0] - 75), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 255), 2)
                cv2.putText(Hole_OrgFrame, "hole_flag:" + str(hole_flag),
                            (10, Hole_OrgFrame.shape[0] - 95), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 255), 2)

                cv2.imshow("Hole_OrgFrame", Hole_OrgFrame)
                cv2.waitKey(10)


          # chest 红球处理
            Chest_ball_x = 0
            Chest_ball_y = 0


            Chest_Ball_hsv = cv2.cvtColor(Ball_OrgFrame, cv2.COLOR_BGR2HSV)
            Chest_Ball_hsv = cv2.GaussianBlur(Chest_Ball_hsv, (3, 3), 0)
            
            Chest_Ball_Imask = cv2.inRange(Chest_Ball_hsv, color_dist['ball_red']['Lower'], color_dist['ball_red']['Upper'])
            Chest_Ball_Imask = cv2.erode(Chest_Ball_Imask, None, iterations=2)
            Chest_Ball_Imask = cv2.dilate(Chest_Ball_Imask, np.ones((3, 3), np.uint8), iterations=2)
            
            # cv2.imshow('ball_mask', Chest_Ball_Imask)    # ball mask
            # print('Press a key to continue:')
            # cv2.waitKey(0)

            _, cnts2, hierachy2 = cv2.findContours(Chest_Ball_Imask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if cnts2 is not None:
                cnt_large3 = getAreaMaxContour2(cnts2, 10)
            else:
                print("1135L cnt_large is None")
                continue
            
            # 圆球轮廓  计算角度 Chest_ball_angle
            if cnt_large3 is not None:
                Chest_ball_flag = True
                (Chest_circle_x, Chest_circle_y), Chest_radius = cv2.minEnclosingCircle(cnt_large3)
                Chest_ball_center = (int(Chest_circle_x), int(Chest_circle_y))
                Chest_radius = int(Chest_radius)
                cv2.circle(Ball_OrgFrame, Chest_ball_center, Chest_radius, (100, 200, 20), 2)
                cv2.line(Ball_OrgFrame, Chest_ball_center, top_center, (0, 100, 0), 2)
                # ellipse = cv2.fitEllipse(cnt_large)
                # cv2.ellipse(OrgFrame,ellipse,(255,255,0),2)
                if (Chest_ball_center[0] - top_center[0]) == 0:
                    Chest_ball_angle = 90
                else:
                    # *Chest_ball_angle*  (y1-y0)/(x1-x0)
                    Chest_ball_angle = - math.atan((Chest_ball_center[1] - top_center[1]) / (Chest_ball_center[0] - top_center[0])) * 180.0 / math.pi
                Chest_ball_x = int(Chest_circle_x)   # *ball_x*
                Chest_ball_y = int(Chest_circle_y) # *ball_y*
            else:
                Chest_ball_flag = False
                Chest_ball_y = 0

            if img_debug:
                cv2.putText(Ball_OrgFrame, "step:" + str(step),
                            (10, Ball_OrgFrame.shape[0] - 35), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 255), 2)
                cv2.putText(Ball_OrgFrame, "Chest_ball_x:" + str(Chest_ball_x),
                            (10, Ball_OrgFrame.shape[0] - 75), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 255), 2)
                cv2.putText(Ball_OrgFrame, "Chest_ball_y:" + str(Chest_ball_y),
                            (220, Ball_OrgFrame.shape[0] - 75), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 255), 2)
                cv2.putText(Ball_OrgFrame, "Chest_ball_flag:" + str(Chest_ball_flag),
                            (10, Hole_OrgFrame.shape[0] - 95), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 255), 2)
                cv2.putText(Ball_OrgFrame, "ball_angle:" + str(Chest_ball_angle),
                            (10, Ball_OrgFrame.shape[0] - 115), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 255), 2)

                cv2.imshow("Ball_OrgFrame", Ball_OrgFrame)
                cv2.waitKey(10)
            # if img_debug:
            #     picnum += 1
            #     picname = 'photo_save' + str(picnum) + '.bmp'
            #     cv2.imwrite(picname,Ball_OrgFrame) #保存图片

        else:
            break


        act_move()




##################################################第八关：过洞###############################################
def square_hole():
    global ChestOrg_copy, state, state_sel, step, reset, skip, debug
    
    r_w = chest_r_width
    r_h = chest_r_height
    state_sel = 'hole'

    step = 0
    state = 8
    while state == 8:  # 初始化
        
     # 开始处理图像
        if True:   # head发现黄色区域
            t1 = cv2.getTickCount()

            HeadOrg_copy = HeadOrg_img.copy()
            ChestOrg_copy=np.rot90(ChestOrg_img)
            ChestOrg_copy = ChestOrg_copy.copy()
            cv2.rectangle(ChestOrg_copy,(0,580),(480,640),(255,255,255),-1)     # 底部涂白，遮盖双脚

            handling = cv2.resize(ChestOrg_copy, (r_w, r_h), interpolation=cv2.INTER_CUBIC)  # 将图片缩放
            frame_gauss = cv2.GaussianBlur(handling, (3, 3), 0)         # 高斯模糊
            frame_hsv = cv2.cvtColor(frame_gauss, cv2.COLOR_BGR2HSV)    # 将图片转换到HSV空间

         # yellow  yellow   Yellow_
            frame = cv2.inRange(frame_hsv, color_range['yellow_hole'][0],color_range['yellow_hole'][1])  # 对原图像和掩模(颜色的字典)进行位运算
            opened = cv2.morphologyEx(frame, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))     # 开运算 去噪点
            closed = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, np.ones((9, 9), np.uint8))   # 闭运算 封闭连接
            (_, contours, hierarchy) = cv2.findContours(closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)  # 找出轮廓
            cv2.drawContours(handling, contours, 0, (0, 255, 0), 3)   # 画出轮廓fftest
            area_sum = getAreaSumContour(contours)
            percent = round(100 * area_sum / (r_w * r_h), 2)
            
            areaMaxContour, area_max = getAreaMaxContour1(contours)  # 找出最大轮廓
            if areaMaxContour is not None:
                Yellow_bottom_left = areaMaxContour[0][0]  #初始化
                Yellow_bottom_right = areaMaxContour[0][0] #初始化
                Yellow_top_right = areaMaxContour[0][0]  # 右上角点坐标
                Yellow_top_left = areaMaxContour[0][0]  # 左上角点坐标

                for c in areaMaxContour:  # 遍历找到四个顶
                    # 底边两顶点
                    if c[0][0] + 1 * (r_h - c[0][1]) < Yellow_bottom_left[0] + 1.* (r_h - Yellow_bottom_left[1]):
                        Yellow_bottom_left = c[0]
                    if c[0][0] + 1 * c[0][1] > Yellow_bottom_right[0] + 1 * Yellow_bottom_right[1]:
                        Yellow_bottom_right = c[0]
                    
                    # 上边两定点
                    if c[0][0] + 1.5 * c[0][1] < Yellow_top_left[0] + 1.5 * Yellow_top_left[1]:
                        Yellow_top_left = c[0]
                    if (r_w - c[0][0]) + 1.5 * c[0][1] < (r_w - Yellow_top_right[0]) + 1.5 * Yellow_top_right[1]:
                        Yellow_top_right = c[0]
                
                Yellow_angle_bottom = - math.atan((Yellow_bottom_right[1] - Yellow_bottom_left[1]) / (Yellow_bottom_right[0] - Yellow_bottom_left[0])) * 180.0 / math.pi
                Yellow_angle_top = - math.atan((Yellow_top_right[1] - Yellow_top_left[1]) / (Yellow_top_right[0] - Yellow_top_left[0])) * 180.0 / math.pi
                Yellow_bottom_center_x = int((Yellow_bottom_right[0] + Yellow_bottom_left[0]) / 2)
                Yellow_bottom_center_y = int((Yellow_bottom_right[1] + Yellow_bottom_left[1]) / 2)

                # Yellow_top_center_x = int((Yellow_top_right[0] + Yellow_top_left[0]) / 2)
                # Yellow_top_center_y = int((Yellow_top_right[1] + Yellow_top_left[1]) / 2)
                if img_debug:
                    cv2.circle(handling, (Yellow_bottom_right[0], Yellow_bottom_right[1]), 5, [0, 255, 255], 2)
                    cv2.circle(handling, (Yellow_bottom_left[0], Yellow_bottom_left[1]), 5, [0, 255, 255], 2)
                    cv2.circle(handling, (Yellow_bottom_center_x, Yellow_bottom_center_y), 5, [0, 0, 255], 2)

                    # cv2.circle(handling, (Yellow_top_right[0], Yellow_top_right[1]), 5, [255, 0, 255], 2)
                    # cv2.circle(handling, (Yellow_top_left[0], Yellow_top_left[1]), 5, [255, 0, 255], 2)
                    # cv2.circle(handling, (Yellow_top_center_x, Yellow_top_center_y), 5, [255, 0, 0], 2)

            else:
                Yellow_angle_bottom = 0
                # Yellow_angle_top = 0
                Yellow_bottom_center_x = 0.5 * r_w
                Yellow_bottom_center_y = 0


         # black  black  black  black   Black_
            Black_angle_bottom = 0
            Black_bottom_center_x = 0
            Black_bottom_center_y = 0
            frame = cv2.inRange(frame_hsv, color_range['black_hole'][0],color_range['black_hole'][1])  # 对原图像和掩模(颜色的字典)进行位运算
            opened = cv2.morphologyEx(frame, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))  # 开运算 去噪点
            closed = cv2.morphologyEx(frame, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))  # 闭运算 封闭连接

            (_, contours, hierarchy) = cv2.findContours(closed, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)  # 找出轮廓cv2.CHAIN_APPROX_NONE
            
            # areaMaxContour, area_max = getAreaMaxContour1(contours)  # 找出最大轮廓
            areaMaxContour = getSumContour(contours,100)  # 得到总轮廓
            if areaMaxContour is not None:
                Black_bottom_left = areaMaxContour[0][0]  #初始化
                Black_bottom_right = areaMaxContour[0][0] #初始化

                for c in areaMaxContour:  # 遍历找到四个顶
                    # 底边两顶点
                    if c[0][0] + 1 * (r_h - c[0][1]) < Black_bottom_left[0] + 1.* (r_h - Black_bottom_left[1]):
                        Black_bottom_left = c[0]
                    if c[0][0] + 1 * c[0][1] > Black_bottom_right[0] + 1 * Black_bottom_right[1]:
                        Black_bottom_right = c[0]

                Black_angle_bottom = - math.atan((Black_bottom_right[1] - Black_bottom_left[1]) / (Black_bottom_right[0] - Black_bottom_left[0])) * 180.0 / math.pi
                Black_bottom_center_x = (Black_bottom_right[0] + Black_bottom_left[0]) / 2
                Black_bottom_center_y = (Black_bottom_right[1] + Black_bottom_left[1]) / 2
                rect = cv2.minAreaRect(areaMaxContour)
                # center, w_h, angle = rect  # 中心点 宽高 旋转角度
                box = np.int0(cv2.boxPoints(rect))  # 点的坐标
                if img_debug:
                    cv2.circle(handling, (Black_bottom_right[0], Black_bottom_right[1]), 5, [0, 255, 255], 2)
                    cv2.circle(handling, (Black_bottom_left[0], Black_bottom_left[1]), 5, [0, 255, 255], 2)
                    cv2.circle(handling, (int(Black_bottom_center_x), int(Black_bottom_center_y)), 5, [255, 0, 0], 2)
                    cv2.drawContours(handling, [box], 0, (0, 0, 255), 2)  # 将大矩形画在图上
            else:#没有识别到黑方洞
                angle = 0
                Black_center_x = 0.5*r_w
                Black_center_y = 0


            if img_debug:
                t2 = cv2.getTickCount()
                time_r = (t2 - t1) / cv2.getTickFrequency()
                fps = 1.0 / time_r
                cv2.putText(handling, "step:" + str(step), (30, 440), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 0), 2)  # (0, 0, 255)BGR
                cv2.putText(handling, "fps:" + str(int(fps)), (30, 460), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 0), 2)  # (0, 0, 255)BGR
                cv2.putText(handling, 'area: ' + str(percent) + '%', (30, 420), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 2)

                cv2.putText(handling, 'Yellow_center(x,y): ' + str(Yellow_bottom_center_x) + ', ' + str(Yellow_bottom_center_y), (30, 400), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 2)
                cv2.putText(handling, 'Black_center(x,y): ' + str(Black_bottom_center_x) + ', ' + str(Black_bottom_center_y), (30, 380), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 2)
                cv2.putText(handling, 'Yellow_angle_bottom: ' + str(Yellow_angle_bottom) , (30, 350), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 2)
                cv2.putText(handling, 'Black_angle_bottom: ' + str(Black_angle_bottom) , (30, 330), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 2)
                cv2.drawContours(handling, contours, -1, (255, 0, 255), 3)
                
                cv2.imshow('handling', handling)  # 显示图像
                
                # cv2.imshow('yellow_mask', closed)  # 显示图像
                # cv2.imshow('black_mask', closed)  # 显示图像
                # cv2.imshow('Head_Camera', HeadOrg_copy)  # 显示图像
                k = cv2.waitKey(10)
                if k == 27:
                    cv2.destroyWindow('closed_pic')
                    cv2.destroyWindow('org_img_copy')
                    break
                elif k == ord('s'):
                    print("save picture123")
                    cv2.imwrite("picture123.jpg",HeadOrg_copy) #保存图片


        if step == 0:   # 大步2前进 黄色下底边调整角度  调整好角度后继续大步前进
            if Black_bottom_center_y > 340:    # 黑色方框
                print("2621L step 111")
                step = 1
            if Yellow_bottom_center_y < 320:
                print("2621L head_angle_dis ",Yellow_bottom_center_y)
                head_angle_dis()    # headfftest
                print("226L 快走前进 fastForward05",Yellow_bottom_center_y)
                action_append("fastForward05")
                # print("313L 前进33333")
                # action_append( "forwardSlow0403")
                # action_append( "forwardSlow0403")
                # action_append( "forwardSlow0403")
            elif Yellow_bottom_center_y < 440:
                print("2630L head_angle_dis ",Yellow_bottom_center_y)
                head_angle_dis()    # headfftest
                print("2585L 快走前进 fastForward03 ",Yellow_bottom_center_y)
                action_append("fastForward03")
            else: # Yellow_bottom_center_y >= 440
                if Yellow_angle_bottom < -4:
                    if Yellow_angle_bottom < -10:
                        print(Yellow_angle_bottom, " Yellow_angle_bottom < -10   向右转动 turn001R")
                        action_append( "turn001R")
                    else:
                        print(Yellow_angle_bottom, " Yellow_angle_bottom < -2   向右转动")
                        action_append( "turn001R")
                elif Yellow_angle_bottom > 4:
                    if Yellow_angle_bottom > 10:
                        print(Yellow_angle_bottom, " Yellow_angle_bottom > 10  向左转动 turn001L")
                        action_append( "turn001L")
                    else:
                        print(Yellow_angle_bottom, " Yellow_angle_bottom > 2  向左转动")
                        action_append( "turn001L")
                elif -4 <= Yellow_angle_bottom <= 4:
                    # 调整居中角度
                    # if Yellow_bottom_center_x < 200:
                    #     print("355L 向左移动 Left02move")
                    #     action_append( "Left02move")
                    # elif Yellow_bottom_center_x > 280:
                    #     print("358L 向右移动 Right02move")
                    #     action_append( "Right02move")
                    # el
                    if Yellow_bottom_center_y < 360:
                        print("349L Yellow_bottom_center_y < 66% 前进")
                        action_append( "forwardSlow0403")
                    else:
                        print("2665L step goto 11111")
                        step = 1
        
        elif step == 1: # 依据yellow调整左右位置  到接触黑色
            if Black_bottom_center_y > 340:    # 黑色方框
                print("ste 222")
                step = 2
            elif Yellow_bottom_center_x < 200:
                print("369L 向左移动 Left02move")
                action_append( "Left02move")
            elif Yellow_bottom_center_x > 280:
                print("372L 向右移动 Right02move")
                action_append( "Right02move")
            else:
                print("363L 继续前进")
                action_append( "forwardSlow0403")
        
        elif step == 2:  # 看黑色下顶点,调整方向和位置   
            # angle
            if Black_angle_bottom < -2.0:  # 右转
                if Black_angle_bottom < -6.0:  # 大右转
                    print("386L  Black_angle_bottom < -6 右转 turn001R ",Black_angle_bottom)
                    action_append( "turn001R")
                elif Black_angle_bottom < -6.0:  # 大右转
                    print("386L  Black_angle_bottom < -4 右转 turn001R ",Black_angle_bottom)
                    action_append( "turn001R")
                else:
                    print("389L  Black_angle_bottom < -2 右转 turn000R ",Black_angle_bottom)
                    action_append( "turn000R")

                # time.sleep(1)   # timefftest
            elif Black_angle_bottom > 2.0:  # 左转
                if Black_angle_bottom > 6.0:  # 大左转
                    print("393L  Black_angle_bottom > 6 大左转 turn001L ",Black_angle_bottom)
                    action_append( "turn001L")
                elif Black_angle_bottom > 6.0:  # 大左转
                    print("393L  Black_angle_bottom > 4 大左转 turn001L ",Black_angle_bottom)
                    action_append( "turn001L")
                else:
                    print("2709L  Black_angle_bottom > 2 左转 turn000L ",Black_angle_bottom)
                    action_append( "turn000L")

                # time.sleep(1)   # timefftest
            # x 225 # 240
            elif Black_bottom_center_x > 235:   # 小右移 249.6
                if Black_bottom_center_x > 245:   # 大右移 
                    print("389L  Black_bottom_center_x > 0.54 大右移 Right02move ",Black_bottom_center_x)
                    action_append( "Right02move")
                else:
                    print("392L  Black_bottom_center_x > 0.54 小右移 Right1move ",Black_bottom_center_x)
                    action_append( "Right1move")
            elif Black_bottom_center_x < 215: # 小左移 230.4
                if Black_bottom_center_x < 205: # 大左移 
                    print("2722L Black_bottom_center_x < 0.48 大左移 Left02move ",Black_bottom_center_x)
                    action_append( "Left02move")
                else:
                    print("399L Black_bottom_center_x < 0.48 小左移 Left1move ",Black_bottom_center_x)
                    action_append( "Left1move")
            # y
            elif Black_bottom_center_y > 480:
                print("step 33333")
                step = 3
            elif Black_bottom_center_y <= 430:
                print("383L 继续接近黑线,请稍微往前挪动 forwardSlow0403")    # <480 Forwalk01
                action_append( "forwardSlow0403")
            elif Black_bottom_center_y <= 450:
                print("383L 继续接近黑线,请稍微往前挪动 Forwalk01")    # Forwalk01
                action_append( "Forwalk01")
            else:
                print("386L 继续接近黑线,请稍微往前挪动 Forwalk00")
                action_append( "Forwalk00")

        elif step == 3: # 依据黑线调整完角度和位置后继续前进
            if Black_bottom_center_y < 500:  # 小步前挪
                print("2627L  Black_bottom_center_y < 500 小步前挪 Forwalk01")
                action_append( "Forwalk01")
            elif Black_bottom_center_y < 560:  # 小步前挪
                print("2630L  Black_bottom_center_y < 560 小步前挪 Forwalk00")
                action_append( "Forwalk00")
            elif Black_bottom_center_y >= 580:
                print("417L  Black_bottom_center_y > 580 后退一点点 Back0Run")
                action_append("Back0Run")
            elif 500 <= Black_bottom_center_y <= 580 :
                print(" 趴地过关  ")
                
                action_append("Stand")
                action_append("lieForward")

                
                print("307 趴地过关  ")
                action_append( "Stand")
                action_append( "forwardSlow0403")
                action_append( "forwardSlow0403")
                action_append( "forwardSlow0403")
                action_append( "Stand")

                cv2.destroyAllWindows()
                break




#################################################第九关：沟壑###############################################
def gap_line():
    global org_img, state, step, reset, skip, debug , chest_ret

    r_w = chest_r_width
    r_h = chest_r_height

    step = 0
    state = 9
    while state == 9:
        
        # 分析图像
        t1 = cv2.getTickCount()
        
        if True:

            org_img = ChestOrg_img.copy()
            org_img = np.rot90(org_img)
            handling = org_img.copy()
            

            border = cv2.copyMakeBorder(org_img, 8, 8, 6, 6, borderType=cv2.BORDER_CONSTANT,value=(255, 255, 255))  # 扩展白边，防止边界无法识别
            org_img_copy = cv2.resize(border, (r_w, r_h), interpolation=cv2.INTER_CUBIC)  # 将图片缩放
            frame_gauss = cv2.GaussianBlur(org_img_copy, (3, 3), 0)  # 高斯模糊
            frame_hsv = cv2.cvtColor(frame_gauss, cv2.COLOR_BGR2HSV)  # 将图片转换到HSV空间
            frame = cv2.inRange(frame_hsv, color_range['black_gap'][0],color_range['black_gap'][1])  # 对原图像和掩模(颜色的字典)进行位运算
            closed = cv2.morphologyEx(frame, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))  # 闭运算 封闭连接
            opened = cv2.morphologyEx(frame, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))  # 开运算 去噪点

            (image, contours, hierarchy) = cv2.findContours(closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)  # 找出轮廓
            # print(len(contours))    # 轮廓个数

            
            
            # 拆分上下两个轮廓
            bottom_center = [0, 0]
            bottom_A = [0, 0]
            bottom_B = [0, 0]
            line_A__dot = [0, 0]
            line_B__dot = [0, 0]
            line_center_y = 0
            if contours is not None:
                for cn in range(len(contours)): # 遍历轮廓  轮廓cn
                    contour_area_temp = math.fabs(cv2.contourArea(contours[cn]))  # 计算轮廓面积
                    rect = cv2.minAreaRect(contours[cn])
                    box = np.int0(cv2.boxPoints(rect))  # 点的坐标
                    if contour_area_temp < 1000:
                        color_set = [0, 0, 255]
                        cv2.drawContours(handling, contours[cn], -1, (0, 0, 255), 1)    # 画出轮廓fftest
                        cv2.drawContours(handling,[box],0,color_set,2)                  # 画外界矩形
                        # 排除面积小轮廓，排除部分位置轮廓 fftest
                        continue
                    else:
                        color_set = [0, 255, 0]
                        cv2.drawContours(handling, contours[cn], -1, (0, 0, 255), 1)    # 画出轮廓fftest
                        cv2.drawContours(handling,[box],0,color_set,2)                  # 画外界矩形

                        dot_assemble =contours[cn][0]
                        old_dot = dot_assemble[0]
                        con_top_left = old_dot
                        con_top_right   = old_dot
                        con_bottom_left = old_dot
                        con_bottom_right   = old_dot 
                        for c in contours[cn]:  # 遍历轮廓contours[cn]的点
                            # 左上  右上
                            if math.pow(c[0][0] - 0, 2) + math.pow(c[0][1] - 0,2) < math.pow(con_top_left[0] - 0,2) + math.pow(con_top_left[1] - 0,2):
                                con_top_left = c[0]
                            if math.pow(r_w - c[0][0], 2) + math.pow(c[0][1] - 0,2) < math.pow(r_w - con_top_right[0],2) + math.pow(con_top_right[1] - 0,2):
                                con_top_right = c[0]

                            # 左下  右下
                            if math.pow(c[0][0] - 0, 2) + math.pow(r_h - c[0][1],2) < math.pow(con_bottom_left[0] - 0,2) + math.pow(r_h - con_bottom_left[1],2):
                                con_bottom_left = c[0]
                            if math.pow(r_w - c[0][0], 2) + math.pow(r_h - c[0][1],2) < math.pow(r_w - con_bottom_right[0],2) + math.pow(r_h - con_bottom_right[1],2):
                                con_bottom_right = c[0]
                            

                        if img_debug:
                            cv2.circle(org_img_copy, (con_top_left[0], con_top_left[1]), 5, [0, 0, 255], 2)
                            cv2.circle(org_img_copy, (con_bottom_left[0], con_bottom_left[1]), 5, [0, 255, 0], 2)
                            cv2.circle(org_img_copy, (con_bottom_right[0], con_bottom_right[1]), 5, [255, 0, 0], 2)
                            cv2.circle(org_img_copy, (con_top_right[0], con_top_right[1]), 5, [0, 255, 255], 2)
                            cv2.imshow('org_img_copy', org_img_copy)  # 显示图像
                            cv2.waitKey(1)
                 
                 # 轮廓外接矩形四个点和中心点
                    box_A = [box[0,0],box[0,1]]
                    box_B = [box[1,0],box[1,1]]
                    box_C = [box[2,0],box[2,1]]
                    box_D = [box[3,0],box[3,1]]
                    
                    # 外接矩形找出底边两个点,即y坐标排序  1 < 2 
                    if box_A[1]<box_B[1]:
                        box_bottom2 = box_B
                        box_bottom1 = box_A
                    else:
                        box_bottom2 = box_A
                        box_bottom1 = box_B

                    if box_bottom2[1]<box_C[1]:
                        box_bottom1 = box_bottom2
                        box_bottom2 = box_C
                    elif box_bottom1[1]<box_C[1]:
                        box_bottom2 = box_bottom2
                        box_bottom1 = box_C

                    if box_bottom2[1]<box_D[1]:
                        box_bottom1 = box_bottom2
                        box_bottom2 = box_D
                    elif box_bottom1[1]<box_D[1]:
                        box_bottom2 = box_bottom2
                        box_bottom1 = box_D

                    if box_bottom1[0] > box_bottom2[0]:
                        box_bottom1,box_bottom2 = box_bottom2,box_bottom1
                    
                    # box_center   去轮廓外接矩形底边中心y大的轮廓
                    if bottom_center[1] < (box_bottom1[1]+box_bottom2[1])/2:
                        bottom_center = [int((box_bottom1[0]+box_bottom2[0])/2),int((box_bottom1[1]+box_bottom2[1])/2)]
                        bottom_A = box_bottom2
                        bottom_B = box_bottom1
                        line_B__dot = con_top_right
                        line_A__dot = con_top_left
                        ABline_angle = - math.atan((line_B__dot[1] - line_A__dot[1]) / (line_B__dot[0] - line_A__dot[0])) * 180.0 / math.pi
                        bottom_percent = round(100 * contour_area_temp / (r_w * r_h), 2)
                        line_angle = - math.atan((bottom_B[1] - bottom_A[1]) / (bottom_B[0] - bottom_A[0])) * 180.0 / math.pi
                        line_center_y = bottom_center[1]
                    
                    
                    if img_debug:
                        cv2.circle(handling, (box_bottom1[0],box_bottom1[1]), 3, (0, 255, 255), 1) # 黄圆点标记
                        cv2.circle(handling, (box_bottom2[0],box_bottom2[1]), 3, (0, 255, 255), 1) # 黄圆点标记
                        cv2.circle(handling, (bottom_center[0] ,bottom_center[1]), 5, (255, 255, 255), -1) # 绿圆点标记
                        cv2.line(handling, (bottom_A[0],bottom_A[1]), (bottom_B[0],bottom_B[1]), (0, 255, 255), thickness=2)
                        cv2.putText(handling, 'line_angle: ' + str(round(line_angle, 2)), (30, 540), cv2.FONT_HERSHEY_SIMPLEX, 0.7,(0, 0, 255), 2)
                        cv2.putText(handling, 'line_center_y: ' + str(round(line_center_y, 2)), (30, 520), cv2.FONT_HERSHEY_SIMPLEX, 0.7,(0, 0, 255), 2)  # if center
                        cv2.putText(handling, 'bottom_percent: ' + str(bottom_percent) + '%', (30, 500), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255),2)
                        
                        cv2.line(org_img_copy, (line_A__dot[0],line_A__dot[1]), (line_B__dot[0],line_B__dot[1]), (0, 255, 255), thickness=2)
                        cv2.putText(org_img_copy, 'ABline_angle: ' + str(round(ABline_angle, 2)), (30, 540), cv2.FONT_HERSHEY_SIMPLEX, 0.7,(0, 0, 255), 2)
    
                        cv2.imshow('org_img_copy', org_img_copy)  # 显示图像
                        cv2.imshow('handling', handling)  # 显示图像
                        cv2.waitKey(1)

            else:
                angle_top = 0
                angle_bottom = 0
                top_center_y = 0
                line_center_y = 0




        
        # 决策执行动作
        if step == 0:   # 前进几步，然后调整方向
            # 已经侧边看齐

            print("318L 快走前进 fastForward05 ",line_center_y)
            action_append("fastForward05")
            step = 1
        
        elif step == 1: 
            if line_center_y <= 380 :
                print("339L 大步前进 forwardSlow0403 ",line_center_y)
                action_append("forwardSlow0403")
                # action_append("forwardSlow0403")
                # action_append("forwardSlow0403")
                # action_append("forwardSlow0403")
                # action_append("forwardSlow0403")
            elif line_center_y >= 480 and bottom_percent >= 2:
                print("gap 22222")
                step = 2
            else:
                if ABline_angle > 2:
                    if ABline_angle > 4.0:  # 大左转
                        print("336L  ABline_angle > 4 大左转 turn001L ",ABline_angle)
                        action_append( "turn001L")
                    else:
                        print("339L  ABline_angle > 2 左转 turn000L ",ABline_angle)
                        action_append( "turn000L")
                elif ABline_angle < -2:
                    if ABline_angle < -4.0:  # 大右转
                        print("340L  ABline_angle < -4 右转 turn001R ",ABline_angle)
                        action_append( "turn001R")
                    else:
                        print("343L  ABline_angle < -2 右转 turn000R ",ABline_angle)
                        action_append( "turn000R")
                elif -2 <= ABline_angle <= 2:
                    print(" -2 <= ABline_angle <= 2  前挪动，前挪动 ")
                    print("350L 已调整角度,大步前进 forwardSlow0403 ",line_center_y)
                    action_append("forwardSlow0403")
        elif step == 2: #看下顶点 小步挪
            if line_center_y < 590:
                if ABline_angle > 2:
                    if ABline_angle > 4.0:  # 大左转
                        print("336L  ABline_angle > 4 大左转 turn001L ",ABline_angle)
                        action_append( "turn001L")
                    else:
                        print("339L  ABline_angle > 2 左转 turn000L ",ABline_angle)
                        action_append( "turn000L")
                elif ABline_angle < -2:
                    if ABline_angle < -4.0:  # 大右转
                        print("340L  ABline_angle < -4 右转 turn001R ",ABline_angle)
                        action_append( "turn001R")
                    else:
                        print("343L  ABline_angle < -2 右转 turn000R ",ABline_angle)
                        action_append( "turn000R")
                elif -2 <= ABline_angle <= 2:
                    print("354L 向前挪动 Forwalk00")
                    action_append("Forwalk00")
            else:
                print("2987L 前挪一点 跨")
                action_append("Forwalk00")
                t1 = cv2.getTickCount()
                print("157 过去，下一关")
                
                action_append("Stand")
                action_append("acrossGap")
                
                t2 = cv2.getTickCount()
                time_r = (t2 - t1)
                print("across 花费时长 ",time_r)
                
                action_append( "Stand")
                action_append( "Forwalk01")

                step = 2
                
                cv2.destroyAllWindows()
                break



################################################第一关：起点#############################################
def start_door():
    global ChestOrg_img, state, state_sel, step, reset, skip, img_debug
    state_sel = 'door'
    state = 1
    if state == 1:  # 初始化
        print("/-/-/-/-/-/-/-/-/-进入start_door")
        step = 0
    else:
        return

    while state == 1 :
        if step == 0: #判断门是否抬起
            t1 = cv2.getTickCount() # 时间计算
            

            org_img_copy = ChestOrg_img.copy()
            org_img_copy = np.rot90(org_img_copy)
            handling = org_img_copy.copy()
            

            border = cv2.copyMakeBorder(handling, 12, 12, 16, 16, borderType=cv2.BORDER_CONSTANT,value=(255, 255, 255))     # 扩展白边，防止边界无法识别
            handling = cv2.resize(border, (chest_r_width, chest_r_height), interpolation=cv2.INTER_CUBIC)                   # 将图片缩放
            frame_gauss = cv2.GaussianBlur(handling, (21, 21), 0)       # 高斯模糊
            frame_hsv = cv2.cvtColor(frame_gauss, cv2.COLOR_BGR2HSV)    # 将图片转换到HSV空间
            
            frame_door_yellow = cv2.inRange(frame_hsv, color_range['yellow_door'][0], color_range['yellow_door'][1])    # 对原图像和掩模(颜色的字典)进行位运算
            frame_door_black = cv2.inRange(frame_hsv, color_range['black_door'][0], color_range['black_door'][1])       # 对原图像和掩模(颜色的字典)进行位运算


            frame_door = cv2.add(frame_door_yellow, frame_door_black)    
            open_pic = cv2.morphologyEx(frame_door, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))      # 开运算 去噪点
            closed_pic = cv2.morphologyEx(open_pic, cv2.MORPH_CLOSE, np.ones((50, 50), np.uint8))   # 闭运算 封闭连接
            # print(closed_pic)

            (image, contours, hierarchy) = cv2.findContours(closed_pic, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)  # 找出轮廓
            areaMaxContour, area_max = getAreaMaxContour1(contours)  # 找出最大轮廓
            percent = round(100 * area_max / (chest_r_width * chest_r_height), 2)  # 最大轮廓的百分比
            if areaMaxContour is not None:
                rect = cv2.minAreaRect(areaMaxContour)  # 矩形框选
                box = np.int0(cv2.boxPoints(rect))      # 点的坐标
                if img_debug:
                    cv2.drawContours(handling, [box], 0, (153, 200, 0), 2)  # 将最小外接矩形画在图上
            if img_debug:
                cv2.putText(handling, 'area: ' + str(percent) + '%', (30, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                t2 = cv2.getTickCount()
                time_r = (t2 - t1) / cv2.getTickFrequency()
                fps = 1.0 / time_r
                cv2.putText(handling, "fps:" + str(int(fps)), (30, 150), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 255), 2)
                cv2.imshow('handling', handling)  # 显示图像

                cv2.imshow('frame_door_yellow', frame_door_yellow)  # 显示图像
                cv2.imshow('frame_door_black', frame_door_black)    # 显示图像
                cv2.imshow('closed_pic', closed_pic)  # 显示图像

                k = cv2.waitKey(10)
                if k == 27:
                    cv2.destroyWindow('closed_pic')
                    cv2.destroyWindow('handling')
                    break
                elif k == ord('s'):
                    print("save picture123")
                    cv2.imwrite("picture123.jpg",org_img_copy) #保存图片



            # 根据比例得到是否前进的信息
            if percent > 1:    #检测到横杆
                print(percent,"%")
                print("有障碍 等待 contours len：",len(contours))
                time.sleep(0.1)
            else:
                print(percent)

                # print("231L 执行3步")
                # action_append("forwardSlow0403")
                # action_append("forwardSlow0403")
                # action_append("forwardSlow0403")

                print("231L 执行快走555")
                action_append("fastForward05")
                
                step = 1
                
        elif step == 1:  # 寻找下一关卡

            print("判断下一关是什么~~~~~~~~~~~~~~~~")
            if state_sel is not None:
                print('state_sel: %s' % state_sel)
                print("开启下一关")
                # action_append("forwardSlow0403")
                # action_append("Stand")

                step = 0
                cv2.destroyAllWindows()

                break
            else:
                print("直行0步")
                # action_append("forwardSlow0403")




if __name__ == '__main__':
    while len(CMDcontrol.action_list) > 0 :
        print("等待启动")
        time.sleep(1)
    action_append("HeadTurnMM")

    while True:
        if ChestOrg_img is not None and chest_ret:
            k = cv2.waitKey(10)
            if k == 27:
                cv2.destroyWindow('camera_test')
                break



            print("start door START")
            start_door()

            floor()     # 台阶

            obstacle()  # 地雷

            baffle()

            head_angle_dis()
            Greenbridge()


            # --------------------------------------------------

            head_angle_dis()
            kick_ball()

            head_angle_dis()
            print("3123L 快走前进 fastForward 3 3 3")
            action_append("fastForward05")

            head_angle_dis()
            square_hole()

            head_angle_dis()
            gap_line()

            print("end door end door ")
            start_door()



            while(1):
                print("结束")
                time.sleep(10000)
            
        else:
            print('image is empty chest_ret:',chest_ret)
            time.sleep(0.01)
            cv2.destroyAllWindows()

    
