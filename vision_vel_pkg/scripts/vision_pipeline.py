#!/usr/bin/env python3
# coding=utf-8
import cv2
import numpy as np
import rospy
from socket import *
from geometry_msgs.msg import Twist,PoseStamped
from qingzhou_cloud.msg import trafficlight
from std_msgs.msg import Float32
import time
import math
import threading
from queue import Queue
import subprocess
from actionlib_msgs.msg import GoalID


addr = ('255.255.255.255', 6666) 
s = socket(AF_INET, SOCK_DGRAM)
s.setsockopt(SOL_SOCKET, SO_BROADCAST, 1)
# 全体点拟合版
font = cv2.FONT_HERSHEY_DUPLEX  # 字体

K = np.array([[198.5257, 0, 153.7481],
              [0, 264.7291, 138.2082],
              [0, 0, 1.0000]], dtype=np.float32)  # 相机内参矩阵240

K1 = np.array([[289.6046 ,0 ,237.4473],    #480*360
                [0 , 385.5118 ,201.0799],
                [0,0 ,1.0000]],dtype=np.float32)
Dist1 = np.array([-0.3843, 0.1656,0.,0.,0.],dtype=np.float32)
Dist = np.array([-0.3468, 0.1280, 0., 0., 0.], dtype=np.float32)  # 相机畸变参数

H = np.array([[-0.1669762, -0.69926321, 122.65788084],
              [0.01309059, -0.12283827, -18.74868092],
              [0.00000088, -0.00710999, 1.]], dtype=np.float32)



# color range

kernel1 = np.ones((2, 2), np.uint8)
flag_s = False
flag_t = False
flag_z = False
t = 1
def gstreamer_pipeline(
    sensor_id=0,
    capture_width=480,
    capture_height=360,
    display_width=480,
    display_height=360,
    framerate=10,
    flip_method=0,
):
    return (
        "nvarguscamerasrc sensor-id=%d !"
        "video/x-raw(memory:NVMM), width=(int)%d, height=(int)%d, framerate=(fraction)%d/1 ! "
        "nvvidconv flip-method=%d ! "
        "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
        "videoconvert ! "
        "video/x-raw, format=(string)BGR ! appsink"
        % (
            sensor_id,
            capture_width,
            capture_height,
            framerate,
            flip_method,
            display_width,
            display_height,
        )
    )





def warp(img):
    """
    :param img: 输入图像
    :return: 逆透视之后的图像200x200
    """
    undist_img = cv2.undistort(img, K, Dist)      # 去畸变
    return undist_img

def warp1(img):
        warped_img = cv2.warpPerspective(img, H, (200, 200))
        return warped_img




#################################################################s弯调参######################################################################
##############################################################################################################################################
def get_xy(lane_x,lane_y):               #返回s弯偏移量

    # 多项式回归
    lane_mid = np.polyfit(lane_x, lane_y, deg=1)
    # lane_mid_function = np.poly1d(lane_mid)
    x_min = min(lane_x)
    x_max = max(lane_x)
    x_vals = np.linspace(x_min, x_max, 200)
    y_vals = np.polyval(lane_mid, x_vals)
    lane_mid_function = np.poly1d(lane_mid)
    p = (175-lane_mid[1])/lane_mid[0]
    #p = lane_mid_function(170)

    return p, lane_mid, x_vals, y_vals

left_fudu = 240   #左转幅度，越小转越大  低速：250     中高速：220
right_fudu = 350  #右转幅度                 ：380            330
center = 93  #中心位置                      ：93              93
canny_threshold1 = 100  #80
canny_threshold2 = 180
dianyabuchang = 1
#####################################################################################################################################################
def contour_select(contours,mask_green1,hsv):
    # 将轮廓进行筛选，可以根据大小、形状等要求进行选择
    roi_list = []
    mask = np.zeros_like(hsv)
    for index, contour in enumerate(contours):
        # 计算每个轮廓面积
        cv2.drawContours(mask, [contour], -1, (255, 255, 255), thickness=cv2.FILLED)
        contour_area_hsv = cv2.bitwise_and(hsv, mask)
        min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(contour_area_hsv[..., 2], mask[..., 0])
        hsv_h, s, v = contour_area_hsv[max_loc[1], max_loc[0]]
        Area = cv2.contourArea(contour)
        Hull = cv2.convexHull(contour, False)
        HullArea = cv2.contourArea(Hull)
        # 根据特定要求筛选ROI，例如大小、形状等
        x, y, w, h = cv2.boundingRect(contour)
        # if 500 >= Area >0 and HullArea <= 400 and 0.2 <= w/h <= 5 and v >= 200:    # 场景有绿色干扰物时
        # if 500 >= Area >10 and HullArea <= 500 and 0.5 <= w/h <= 2.5 :                 # 正常时
        # if 1000 >= Area > 0 :                                                        # 场景很干净时
        if v >= 180:
            roi_list.append(contour)
    if len(roi_list) != 0:
        flag = True
    else:
        flag = False

    return roi_list, flag




def ocr(text):
    if "航" in text and "天" in text or "天" in text or "夭" in text or "大" in text or "犬" in text or "关" in text or "尺" in text or "人" in text or "灭" in text or "无" in text:
        print("航天")
        wenzi = 1
    elif "三" in text or "院" in text or "二" in text or "阮" in text or "说" in text:
        print("三院")
        wenzi = 1
    elif "星" in text and "航" in text or "星" in text or "扯" in text or "埕"in text or "皇" in text or "阜" in text or "易" in text or "春" in text or "屈" in text or "吊" in text:
        print("星航")
        wenzi = 2
    elif "机" in text or "电" in text or "巳" in text or "枝" in text or "申" in text or "权" in text or "屯" in text or "由" in text:
        print("机电")
        wenzi = 2
    else:
        wenzi = 3
    return wenzi


class MovingAverageFilter:
    def __init__(self, window_size):
        self.window_size = window_size
        self.data_queue = Queue(maxsize=window_size)
        
    def moving_average(self, data):
        window = np.ones(self.window_size) / self.window_size
        return np.convolve(data, window, mode='same')
    
    def filter(self, data):
        self.data_queue.put(data)
        
        if self.data_queue.full():
            window_data = np.array(list(self.data_queue.queue))
            filtered_value = self.moving_average(window_data)
            filtered_data = filtered_value[self.window_size//2]
            
            self.data_queue.get()
        else:
            filtered_data = data
        
        return filtered_data

    def reset(self):
        self.data_queue.queue.clear()

#TODO:红绿灯参数     #bgr_mean
Red = np.array([49, 37, 147])  
Yellow = np.array([11, 81, 178.])
Green = np.array([44, 85, 22])
Colors = (Red, Yellow, Green)
ColorsName = ('Red', 'Yellow', 'Green')
DistThreshold = 1600  # 颜色距离阈值


def JudgeLightColor(Light):
    Dist = np.empty((0,))
    for Color in Colors:
        Dist = np.append(Dist, np.sum(abs(Color - Light) ** 2))
    return np.argmin(Dist), np.min(Dist)

def get_vel():
    vel_msg = Twist()
    trafficlight_msg = trafficlight()
    daoche_msg = trafficlight()
    i = 0
    limit = 5
    stop = 7
    wu = 0
    meiyou = 0
    global shibie_pub
    global daoche_pub
    global vel_pub  
    global trafficlight_pub
    global daoche_pub
    global flag_s     #s弯标志位
    global flag_t     #红绿灯标志位
    global flag_z     #文字识别标志位
    global t          #是否识别红绿灯标志位
    global dianyabuchang
    p = 0
    window_title = "CSI Camera"   #窗口名
    video_capture = cv2.VideoCapture(gstreamer_pipeline(flip_method=0), cv2.CAP_GSTREAMER)    #调用摄像头
    if video_capture.isOpened():
        try:
            # window_handle = cv2.namedWindow(window_title, cv2.WINDOW_AUTOSIZE)
            while True:
                time.sleep(0.002)
                ret_val, frame = video_capture.read()
                if ret_val:
                    img = cv2.resize(frame.copy(),(320,240))
                    image = warp(img)     #去畸变后的图像
####################################################S弯识别部分#########################################################################
#######################################################################################################################################
                    if flag_s:
                        print(i)       
                        kp = 2.86*dianyabuchang
                        cancel_pub =  rospy.Publisher("/move_base/cancel",GoalID,queue_size=10)
                        empty_goal = GoalID()
                        cancel_pub.publish(empty_goal)
                        img = cv2.resize(frame.copy(),(320,240))
                        img = warp(img)
                        img = warp1(img)
                        # img = cv2.GaussianBlur(img,(3,3),0)
                        # kernel = np.array([[-1, 0, 1],
                        #                    [-2, 0, 2],
                        #                    [-1, 0, 1]], dtype=np.float32)
                        # img = cv2.filter2D(frame, ddepth=cv2.CV_32F, kernel=kernel)
                        # img = cv2.convertScaleAbs(img)
                        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
                        contours_img = cv2.Canny(gray,canny_threshold1,canny_threshold2)
                        kernel = np.ones((2,2),np.uint8)
                        contours_img = cv2.dilate(contours_img,kernel,iterations=1)
                        contours_img = cv2.erode(contours_img,kernel,iterations=1)
                        contours_img = cv2.dilate(contours_img,kernel,iterations=1)
                        contours, hierarchy = cv2.findContours(contours_img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
                        target_contours = []
                        boxs = []
                        center_x = []
                        center_y = []
                        for contour in contours:
                            area = cv2.contourArea(contour)
                            rect = cv2.minAreaRect(contour)
                            Hull = cv2.convexHull(contour)
                            Hullarea = cv2.contourArea(Hull)
                            (x, y), (w, h), _ = rect
                            if 70 < area < 200 and 0.2<w/h <5 and area/Hullarea > 0.7:
                                target_contours.append(contour)
                                center_x.append(x)
                                center_y.append(y)
                        if len(target_contours) > 1:
                            result = cv2.drawContours(img.copy(), target_contours, -1, (0, 0, 255), 1)
                            pian, xishu, x_val, y_val = get_xy(center_x,center_y)
                            if pian <center:
                                pianyi = (pian-center)/left_fudu
                            elif pian >=center:
                                pianyi = (pian-center)/right_fudu
                            if pianyi >= 0.75:
                                vel_msg.linear.x = 1
                            elif pianyi <= -0.75:
                                vel_msg.linear.x = 1
                            else :
                                vel_msg.angular.z = -pianyi*kp #上午1.60
                                vel_msg.linear.x = 1
                            i = 0
                            points = []  # 获得曲线坐标
                            for index, x in enumerate(x_val):
                                points.append([x, y_val[index]])
                            points = np.array(points, np.int32) #上午1.60
                            print(pianyi,vel_msg.angular.z)
                            # vel_pub.publish(vel_msg)
                            result = cv2.polylines(result, [points], False, (0, 0, 255), 2)
                        elif len(target_contours) == 1:
                            vel_msg.linear.x = 1
                            # vel_pub.publish(vel_msg)
                            if i < 3:
                                i = 0
                            result = img
                            print('一个')
                        else:
                            if i < limit:
                                vel_msg.angular.z = -0.67
                            i = i + 1
                            # vel_pub.publish(vel_msg)
                            result = img
                        # if limit < i < stop:
                        #     vel_msg.angular.z = 0
                        #     # vel_pub.publish(vel_msg)
                        if  i > limit:
                            filter.reset()
                            print("滤波器重置成功!")
                            # flag_01=1
                            vel_msg.linear.x = 0.3
                            vel_msg.angular.z = 0
                            vel_pub.publish(vel_msg)
                            # vel_msg.linear.x = 0
                            # vel_msg.angular.z = 0
                            # vel_pub.publish(vel_msg)
                        if i > stop:
                            flag_s = False
                            # vel_msg.linear.x = 0.3
                            # vel_msg.angular.z = 0
                            # vel_pub.publish(vel_msg)
                            flag_z = True
                            i=0
                        window_size = 4
                        filter = MovingAverageFilter(window_size)
                        vel_msg.angular.z = filter.filter(vel_msg.angular.z)
                        vel_pub.publish(vel_msg)
                        image = result
#####################################################红绿灯识别部分########################################################################           
##########################################################################################################################################
                    if flag_t and t%2 !=0 :
            ################################# RGB法 ###################################
                        undist_img = cv2.undistort(frame, K1, Dist1)      # 去畸变
                        LightImg = undist_img     #去畸变后的图像
                        LightImgGray = cv2.cvtColor(LightImg, cv2.COLOR_BGR2GRAY)
                        th, MaskImg = cv2.threshold(LightImgGray, 200, 255, cv2.THRESH_TOZERO)
                        # lower_mask = np.array([0,0,201])
                        # upper_mask = np.array([180,255,255])
                        # MaskImg = cv2.inRange(img,lower_mask,upper_mask)
                        # MaskImg = cv2.morphologyEx(MaskImg, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
                        # MaskImg = cv2.morphologyEx(MaskImg, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))
                        # cv2.imshow("Mask",MaskImg)
                        contours, hierarchy = cv2.findContours(MaskImg, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
                        sel_contours = []
                        LightColors = []
                        number = 0
                        # 根据面积筛选轮廓
                        for index, contour in enumerate(contours):
                            Area = cv2.contourArea(contour)
                            Hull1 = cv2.convexHull(contour, False)
                            HullArea1 = cv2.contourArea(Hull1)
                            if 0 < Area < 1000 and Area / HullArea1 > 0.8:
                                number = number +1
                                sel_contours.append(contour)
                                # 形态学提取外轮廓区域
                                MaskImg = np.zeros_like(LightImgGray)
                                cv2.drawContours(MaskImg, [contour], -1, 255, cv2.FILLED)
                                kernel = np.ones((int(13), int(13)), np.uint8)
                                dilation = cv2.dilate(MaskImg, kernel, iterations=1)  # 膨胀
                                MaskImg = dilation - MaskImg
                                MaskImg = cv2.cvtColor(MaskImg, cv2.COLOR_GRAY2BGR)
                                OutSide = LightImg & MaskImg
                                Index = np.argwhere(np.sum(OutSide, axis=2) > 0)
                                GrayLevel = OutSide[Index[:, 0], Index[:, 1], :]
                                Light = np.mean(GrayLevel, axis=0)
                                Color, Dist = JudgeLightColor(Light)
                                if Dist < DistThreshold:  # 颜色空间L2距离足够小，完成颜色判断
                                    LightColors.append(Color)
                        # %% 显示交通灯小块区域
                        print("识别到亮点数量:"+str(number))
                        print(LightColors)
                        cv2.drawContours(LightImg, sel_contours, -1, (255, 0, 0), 3)
                        cv2.putText(LightImg, str([ColorsName[LightColor] for LightColor in LightColors]), (10, 80),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)
                        image = LightImg
                        if 0 in LightColors:
                            print("红灯")
                            trafficlight_msg.trafficstatus = 2   #红灯
                            trafficlight_pub.publish(trafficlight_msg)
                            wu = 0
                        else:
                            wu += 1
                        if 2 in LightColors or wu> 100:
                            t += 1
                            trafficlight_msg.trafficstatus = 1   #绿灯
                            print("绿灯")
                            trafficlight_pub.publish(trafficlight_msg)
    ############################################# HSV法 #############################################
                        # image = img
                        # kernel = np.ones((2,2),np.uint8)
                        # hsv = cv2.cvtColor(image,cv2.COLOR_BGR2HSV)
                        # lower_green = np.array([60,30,145])
                        # upper_green = np.array([78,222,255])
                        # mask_green1 = cv2.inRange(hsv,lower_green,upper_green)

                        # mask_green = cv2.erode(mask_green1,kernel,iterations=2)
                        # mask_green = cv2.dilate(mask_green,kernel,iterations=7)#绿色掩膜
                        # green_contours, _ = cv2.findContours(mask_green, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                        # if len(green_contours)!=0:
                        #     green_contours,flagg = contour_select(green_contours,mask_green1,hsv)
                        # else:
                        #     flagg = False
                        # # 绘制红色和绿色轮廓
                        # if flagg == False:
                        #     p += 1
                        # else:
                        #     p = 0
                        #     t += 1
                        # if p >= 5:
                        #     trafficlight_msg.trafficstatus = 2
                        #     trafficlight_pub.publish(trafficlight_msg)
                        # else:
                        #     trafficlight_msg.trafficstatus = 1
                        #     trafficlight_pub.publish(trafficlight_msg)
                        # cv2.drawContours(image, green_contours, -1, (0, 255, 0), 2)
                        # print(flagg)
            ####################################################################################################################
            ####################################################################################################################            
                    if flag_z == True:
                        undist_img = cv2.undistort(frame, K1, Dist1)      # 去畸变
                        img = undist_img[80:240,120:320]   #y ,x
                        cv2.imwrite("word.jpg", img)
                        time_start=time.time()
                        path = "./OcrLiteOnnx word.jpg ./ocr-lite-onnx/models 1.0 0.5 0.3 3 1.3 1.3 1.6 1.6"
                        ret = subprocess.getstatusoutput(path)
                        print(ret[1])
                        wen = ocr(ret[1])
                        time_end=time.time()
                        print('Running time:{} seconds'.format(time_start - time_end))                
                        if wen == 1:
                            daoche_msg.trafficstatus = 1
                            meiyou = 0
                            daoche_pub.publish(daoche_msg)
                            flag_z = False
                        elif wen == 2:
                            daoche_msg.trafficstatus = 0
                            meiyou = 0
                            flag_z = False
                            daoche_pub.publish(daoche_msg)
                        elif meiyou > 3:
                            daoche_msg.trafficstatus = 1
                            meiyou = 0
                            flag_z = False
                            daoche_pub.publish(daoche_msg)
                        else:
                            meiyou = meiyou + 1
                        image = warp(img)     #去畸变后的图像
                    _, send_data = cv2.imencode('.jpg', image,[cv2.IMWRITE_JPEG_QUALITY,50])
                    s.sendto(send_data, addr)
                    # cv2.imshow(window_title, image)
                    # if cv2.waitKey(1) & 0xFF == ord('q'):  # 按键盘Q键退出
                    #     break
#----------------------------------------------------------------------------------------------------------------------------------------#
##########################################################################################################################################
                else:
                    video_capture.release()
                    break 
        finally:
            vel_msg.linear.x = 0
            vel_msg.angular.z = 0
            vel_pub.publish(vel_msg)
            # video_capture.release()
            # cv2.destroyAllWindows()
    else:
        print("Error: Unable to open camera")

def location():                    ##根据位置信息返回标志位
    sub = rospy.Subscriber("/send_tf_xy",trafficlight,receive_xy, queue_size=10)
    rospy.spin()
    
def volt():
    vol = rospy.Subscriber("/battery", Float32, calcu, queue_size=10)

def calcu(dianya):
    global dianyabuchang
    dianyabuchang = dianya.data/11.5
    # print(dianyabuchang)
    


def receive_xy(tf_xy):
    global flag_s
    global flag_t
    global flag_z
    global t
    try:
        #红绿灯开启区域 
        xt_min=2
        xt_max=3
        yt_min=-5.8
        yt_max=-4.6
        #s弯开启区域
        xs_min=0.5
        xs_max=1.5
        ys_min=-4.3
        ys_max=-3.8
        #文字识别区域
        # xz_min=-1.6
        # xz_max=-0.7
        # yz_min=-0.4
        # yz_max=-0.28

        #判断是否检测红绿灯
        if  xt_min <tf_xy.X < xt_max and yt_min< tf_xy.Y< yt_max :
            flag_t=True
        else:
            flag_t=False
        #判断是否检测s弯
        if  xs_min <tf_xy.X < xs_max and ys_min< tf_xy.Y< ys_max:
            flag_s=True
            flag_z = False
            if t%2==0:
                t+=1
        # if  xz_min <tf_xy.X < xz_max and yz_min< tf_xy.Y< yz_max:
        #     flag_z = True
    except Exception as e:
        rospy.logwarn("警告:%s",e)

if __name__ == "__main__":
    rospy.init_node("visionvel_node")
    vel_pub = rospy.Publisher("/cmd_vel",Twist,queue_size=10)
    daoche_pub = rospy.Publisher("daoche",trafficlight,queue_size=10)
    shibie_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=10)
    sub = rospy.Subscriber("/send_tf_xy",trafficlight,queue_size=10)
    vol = rospy.Subscriber("/battery",Float32,queue_size=10)
    trafficlight_pub = rospy.Publisher("trafficLight", trafficlight,queue_size=10)
    thread1 = threading.Thread(name='t2',target= location)
    thread1.start()   #启动线程1
    volt()
    get_vel()
