#!/usr/bin/env python3
# coding=utf-8
import cv2
import numpy as np
import rospy
from socket import *
import pytesseract
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist,TransformStamped
from qingzhou_cloud.msg import trafficlight
import time
import math
import threading
from actionlib_msgs.msg import GoalID

addr = ('255.255.255.255', 6666) 
s = socket(AF_INET, SOCK_DGRAM)
s.setsockopt(SOL_SOCKET, SO_BROADCAST, 1)

# 全体点拟合版
font = cv2.FONT_HERSHEY_DUPLEX  # 字体

K = np.array([[198.5257, 0, 153.7481],
              [0, 264.7291, 138.2082],
              [0, 0, 1.0000]], dtype=np.float32)  # 相机内参矩阵

Dist = np.array([-0.3468, 0.1280, 0., 0., 0.], dtype=np.float32)  # 相机畸变参数

# color range

kernel1 = np.ones((2, 2), np.uint8)
ColorsName = ('Red', 'Yellow', 'Green')
flag_s = False
flag_t = False
flag_z = False
t = 1

def gstreamer_pipeline(
    sensor_id=0,
    capture_width=320,
    capture_height=240,
    display_width=320,
    display_height=240,
    framerate=30,
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


def get_xy(lane_x,lane_y):               #返回s弯偏移量

    # 多项式回归
    lane_mid = np.polyfit(lane_x, lane_y, deg=1)
    # lane_mid_function = np.poly1d(lane_mid)
    x_min = min(lane_x)
    x_max = max(lane_x)
    x_vals = np.linspace(x_min, x_max, 200)
    y_vals = np.polyval(lane_mid, x_vals)
    lane_mid_function = np.poly1d(lane_mid)
    p = (35-lane_mid[1])/lane_mid[0]
    # p = lane_mid_function(100)

    return p, lane_mid, x_vals, y_vals


def contour_select(contours,mask_green1,hsv):
    # 将轮廓进行筛选，可以根据大小、形状等要求进行选择
    roi_list = []
    mask = np.zeros_like(hsv)
    for index, contour in enumerate(contours):
        # 计算每个轮廓面积
        # cv2.drawContours(mask, [contour], -1, (255, 255, 255), thickness=cv2.FILLED)
        # contour_area_hsv = cv2.bitwise_and(hsv, mask)
        # min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(contour_area_hsv[..., 2], mask[..., 0])
        # hsv_h, s, v = contour_area_hsv[max_loc[1], max_loc[0]]
        Area = cv2.contourArea(contour)
        Hull = cv2.convexHull(contour, False)
        HullArea = cv2.contourArea(Hull)
        # 根据特定要求筛选ROI，例如大小、形状等
        x, y, w, h = cv2.boundingRect(contour)
        #if 500 >= Area >10 and HullArea <= 400 and 0.8 <= w/h <= 1.5 and v >= 205:    # 场景有绿色干扰物时
        if 500 >= Area >10 and HullArea <= 500 and 0.5 <= w/h <= 2.5 :                 # 正常时
        # if 1000 >= Area > 0 :                                                        # 场景很干净时
            roi_list.append(contour)
    if len(roi_list) != 0:
        flag = True
    else:
        flag = False

    return roi_list, flag



def ocr(text):
    if "航" in text and "天" in text or "天" in text or "夭" in text or "大" in text:
        print("航天")
        wenzi = 1
    elif "三" in text or "院" in text or "二" in text or "阮" in text:
        print("三院")
        wenzi = 1
    elif "星" in text and "航" in text or "星" in text or "扯" in text or "埕"in text or "皇" in text or "阜" in text or "易" in text or "春" in text or "屈" in text or "吊" in text:
        print("星航")
        wenzi = 2
    elif "机" in text or "电" in text or "巳" in text or "枝" in text or "申" in text or "权" in text or "屯" in text:
        print("机电")
        wenzi = 2
    else:
        wenzi = 3
    return wenzi

last_flag = False
flag_01=1
def get_vel():
    vel_msg = Twist()
    trafficlight_msg = trafficlight()
    daoche_msg = trafficlight()
    green = 0
    i = 0
    limit = 45
    stop = 67
    global daoche_pub
    global vel_pub  
    global trafficlight_pub
    global daoche_pub
    global flag_01 
    global flag_s     #s弯标志位
    global flag_t     #红绿灯标志位
    global flag_z     #文字识别标志位
    global last_flag 
    global t          #是否识别红绿灯标志位
    window_title = "CSI Camera"   #窗口名
    p = 0
    kernel1 = np.ones((2, 2), np.uint8)
    video_capture = cv2.VideoCapture(gstreamer_pipeline(flip_method=0), cv2.CAP_GSTREAMER)    #调用摄像头
    if video_capture.isOpened():
        try:
            # window_handle = cv2.namedWindow(window_title, cv2.WINDOW_AUTOSIZE)
            while True:
                ret_val, frame = video_capture.read()
                if ret_val:
                    img = frame.copy()
                    img = warp(img)     #去畸变后的图像
                    img_ori =img
                    image = img
####################################################S弯识别部分#########################################################################
#######################################################################################################################################
                    if flag_s:
                        print(i)       
                        if flag_01==1:
                            flag_01=0
                            cancel_pub =  rospy.Publisher("/move_base/cancel",GoalID,queue_size=10)
                            empty_goal = GoalID()
                            cancel_pub.publish(empty_goal)
                        img = img[170:240, 0:320]  #(50,320)
                        pt2 = (80, 0)
                        pt1 = (240, 0)
                        pt3 = (0, 70)
                        pt4 = (320, 70)
                        # 将四个顶点坐标放入数组中
                        pts = np.array([pt1, pt2, pt3, pt4], np.int32)
                        # 创建一个掩膜
                        mask = np.zeros_like(img)
                        # 在掩膜上绘制填充的梯形
                        cv2.fillPoly(mask, [pts], 255)
                        img = cv2.bitwise_and(img,img,mask)
                        frame = cv2.GaussianBlur(img,(3,3),0)
                        #frame = cv2.bilateralFilter(img, d, sigma_color, sigma_space)
                        kernel = np.array([[-1, -2, -1],
                                        [0, 0, 0],
                                        [1, 2, 1]], dtype=np.float32)
                        img = cv2.filter2D(frame, ddepth=cv2.CV_32F, kernel=kernel)
                        img = cv2.convertScaleAbs(img)
                        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
                        _, binary = cv2.threshold(gray, 55, 255, cv2.THRESH_BINARY)
                        binary = cv2.dilate(binary, kernel=kernel1, iterations=1)
                        contours, hierarchy = cv2.findContours(binary, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)  # 寻找轮廓线
                        target_contours = []  # 去掉大轮廓，保留车道线轮廓,list形式
                        lane_x = []
                        lane_y = []
                        for index, contour in enumerate(contours):
                            Hull = cv2.convexHull(contour, False)
                            # 多边形逼近
                            approx = cv2.approxPolyDP(Hull, 0.1 * cv2.arcLength(Hull, True), True)
                            # 计算周长、面积和边界框
                            perimeter = cv2.arcLength(approx, True)
                            area = cv2.contourArea(approx)
                            x, y, w, h = cv2.boundingRect(approx)
                            if 70 > perimeter > 25 and 400 > area > 40 and 0.6 < w / h < 2.5:
                                target_contours.append(approx)
                                center_x = int(x + w / 2)
                                center_y = int(y + h / 2)
                                lane_x.append(center_x)
                                lane_y.append(center_y)
                        if len(target_contours) > 1:
                            result = cv2.drawContours(frame.copy(), target_contours, -1, (0, 0, 255), 1)
                            pian, xishu, x_val, y_val = get_xy(lane_x,lane_y)
                            pianyi = (pian-160)/160
                            i = 0
                            # if xishu[0] < 0 and pianyi > 0:
                            #     pinayi = pianyi + 0.7
                            # if pianyi >1:
                            #     pianyi = 1
                            # elif pianyi <-1:
                            #     pianyi = -1
                            # else:
                            #     pianyi = pianyi
                            points = []  # 获得曲线坐标
                            for index, x in enumerate(x_val):
                                points.append([x, y_val[index]])
                            points = np.array(points, np.int32)
                            vel_msg.angular.z = -pianyi*1.62
                            if vel_msg.angular.z < 0:
                                vel_msg.angular.z = vel_msg.angular.z-0.15
                            else:
                                vel_msg.angular.z = vel_msg.angular.z
                            print(pianyi,vel_msg.angular.z)
                            vel_msg.linear.x = 1
                            vel_pub.publish(vel_msg)
                            result = cv2.polylines(result, [points], False, (0, 0, 255), 2)
                        elif len(target_contours)==1:
                            vel_msg.linear.x = 1
                            vel_pub.publish(vel_msg)
                            if i < 3:
                                i = 0
                            result = img
                            print('一个')
                        else:
                            if i < limit:
                                vel_msg.angular.z = -0.45
                            i = i + 1
                            vel_pub.publish(vel_msg)
                            result = frame
                        if limit < i < stop:
                            vel_msg.linear.x = 0.5
                            vel_msg.angular.z = 0
                            vel_pub.publish(vel_msg)
                        if   i > stop:
                            flag_s = False
                            flag_z = True
                            # flag_01=1
                            i=0
                            vel_msg.linear.x = 0
                            vel_msg.angular.z = 0
                            vel_pub.publish(vel_msg)
                        image = result
#####################################################红绿灯识别部分########################################################################           
##########################################################################################################################################
                    if flag_t and t%2 !=0 :
                        image = img
                        kernel = np.ones((2,2),np.uint8)
                        # lower_red1 = np.array([174,55,86])
                        # upper_red1 = np.array([180,205,255])
                        # lower_red2 = np.array([0,55,86])
                        # upper_red2 = np.array([2,205,255])
                        hsv = cv2.cvtColor(image,cv2.COLOR_BGR2HSV)
                        # mask_red1 = cv2.inRange(hsv,lower_red1,upper_red1)
                        # mask_red2 = cv2.inRange(hsv,lower_red2,upper_red2)
                        # mask_red = cv2.add(mask_red2,mask_red1)
                        # mask_red = cv2.erode(mask_red,kernel=kernel,iterations=6)
                        # mask_red = cv2.dilate(mask_red,kernel,iterations=6)#红色掩膜
                        lower_green = np.array([60,30,145])
                        upper_green = np.array([78,222,255])
                        mask_green1 = cv2.inRange(hsv,lower_green,upper_green)

                        mask_green = cv2.erode(mask_green1,kernel,iterations=2)
                        mask_green = cv2.dilate(mask_green,kernel,iterations=7)#绿色掩膜
                        # red_contours, _ = cv2.findContours(mask_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                        green_contours, _ = cv2.findContours(mask_green, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                        # if len(red_contours)!=0:
                        #     red_contours,flagr = contour_select(red_contours)
                        # else:
                        #     flagr = False
                        if len(green_contours)!=0:
                            green_contours,flagg = contour_select(green_contours,mask_green1,hsv)
                        else:
                            flagg = False
                        # 绘制红色和绿色轮廓
                        if flagg == False:
                            p += 1
                        else:
                            p = 0
                            t += 1
                        if p >= 7:
                            trafficlight_msg.trafficstatus = 2
                            trafficlight_pub.publish(trafficlight_msg)
                        else:
                            trafficlight_msg.trafficstatus = 1
                            trafficlight_pub.publish(trafficlight_msg)

                        # if flagg == True:
                        #     trafficlight_msg.trafficstatus = 1
                        #     trafficlight_pub.publish(trafficlight_msg)
                        #     t += 1
                        # else:
                        #     trafficlight_msg.trafficstatus = 2
                        #     trafficlight_pub.publish(trafficlight_msg)
                            
                        # if flagr == True:
                        #     trafficlight_msg.trafficstatus = 2        
                        #     trafficlight_pub.publish(trafficlight_msg)
                        # if flagg == True:
                        #     green = green + 1
                        # if green > 2:
                        #     flagr = False
                        #     green = 0
                        #     trafficlight_msg.trafficstatus = 1
                        #     trafficlight_pub.publish(trafficlight_msg)
                        # cv2.drawContours(image, red_contours, -1, (0, 0, 255), 2)
                        cv2.drawContours(image, green_contours, -1, (0, 255, 0), 2)
                        print(flagg)
            ####################################################################################################################
            ####################################################################################################################            
                    if flag_z == True and last_flag == False:
                        img_word = img_ori[40:120, 100:220]
                        img_gray = cv2.cvtColor(img_word, cv2.COLOR_BGR2GRAY)
                        th = cv2.adaptiveThreshold(img_gray, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, 21, 10)
                        th = cv2.boxFilter(th,-1,(1,1),normalize=False)
                        cv2.imwrite("word.jpg", th)
                        time_start=time.time()
                        text = pytesseract.image_to_string("word.jpg",lang='chi_sim',config='--psm 6')
                        time_end=time.time()
                        print('Running time:{} seconds'.format(time_start - time_end))
                        print(text)
                        wen = ocr(text)
                        print(wen)
                        if wen == 1:
                            daoche_msg.trafficstatus = 1
                        else:
                            daoche_msg.trafficstatus = 0
                        flag_01=1
                        daoche_pub.publish(daoche_msg)
                    last_flag = flag_z
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
    

def receive_xy(tf_xy):
    global flag_s
    global flag_t
    global flag_z
    global last_flag
    global t
    try:
        #红绿灯开启区域 
        xt_min=2
        xt_max=3
        yt_min=-5.8
        yt_max=-4.8
        #s弯开启区域
        xs_min=0.5
        xs_max=1.5
        ys_min=-4.3
        ys_max=-3.8
        #文字识别区域
        xz_min=-1.6
        xz_max=-0.7
        yz_min=-0.4
        yz_max=-0.28

        #判断是否检测红绿灯
        # if  xt_min <tf_xy.X < xt_max and yt_min< tf_xy.Y< yt_max :
        #     flag_t=True
        # else:
        #     flag_t=False
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
    sub = rospy.Subscriber("/send_tf_xy",trafficlight,queue_size=10)
    trafficlight_pub = rospy.Publisher("trafficLight", trafficlight,queue_size=10)
    thread1 = threading.Thread(name='t2',target= location)
    thread1.start()   #启动线程1
    get_vel()
#!/usr/bin/env python3
# coding=utf-8
import cv2
import numpy as np
import rospy
from socket import *
import pytesseract
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist,TransformStamped
from qingzhou_cloud.msg import trafficlight
import time
import math
import threading
from actionlib_msgs.msg import GoalID

addr = ('255.255.255.255', 6666) 
s = socket(AF_INET, SOCK_DGRAM)
s.setsockopt(SOL_SOCKET, SO_BROADCAST, 1)

# 全体点拟合版
font = cv2.FONT_HERSHEY_DUPLEX  # 字体

K = np.array([[198.5257, 0, 153.7481],
              [0, 264.7291, 138.2082],
              [0, 0, 1.0000]], dtype=np.float32)  # 相机内参矩阵

Dist = np.array([-0.3468, 0.1280, 0., 0., 0.], dtype=np.float32)  # 相机畸变参数

# color range

kernel1 = np.ones((2, 2), np.uint8)
ColorsName = ('Red', 'Yellow', 'Green')
flag_s = False
flag_t = False
flag_z = False
t = 1

def gstreamer_pipeline(
    sensor_id=0,
    capture_width=320,
    capture_height=240,
    display_width=320,
    display_height=240,
    framerate=30,
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


def get_xy(lane_x,lane_y):               #返回s弯偏移量

    # 多项式回归
    lane_mid = np.polyfit(lane_x, lane_y, deg=1)
    # lane_mid_function = np.poly1d(lane_mid)
    x_min = min(lane_x)
    x_max = max(lane_x)
    x_vals = np.linspace(x_min, x_max, 200)
    y_vals = np.polyval(lane_mid, x_vals)
    lane_mid_function = np.poly1d(lane_mid)
    p = (35-lane_mid[1])/lane_mid[0]
    # p = lane_mid_function(100)

    return p, lane_mid, x_vals, y_vals


def contour_select(contours,mask_green1,hsv):
    # 将轮廓进行筛选，可以根据大小、形状等要求进行选择
    roi_list = []
    mask = np.zeros_like(hsv)
    for index, contour in enumerate(contours):
        # 计算每个轮廓面积
        # cv2.drawContours(mask, [contour], -1, (255, 255, 255), thickness=cv2.FILLED)
        # contour_area_hsv = cv2.bitwise_and(hsv, mask)
        # min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(contour_area_hsv[..., 2], mask[..., 0])
        # hsv_h, s, v = contour_area_hsv[max_loc[1], max_loc[0]]
        Area = cv2.contourArea(contour)
        Hull = cv2.convexHull(contour, False)
        HullArea = cv2.contourArea(Hull)
        # 根据特定要求筛选ROI，例如大小、形状等
        x, y, w, h = cv2.boundingRect(contour)
        #if 500 >= Area >10 and HullArea <= 400 and 0.8 <= w/h <= 1.5 and v >= 205:    # 场景有绿色干扰物时
        if 500 >= Area >10 and HullArea <= 500 and 0.5 <= w/h <= 2.5 :                 # 正常时
        # if 1000 >= Area > 0 :                                                        # 场景很干净时
            roi_list.append(contour)
    if len(roi_list) != 0:
        flag = True
    else:
        flag = False

    return roi_list, flag



def ocr(text):
    if "航" in text and "天" in text or "天" in text or "夭" in text or "大" in text:
        print("航天")
        wenzi = 1
    elif "三" in text or "院" in text or "二" in text or "阮" in text:
        print("三院")
        wenzi = 1
    elif "星" in text and "航" in text or "星" in text or "扯" in text or "埕"in text or "皇" in text or "阜" in text or "易" in text or "春" in text or "屈" in text or "吊" in text:
        print("星航")
        wenzi = 2
    elif "机" in text or "电" in text or "巳" in text or "枝" in text or "申" in text or "权" in text or "屯" in text:
        print("机电")
        wenzi = 2
    else:
        wenzi = 3
    return wenzi

last_flag = False
flag_01=1
def get_vel():
    vel_msg = Twist()
    trafficlight_msg = trafficlight()
    daoche_msg = trafficlight()
    green = 0
    i = 0
    limit = 20
    stop = 30
    global daoche_pub
    global vel_pub  
    global trafficlight_pub
    global daoche_pub
    global flag_01 
    global flag_s     #s弯标志位
    global flag_t     #红绿灯标志位
    global flag_z     #文字识别标志位
    global last_flag 
    global t          #是否识别红绿灯标志位
    window_title = "CSI Camera"   #窗口名
    p = 0
    kernel1 = np.ones((2, 2), np.uint8)
    video_capture = cv2.VideoCapture(gstreamer_pipeline(flip_method=0), cv2.CAP_GSTREAMER)    #调用摄像头
    if video_capture.isOpened():
        try:
            # window_handle = cv2.namedWindow(window_title, cv2.WINDOW_AUTOSIZE)
            while True:
                ret_val, frame = video_capture.read()
                if ret_val:
                    img = frame.copy()
                    img = warp(img)     #去畸变后的图像
                    img_ori =img
                    image = img
####################################################S弯识别部分#########################################################################
#######################################################################################################################################
                    if flag_s:
                        print(i)       
                        if flag_01==1:
                            flag_01=0
                            cancel_pub =  rospy.Publisher("/move_base/cancel",GoalID,queue_size=10)
                            empty_goal = GoalID()
                            cancel_pub.publish(empty_goal)
                        img = img[170:240, 0:320]  #(50,320)
                        pt2 = (80, 0)
                        pt1 = (240, 0)
                        pt3 = (0, 70)
                        pt4 = (320, 70)
                        # 将四个顶点坐标放入数组中
                        pts = np.array([pt1, pt2, pt3, pt4], np.int32)
                        # 创建一个掩膜
                        mask = np.zeros_like(img)
                        # 在掩膜上绘制填充的梯形
                        cv2.fillPoly(mask, [pts], 255)
                        img = cv2.bitwise_and(img,img,mask)
                        frame = cv2.GaussianBlur(img,(3,3),0)
                        #frame = cv2.bilateralFilter(img, d, sigma_color, sigma_space)
                        kernel = np.array([[-1, -2, -1],
                                        [0, 0, 0],
                                        [1, 2, 1]], dtype=np.float32)
                        img = cv2.filter2D(frame, ddepth=cv2.CV_32F, kernel=kernel)
                        img = cv2.convertScaleAbs(img)
                        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
                        _, binary = cv2.threshold(gray, 55, 255, cv2.THRESH_BINARY)
                        binary = cv2.dilate(binary, kernel=kernel1, iterations=1)
                        contours, hierarchy = cv2.findContours(binary, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)  # 寻找轮廓线
                        target_contours = []  # 去掉大轮廓，保留车道线轮廓,list形式
                        lane_x = []
                        lane_y = []
                        for index, contour in enumerate(contours):
                            Hull = cv2.convexHull(contour, False)
                            # 多边形逼近
                            approx = cv2.approxPolyDP(Hull, 0.1 * cv2.arcLength(Hull, True), True)
                            # 计算周长、面积和边界框
                            perimeter = cv2.arcLength(approx, True)
                            area = cv2.contourArea(approx)
                            x, y, w, h = cv2.boundingRect(approx)
                            if 70 > perimeter > 25 and 400 > area > 40 and 0.6 < w / h < 2.5:
                                target_contours.append(approx)
                                center_x = int(x + w / 2)
                                center_y = int(y + h / 2)
                                lane_x.append(center_x)
                                lane_y.append(center_y)
                        if len(target_contours) > 1:
                            result = cv2.drawContours(frame.copy(), target_contours, -1, (0, 0, 255), 1)
                            pian, xishu, x_val, y_val = get_xy(lane_x,lane_y)
                            pianyi = (pian-160)/160
                            i = 0
                            # if xishu[0] < 0 and pianyi > 0:
                            #     pinayi = pianyi + 0.7
                            # if pianyi >1:
                            #     pianyi = 1
                            # elif pianyi <-1:
                            #     pianyi = -1
                            # else:
                            #     pianyi = pianyi
                            points = []  # 获得曲线坐标
                            for index, x in enumerate(x_val):
                                points.append([x, y_val[index]])
                            points = np.array(points, np.int32)
                            vel_msg.angular.z = -pianyi*1.62
                            if vel_msg.angular.z < 0:
                                vel_msg.angular.z = vel_msg.angular.z-0.15
                            else:
                                vel_msg.angular.z = vel_msg.angular.z
                            print(pianyi,vel_msg.angular.z)
                            vel_msg.linear.x = 1
                            vel_pub.publish(vel_msg)
                            result = cv2.polylines(result, [points], False, (0, 0, 255), 2)
                        elif len(target_contours)==1:
                            vel_msg.linear.x = 1
                            vel_pub.publish(vel_msg)
                            if i < 3:
                                i = 0
                            result = img
                            print('一个')
                        else:
                            if i < limit:
                                vel_msg.angular.z = -0.45
                            i = i + 1
                            vel_pub.publish(vel_msg)
                            result = frame
                        if limit < i < stop:
                            vel_msg.linear.x = 0.2
                            vel_msg.angular.z = -1.0
                            vel_pub.publish(vel_msg)
                        if   i > stop:
                            flag_s = False
                            flag_z = True
                            # flag_01=1
                            i=0
                            vel_msg.linear.x = 0
                            vel_msg.angular.z = 0
                            vel_pub.publish(vel_msg)
                        image = result
#####################################################红绿灯识别部分########################################################################           
##########################################################################################################################################
                    if flag_t and t%2 !=0 :
                        image = img
                        kernel = np.ones((2,2),np.uint8)
                        # lower_red1 = np.array([174,55,86])
                        # upper_red1 = np.array([180,205,255])
                        # lower_red2 = np.array([0,55,86])
                        # upper_red2 = np.array([2,205,255])
                        hsv = cv2.cvtColor(image,cv2.COLOR_BGR2HSV)
                        # mask_red1 = cv2.inRange(hsv,lower_red1,upper_red1)
                        # mask_red2 = cv2.inRange(hsv,lower_red2,upper_red2)
                        # mask_red = cv2.add(mask_red2,mask_red1)
                        # mask_red = cv2.erode(mask_red,kernel=kernel,iterations=6)
                        # mask_red = cv2.dilate(mask_red,kernel,iterations=6)#红色掩膜
                        lower_green = np.array([60,30,145])
                        upper_green = np.array([78,222,255])
                        mask_green1 = cv2.inRange(hsv,lower_green,upper_green)

                        mask_green = cv2.erode(mask_green1,kernel,iterations=2)
                        mask_green = cv2.dilate(mask_green,kernel,iterations=7)#绿色掩膜
                        # red_contours, _ = cv2.findContours(mask_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                        green_contours, _ = cv2.findContours(mask_green, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                        # if len(red_contours)!=0:
                        #     red_contours,flagr = contour_select(red_contours)
                        # else:
                        #     flagr = False
                        if len(green_contours)!=0:
                            green_contours,flagg = contour_select(green_contours,mask_green1,hsv)
                        else:
                            flagg = False
                        # 绘制红色和绿色轮廓
                        if flagg == False:
                            p += 1
                        else:
                            p = 0
                            t += 1
                        if p >= 7:
                            trafficlight_msg.trafficstatus = 2
                            trafficlight_pub.publish(trafficlight_msg)
                        else:
                            trafficlight_msg.trafficstatus = 1
                            trafficlight_pub.publish(trafficlight_msg)

                        # if flagg == True:
                        #     trafficlight_msg.trafficstatus = 1
                        #     trafficlight_pub.publish(trafficlight_msg)
                        #     t += 1
                        # else:
                        #     trafficlight_msg.trafficstatus = 2
                        #     trafficlight_pub.publish(trafficlight_msg)
                            
                        # if flagr == True:
                        #     trafficlight_msg.trafficstatus = 2        
                        #     trafficlight_pub.publish(trafficlight_msg)
                        # if flagg == True:
                        #     green = green + 1
                        # if green > 2:
                        #     flagr = False
                        #     green = 0
                        #     trafficlight_msg.trafficstatus = 1
                        #     trafficlight_pub.publish(trafficlight_msg)
                        # cv2.drawContours(image, red_contours, -1, (0, 0, 255), 2)
                        cv2.drawContours(image, green_contours, -1, (0, 255, 0), 2)
                        print(flagg)
            ####################################################################################################################
            ####################################################################################################################            
                    if flag_z == True and last_flag == False:
                        img_word = img_ori[60:120, 100:220]
                        img_gray = cv2.cvtColor(img_word, cv2.COLOR_BGR2GRAY)
                        th = cv2.adaptiveThreshold(img_gray, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, 21, 10)
                        th = cv2.boxFilter(th,-1,(1,1),normalize=False)
                        cv2.imwrite("word.jpg", th)
                        time_start=time.time()
                        text = pytesseract.image_to_string("word.jpg",lang='chi_sim',config='--psm 6')
                        time_end=time.time()
                        print('Running time:{} seconds'.format(time_start - time_end))
                        print(text)
                        wen = ocr(text)
                        print(wen)
                        if wen == 1:
                            daoche_msg.trafficstatus = 1
                        else:
                            daoche_msg.trafficstatus = 0
                        flag_01=1
                        daoche_pub.publish(daoche_msg)
                    last_flag = flag_z
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
    

def receive_xy(tf_xy):
    global flag_s
    global flag_t
    global flag_z
    global last_flag
    global t
    try:
        #红绿灯开启区域 
        xt_min=2
        xt_max=3
        yt_min=-5.8
        yt_max=-4.8
        #s弯开启区域
        xs_min=0.5
        xs_max=1.5
        ys_min=-4.3
        ys_max=-3.8
        #文字识别区域
        xz_min=-1.6
        xz_max=-0.7
        yz_min=-0.4
        yz_max=-0.28

        #判断是否检测红绿灯
        # if  xt_min <tf_xy.X < xt_max and yt_min< tf_xy.Y< yt_max :
        #     flag_t=True
        # else:
        #     flag_t=False
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
    sub = rospy.Subscriber("/send_tf_xy",trafficlight,queue_size=10)
    trafficlight_pub = rospy.Publisher("trafficLight", trafficlight,queue_size=10)
    thread1 = threading.Thread(name='t2',target= location)
    thread1.start()   #启动线程1
    get_vel()
