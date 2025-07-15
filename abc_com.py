#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
abc_track_node.py
A구역: 카메라로 차선 인식
B구역: 라이다 corridor 주행
C구역: odom 기반 목표 이동
"""
from __future__ import division, print_function
import rospy, math
import cv2
import numpy as np
from sensor_msgs.msg import CompressedImage, CameraInfo, Image, LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError
from visualization_msgs.msg import Marker

# 이전 프레임 베이스 저장
prev_left_base = None
prev_right_base = None

class ABCTrackNode(object):
    def __init__(self):
        rospy.init_node('abc_track_node')
        self.rate = rospy.Rate(20)
        self.mode = 'A_CAM'

        # CV bridge and image 상태
        self.bridge = CvBridge()
        self.frame = None
        self.map1 = self.map2 = None
        self.bev_M = None
        self.got_image = False

        # LIDAR/Odom 상태
        self.avg_left = self.avg_right = float('inf')
        self.current_x = self.current_y = self.yaw = 0.0
        self.got_scan = self.got_odom = False
        self.init_x = self.init_y = self.init_yaw = None
        self.goal_x = self.goal_y = None
        self.arrived = False

        # --- A구역 파라미터 ---
        self.fixed_points = rospy.get_param('~fixed_points', [(9,479),(638,475),(517,251),(118,252)]) #(32,470),(600,470),(480,258),(118,266)
        self.road_width_m = rospy.get_param('~road_width_m', 0.7)
        self.bev_length_m = rospy.get_param('~bev_length_m', 1.7)
        self.mask_scale   = rospy.get_param('~mask_scale', 0.5)
        self.lower_hls    = rospy.get_param('~lower_hls', [95,170,20]) #today
        self.upper_hls    = rospy.get_param('~upper_hls', [190,225,240]) 

	# self.lower_hls    = rospy.get_param('~lower_hls', [140,180,20]) ->해 쨍쨍
        # self.upper_hls    = rospy.get_param('~upper_hls', [182,217,220])
        
        # sliding window 및 원형 필터 파라미터
        self.blur_ksize = (21, 21)
        self.min_radius = 25
        self.max_radius = 56
        self.nwindows   = 18
        self.margin     = 60
        self.minpix     = 50
        # Pure Pursuit 제어 파라미터
        self.x_la       = 0.6 #0.4
        self.real_shift = 0.3 #0.45

        # BEV 이미지 크기
        self.bev_w = 640
        self.bev_h = 480
        self.x_m_per_pixel = self.road_width_m / float(self.bev_w)
        self.y_m_per_pixel = self.bev_length_m / float(self.bev_h)

        # --- B구역 파라미터 ---
        self.wall_threshold = rospy.get_param('~wall_threshold', 0.6)
        self.angle_tol_deg  = rospy.get_param('~angle_tolerance_deg', 5)
        self.kp             = rospy.get_param('~kp', 0.5) #1.0
        self.lidar_speed    = rospy.get_param('~linear_speed', 0.6) #0.5

        # --- C구역 파라미터 ---
        self.goal_x_rel        = rospy.get_param('~goal_x_rel', 2.0)
        self.goal_y_rel        = rospy.get_param('~goal_y_rel', 3.0)
        self.goto_angle_thresh = rospy.get_param('~angle_threshold', 0.1)
        self.goto_ang_gain     = rospy.get_param('~angular_gain', 0.5)
        self.goto_speed_far    = rospy.get_param('~speed_far', 0.3)
        self.goto_speed_close  = rospy.get_param('~speed_close', 0.3)
        self.arrive_dist_thresh= rospy.get_param('~arrive_dist_thresh', 0.25)

        # Publishers & Subscribers
        self.cmd_pub       = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.debug_bev_pub = rospy.Publisher('/car/bev_image', Image, queue_size=1)
        self.debug_sw_pub  = rospy.Publisher('/car/sliding_window_image', Image, queue_size=1)
        self.marker_pub    = rospy.Publisher('goal_marker', Marker, queue_size=1)

        rospy.Subscriber('/csi_cam_0/camera_info', CameraInfo, self.callback_cam_info)
        rospy.Subscriber('/csi_cam_0/image_raw/compressed', CompressedImage, self.callback_image)
        rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        rospy.on_shutdown(self.shutdown)

        self.run()

    def shutdown(self):
        self.cmd_pub.publish(Twist())
        rospy.loginfo('Node stopped')

    # --- 콜백 함수 ---
    def callback_cam_info(self, info):
        K = np.array(info.K).reshape(3,3)
        D = np.array(info.D)
        w, h = info.width, info.height
        self.map1, self.map2 = cv2.initUndistortRectifyMap(K, D, None, K, (w,h), cv2.CV_16SC2)
        src = np.float32([self.fixed_points[i] for i in [0,1,3,2]])
        dst = np.float32([[0,h],[w,h],[0,0],[w,0]])
        self.bev_M = cv2.getPerspectiveTransform(src, dst)

    def callback_image(self, msg):
        try:
            img = self.bridge.compressed_imgmsg_to_cv2(msg, 'bgr8')
            if self.map1 is not None:
                img = cv2.remap(img, self.map1, self.map2, interpolation=cv2.INTER_LINEAR)
            self.frame = img
        except CvBridgeError as e:
            rospy.logerr('CvBridge Error: %s', e)

    def scan_callback(self, msg):
        ranges, amin, ainc = msg.ranges, msg.angle_min, msg.angle_increment
        tol = int(math.radians(self.angle_tol_deg) / ainc)
        li = int((math.radians(-105) - amin) / ainc)
        ri = int((math.radians(105) - amin) / ainc)
        def perp(center, idx):
            ds=[]
            for j in range(idx-tol, idx+tol+1):
                r = ranges[j]
                if not math.isinf(r):
                    beam = amin + j*ainc
                    ds.append(r * abs(math.cos(beam - center)))
                else:
                    continue
            return ds
        lv = perp(math.radians(-105), li)
        rv = perp(math.radians(105), ri)
        if any(not math.isinf(v) for v in lv) and any(not math.isinf(v) for v in rv):
	    self.avg_left  = sum(lv) / len(lv)
	    self.avg_right = sum(rv) / len(rv)
	elif max(ranges) == 0.0:
            self.avg_left  = 1.0
            self.avg_right = 1.0
	
        self.got_scan = True

    def odom_callback(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        self.current_x, self.current_y = p.x, p.y
        siny = 2*(q.w*q.z + q.x*q.y)
        cosy = 1-2*(q.y*q.y + q.z*q.z)
        self.yaw = math.atan2(siny, cosy)
        self.got_odom = True

    # --- 메인 루프 ---
    def run(self):
        while not rospy.is_shutdown():
            # 모드 전환
            if self.mode=='A_CAM' and self.got_scan:
                if self.avg_left < self.wall_threshold and self.avg_right < self.wall_threshold:
                    self.mode = 'B_LIDAR'
                    rospy.loginfo('Switch to B_LIDAR')
            if self.mode=='B_LIDAR' and self.got_odom :
                if (self.avg_left > self.wall_threshold and self.avg_right > self.wall_threshold):
                    self.init_x, self.init_y, self.init_yaw = self.current_x, self.current_y, self.yaw
                    self.goal_x = self.init_x + math.cos(self.init_yaw)*self.goal_x_rel - math.sin(self.init_yaw)*self.goal_y_rel
                    self.goal_y = self.init_y + math.sin(self.init_yaw)*self.goal_x_rel + math.cos(self.init_yaw)*self.goal_y_rel
                    self.mode = 'C_ODOM'
                    rospy.loginfo('Switch to C_ODOM')
            # 모드별 처리
            if self.mode=='A_CAM':     self.process_a()
            elif self.mode=='B_LIDAR': self.process_b()
            else:                      self.process_c()
            self.rate.sleep()

    # --- A구역 처리 (카메라 라인) ---
    def process_a(self):
	
        if self.frame is None or self.bev_M is None: return
        bev = cv2.warpPerspective(self.frame, self.bev_M, (self.frame.shape[1], self.frame.shape[0]))
        out, lx, ly, rx, ry = self.sliding_window(bev)
        if (lx and len(lx)>0) or (rx and len(rx)>0):
            cx = bev.shape[1]//2
            lx_m = (cx - np.array(lx)) * self.x_m_per_pixel
            ly_m = (bev.shape[0] - np.array(ly)) * self.y_m_per_pixel
            rx_m = (cx - np.array(rx)) * self.x_m_per_pixel
            ry_m = (bev.shape[0] - np.array(ry)) * self.y_m_per_pixel
            lf = np.polyfit(ly_m, lx_m, 1) if len(lx_m)>=2 else None
            rf = np.polyfit(ry_m, rx_m, 3) if len(rx_m)>=4 else None
            py, px = self.calculate_path(rf, lf, self.x_la, self.real_shift)
            if px is not None:
                pf = np.polyfit(py, px, 3)
                y_la = np.polyval(pf, self.x_la)
                steer = self.calculate_steering_angle(self.x_la, y_la)
                self.publish_control(steer)
        else:
            self.publish_control(0, throttle=0.65)
        try:
            self.debug_bev_pub.publish(self.bridge.cv2_to_imgmsg(bev,'bgr8'))
            self.debug_sw_pub.publish(self.bridge.cv2_to_imgmsg(out,'bgr8'))
        except CvBridgeError:
            pass

    # --- B구역 처리 (라이다 corridor) ---
    def process_b(self):
        err = max(min(self.avg_left-self.avg_right, 1.0), -1.0)
        cmd = Twist(); cmd.linear.x = self.lidar_speed; cmd.angular.z = self.kp*err
        self.cmd_pub.publish(cmd)
        rospy.loginfo('Corridor L:%.2f R:%.2f err:%.2f', self.avg_left, self.avg_right, err)

    # --- C구역 처리 (odom go-to) ---
    def process_c(self):
        if self.goal_x is None: return
        if not self.arrived:
            m = Marker(); m.header.frame_id='odom'; m.header.stamp=rospy.Time.now()
            m.ns, m.id = 'goals', 0; m.type, m.action = Marker.SPHERE, Marker.ADD
            m.pose.position.x, m.pose.position.y = self.goal_x, self.goal_y
            m.pose.orientation.w = 1.0; m.scale.x=m.scale.y=m.scale.z=0.2
            m.color.r, m.color.a = 1.0, 1.0
            self.marker_pub.publish(m)
        dx = self.goal_x - self.current_x; dy = self.goal_y - self.current_y
        dist, ang = math.hypot(dx, dy), math.atan2(dy, dx)
        diff = math.atan2(math.sin(ang - self.yaw), math.cos(ang - self.yaw))
        cmd = Twist()
        if dist > self.arrive_dist_thresh:
            if abs(diff) > self.goto_angle_thresh:
                cmd.angular.z = self.goto_ang_gain*diff; cmd.linear.x = self.goto_speed_far
            else:
                cmd.linear.x = self.goto_speed_close
        else:
            if not self.arrived: rospy.loginfo('Arrived'); self.arrived = True
        self.cmd_pub.publish(cmd)

    # --- 유틸 함수 ---
    def publish_control(self, steer_deg, throttle=0.7):
        twist = Twist(); twist.linear.x = throttle; twist.angular.z = math.radians(steer_deg)
        self.cmd_pub.publish(twist)

    def sliding_window(self, img):
        global prev_left_base, prev_right_base
        mask = self.mask_hls_areas(img)
        mask = self.remove_circular_objects(mask)
        h, w = mask.shape; base_h = h*4//5
        hist = np.sum(mask[base_h:], axis=0)
        mid = w//2; leftx = np.argmax(hist[:mid]); rightx = np.argmax(hist[mid:]) + mid
        if prev_left_base and not (0<leftx<=w//3): leftx = prev_left_base
        if prev_right_base and not (rightx>=w*17//30): rightx = prev_right_base
        prev_left_base, prev_right_base = leftx, rightx
        out = np.dstack((mask,)*3); win_h = h//self.nwindows
        nonzeroy, nonzerox = mask.nonzero(); lpx,lpy,rpx,rpy = [],[],[],[]
        for i in range(self.nwindows):
            ylow, yhigh = h-(i+1)*win_h, h-i*win_h
            xl, xh = leftx-self.margin, leftx+self.margin
            rl, rh = rightx-self.margin, rightx+self.margin
            gl = ((nonzeroy>=ylow)&(nonzeroy<yhigh)&(nonzerox>=xl)&(nonzerox<xh)).nonzero()[0]
            if len(gl)>self.minpix:
                leftx = int(np.mean(nonzerox[gl])); lpx.append(leftx); lpy.append(int(np.mean(nonzeroy[gl])));
                cv2.rectangle(out, (xl, ylow), (xh, yhigh), (255,0,0), 2)
            gr = ((nonzeroy>=ylow)&(nonzeroy<yhigh)&(nonzerox>=rl)&(nonzerox<rh)).nonzero()[0]
            if len(gr)>self.minpix:
                rightx = int(np.mean(nonzerox[gr])); rpx.append(rightx); rpy.append(int(np.mean(nonzeroy[gr])));
                cv2.rectangle(out, (rl, ylow), (rh, yhigh), (0,0,255), 2)
        return out, lpx, lpy, rpx, rpy

    def mask_hls_areas(self, img):
        small = cv2.resize(img, (int(img.shape[1]*self.mask_scale), int(img.shape[0]*self.mask_scale)), interpolation=cv2.INTER_AREA)
        blur = cv2.GaussianBlur(small, self.blur_ksize, 0)
        hls = cv2.cvtColor(blur, cv2.COLOR_BGR2HLS)
        mask = cv2.inRange(hls, np.array(self.lower_hls), np.array(self.upper_hls))
        return cv2.resize(mask, (img.shape[1], img.shape[0]), interpolation=cv2.INTER_NEAREST)

    def remove_circular_objects(self, mask):
        cnts,_ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for c in cnts:
            if len(c)>=5:
                (_, _), r = cv2.minEnclosingCircle(c)
                if self.min_radius<r<self.max_radius:
                    cv2.drawContours(mask, [c], -1, 0, -1)
        return mask

    def calculate_path(self, rf, lf, x_la, shift):
        if rf is not None:
            Rpx = 3*rf[0]*x_la**2 + 2*rf[1]*x_la + rf[2]
            th = math.atan(Rpx)
            ysh = shift * math.cos(th)
            xsh = -shift * math.sin(th)
            py = np.linspace(0, self.bev_length_m, num=720)
            fx = rf[0]*py**3 + rf[1]*py**2 + rf[2]*py + rf[3]
            return py+xsh, fx+ysh
        if lf is not None:
            py = np.linspace(0, self.bev_length_m, num=720)
            fx = lf[0]*py + lf[1]
            return py, fx-shift
        return None, None

    def calculate_steering_angle(self, x_la, y_la):
        return math.degrees(math.atan2(2*0.55*y_la, x_la**2+y_la**2))

    def calculate_steering_angle_from_slope(self, s):
        d = math.degrees(math.atan(s))
        return -16 if d<=-20 else d

if __name__=='__main__':
    try:
        ABCTrackNode()
    except rospy.ROSInterruptException:
        pass

