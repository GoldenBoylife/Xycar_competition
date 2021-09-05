#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import cv2, random, math, time

# You are to find "left and light position" of road lanes
def process_image(img,left,right,mid):
    if cap.isOpened() :      #캡처 객체 초기화 확인,정상시 True
        while True:
            ret, img = cap.read()
            ##roi 범위 만들기 
            roi_1= 350
            roi_2=370
            roi =img[roi_1:roi_2 :]

            ##gray + blur 효과
            gray= cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)  
            blur = cv2.GaussianBlur(gray, (5,5), 0)

            ##canny edge
            low_threshold = 50
            high_threshold = 200
            edges = cv2.Canny(blur,low_threshold,high_threshold)

            ##팽창
            kernel = np.ones((3,3),np.uint8)
            dila = cv2.dilate(edges, kernel, iterations=3)

            view = cv2.cvtColor(dila, cv2.COLOR_GRAY2BGR)
            ##영상 가운데 Red box
            cv2.rectangle(img,(mid-5,roi_1),(mid+5,roi_2), (0,0,255),2)
            cv2.rectangle(view,(mid-5,0),(mid+5,20), (0,0,255),3)

            found_left, found_right = False, False
            prev_left, prev_right = left, right
            state = "BOTH"
            ##왼차선인식 박스 만들기  x 0~80 범위까지 상자만들기
            for l in range(0, 80, 5):
                ##이전 왼쪽위치값 기준으로 오른쪽에서 찾기
                if left+l+10 < mid and left+l-10 >= 0:
                    area = dila[5:15 , left+l-10:left+l+10]
                    #상자area 만듬
                    ##상자area에서 흰색 픽셀 90개 넘어가면 위치검출
                    if cv2.countNonZero(area) > 90:
                        left = left+l
                        #왼차선 x위치 업뎃.
                        found_left = True
                        break
                ##이전 왼쪽차선 위치값 기준으로 왼쪽 찾기
                if left-l+10 < mid and left-l-10 >= 0:
                    area = dila[5:15 , left-l-10:left-l+10]
                    if cv2.countNonZero(area) > 90:
                        left = left-l
                        found_left = True
                        break
            ##오른차선인식 박스만들기 x 0~110
            for r in range(0, 111, 5):
                ##이전 오른쪽위치값 기준으로 오른쪽에서 찾기
                if right+r+10 < 640 and right+r-10 > mid:
                    area = dila[5:15 , right+r-10:right+r+10]
                    if cv2.countNonZero(area) > 90:
                        right = right+r
                        found_right = True
                        break
                ##이전 오른쪽위치값 기준으로 오른쪽에서 찾기
                if right-r+10 < 640 and right-r-10 > mid:
                    area =dila[5 :15 , right-r-10:right-r+10]
                    if cv2.countNonZero(area) > 90:
                        right = right-r
                        found_right = True
                        break
            
            ##왼차선 인식x, 오른 차선인식o 일때 
            if not found_left and found_right:
                left += right - prev_right
                #오른차선 증가한만큼, 왼차선 증가
                state = "X   O"

            ##오른 차선 인식x, 왼차선 인식o 일 때    
            if not found_right and found_left:
                right += left - prev_left
                #왼차선 증가한 만큼 오른차선 증가
                state = "O   X"

            cent = (left + right)//2
            #차선 가운데 위치
            print (left, right, state)

            lsquare = cv2.rectangle(img,( left- 10 , 5 +roi_1),(left+10,15+roi_1),(0,255,0), 3)
            #img에다가 왼차선검출한 녹색상자 표시함.roi_1은 y보정값
            rsquare = cv2.rectangle(img,(right-10, 5+roi_1),(right+10, 15+roi_1), (0,255,0), 3)
            #img에다가 오른차선검출한 녹색상자 표시함. roi_1은 y보정값
            cent_square= cv2.rectangle(img,(cent+10,5+roi_1), (cent-10,15+roi_1),(255,0,0),3)
            #img에다가 검출한 차선 가운데 파란색 상자 표시함
            steer_angle = -0.33*(cent-mid)
            #화면 가운데와 차선 가운데 차이값에 비레해서 angle 조종

            lsquare = cv2.rectangle(view,( left- 10 , 5 ),(left+10,15),(0,255,0), 3)
            #img에다가 왼차선검출한 녹색상자 표시함.roi_1은 y보정값
            rsquare = cv2.rectangle(view,(right-10, 5),(right+10, 15), (0,255,0), 3)
            #img에다가 오른차선검출한 녹색상자 표시함. roi_1은 y보정값
            cent_square= cv2.rectangle(view,(cent+10,5), (cent-10,15),(255,0,0),3)
            #img에다가 검출한 차선 가운데 파란색 상자 표시함
            steer_angle = -0.33*(cent-mid)
            #화면 가운데와 차선 가운데 차이값에 비레해서 angle 조종
            if ret:
                cv2.imshow("view", view)
                cv2.waitKey(10)
            else:
                break
            draw_steer(img,steer_angle)

def draw_steer(img, steer_angle):
    origin_Height = arrow_pic.shape[0]
    origin_Width = arrow_pic.shape[1]
    steer_wheel_center = origin_Height * 0.74
    #중심점
 
    arrow_Height = 444/2
    arrow_Width = (arrow_Height * 462)/728
    matrix = cv2.getRotationMatrix2D((origin_Width/2, steer_wheel_center), (steer_angle) * 1.5, 0.7)    
    arrow_pic_ = cv2.warpAffine(arrow_pic, matrix, (origin_Width+60, origin_Height))
    arrow_pic_ = cv2.resize(arrow_pic_, dsize=(arrow_Width, arrow_Height), interpolation=cv2.INTER_AREA)
    gray_arrow = cv2.cvtColor(arrow_pic_, cv2.COLOR_BGR2GRAY)
    _, mask = cv2.threshold(gray_arrow, 1, 255, cv2.THRESH_BINARY_INV)
    arrow_roi = img[arrow_Height: 444, (433/2 - arrow_Width/2) : (433/2 + arrow_Width/2)]
    arrow_roi = cv2.add(arrow_pic_, arrow_roi, mask=mask)
    res = cv2.add(arrow_roi, arrow_pic_)
    img[(444 - arrow_Height): 444, (433/2 - arrow_Width/2): (433/2 + arrow_Width/2)] = res
    cv2.imshow('steer', img)


if __name__ == '__main__':
    left, right, mid = 110, 630, 340
    ##파일 경로로 실행, 동영상 파일, steer 영상 파일.
    video_file = "kmu_track(mp4)_fast.mp4"
    png_file = "steer_arrow.png"
    cap= cv2.VideoCapture(video_file)
    #비디오 프레임 하나하나 캡첫해서 cap으로 저장 
    time.sleep(3)
    arrow_pic = cv2.imread(png_file, cv2.IMREAD_COLOR)
    
    while not rospy.is_shutdown():
        process_image(cap,left, right,mid)
        if cv2.waitKey(3) & 0xFF == ord('q'):
            break

