import cv2
import numpy as np

img = cv2.imread('breaker.jpeg',cv2.IMREAD_COLOR)
roi = img[95:98 , 200:553] # 차단기의 막대부분.
std_color = (129, 121, 214) # 차단기 막대의 색 평균값.
isbreaker = False 
blue = 0
green = 0
red = 0
num_pixel = 4 * 353

#roi의 색 평균값 구하기
for i in range(95,99):
	for j in range(200,554):
		blue +=  img[i][j][0]
		green += img[i][j][1]
		red += img[i][j][2]

blue_avg = blue / num_pixel
green_avg = green / num_pixel
red_avg = red / num_pixel

#roi의 색 평균값과 기준으로 잡아놓은 색의 값 비교 (오차범위 내에 들어오는지)
if  (std_color[0] - 10 < blue_avg < std_color[0] + 10) and (std_color[1] - 10 < green_avg < std_color[1] + 210) and (std_color[2] - 10 < red_avg < std_color[2] + 10) :	
	isbreaker = True  # isbreker = True 이면 차동차 정지. 

print isbreaker
#한계점 - 차단기 규격이 통일되어있지 않음 ==> 빨간색과 하얀색 비율에따라 인식 못할수도있음. 
