# -제4회 국제 대학생 EV 자율주행 경진대회-
**주행 트랙**  

<img width="517" height="362" alt="image" src="https://github.com/user-attachments/assets/b563cc24-c505-4073-853b-18eb6e3517e2" />  

<img src="https://github.com/user-attachments/assets/4901b476-a060-45a5-a4ac-848052b0d463" width="517" height="462"/>  
  
---

**팀장**: 김윤식  
**팀원**: 김원준, 옥정우  
| 항목 | 내용 |
|------|------|
| 차량 타입 | 1/10 스케일 JetRacer ROS AI Kit (아커만 조향) |
| 제어 보드 | Jetson Nano (ROS1 Melodic, Ubuntu 18.04) |
| 센서 | IMX219-160 카메라, 2D LiDAR (e.g. RPLIDAR A1) |
| 통신 방식 | Ethernet/Wi-Fi (SSH 접속) |
| 전원 공급 | 18650 리튬 이온 배터리 (3.7V, 3500mAh) |  

<img src="https://github.com/user-attachments/assets/11c8e007-86b1-4f64-adf5-7efc31882517" alt="Jetracer ROS AI Kit" width="517" height="462"/>

---
총 A, B, C 세 개의 구간으로 구성되어, 각 구간에 따라 사용하는 센서와 주행 방식이 다름  

**구역별 주행 규칙**

🔹 A구역 (카메라 전용 구간)
- 사용 센서: 카메라
- A 구역 정보: 차선 간격 65cm / 흰색 실선 5cm
- 주행 방식: 흰색 차선을 따라 이탈하지 않고 B구간 까지 주행
- 제한 사항:
  - LiDAR 사용 금지
  - 심사위원이 장애물 위치를 랜덤으로 배치하여 Localization 방해

🔹 B구역 (LiDAR 전용 구간)
- 사용 센서: 2D LiDAR
- B 구역 정보: 차선 간격 70cm / 높이 30cm 장벽 
- 주행 방식: 벽에 부딪히지 않고 라이다만을 사용하여 장벽이 없어질때까지 주행
- 제한 사항:
  - 카메라 사용 금지 (가짜 차선 있음)
- 특징:
  - 30cm 높이 장벽 인식 및 corridor 주행

🔹 C구역 (오도메트리 기반 구간)
- 사용 센서: odom만 사용
- C 구역 정보: B의 마지막 차선 중간 지점으로부터 거리 정보를 주면 해당 거리 만큼 이동
- 주행 방식: 차량 기준 상대 좌표로 목표 지점 주행
- 특징:
  - go-to-goal 방식 주행 (직선 + 회전)
 
  참고: waypoint 주행과 학습 주행을 막기 위해 연습주행 다음 날 트랙이 변경됨

-----------------
BEV pixel_hls.py
-----------------

A구역에서 사용되는 카메라 기반 자율주행 알고리즘을 위한 BEV(Bird’s Eye View) 변환 좌표 설정 및 흰색 차선 색상 범위(HLS) 수동 측정을 위해 제작  

**BEV 좌표 추출**: 마우스로 차선 4점 클릭하여 상시 변하는 카메라 시야에 맞게 BEV 변환 좌표를 수동으로 수집  

**흰색 차선의 밝기/조도 대응**: 실외 환경(햇빛, 그림자 등)에 따라 변화하는 차선의 색상을 HLS 공간에서 수치화하여, 적절한 lower, upper 필터값을 설정

-----------------------
abc_com.py
--------------
A/B/C 구간 통합 주행 노드

process_a() {  
  → BEV 변환: warpPerspective  
  → HLS 마스크: mask_hls_areas()  
  → 슬라이딩 윈도우: sliding_window()  
  → 곡선 피팅: np.polyfit  
  → 경로 생성: calculate_path()  
  → 조향 계산: calculate_steering_angle() #pure pursuit제어  
  → 제어 발행: publish_control()  
}  

**차선 중앙 유지를 위한 알고리즘**  
오른쪽 차선이 있으면 calculate_path()에서 해당 차선의 접선 각(th)를 구하고, 법선 방향으로 real_shift(차선 중심까지의 차이)를 적용해 차선 ‘중앙선’ 경로(px, py) 생성  
오른쪽이 없고 왼쪽만 있으면 왼쪽 차선을 기준으로 -shift만큼 평행이동해 중앙선 경로를 생성  
→ 차량의 진동을 해결하기 위해 pure pursuit 목표점(look-ahead) 값을 튜닝  


process_b() {  
좌/우 벽의 거리 차이를 이용해 차량이 중앙에서 얼마나 벗어났는지 계산  
거리 차이를 기반으로 비례 제어(P 제어)를 적용해 조향 각도 조절  
}  

process_c() {  
차량의 현재 위치/방향과 목표 좌표 간의 거리와 각도 차이 계산  
방향 정렬 → 직진 주행 → 목표 도달 시 정지  
}  

A → B 전환 트리거  
LiDAR에서 좌우 벽의 평균 거리가 모두(AND) wall_threshold 미만일 때 변환  
※전환 정확률을 높이기 위해 둘 중 하나(OR)로 할 경우 A구역에서 차선 옆 임의의 장애물에 의해 B구역으로 전환 될 수 있음  

B → C 전환 트리거  
양쪽 벽이 모두 사라질 경우 통로에서 빠져나왔음을 의미하여 해당 Odom 좌표로 이동  

---

**주행 영상**  

https://github.com/user-attachments/assets/bad66ce5-53da-4e18-9b8b-f3927af6e2bb


**RQT 화면**  

https://github.com/user-attachments/assets/28122368-daee-4000-979e-f9e78e4d8609




