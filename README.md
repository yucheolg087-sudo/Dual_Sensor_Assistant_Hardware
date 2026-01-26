# Eye-Vision: 초음파/ToF 듀얼 센서 보행 보조 기기

시각장애인의 보행 사각지대(상단 충돌, 하단 낙상)를 해소하기 위한 아두이노 웨어러블 디바이스 프로젝트입니다.

[📂 소스 코드(ino) 확인하기](./Dual_Sensor_Assist_EyeVision.ino)

<br/>

![Arduino](https://img.shields.io/badge/Arduino-UNO-00979D?style=flat-square&logo=Arduino&logoColor=white)
![C++](https://img.shields.io/badge/C%2B%2B-00599C?style=flat-square&logo=c%2B%2B&logoColor=white)

<br/>

## 🏆 수상 내역
제 1회 U7 메이커스 경진대회 대상 (안양시장상)

<p align="center">
  <img src="./eyevision_award_certificate.jpg" alt="상장" width="50%">
</p>

<br/>

## 프로젝트 소개
기존의 흰 지팡이는 바닥의 장애물은 감지할 수 있지만, 얼굴 높이의 튀어나온 물체나 갑작스러운 계단(낙상) 같은 사각지대에는 취약하다는 점을 발견했습니다.

단순히 거리를 재는 것을 넘어, "보행자가 안전하다고 느낄 수 있는가?"에 집중하여 상단(초음파)과 하단(ToF)을 동시에 감지하고 위험 상황을 즉각 알리는 듀얼 센서 시스템을 설계했습니다.

* 개발 기간: 2025.09 ~ 2026.01
* 팀 구성: 3인 (Team Leader, HW, SW)
* 담당 역할:
    * 시스템 통합: 센서 데이터 처리 및 전체 회로 설계
    * 알고리즘 구현: 낙상 감지를 위한 미분 로직 및 EMA 필터 적용
    * HW 최적화: 전력 노이즈 개선 및 배터리 효율 관리

<br/>

## 하드웨어 구성 (HW Specs)

### 시스템 회로도
| Schematic | Logic Flow |
| :---: | :---: |
| ![회로도](./eyevision_circuit_diagram.png) | ![로직도](./eyevision_logic_flowchart.png) |

### Pin Map & Parts
* MCU: Arduino Nano (ATmega328P)
* D2, D3: HC-SR04 Echo/Trig (상단 감지용)
* A4, A5: VL53L1X SDA/SCL (하단 낙상 감지용 - I2C)
* D9: Active Buzzer (PWM 경고음 출력)
* Power: 9V Battery + 5V Regulator (with 100uF Capacitor)

<br/>

## 핵심 구현 기술 (Key Features)

### 1. 딜레이 없는 실시간 감지 (Non-blocking)
delay() 함수를 사용하면 센서가 값을 읽는 동안 전체 시스템이 멈추는 문제가 발생했습니다. 이를 해결하기 위해 millis() 함수를 활용한 타임 스탬프 방식의 스케줄러를 구현하여, 센싱과 알림이 끊김 없이 동시에 이루어지도록 했습니다.

### 2. 신호 처리 필터링 (EMA Filter)
보행 중 발생하는 진동으로 인해 센서 값이 튀는 현상이 있었습니다. 단순 평균 필터는 반응이 느려지는 단점이 있어, 지수 이동 평균(EMA) 필터를 적용했습니다. 이를 통해 노이즈는 잡으면서도 장애물 감지 반응 속도는 유지할 수 있었습니다.

<br/>

## 트러블 슈팅 (Challenges & Solutions)
개발 과정에서 겪었던 가장 큰 문제들과 해결 과정입니다.

### ⚠️ 문제 1: 센서 값이 0으로 튀는 고스트 노이즈
저가형 초음파 센서와 모터를 같이 쓸 때 전압 강하로 인해 센서 값이 0으로 찍히는 오작동이 빈번했습니다.
* 해결: 전원 라인에 100uF 전해 커패시터와 0.1uF 세라믹 커패시터를 병렬로 연결하여 전원 리플을 물리적으로 제거했고, SW적으로는 비정상적인 값(Outlier)을 무시하는 예외 처리 로직을 추가하여 해결했습니다.

### ⚠️ 문제 2: 계단 낙상 감지의 어려움
초기에는 "거리 > 50cm"면 위험하다고 코딩했습니다. 하지만 계단마다 높이가 다르고, 경사로에서는 오작동이 발생했습니다.
* 해결 (핵심): 절대적인 '거리(Distance)'가 아니라 '거리의 변화량(Velocity)'에 집중했습니다. 보행 중 바닥과의 거리가 아주 짧은 시간 내에 급격히 멀어지는 순간(미분값 급증)을 낙상으로 판단하도록 알고리즘을 변경했고, 테스트 결과 오탐지율을 획기적으로 줄였습니다.

<br/>

## 테스트 결과 및 시연

### 프로토타입 & 착용 테스트
| 내부 하드웨어 | 실제 착용 보행 |
| :---: | :---: |
| <img src="./eyevision_hardware_prototype.jpg" width="45%"> | <img src="./eyevision_wearing_demo.jpg" width="45%"> |

### 실제 작동 영상 (YouTube)
개인정보 보호를 위해 시연 영상은 일부 블러 처리되었습니다. 아래 링크를 클릭하면 기능별 영상을 확인할 수 있습니다.

* [1. 하단 낙상 감지 테스트 영상 보기](https://youtu.be/Dz_auRCq3no)
    * 계단 진입 시 ToF 센서가 급격한 거리 변화를 감지하여 즉시 경고
* [2. 상단 장애물 감지 테스트 영상 보기](https://youtu.be/06Roy1dTxnc)
    * 전방 1.5m 내 장애물(사람, 벽) 감지 시 충돌 방지 알림

### 최종 성과
* 반응 속도: 0.1초 이내 (10Hz Sampling)
* 탐지 정확도: 낙상 감지율 95% 이상 달성
* 수상: 2025 U7 메이커스 경진대회 대상
