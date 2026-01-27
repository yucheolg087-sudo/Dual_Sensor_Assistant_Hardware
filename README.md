# Eye-Vision: 시각장애인 보행 보조 디바이스(하드웨어)

[📂 전체 소스 코드(ino) 다운로드](./Dual_Sensor_Assist_EyeVision.ino)

## TL;DR (3줄 요약)
- 문제: 시각장애인의 보행 사각지대(상단 충돌, 하단 낙상) 해결 필요
- 기술: 초음파(상단) + ToF(하단) 센서 융합 낙상 감지 알고리즘 적용
- 성과:   U7 메이커스 경진대회 대상

---

![Arduino](https://img.shields.io/badge/Arduino-Nano-00979D?style=flat-square&logo=Arduino&logoColor=white)
![C++](https://img.shields.io/badge/C%2B%2B-Embedded-00599C?style=flat-square&logo=c%2B%2B&logoColor=white)

---

## 수상 내역
제 1회 U7 메이커스 경진대회 대상 (안양시장상) 수상

<p align="center">
  <img src="./eyevision_award_certificate.jpg" alt="상장" width="50%">
</p>

---

## 1. 프로젝트 소개

기존의 흰 지팡이는 지면 장애물 감지에는 효과적이지만, 다음과 같은 보행 사각지대에는 취약합니다.
1. 머리, 가슴 높이의 튀어나온 상단 장애물 (간판, 트럭 짐칸 등)
2. 갑작스러운 계단이나 턱 같은 하단 낙차 구간

본 프로젝트는 보행자가 인지하기 어려운 사각지대의 위험 요소를 실시간으로 파악하고 버저로 즉각적인 알림을 제공하는데 초점을 두었습니다.

* 개발 기간: 2025.09 ~ 2026.01
* 팀 구성: 3인 (Team Leader, HW, SW)
* 담당 역할: 팀장 
    * 시스템 통합: 센서 데이터 처리 및 전체 회로 설계
    * 알고리즘 구현: 낙상 감지를 위한 미분 로직 및 EMA 필터 적용
    * HW 최적화: 전력 노이즈 개선 및 배터리 효율 관리
---

## 2. 시스템 아키텍처 (System Overview)

### 하드웨어 구성
- MCU: Arduino Nano (ATmega328P) - 소형화 및 전력 효율 고려
- 상단 센서: 초음파 센서 (HC-SR04) - 넓은 범위의 전방 장애물 탐지
- 하단 센서: ToF 거리 센서 (VL53L0X) - 정밀한 거리 감지
- 출력: Active Buzzer - 위험 단계별 경고음 패턴 차별화

### 시스템 블록도 & 로직
| Circuit Schematic | Logic Flowchart |
|:---:|:---:|
| <img src="./eyevision_circuit_diagram.png" width="100%"> | <img src="./eyevision_logic_flowchart.png" width="100%"> |

---

## 3. 핵심 구현 기술 (Key Tech)

### 3.1 Non-blocking 실시간 스케줄링
delay() 함수 사용 시 센싱 중 시스템이 멈추는(Blocking) 문제가 발생합니다. 이를 해결하기 위해 millis() 기반의 논블로킹 구조를 도입하여, 센싱(10Hz) / 필터링 / 경고음 출력이 서로 간섭 없이 독립적으로 수행되도록 구현했습니다.

### 3.2 노이즈 제어: EMA 필터
보행 중 발생하는 진동으로 센서 값이 튀는 현상을 잡기 위해 지수 이동 평균(EMA) 필터를 적용했습니다.
> 수식: S(t) = α * Y(t) + (1 - α) * S(t-1) (α = 0.3)

최신 데이터에 가중치를 두어 노이즈는 평활화하면서도, 장애물 출현 시 반응 속도가 느려지지 않도록 최적화했습니다.

### 3.3 낙상 감지 알고리즘 (Velocity Checking)
단순히 "거리가 50cm 이상이면 위험"이라는 절대값 기준은 각 지형마다(계단, 턱, 경사로) 높이가 달라 오탐지가 잦았습니다.
이를 해결하기 위해 '바닥과의 거리가 변하는 속도(미분값)'를 핵심 지표로 사용했습니다.

- 로직: 보행 중 바닥 거리가 아주 짧은 시간(dt) 내에 급격히 멀어지는 순간을 낙상으로 판단
- 결과: 불규칙한 지형에서도 오탐지 없이 높은은 낙상 감지 성공률 확보

---

## 4. 기술적 의사결정 (Technical Decisions)
프로젝트 진행 중 고민했던 부품 선정 및 설계 이유입니다.

### Q1. 왜 하단 센서로 초음파 대신 ToF를 썼는가?
초음파 센서는 음파가 퍼지는 특성(방사각) 때문에 계단 모서리나 좁은 틈을 정확히 감지하지 못했습니다. 반면 ToF(VL53L1X)는 직진성이 강한 레이저 방식이라 계단 끝부분(Edge)을 칼같이 감지할 수 있어, 안전이 최우선인 하단 감지용으로 채택했습니다.

### Q2. 왜 Arduino Nano를 선택했는가?
초기 프로토타입은 Uno로 개발했으나, 시각장애인이 착용하기에는 부피가 크고 무거웠습니다. 성능(ATmega328P)은 동일하면서 크기는 1/4 수준인 Nano를 사용하여 웨어러블 기기에 적합한 소형화를 달성했습니다.

---

## 5. 테스트 결과 (Result & Demo)

### 하드웨어 및 착용 모습
| Hardware Prototype | Wearing Test |
| :---: | :---: |
| <img src="./eyevision_hardware_prototype.jpg" width="100%"> | <img src="./eyevision_wearing_demo.jpg" width="100%"> |

### 기능별 시연 영상
> 이미지를 클릭하면 유튜브 영상으로 이동합니다. (개인정보 보호를 위해 블러 처리됨)

| 1. 하단 낙상 감지 (ToF) | 2. 상단 충돌 방지 (Ultrasonic) |
| :---: | :---: |
| [![하단](https://img.youtube.com/vi/Dz_auRCq3no/0.jpg)](https://youtu.be/Dz_auRCq3no) | [![상단](https://img.youtube.com/vi/06Roy1dTxnc/0.jpg)](https://youtu.be/06Roy1dTxnc) |
| 계단 진입 시 급격한 거리 변화 감지 | 전방 장애물(사람/벽) 인식 후 경고 |

---

## 6. 향후 과제 (Future Works)
1. 주변 소음 발생 시 청각 알림 인지 어려움 -> 이어폰 출력 단자(또는 블루투스) 추가
2. 경보 전달의 단순성으로 인한 직관성 부족 -> 진동모터나 DFplayer 음성 출력으로 피드백 다양성 확대 
3. 센서 기울기, 사용자 움직임에 따른 오측정 -> 기울기와 가속도를 측정가능한 자이로 센서 도입
