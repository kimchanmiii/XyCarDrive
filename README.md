# XyCarDrive

IT 융합 기술의 발달로 전 세계적으로 자율 주행에 대한 관심이 높아지면서 관련 연구가 활발하게 진행되고 있다. 자율 주행을 구현하기 위한 기초로는 주변상황을 인식하여 분석해 적절한 제어를 하는 것이 중요하다. 이를 위해 카메라, 초음파, 라이다(LiDAR), 레이더, GPS 등 다양한 센서를 입력장치로 사용하는데 이 중 카메라는 주변 상황과 차선을 인식하는데, 초음파 센서는 장애물을 탐지하는데, 저비용으로 다양한 환경에서 사용할 수 있다는 장점이 있다.

카메라를 이용한 차선 인식은 자율 주행을 위한 기본 동작이다. 카메라를 통해 보여지는 차선의 경우 항상 직선이지 않고, 차량의 진동/떨림/위치 등의 영향과 주변 조도에 따른 빛 번짐 등의 환경적인 영향 때문에 처리해야 될 예외사항이 많이 발생하게 되어 다양한 방식이 존재하는데, HSV 이진화를 이용하는 방법, 허프(Hough) 변환을 이용하는 방법, 히스토그램(Histogram)을 이용하는 방법 등이 있다. 본 연구에서는 허프 변환을 중점으로 사용해 이미지에서 차선을 효과적으로 검출하여 구동체를 제어하고자 한다.
