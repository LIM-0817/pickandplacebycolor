import rclpy as rp
from rclpy.node import Node 

import cv2
from cv_bridge import CvBridge

from std_msgs.msg import String
from sensor_msgs.msg import Image

import tf2_ros
import tf_transformations

import numpy as np


class ColorDetector(Node):
    def __init__(self):
        super().__init__('color_detector')

        # 카메라 성능 수치 = Intrinsic parameter
        # 얘네들은 urdf에서 정의한 fov, image, clip 값들로 계산함.
        self.fx = 585.0     # 초점 거리: 렌즈 중심에서 이미지 센서까지 거리(픽셀 단위)
        self.fy = 588.0    
        self.cx = 320.0     # 주점: 렌즈의 정중앙 좌표(픽셀 단위)
        self.cy = 160.0

        # 카메라의 이미지 subscription
        self.create_subscription(Image, '/camera/image_raw', self.callback_image_sub, 10)

        # 각 상자의 색과 좌표를 string 형식으로 발행
        self.coordinate_pub = self.create_publisher(String, '/color_coordinates', 10)

        # CvBridge 선언
        self.bridge = CvBridge()

        # TF
        self.buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.buffer, self)

    def callback_image_sub(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')  # 이미지 받아와서 cv2의 bgr8로 encoding
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)    # HSV로 형식 변환

        # (그대로 가져옴), RGB각각을 HSV로 구분할 때의 H, S, V 각각마다의 범위를 지정 )
        color_ranges = {
            "R": [(0, 120, 70), (10, 255, 255)],
            "G": [(55, 200, 200), (60, 255, 255)],
            "B": [(90, 200, 200), (128, 255, 255)]
        }

        # 각각의 color range 내에 해당하는 영역만 추출
        for color_name, (lower, upper) in color_ranges.items():
            mask = cv2.inRange(hsv, lower, upper)       # lower, upper 사이의 값은 하얀색으로, 나머지는 검정으로 바꿔주는 함수

            # 노이즈를 없앰
            mask = cv2.erode(mask, None, iterations=2)  # 가장자리를 2번 깎아 노이즈 제거
            mask = cv2.dilate(mask, None, iterations=2) # 가장자리를 깎았으므로 dilate를 통해 원래 객체만큼 크기를 다시 키움

            # 가장자리 따기
            # mode = cv2.RETR_EXTERNAL: 물체의 가장 바깥쪽만 땀(도넛이면 중간 구멍은 x) -> 어느 물체가 어디에 속하는지 hierarchy는 안나옴. 따라서 _로 반환 받음.
            # contours는 파이썬 list임. len(contours) = 물체 개수, contours[0] = 첫번째 물체의 외곽선 좌표들, contours[1] = 두번째 ...
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE) 

            if len(contours) > 0:
                for cnt in contours:                        # contours의 외곽선 각각의 좌표들을 따로 가져옴.
                    if cv2.contourArea(cnt) > 100:          # 해당 외곽선 넓이가 100픽셀 이상이어야 처리 
                        x, y, w, h = cv2.boundingRect(cnt)  # cnt의 맨왼쪽 위 좌표(x, y)와 너비, 높이
                        center_x = x + w//2                 # 중심점 찾기(!주의! 컴퓨터 이미지에서는 아래로 갈수록 y가 커짐)
                        center_y = y + h//2

                        # 카메라로 받아오는 이미지(frame)에 물체 인식된 걸 보여주기 위해 네모상자 그리기
                        cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 255), 2) # (네모 위치, 노란색, 두께 2)
                        cv2.putText(frame, color_name, (x, y-10), 
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
                        
                        # pixel로 표시된 좌표를 실제 좌표(m단위)로 변환
                        # 이 물체가 10cm 앞에 있다고 가정하고, 픽셀 위치를 계산해서 로봇이 움직여야 할 X(앞뒤), Y(좌우) 좌표를 뽑아내는 코드
                        Z = 0.1  # 상자의 높이가 10cm
                        Y = (center_x - self.cx) * Z / self.fx * -10
                        X = (center_y - self.cy) * Z / self.fy

                        # camera_link의 관점을 panda_link0의 관점으로 받기 위한 TF 정보 꺼내오기 
                        try: 
                            t = self.buffer.lookup_transform(   # t = TransformStamped 객체
                                'panda_link0',
                                'camera_link',
                                rp.time.Time(),
                                rp.duration.Duration(seconds=0.5) 
                            )

                            # 받아온 TF 정보 값을 변수에 저장
                            x = t.transform.translation.x
                            y = t.transform.translation.y
                            z = t.transform.translation.z

                            t_3 = [x, y, z]                  # TF matrix의 3번째 열

                            q_x = t.transform.rotation.x
                            q_y = t.transform.rotation.y
                            q_z = t.transform.rotation.z
                            q_w = t.transform.rotation.w

                            # TF matrix만들기(4x4 ndarray)
                            T_matrix = tf_transformations.quaternion_matrix([q_x, q_y, q_z, q_w])
                            #    형태:
                            #    [ R R R 0 ]
                            #    [ R R R 0 ]
                            #    [ R R R 0 ]
                            #    [ 0 0 0 1 ]   

                            # 슬라이싱으로 [0, 3], [1, 3], [2, 3] 채워넣기
                            T_matrix[:3, 3] = t_3
                            #    형태:
                            #    [ R R R x ]
                            #    [ R R R y ]
                            #    [ R R R z ]
                            #    [ 0 0 0 1 ] 
                            
                            # 행렬곱으로 X, Y, Z(카메라 관점에서 물체의 위치)를 panda_link0관점의 위치로 변경
                            pt_cam = np.array([X, Y, Z, 1.0])   # (4, )
                            pt_base = T_matrix @ pt_cam         # (4, )

                            # R, B, G에 대한 fine_tuning: x 좌표의 위치 약간 조정
                            if color_name == "B":
                                pt_base[1] -= 0.0215
                            elif color_name == "R": 
                                pt_base[1] -= 0.01
                            elif color_name == "G":
                                pt_base[1] += 0.03

                            # 각 color의 상자에 대한 좌표 publish(self.coordinate_pub), log 띄우기 
                            coord = String()
                            coord.data = f'{color_name}, {pt_base[0]:.3f}, {pt_base[1]:.3f}, {pt_base[2]:.3f}'
                            self.coordinate_pub.publish(coord)
                            self.get_logger().info(coord.data)
                        
                        except (tf2_ros.LookupException, 
                                    tf2_ros.ConnectivityException, 
                                    tf2_ros.ExtrapolationException) as e:
                                self.get_logger().warn(f"TF lookup failed: {e}")

                        except Exception as e:
                            self.get_logger().info(f'Unexpected Error:{e}')
                    else: 
                        self.get_logger().info(f'{color_name} box blocked by robot?')
                        pass
            else: 
                self.get_logger().info(f'{color_name} box blocked by robot?')
                pass
                

        # Color Detection 창 띄우고 fram을 보여주기
        # !!!지금 callback함수가 실행될 때의 코드 이므로 while True 선언 절대 x!!!!
        try:
            cv2.namedWindow("Color Detection", cv2.WINDOW_NORMAL)
            cv2.resizeWindow("Color Detection", 640, 320)
            cv2.imshow("Color Detection", frame)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().warn(f'OpenCV Error: {e}')


def main(args=None):
    rp.init(args=args)
    node = ColorDetector()

    try: 
        rp.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rp.shutdown()
        cv2.destroyAllWindows() # openCV에서 window 열었으므로 닫아주기


if __name__ == '__main__':
    main()
            