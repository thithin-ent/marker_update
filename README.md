# marker_update

Marker_slam은 lidar를 기반으로 한 map에서 ArUco 마커의 위치를 자동으로 등록하기 위한 패키지입니다. Marker_slam 패키지를 실행하기 위해선 다른 localization이나 slam 패키지를 통해 로봇의 위치를 추정하여 map과 base_link간의 transform정보가 있어야 하고, robot_state_publisher를 통해 base_link와 camera_link간의 transform정보가 있어야합니다. Marker_slam은 이미 만들어진 Grid map에서 lidar localization을 이용하여 로봇위치를 추정한 후 마커를 mapping할 수 있습니다.

<p align="center"><img src="/fig/image.gif"></p>

# How to use
marker_slam 기능을 사용하기 위해서는 먼저 lidar를 이용한 map이 필요합니다. lidar로 map을 제작하기 위한 명령어는 다음과 같습니다.

    roslaunch marker_slam lidar_mapping.launch

lidar로 map을 제작한 후 다음 명령어로 map을 저장합니다.

    roslaunch marker_slam lidar_map_save.launch

이후 lidar로 만든 map을 기반으로 마커의 맵을 제작합니다. 해당 명령어는 다음과 같습니다.

    roslaunch marker_slam marker_update.launch


# Subscribed Topic & 입력 정보
/Fiducial_transfrom (indoor_2d_nav_aruco_detect/FiducialTransformArray_i2n)
 - 카메라에서 감지된 마커의 위치정보입니다. aruco_detect의 Fiducial_transfrom에서 marker의 id정보가 추가된 형태입니다.

/tf (tf/tfMessage)
 - Transforms정보입니다. base_link → camera_link, map → base_link의 관계가 tf에서 발행되고 있어야 합니다.

/initailpose (geometry_msgs/PoseWithCovarianceStamped)
 - 로봇의 위치를 재조정하기 위한 정보입니다.

# published Topic & 출력 정보
Marker map
 - Gridmap을 기반으로 한 marker map의 좌표 위치입니다. 저장되는 정보는 marker의 번호, x, y, z, 쿼터니안 회전좌표가 저장됩니다. Marker_slam의 marker폴더 위치에 .txt파일 형태로 저장됩니다.

# Parameters
Landmark_N (int)
 - 저장할 마커의 최대 개수입니다. 마커의 최대 개수는 마커 번호에 영향을 받기 때문에, 만일 id 100의 ArUco 마커를 사용하기 위해선 Landmark_N은 최소 101이상이 되어야 합니다.

Marker_txt (str)
 - 저장되는 marker map의 이름입니다. Marker_slam의 marker폴더 위치에 지정된 이름으로 저장됩니다.
