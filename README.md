# ROS 2 C++ 실습 워크스페이스

ROS 2 Humble에서 C++ 토픽, 서비스, 액션을 실습하는 워크스페이스입니다. 현재 MoveTo 액션 서버와 클라이언트로 목표 좌표를 보내고, 이동 피드백을 RViz에서 확인할 수 있습니다.

| 패키지 | 내용 |
| --- | --- |
| `cpp_pubsub` | C++ 토픽 발행·구독 예제 |
| `cpp_service` | 덧셈 및 그리퍼 서비스 서버 예제 |
| `custom_interfaces` | `AddNums.srv`, `MoveTo.action` 정의 |
| `move_action_cpp` | MoveTo 서버·클라이언트 및 RViz 설정 |
| `bfs_pick_place` | BFS Pick & Place 실습용 패키지 틀 |

## 환경과 빌드

ROS 2 Humble과 `colcon`이 필요합니다. RViz 예제를 실행하려면 Humble 환경에 `rviz2`도 설치되어 있어야 합니다. 아래 명령은 Humble이 `~/ros2_humble/install`에 설치된 환경을 기준으로 합니다.

```bash
git clone https://github.com/JeoKSJ/ros2_cpp.git ~/ros2_cpp
cd ~/ros2_cpp
source ~/ros2_humble/install/setup.bash
colcon build --symlink-install
source install/setup.bash
```

이미 클론했다면 `cd ~/ros2_cpp`에서 `git pull --ff-only`를 실행한 뒤 다시 빌드합니다.

## MoveTo 액션 실행

새 터미널마다 아래 환경을 불러옵니다.

```bash
source ~/ros2_humble/install/setup.bash
source ~/ros2_cpp/install/setup.bash
```

첫 번째 터미널에서 서버를 실행합니다.

```bash
ros2 run move_action_cpp move_action_server
```

RViz에서 확인하려면 클라이언트 실행 전에 별도 터미널에서 RViz를 엽니다.

```bash
rviz2 -d "$(ros2 pkg prefix move_action_cpp)/share/move_action_cpp/config/move_to.rviz"
```

다른 터미널에서 목표 좌표 `x y z`를 보냅니다.

```bash
ros2 run move_action_cpp move_action_client 0.3 0.2 0.1
```
