#include <cmath>
#include <functional>
#include <stdexcept>
#include <string>

#include "custom_interfaces/action/move_to.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "visualization_msgs/msg/marker.hpp"

int main(int argc, char ** argv)
{
    // 실행 흐름: 좌표 입력 → 서버 연결 → 목표 전송 → 피드백 표시 → 결과 확인.
    // 실제 이동은 서버가 담당하고, 이 클라이언트는 요청과 시각화를 담당한다.
    // ROS 2를 초기화하고 --ros-args 등 ROS 전용 인자를 제거한 인자 목록을 얻는다.
    const auto args = rclcpp::init_and_remove_ros_arguments(argc, argv);
    // using은 긴 타입 이름에 짧은 별칭을 붙인다.
    // MoveTo 액션은 Goal(목표), Feedback(진행 상황), Result(최종 결과)로 구성된다.
    using MoveTo = custom_interfaces::action::MoveTo;
    // 서버가 수락한 목표를 추적하는 클라이언트 측 객체의 타입이다.
    using GoalHandle = rclcpp_action::ClientGoalHandle<MoveTo>;
    using Marker = visualization_msgs::msg::Marker;
    // shared_ptr로 노드를 생성하여 클라이언트, 퍼블리셔, 실행기에서 함께 사용한다.
    const auto node = std::make_shared<rclcpp::Node>("move_to_action_client");

    // 예: ros2 run move_action_cpp move_action_client 1.0 2.0 3.0
    MoveTo::Goal goal;
    try {
        // args[0]은 프로그램 이름이므로 좌표 3개를 포함하면 총 4개여야 한다.
        if (args.size() != 4) {
            throw std::invalid_argument("expected three coordinates");
        }
        // 문자열을 double로 바꾸는 람다(이름 없이 정의하는 함수).
        // []는 바깥 지역 변수를 캡처하지 않는다는 뜻이다.
        const auto coordinate = [](const std::string & value) {
            std::size_t end = 0;
            // end에는 변환이 끝난 문자 위치가 저장된다. 변환 불가/범위 초과 시 예외 발생.
            const double number = std::stod(value, &end);
            // "1.2abc"처럼 뒤에 문자가 남거나 NaN, 무한대인 입력을 거부한다.
            if (end != value.size() || !std::isfinite(number)) {
                throw std::invalid_argument("expected a finite number");
            }
            return number;
        };
        // 검사한 좌표를 서버에 보낼 목표 메시지에 저장한다.
        goal.x = coordinate(args[1]);
        goal.y = coordinate(args[2]);
        goal.z = coordinate(args[3]);
    } catch (const std::exception & error) {
        // 잘못된 입력은 여기에서 처리하고 ROS를 정리한 뒤 종료한다.
        RCLCPP_ERROR(node->get_logger(), "%s. Usage: move_action_client x y z", error.what());
        rclcpp::shutdown();
        return 2;
    }

    // 서버와 액션 타입(MoveTo), 액션 이름(/move_to)이 일치해야 통신할 수 있다.
    const auto client = rclcpp_action::create_client<MoveTo>(node, "/move_to");
    // 서버가 사용 가능해질 때까지 최대 5초 기다린다.
    if (!client->wait_for_action_server(std::chrono::seconds(5))) {
        RCLCPP_ERROR(node->get_logger(), "/move_to is unavailable");
        rclcpp::shutdown();
        return 1;
    }

    // RViz에서 이 토픽을 Marker로 구독하면 현재 위치를 볼 수 있다.
    // 10은 기본 KeepLast QoS에서 보관할 메시지 개수(깊이)이다.
    const auto publisher = node->create_publisher<Marker>("/move_to_marker", 10);
    Marker marker;
    // 위치 좌표는 world 좌표계를 기준으로 해석한다.
    marker.header.frame_id = "world";
    // ns와 id의 조합이 마커를 식별한다. 같은 값을 쓰면 기존 마커가 갱신된다.
    marker.ns = "move_to";
    marker.id = 0;
    marker.type = Marker::CUBE;  // 구체(SPHERE)나 큐브(CUBE) 등 다양한 모양이 있다.
    marker.action = Marker::ADD;
    // 자세는 쿼터니언으로 표현한다. (x, y, z, w) = (0, 0, 0, 1)은 회전 없음.
    marker.pose.orientation.w = 1.0;
    // 구체의 각 축 지름을 0.7로 설정한다.
    marker.scale.x = marker.scale.y = marker.scale.z = 0.7;
    // 색상 성분은 0~1 범위. r은 기본값 0이며, a=1은 완전 불투명이다.
    marker.color.g = 0.8;
    marker.color.b = 1.0;
    marker.color.a = 1.0;

    // 콜백에서 finished를 바꾸면 아래의 이벤트 처리 반복문이 끝난다.
    bool finished = false;
    // 기본 종료 코드는 실패(1). 최종 성공을 확인했을 때만 0으로 바꾼다.
    int exit_code = 1;
    // 목표 수락 여부, 진행 상황, 최종 결과를 처리할 콜백을 등록한다.
    rclcpp_action::Client<MoveTo>::SendGoalOptions options;
    // ① 서버가 목표를 수락/거절했을 때 실행된다.
    // [&]는 사용하는 바깥 지역 변수를 참조로 캡처하므로 원래 변수를 수정할 수 있다.
    options.goal_response_callback = [&](GoalHandle::SharedPtr handle) {
        // 빈 핸들은 목표가 거절되었다는 뜻이다. 수락 시에는 결과를 계속 기다린다.
        if (!handle) {
            RCLCPP_ERROR(node->get_logger(), "Goal rejected");
            finished = true;
        }
    };
    // ② 서버가 이동 중 피드백을 보낼 때마다 실행된다.
    // 첫 번째 인자인 목표 핸들은 이 콜백에서 쓰지 않으므로 이름을 생략했다.
    options.feedback_callback =
        [&](GoalHandle::SharedPtr, const std::shared_ptr<const MoveTo::Feedback> feedback) {
            // 목표 위치가 아니라 서버가 알려준 '현재 위치'로 마커를 갱신한다.
            marker.header.stamp = node->now();
            marker.pose.position.x = feedback->current_x;
            marker.pose.position.y = feedback->current_y;
            marker.pose.position.z = feedback->current_z;
            publisher->publish(marker);
        };
    // ③ 액션이 종료되어 최종 결과가 도착하면 실행된다.
    options.result_callback = [&](const GoalHandle::WrappedResult & result) {
        // ROS 액션 상태와 사용자 정의 success가 모두 성공이어야 한다.
        // &&는 왼쪽부터 평가하므로 결과 포인터가 있을 때만 success에 접근한다.
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED &&
            result.result && result.result->success)
        {
            RCLCPP_INFO(node->get_logger(), "%s", result.result->message.c_str());
            exit_code = 0;
        } else {
            // 실패/취소 등의 종료 상태와 서버 메시지를 출력한다.
            // 삼항 연산자 조건 ? A : B로 결과가 없을 때는 빈 문자열을 사용한다.
            RCLCPP_ERROR(node->get_logger(), "Goal ended with code %d: %s",
                static_cast<int>(result.code),
                result.result ? result.result->message.c_str() : "");
        }
        // 성공 여부와 관계없이 최종 결과를 받았으므로 대기를 끝낸다.
        finished = true;
    };

    // 실행기는 노드의 이벤트를 처리한다. 단일 스레드이므로 콜백은 순차 실행된다.
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    // 비동기 요청: 최종 결과를 기다리며 여기서 멈추지 않고 다음 줄로 진행한다.
    client->async_send_goal(goal, options);
    // ROS가 정상 상태이고 작업이 끝나지 않은 동안 콜백을 처리한다.
    // spin_once의 100ms는 작업을 기다리는 최대 시간이며, 주기 타이머는 아니다.
    // 서버 연결 대기와 달리 이 반복문에는 전체 작업 제한 시간이 없다.
    while (rclcpp::ok() && !finished) {
        executor.spin_once(std::chrono::milliseconds(100));
    }
    if (rclcpp::ok()) {
        // 정상 종료 시 ROS 자원을 정리한다. Ctrl+C 등으로 이미 종료됐다면 생략한다.
        rclcpp::shutdown();
    }
    // 프로세스 종료 코드: 0=성공, 1=연결/목표 수행 실패, 2=입력 오류(위에서 반환).
    return exit_code;
}

/*
1. Node 생성
2. Goal 값 준비
3. Action Client 생성
4. Server 사용 가능할 때까지 대기
5. callback 3개 등록
   - goal_response_callback
   - feedback_callback
   - result_callback
6. async_send_goal()
7. spin_once()로 callback 처리
8. result 도착
9. finished = true
10. 종료
*/