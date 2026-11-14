#include <rclcpp/rclcpp.hpp>                    // ROS2 C++ 클라이언트 라이브러리 헤더
#include <rclcpp_action/rclcpp_action.hpp>      // ROS2 C++ 액션 서버 헤더

#include "custom_interfaces/action/move_to.hpp" // 사용자 정의 액션 메시지 헤더

#include <thread>                               // C++ 스레드 헤더
#include <chrono>                               // C++ 시간 관련 헤더
#include <cmath>

struct Position
{
    double x;
    double y;
    double z;
};

class MoveToActionServer : public rclcpp::Node
{
public:
    using MoveTo = custom_interfaces::action::MoveTo;
    using GoalHandleMoveTo =
        rclcpp_action::ServerGoalHandle<MoveTo>;

    MoveToActionServer()
        : Node("move_to_action_server")
    {
        action_server_ = rclcpp_action::create_server<MoveTo>(
            this,
            "move_to",
            std::bind(
                &MoveToActionServer::handleGoal,
                this,
                std::placeholders::_1,
                std::placeholders::_2
            ),
            std::bind(
                &MoveToActionServer::handleCancel,
                this,
                std::placeholders::_1
            ),
            std::bind(
                &MoveToActionServer::handleAccepted,
                this,
                std::placeholders::_1
            )
        );
        RCLCPP_INFO(
            this->get_logger(),
            "MoveTo Action Server 시작"
        );
    }
private:
    rclcpp_action::Server<MoveTo>::SharedPtr action_server_;
    Position current_position_{0.0, 0.0, 0.0}; // 현재 위치 초기화

    rclcpp_action::GoalResponse handleGoal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const MoveTo::Goal> goal)
    {
        (void)uuid;

        RCLCPP_INFO(
            this->get_logger(),
            "Goal 요청: x=%.2f, y=%.2f, z=%.2f",
            goal->x,
            goal->y,
            goal->z
        );

        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }
    rclcpp_action::CancelResponse handleCancel(
        const std::shared_ptr<GoalHandleMoveTo> goal_handle)
    {
        (void)goal_handle;

        RCLCPP_INFO(
            this->get_logger(),
            "Goal 취소 요청"
        );

        return rclcpp_action::CancelResponse::ACCEPT;
    }
    void handleAccepted(
        const std::shared_ptr<GoalHandleMoveTo> goal_handle)
    {
        (void)goal_handle;

        RCLCPP_INFO(
            this->get_logger(),
            "Goal 수락됨"
        );

        std::thread{            // 별도의 스레드에서 execute() 실행
            std::bind(
                &MoveToActionServer::execute,
                this,
                std::placeholders::_1
            ),
            goal_handle
        }.detach();
    }
    void execute(
        const std::shared_ptr<GoalHandleMoveTo> goal_handle)
    {
        const auto goal = goal_handle->get_goal();

        auto feedback =
            std::make_shared<MoveTo::Feedback>();

        auto result =
            std::make_shared<MoveTo::Result>();
        
        const double hz = 30.0; // 30Hz로 이동 시뮬레이션
        rclcpp::Rate rate(hz);  // 30Hz로 루프 실행

        // const double duration_sec = 5.0; // 5초 동안 이동
        Position start_position = current_position_;
        
        double dx = goal->x - start_position.x;
        double dy = goal->y - start_position.y;
        double dz = goal->z - start_position.z;
        double distance = std::sqrt(dx * dx + dy * dy + dz * dz);
        double speed = 10.0; // 이동 속도 (좌표 단위 / sec)
        double duration_sec = distance / speed; // 이동 시간 계산
        
        int total_steps = static_cast<int>(duration_sec * hz);

        if (total_steps < 1){
            total_steps = 1; // 최소 단계 수를 1로 설정
        }

        // 이동을 total_steps 단계로 가상 구현
        for (int i = 1; i <= total_steps; i++)
        {
            
            // 취소 요청 확인
            if (goal_handle->is_canceling())
            {
                result->success = false;
                result->message = "이동이 취소되었습니다.";

                goal_handle->canceled(result);

                RCLCPP_INFO(
                    this->get_logger(),
                    "Goal 취소됨"
                );

                return;
            }

            // 현재 단계 비율 계산
            // double ratio = static_cast<double>(i) / total_steps;    // 현재 단계 비율 계산

            // 부드럽게 이동하기 위해 사인 함수를 사용하여 비율 계산
            double t = static_cast<double>(i) / total_steps; // 현재 단계 비율 계산
            double ratio = t * t *(3.0 - 2.0 * t); // Smoothstep 함수 사용

            // 현재 위치를 목표 위치까지 조금씩 증가
            feedback->current_x = start_position.x + dx * ratio;
            feedback->current_y = start_position.y + dy * ratio;
            feedback->current_z = start_position.z + dz * ratio;
            
            // 현재 위치 업데이트
            current_position_.x = feedback->current_x;
            current_position_.y = feedback->current_y;
            current_position_.z = feedback->current_z;

            goal_handle->publish_feedback(feedback);    // 피드백 전송

            // RCLCPP_INFO(
            //     this->get_logger(),
            //     "현재 위치: (%.2f, %.2f, %.2f)",
            //     feedback->current_x,
            //     feedback->current_y,
            //     feedback->current_z
            // );
            
            // 루프 주기 유지
            rate.sleep();
        }

        // 목표 위치에 도달했음을 알림
        current_position_.x = goal->x;
        current_position_.y = goal->y;
        current_position_.z = goal->z;

        // 목표 위치 도착
        result->success = true;
        result->message = "목표 위치에 도착했습니다.";

        goal_handle->succeed(result);

        RCLCPP_INFO(
            this->get_logger(),
            "이동 완료"
        );
    }
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    auto node =
        std::make_shared<MoveToActionServer>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}