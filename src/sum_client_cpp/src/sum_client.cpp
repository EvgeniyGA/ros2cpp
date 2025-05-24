#include "rclcpp/rclcpp.hpp"
#include "sum_interfaces/action/sum_numbers.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

class SumClient : public rclcpp::Node {
public:
  SumClient()
    : Node("sum_client") {
    client_ = rclcpp_action::create_client<sum_interfaces::action::SumNumbers>(this, "sum_numbers");

    // Таймер для отправки цели
    auto timer_callback = [this]() {
      this->send_goal(1.0, 5.0);
    };
    timer_ = this->create_wall_timer(std::chrono::milliseconds(500), timer_callback);
  }

  void send_goal(double a, double b) {
    using namespace std::placeholders;

    if (!client_->wait_for_action_server()) {
      RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
      rclcpp::shutdown();
    }

    auto goal_msg = sum_interfaces::action::SumNumbers::Goal();
    goal_msg.a = a;
    goal_msg.b = b;

    RCLCPP_INFO(this->get_logger(), "Sending goal");

    // Настройка параметров отправки цели
    rclcpp_action::Client<sum_interfaces::action::SumNumbers>::SendGoalOptions send_goal_options;

    send_goal_options.goal_response_callback = [this](std::shared_ptr<rclcpp_action::ClientGoalHandle<sum_interfaces::action::SumNumbers>> goal_handle) {
      if (!goal_handle) {
        RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
      } else {
        RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
      }
    };

    send_goal_options.feedback_callback = [this](std::shared_ptr<rclcpp_action::ClientGoalHandle<sum_interfaces::action::SumNumbers>> goal_handle, const std::shared_ptr<const sum_interfaces::action::SumNumbers::Feedback> feedback) {
      // Преобразуем UUID в строку
      std::ostringstream uuid_str;
      const auto& goal_id = goal_handle->get_goal_id();
      for (const auto& byte : goal_id) {
        uuid_str << std::hex << static_cast<int>(byte);
      }

      // Логируем UUID цели и текущую сумму
      RCLCPP_INFO(this->get_logger(), "Feedback received for goal with UUID: %s", uuid_str.str().c_str());
      RCLCPP_INFO(this->get_logger(), "Current sum: %f", feedback->current_sum);
    };

    send_goal_options.result_callback = [this](const rclcpp_action::ClientGoalHandle<sum_interfaces::action::SumNumbers>::WrappedResult& result) {
      switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
          RCLCPP_INFO(this->get_logger(), "Result received: sum = %f", result.result->sum);
          break;
        case rclcpp_action::ResultCode::ABORTED:
          RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
          break;
        case rclcpp_action::ResultCode::CANCELED:
          RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
          break;
        default:
          RCLCPP_ERROR(this->get_logger(), "Unknown result code");
          break;
      }
      rclcpp::shutdown();
    };

    client_->async_send_goal(goal_msg, send_goal_options);
    // Останавливаем таймер после первой отправки цели
    timer_->cancel();
  }

private:
  rclcpp_action::Client<sum_interfaces::action::SumNumbers>::SharedPtr client_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SumClient>());
  rclcpp::shutdown();
  return 0;
}
