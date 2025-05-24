#include "rclcpp/rclcpp.hpp"
#include "sum_interfaces/action/sum_numbers.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

class SumServer : public rclcpp::Node
{
public:
    SumServer() : Node("sum_server")
    {
        using namespace std::placeholders;

        // Создаем Action-сервер
        server_ = rclcpp_action::create_server<sum_interfaces::action::SumNumbers>(
            this,
            "sum_numbers",
            std::bind(&SumServer::handle_goal, this, _1, _2),
            std::bind(&SumServer::handle_cancel, this, _1),
            std::bind(&SumServer::handle_accepted, this, _1)
        );
    }

private:
    rclcpp_action::Server<sum_interfaces::action::SumNumbers>::SharedPtr server_;

    // Обработчик получения цели (goal)
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        const std::shared_ptr<const sum_interfaces::action::SumNumbers::Goal> goal)
    {
        RCLCPP_INFO(this->get_logger(), "Received goal request: a=%f, b=%f", goal->a, goal->b);
        (void)uuid;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    // Обработчик отмены
    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<sum_interfaces::action::SumNumbers>> /* goal_handle */)
    {
        RCLCPP_INFO(this->get_logger(), "Received cancel request");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    // Обработчик выполнения (расчет суммы)
    void handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<sum_interfaces::action::SumNumbers>> goal_handle)
    {
        std::thread([this, goal_handle]() {
            auto goal = goal_handle->get_goal();
            auto feedback = std::make_shared<sum_interfaces::action::SumNumbers::Feedback>();
            auto result = std::make_shared<sum_interfaces::action::SumNumbers::Result>();

            // Расчет суммы от a до b
            double sum = 0;
            for (double i = goal->a; i <= goal->b; ++i)
            {
                sum += i;
                feedback->current_sum = sum;  // Обновление текущей суммы
                goal_handle->publish_feedback(feedback);
                std::this_thread::sleep_for(std::chrono::milliseconds(500));  // Имитируем длительную операцию
            }

            result->sum = sum;  // Финальный результат
            goal_handle->succeed(result);  // Завершение задачи
        }).detach();
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SumServer>());
    rclcpp::shutdown();
    return 0;
}
