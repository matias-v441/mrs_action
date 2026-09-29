#pragma once
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>

#include <mrs_action_msgs/action/goto.hpp>
#include <mrs_action_msgs/action/path.hpp>
#include <mrs_action_msgs/action/reference_stamped.hpp>

#include <mrs_msgs/srv/vec4.hpp>
#include <mrs_msgs/srv/path_srv.hpp>
#include <mrs_msgs/srv/reference_stamped_srv.hpp>
#include <mrs_msgs/msg/control_manager_diagnostics.hpp>

#include <atomic>
#include <cassert>
#include <chrono>
using namespace std::chrono_literals;
#include <concepts>

namespace action_manager {

struct IActionManager{
    virtual ~IActionManager() = default;
};

template<typename ActionT, typename ServiceT>
struct ActionServiceTraits;

template<typename T>
concept CompleteType = requires {
  sizeof(T);
};

template<typename TraitsT, typename ActionT, typename ServiceT>
concept ValidActionServiceTraits =
  CompleteType<TraitsT> &&
  requires(const typename ActionT::Goal& goal) {
    { TraitsT::action_name } -> std::convertible_to<std::string_view>;
    { TraitsT::service_name } -> std::convertible_to<std::string_view>;
    { TraitsT::make_request_from_goal(goal) }
      -> std::same_as<std::shared_ptr<typename ServiceT::Request>>;
  };

template<typename ActionT, typename ServiceT>
requires ValidActionServiceTraits<ActionServiceTraits<ActionT, ServiceT>, ActionT, ServiceT>
class ActionManager : public IActionManager {
public:
    using GoalHandle = rclcpp_action::ServerGoalHandle<ActionT>;
    using ClientFuture = typename rclcpp::Client<ServiceT>::FutureAndRequestId; 
    using Trait = ActionServiceTraits<ActionT,ServiceT>;

    enum class State{IDLE, REQUESTING, WAITING, FLYING};

    ActionManager(rclcpp::Node &node, float update_interval)
        : node_(&node){

        RCLCPP_INFO(node_->get_logger(), "Started %s", Trait::action_name.data());

        action_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        diag_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

        using ActionManagerT = ActionManager<ActionT,ServiceT>;

        action_server_ = rclcpp_action::create_server<ActionT>(
            node_,
            Trait::action_name.data(),
            std::bind(&ActionManagerT::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&ActionManagerT::handle_cancel, this, std::placeholders::_1),
            std::bind(&ActionManagerT::handle_accepted, this, std::placeholders::_1),
            rcl_action_server_get_default_options(),
            action_group_
            );

        const auto diag_qos = rclcpp::QoS(rclcpp::KeepLast(1))
            .best_effort()
            .durability_volatile();

        rclcpp::SubscriptionOptions sub_options;
        sub_options.callback_group = diag_group_;

        subscription_ = node_->create_subscription<mrs_msgs::msg::ControlManagerDiagnostics>(
            "control_manager/diagnostics",
            diag_qos,
            std::bind(&ActionManagerT::topic_callback, this, std::placeholders::_1),
            sub_options
        );

        client_ = node_->create_client<ServiceT>(Trait::service_name.data());

        update_timer_ = rclcpp::create_timer(
            node_,
            node_->get_clock(),
            rclcpp::Duration::from_seconds(update_interval),
            std::bind(&ActionManagerT::on_update_timer, this),
            action_group_,
            false
        );
    }

private:

    inline State state(){return _state_;}

    void set_state(State new_state){
        if(_state_ == new_state) return;
        State old_state = _state_;
        _state_ = new_state;

        RCLCPP_INFO(node_->get_logger(), "transition %d --> %d", 
            static_cast<unsigned char>(old_state),
            static_cast<unsigned char>(new_state));

        if(goal_handle_ && !goal_handle_->is_canceling()){
            auto fb = std::make_shared<typename ActionT::Feedback>();
            fb->state = static_cast<unsigned char>(state());
            goal_handle_->publish_feedback(fb);
        }
    }

    void on_update_timer(){
        assert(goal_handle_ != nullptr);
        if(goal_handle_->is_canceling()){
            cancel_goal();
            return;
        }
        if(client_future_.has_value()){
            if(client_future_.value().wait_for(0s) == std::future_status::ready){
                try{
                    auto resp = client_future_.value().get();
                    if(resp->success) {
                        future_succeeded_ = true;
                    } else {
                        finish(false, resp->message);
                    }
                }catch(const std::exception&){
                    finish(false, "client future failed");
                }
                client_future_.reset();
            } else {
                RCLCPP_INFO(node_->get_logger(), "Waiting for service response...", static_cast<unsigned char>(_state_));
            }
            if(!future_succeeded_){
                return;
            }
        }

        while(id_cons_ != id_prod_.load(std::memory_order_acquire)) {

            RCLCPP_INFO(node_->get_logger(), "STATE %d RECEIVED have_goal=%d", static_cast<unsigned char>(_state_), have_goal_hist_[id_cons_]);

            bool have_goal = have_goal_hist_[id_cons_];

            switch (state())
            {
            case State::IDLE:
                if(have_goal){
                    set_state(State::WAITING), ++id_cons_;
                }else{
                    set_state(State::REQUESTING), ++id_cons_;
                    future_succeeded_ = false;
                    client_future_ = send_request();
                }
                break;
            case State::REQUESTING:
                if(future_succeeded_ && have_goal){
                    set_state(State::FLYING), ++id_cons_;
                }else{return;}
                break;
            case State::WAITING:
                if(!have_goal){
                    set_state(State::REQUESTING), ++id_cons_;
                    future_succeeded_ = false;
                    client_future_ = send_request();
                }else{return;}
                break;
            case State::FLYING:
                if(!have_goal){
                    finish(true, "finished flying");
                }
                return;
            }
        }
    }

    void topic_callback(const mrs_msgs::msg::ControlManagerDiagnostics msg){
        int id_prod = id_prod_.load(std::memory_order_acquire);
        if(id_prod < 0 || id_prod == static_cast<int>(have_goal_hist_.size())) return;
        bool have_goal = msg.tracker_status.have_goal;
        if(id_prod > 0 && have_goal_hist_[id_prod-1] == have_goal) return;
        have_goal_hist_[id_prod] = have_goal;
        int expected = std::max(0, id_prod);
        id_prod_.compare_exchange_strong(expected, id_prod+1, std::memory_order_acq_rel, std::memory_order_relaxed);
    }

    inline void stop_diag_history(){
        id_prod_.store(-1, std::memory_order_release);
        id_cons_ = -1;
    }

    inline void start_diag_history(){
        id_prod_.store(0, std::memory_order_release);
        id_cons_ = 0;
    }

    inline ClientFuture send_request(){
        return client_->async_send_request(Trait::make_request_from_goal(*goal_handle_->get_goal()));
    }

    void cancel_goal(){
        assert(goal_handle_ != nullptr);
        assert(goal_handle_->is_canceling());
        RCLCPP_INFO(node_->get_logger(), "Canceled");
        set_state(State::IDLE);
        stop_diag_history();
        auto res = std::make_shared<typename ActionT::Result>();
        res->success = false;
        res->message = "canceled";
        goal_handle_->canceled(res);
        goal_handle_.reset();
        handle_accepted_ = false;
        if(update_timer_) update_timer_->cancel();
    }

    void finish(bool success, const std::string& message){
        RCLCPP_INFO(node_->get_logger(), "Finished: %s | %s",
                    success? "success" : "failure", message.data());
        set_state(State::IDLE);
        stop_diag_history();
        auto res = std::make_shared<typename ActionT::Result>();
        res->success = success;
        res->message = message;
        if(res->success){
            goal_handle_->succeed(res);
        }else{
            goal_handle_->abort(res);
        }
        goal_handle_.reset();
        handle_accepted_ = false;
        update_timer_->cancel();
    }

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID &,
        std::shared_ptr<const typename ActionT::Goal>){

        if(handle_accepted_){
            RCLCPP_INFO(node_->get_logger(), "Goal rejected");
            return rclcpp_action::GoalResponse::REJECT;
        }
        RCLCPP_INFO(node_->get_logger(), "Goal accepted");
        handle_accepted_ = true;
        start_diag_history();
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandle>){

        RCLCPP_INFO(node_->get_logger(), "Received cancel request");
        if(goal_handle_ == nullptr || state() == State::REQUESTING || state() == State::FLYING){
            return rclcpp_action::CancelResponse::REJECT;
        }
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle){
        RCLCPP_INFO(node_->get_logger(), "Handle accepted");
        assert(goal_handle_ == nullptr);
        goal_handle_ = goal_handle;
        if(goal_handle_->is_canceling()){
            cancel_goal();
            return;
        }
        update_timer_->reset();
    }

private:

    rclcpp::Node* node_{};

    rclcpp::Subscription<mrs_msgs::msg::ControlManagerDiagnostics>::SharedPtr subscription_{};

    typename rclcpp_action::Server<ActionT>::SharedPtr action_server_{};
    
    rclcpp::CallbackGroup::SharedPtr action_group_{};
    rclcpp::CallbackGroup::SharedPtr diag_group_{};

    typename rclcpp::Client<ServiceT>::SharedPtr client_{};

    std::optional<ClientFuture> client_future_{};

    State _state_{};
    rclcpp::TimerBase::SharedPtr update_timer_{};

    std::array<bool,4> have_goal_hist_{}; // f | t | ftf | tftf
    std::atomic<int> id_prod_{-1};
    int id_cons_{-1};
    bool future_succeeded_{};
    bool handle_accepted_{};
    
    std::shared_ptr<GoalHandle> goal_handle_{};
};

}