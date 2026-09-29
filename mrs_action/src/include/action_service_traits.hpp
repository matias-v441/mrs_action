#pragma once
#include "action_manager.hpp"

namespace action_manager {

using GotoAction = mrs_action_msgs::action::Goto;
using GotoSrv = mrs_msgs::srv::Vec4;

template<>
struct ActionServiceTraits<GotoAction, GotoSrv> {

    static constexpr std::string_view action_name = "~/goto";
    static constexpr std::string_view service_name = "control_manager/goto";

    static std::shared_ptr<GotoSrv::Request>
    make_request_from_goal(const GotoAction::Goal& action_req){
        auto req = std::make_shared<GotoSrv::Request>();
        req->goal = action_req.goal;
        return req;
    }
};

using PathAction = mrs_action_msgs::action::Path;
using PathSrv = mrs_msgs::srv::PathSrv;

template<>
struct ActionServiceTraits<PathAction, PathSrv> {

    static constexpr std::string_view action_name = "~/path";
    static constexpr std::string_view service_name = "trajectory_generation/path";

    static std::shared_ptr<PathSrv::Request>
    make_request_from_goal(const PathAction::Goal& action_req){
        auto req = std::make_shared<PathSrv::Request>();
        req->path = action_req.path;
        return req;
    }
};

using ReferenceAction = mrs_action_msgs::action::ReferenceStamped;
using ReferenceSrv = mrs_msgs::srv::ReferenceStampedSrv;

template<>
struct ActionServiceTraits<ReferenceAction, ReferenceSrv> {

static constexpr std::string_view action_name = "~/reference";
static constexpr std::string_view service_name = "control_manager/reference";

static std::shared_ptr<ReferenceSrv::Request>
make_request_from_goal(const ReferenceAction::Goal& action_req){
    auto req = std::make_shared<ReferenceSrv::Request>();
    req->header = action_req.header;
    req->reference = action_req.reference;
    return req;
}
};

}