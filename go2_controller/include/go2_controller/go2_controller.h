#ifndef GO2_CONTROLLER_H
#define GO2_CONTROLLER_H

#include <memory>
#include <set>
#include <string>

#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/transport/Node.hh>
#include <OsqpEigen/OsqpEigen.h>

#include <gz/plugin/Register.hh>

#include "gz/sim/components/ChildLinkName.hh"
#include "gz/sim/components/Name.hh"
#include "gz/sim/components/Joint.hh"
#include "gz/sim/components/JointAxis.hh"
#include "gz/sim/components/JointForce.hh"
#include "gz/sim/components/JointPosition.hh"
#include "gz/sim/components/JointVelocity.hh"
#include "gz/sim/components/ParentEntity.hh"
#include "gz/sim/components/ParentLinkName.hh"
#include "gz/sim/components/Pose.hh"

#include <gz/math/Pose3.hh>

class Go2Controller : public gz::sim::System,
                      public gz::sim::ISystemConfigure,
                      public gz::sim::ISystemPreUpdate
{
public:
    Go2Controller();

    ~Go2Controller();

    void Configure(const gz::sim::Entity &_entity,
                   const std::shared_ptr<const sdf::Element> &_sdf,
                   gz::sim::EntityComponentManager &_ecm,
                   gz::sim::EventManager &_eventMgr);

    void PreUpdate(const gz::sim::UpdateInfo &_info,
                   gz::sim::EntityComponentManager &_ecm);

private:
    // void CreateComponents(gz::sim::v8::EntityComponentManager &_ecm,
    //                       gz::sim::v8::Entity _joint,
    //                       std::string _joint_name);

    double kp, kd;

    gz::sim::v8::Model model;

    std::string model_name;

    std::vector<gz::sim::v8::Entity> joints;

    Eigen::VectorXd qr, q0;

    std::chrono::steady_clock::duration updatePeriod{0};
};

#endif