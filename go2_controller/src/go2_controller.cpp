#include "go2_controller/go2_controller.h"

Go2Controller::Go2Controller()
{
    std::cout << "Teste" << std::endl;
}

Go2Controller::~Go2Controller()
{
}

void Go2Controller::Configure(const gz::sim::Entity &_entity,
                              const std::shared_ptr<const sdf::Element> &_sdf,
                              gz::sim::EntityComponentManager &_ecm,
                              gz::sim::EventManager &_eventMgr)
{
    this->model = gz::sim::Model(_entity);

    this->model_name = this->model.Name(_ecm);

    this->kp = _sdf->Get<double>("kp");
    std::cout << this->kp << std::endl;
    this->kd = _sdf->Get<double>("kd");
    std::cout << this->kd << std::endl;

    double rate = _sdf->Get<double>("update_rate", 100).first;

    std::chrono::duration<double> period{rate > 0 ? 1 / rate : 0};

    if (_sdf->HasElement("q0"))
        std::cout << "he1" << std::endl;

    if (_sdf->HasElement("b0"))
        std::cout << "he2" << std::endl;

    // std::string b0 = _sdf->Get("b0");

    // std::cout << b0 << std::endl;

    this->updatePeriod =
        std::chrono::duration_cast<std::chrono::steady_clock::duration>(period);

    std::cout << this->model_name << std::endl;
    gz::math::Pose3d p;
    p.Pos().Set(0, 0, 1);

    this->model.SetWorldPoseCmd(_ecm, p);

    this->joints = _ecm.ChildrenByComponents(this->model.Entity(), gz::sim::components::Joint());

    for (const gz::sim::v8::Entity &joint : this->joints)
    {
        // std::cout << joint << std::endl;
    }

    std::cout << "END" << std::endl;
}

// void Go2Controller::CreateComponents(gz::sim::v8::EntityComponentManager &_ecm,
//                                      gz::sim::v8::Entity _joint,
//                                      std::string _joint_name)
// {

//     if (!_ecm.EntityHasComponentType(_joint, gz::sim::v8::components::Joint::typeId))
//     {
//         gzerr << "Entity with name[" << _joint_name
//               << "] is not a joint\n";
//     }
//     else
//     {
//         gzmsg << "Identified joint [" << _joint_name
//               << "] as Entity [" << _joint << "]\n";
//     }

//     this->joints.push_back(_joint);
//     // Create joint position component if one doesn't exist
//     if (!_ecm.EntityHasComponentType(_joint,
//                                      gz::sim::v8::components::JointPosition().TypeId()))
//     {
//         _ecm.CreateComponent(_joint, gz::sim::v8::components::JointPosition());
//         gzmsg << "Create a 'Position Component' for the joint "
//               << _joint_name << ", component: "
//               << _joint << std::endl;
//     }

//     // Create joint velocity component if one doesn't exist
//     if (!_ecm.EntityHasComponentType(_joint,
//                                      gz::sim::v8::components::JointVelocity().TypeId()))
//     {
//         _ecm.CreateComponent(_joint, gz::sim::v8::components::JointVelocity());
//         gzmsg << "Create a 'Velocity Component' for the joint "
//               << _joint_name << ", component: "
//               << _joint << std::endl;
//     }

//     if (!_ecm.EntityHasComponentType(_joint,
//                                      gz::sim::v8::components::JointForceCmd().TypeId()))
//     {
//         _ecm.CreateComponent(_joint, gz::sim::v8::components::JointForceCmd({0}));
//         gzmsg << "Create a 'Force Component' for the joint "
//               << _joint_name << ", component: "
//               << _joint << std::endl;
//     }
// }

void Go2Controller::PreUpdate(const gz::sim::UpdateInfo &_info,
                              gz::sim::EntityComponentManager &_ecm)
{
}

GZ_ADD_PLUGIN(Go2Controller,
              gz::sim::System,
              Go2Controller::ISystemConfigure,
              Go2Controller::ISystemPreUpdate);

GZ_ADD_PLUGIN_ALIAS(Go2Controller, "Go2Controller")