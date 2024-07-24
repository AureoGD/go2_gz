#include "go2_controller/go2_controller.h"

Go2Controller::Go2Controller()
{
    std::cout << "GO2 CONTROLLER" << std::endl;
    this->q0.resize(12, 1);
    this->q0.setZero();
    this->b0.resize(6, 1);
    this->b0.setZero();
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

    this->kd = _sdf->Get<double>("kd");

    double rate = _sdf->Get<double>("update_rate", 100).first;

    std::chrono::duration<double> period{rate > 0 ? 1 / rate : 0};

    std::string dataFromSDF;

    if (_sdf->HasElement("q0"))
    {
        dataFromSDF = _sdf->Get<std::string>("q0");
        this->DataVectorSDF(dataFromSDF, this->q0);
    }
    else
    {
        this->q0 << 0.0, 1.36, -2.65,
            0.0, 1.36, -2.65,
            0.2, 1.36, -2.65,
            0.2, 1.36, -2.65;
    }

    dataFromSDF = "";

    if (_sdf->HasElement("p0"))
    {
        dataFromSDF = _sdf->Get<std::string>("p0");
        this->DataVectorSDF(dataFromSDF, this->b0);
    }
    else
    {
        this->b0 << 0, 0, 2, 0, 0, 0;
    }

    this->updatePeriod =
        std::chrono::duration_cast<std::chrono::steady_clock::duration>(period);

    this->p0.Set(this->b0[0], this->b0[1], this->b0[2], this->b0[3], this->b0[4], this->b0[5]);

    this->model.SetWorldPoseCmd(_ecm, this->p0);

    this->joint_entities = _ecm.ChildrenByComponents(this->model.Entity(), gz::sim::components::Joint());

    for (const gz::sim::v8::Entity &joint : this->joint_entities)
    {
        this->CreateComponents(_ecm, joint);
    }

    for (int idc = 0; idc < this->joint_entities.size(); idc++)
    {
        _ecm.SetComponentData<gz::sim::components::JointPositionReset>(this->joint_entities[idc], {this->q0[idc]});
    }

    this->qr.resize(12, 1);
    this->qr.setZero();
    this->qr << 0.0, 0.67, -1.3, 0.0, 0.67, -1.3,
        0.0, 0.67, -1.3, 0.0, 0.67, -1.3;

    std::cout << "GO2 Controller Configured" << std::endl;
}

void Go2Controller::DataVectorSDF(const std::string _data, Eigen::VectorXd &_vec)
{
    std::string aux;
    int pos = 0;
    for (int idx = 0; idx <= _data.size(); idx++)
    {
        if (_data[idx] != ',' && _data[idx] != '\0')
        {
            aux += _data[idx];
        }
        else
        {
            _vec[pos] = std::stod(aux);
            pos++;
            aux = "";
        }
    }
}

void Go2Controller::CreateComponents(gz::sim::v8::EntityComponentManager &_ecm,
                                     gz::sim::v8::Entity _joint)
{
    this->joint_names.push_back(_ecm.ComponentData<gz::sim::components::Name>(_joint).value());

    if (!_ecm.EntityHasComponentType(_joint,
                                     gz::sim::v8::components::JointPosition().TypeId()))
    {
        _ecm.CreateComponent(_joint, gz::sim::v8::components::JointPosition());
    }

    if (!_ecm.EntityHasComponentType(_joint,
                                     gz::sim::v8::components::JointVelocity().TypeId()))
    {
        _ecm.CreateComponent(_joint, gz::sim::v8::components::JointVelocity());
    }

    if (!_ecm.EntityHasComponentType(_joint,
                                     gz::sim::v8::components::JointForceCmd().TypeId()))
    {
        _ecm.CreateComponent(_joint, gz::sim::v8::components::JointForceCmd({0}));
    }
}

void Go2Controller::PreUpdate(const gz::sim::UpdateInfo &_info,
                              gz::sim::EntityComponentManager &_ecm)
{
    if (this->joint_entities.empty())
        return;

    // Nothing left to do if paused.
    if (_info.paused)
        return;

    auto elapsed = _info.simTime - this->lastUpdateTime;

    if (elapsed > std::chrono::steady_clock::duration::zero() && elapsed < this->updatePeriod)
        return;

    for (int idx = 0; idx < this->joint_entities.size(); idx++)
    {
        const auto *jointPositions = _ecm.Component<gz::sim::v8::components::JointPosition>(this->joint_entities[idx]);
        const auto *jointVelocity = _ecm.Component<gz::sim::v8::components::JointVelocity>(this->joint_entities[idx]);

        if (jointPositions == nullptr || jointPositions->Data().empty() || jointVelocity == nullptr || jointVelocity->Data().empty())
            return;

        double tau = this->kp * (this->qr[idx] - jointPositions->Data()[0]) - this->kd * jointVelocity->Data()[0];
        auto forceComp = _ecm.Component<gz::sim::v8::components::JointForceCmd>(this->joint_entities[idx]);
        *forceComp = gz::sim::v8::components::JointForceCmd({tau});
    }
    this->lastUpdateTime = _info.simTime;
}

GZ_ADD_PLUGIN(Go2Controller,
              gz::sim::System,
              Go2Controller::ISystemConfigure,
              Go2Controller::ISystemPreUpdate);

GZ_ADD_PLUGIN_ALIAS(Go2Controller, "Go2Controller")