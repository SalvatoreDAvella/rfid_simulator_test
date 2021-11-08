#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>

namespace gazebo {
	class ModelControl: public ModelPlugin {
		public:
			void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) {
				this->sdf_ = _sdf;
				// Store the pointer to the model
				this->model = _parent;
				maxspeed = this->sdf_->Get<double>("speed");
				// Store the pointers to the joints
				this->jointR1_ = this->model->GetJoint("rfidantenna_joint");
				this->linkR1_ = this->model->GetLink("rfidantenna");
				// Listen to the update event. This event is broadcast every
				// simulation iteration.
				this->updateConnection = event::Events::ConnectWorldUpdateBegin(
						boost::bind(&ModelControl::OnUpdate, this, _1));

				lowerLimit = this->jointR1_->LowerLimit();
				upperLimit = this->jointR1_->UpperLimit();
				velocity = maxspeed;
			}

			// Called by the world update start event
			void OnUpdate(const common::UpdateInfo & /*_info*/) {
				if (jointR1_->Position(0) >= upperLimit)
					velocity = -maxspeed;
				if (jointR1_->Position(0) <= lowerLimit)
					velocity = maxspeed;

				this->jointR1_->SetVelocity(0, velocity);
			}

		private:
			// Pointer to the model
			physics::ModelPtr model;

			// Pointer to the update event connection
			event::ConnectionPtr updateConnection;

			// Pointers to joints
			physics::JointPtr jointR1_;

			physics::LinkPtr linkR1_;

			sdf::ElementPtr sdf_;

			double lowerLimit;
			double upperLimit;
			double velocity;
			double maxspeed;
	};

	GZ_REGISTER_MODEL_PLUGIN(ModelControl)
}
