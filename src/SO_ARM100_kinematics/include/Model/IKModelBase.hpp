#pragma once

#include "KinematicModel.hpp"

namespace SOArm100::Kinematics::Model
{
class IKModelBase
{
public:
IKModelBase( Model::KinematicModelConstPtr model ) :
	model_( model )
{
	if ( !model || model->IsEmpty() )
		throw std::invalid_argument( "model must not be null or empty" );
}

protected:
Model::KinematicModelConstPtr model_;

const Model::JointChain* GetChain() const {
	if ( !model_ )
		return nullptr;
	return model_->GetChain();
}

const Model::Joint* GetActiveJoint( int index ) const {
	if ( !model_ )
		return nullptr;
	return model_->GetChain()->GetActiveJoint( index ).get();
}
};
}