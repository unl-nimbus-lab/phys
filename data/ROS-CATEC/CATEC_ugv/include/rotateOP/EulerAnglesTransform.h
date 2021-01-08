/*
 * EulerAnglesTransform.h
 *
 *  Created on: 11/12/2012
 *      Author: catec
 */

#ifndef EULERANGLESTRANSFORM_H_
#define EULERANGLESTRANSFORM_H_
#include <Quaternion.h>
#include <Euler.h>
namespace rotateOp {

/*!
 * \brief This class transforms a Euler angle with a specific system to another.
 * Allows to transform form one kind of Euler angle (EULER123, for example) to another (EULER321, for example).
 */
class EulerAnglesTransform {
public:
	/*!
	 * \brief This static method transform a Euler angle with a specific system to another.
	 * One example of use can be the next: transform from EULER123 to EULER321.
	 *
	 *  Euler toTransform(roll,pitch,yaw, TransformationTypes::EULER123);
	 *	Euler transformed = EulerAnglesTransform::transformToSystem(toTransform, TransformationTypes::EULER321);
	 */
	static Euler& transformToSystem(Euler& eulerToTranform,
			TransformationTypes::EulerType type) {
		Quaternion quaternionAux;
		quaternionAux.fromEuler(eulerToTranform.getRoll(),
				eulerToTranform.getPitch(), eulerToTranform.getYaw(),
				eulerToTranform.getEulerType());
		Euler eulerTrans(type);
		eulerTrans.fromQuaternion(quaternionAux);
		return eulerTrans;
	}
};
}

#endif /* EULERANGLESTRANSFORM_H_ */
