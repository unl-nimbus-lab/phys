/*
 * Euler.h
 *
 *  Created on: 11/12/2012
 *      Author: catec
 */

#ifndef EULER_H_
#define EULER_H_
#include <math.h>
#include <iostream>
#include <TransformationTypes.h>
#include <Quaternion.h>

namespace rotateOp {
/*!
 * \brief This class defines a euler angle.
 * the euler angles are three angles to describe the orientation of a rigid body.
 * Euler angles represent three composed rotations that move a reference frame to a given referred frame. Any
 * orientation can be achieved by composing three elemental rotations (rotations around a single axis). The order
 * to rotate the three angles over this axis is important to determine the final position of the body with its reference frame.
 * The Euler angles will be roll, pitch and yaw.
 */
class Euler {
private:
	double roll;
	double pitch;
	double yaw;

	TransformationTypes::EulerType eulerType;

public:

	/*!
	 * \brief Constructor that generates a new Euler object with a specific system.
	 * \param eulerType The euler system that could be EULER123, EULER321, EULER312.
	 */
	Euler(TransformationTypes::EulerType eulerType) {
		this->eulerType = eulerType;
		this->roll = 0;
		this->pitch = 0;
		this->yaw = 0;
	}
	/*!
	 * \brief Constructor that generates a new Euler object with a specific system and rotation.
	 * \param roll the rotation in x axis.
	 * \param pitch the rotation in y axis.
	 * \param yaw the rotation in z axis.
	 * \param eulerType The euler system that could be EULER123, EULER321, EULER312.
	 */
	Euler(double roll, double pitch, double yaw,
			TransformationTypes::EulerType eulerType) {
		this->eulerType = eulerType;
		this->roll = MathHelper::roundDecimal(roll,5);
		this->pitch = MathHelper::roundDecimal(pitch,5);
		this->yaw = MathHelper::roundDecimal(yaw,5);
	}

	/*!
	 * \brief This method sets the euler data with quaternion data.
	 * Sets the roll, pitch, yaw components of the object using quaternion data and a specific euler system (EULER123, EULER312,...).
	 * The euler system (euler321, euler123, ...) must be specified to know what is the transformation to applied.
	 * \param quaternion The quaternion data to set.
	 */
	void fromQuaternion(Quaternion quaternion) {

		switch (eulerType) {
		case TransformationTypes::EULER123:
			roll = MathHelper::roundDecimal(atan2(-2
					* (quaternion.getY() * quaternion.getZ()
							- quaternion.getW() * quaternion.getX()),
					quaternion.getW() * quaternion.getW() - quaternion.getX()
							* quaternion.getX() - quaternion.getY()
							* quaternion.getY() + quaternion.getZ()
							* quaternion.getZ()), 5);
			pitch = MathHelper::roundDecimal(asin(2
					* (quaternion.getX() * quaternion.getZ()
							+ quaternion.getW() * quaternion.getY())), 5);
			yaw = MathHelper::roundDecimal(atan2(-2
					* (quaternion.getX() * quaternion.getY()
							- quaternion.getW() * quaternion.getZ()),
					quaternion.getW() * quaternion.getW() + quaternion.getX()
							* quaternion.getX() - quaternion.getY()
							* quaternion.getY() - quaternion.getZ()
							* quaternion.getZ()), 5);

			break;
		case TransformationTypes::EULER321:
			yaw = MathHelper::roundDecimal(atan2(2
					* (quaternion.getY() * quaternion.getX()
							- quaternion.getZ() * quaternion.getW()),
					quaternion.getW() * quaternion.getW() + quaternion.getX()
							* quaternion.getX() - quaternion.getY()
							* quaternion.getY() - quaternion.getZ()
							* quaternion.getZ()), 5);
			pitch = MathHelper::roundDecimal(asin(-2
					* (quaternion.getX() * quaternion.getZ()
							- quaternion.getW() * quaternion.getY())), 5);
			roll = MathHelper::roundDecimal(atan2(2
					* (quaternion.getZ() * quaternion.getY()
							- quaternion.getW() * quaternion.getX()),
					quaternion.getW() * quaternion.getW() - quaternion.getX()
							* quaternion.getX() - quaternion.getY()
							* quaternion.getY() + quaternion.getZ()
							* quaternion.getZ()), 5);

			break;
		case TransformationTypes::EULER312:
			yaw = MathHelper::roundDecimal(asin(2
					* (quaternion.getY() * quaternion.getZ()
							+ quaternion.getX() * quaternion.getW())), 5);
			roll = MathHelper::roundDecimal(atan2(2
					* (-quaternion.getX() * quaternion.getZ()
							+ quaternion.getY() * quaternion.getW()),
					quaternion.getZ() * quaternion.getZ() - quaternion.getY()
							* quaternion.getY() - quaternion.getX()
							* quaternion.getX() + quaternion.getW()
							* quaternion.getW()), 5);
			pitch = MathHelper::roundDecimal(atan2(2
					* (-quaternion.getX() * quaternion.getY()
							+ quaternion.getZ() * quaternion.getW()),
					quaternion.getY() * quaternion.getY() - quaternion.getZ()
							* quaternion.getZ() + quaternion.getW()
							* quaternion.getW() - quaternion.getX()
							* quaternion.getX()), 5);

			break;
		default:
			break;
		}

	}

	bool operator==(const Euler& aux) const {
		if (roll == aux.roll && yaw == aux.yaw && pitch == aux.pitch
				&& eulerType == aux.eulerType)
			return true;
		else
			return false;
	}

	Euler& operator=(const Euler& aux) {
		roll = aux.roll;
		pitch = aux.pitch;
		yaw = aux.yaw;
		eulerType = aux.eulerType;

		return *this;
	}

	friend std::ostream & operator <<(std::ostream & o, Euler e) {
		switch (e.eulerType) {
		case TransformationTypes::EULER123:
			o << "Euler123: [Roll: " << e.roll << ", Pitch: " << e.pitch
					<< ", Yaw: " << e.yaw << "]";
			break;
		case TransformationTypes::EULER321:
			o << "Euler321: [Yaw: " << e.yaw << ", Pitch: " << e.pitch
					<< ", Roll: " << e.roll << "]";
			break;
		case TransformationTypes::EULER312:
			o << "Euler312: [Yaw: " << e.yaw << ", roll: " << e.roll
					<< ", Pich: " << e.pitch << "]";
			break;
		default:
			o << "Euler: [Roll: " << e.roll << ", Pitch: " << e.pitch
					<< ", Yaw: " << e.yaw << "]";
			break;
		}

		return o;
	}

	double getPitch() const {
		return pitch;
	}

	double getRoll() const {
		return roll;
	}

	double getYaw() const {
		return yaw;
	}

	TransformationTypes::EulerType getEulerType() const {
		return eulerType;
	}
};
}

#endif /* EULER_H_ */
