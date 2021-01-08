/*
 * Quaternion.h
 *
 *  Created on: 11/12/2012
 *      Author: catec
 */

#ifndef QUATERNION_H_
#define QUATERNION_H_

#include <math.h>
#include <iostream>
#include "TransformationTypes.h"
#include "MathHelper.h"


namespace rotateOp {

/*!
 * \brief This class defines a quaternion.
 * A quaternion is a number system that extends the complex number. Quaternions are used in particular for calculations
 * involving three-dimmensional space.
 * Quaternions are been represented in this class as a scalar and a vector: [w, (x,y,z)].
 */
class Quaternion {
private:

	double w;
	double x;
	double y;
	double z;

public:

	/*!
	 * \brief Default constructor. Initialize the quaternion as [1, (0,0,0)].
	 */
	Quaternion() {
		w = 1;
		x = 0;
		y = 0;
		z = 0;

	}

	/*!
	 *\brief This constructor initialize the quaternion as [w, (x,y,z)].
	 *
	 *\param w The scalar.
	 *\param x The x component of the vector.
	 *\param y The y component of the vector.
	 *\param z The z component of the vector.
	 */
	Quaternion(double w, double x, double y, double z) {
		this->w = w;
		this->x = x;
		this->y = y;
		this->z = z;

	}


	/*!
	 * \brief This method sets the quaternion data with euler data.
	 * Sets the quaternion components [w, (x,y,z)] of the object with the specific euler data (roll, pitch, yaw).
	 * The euler system (euler321, euler123, ...) must be specified to know what is the transformation to applied.
	 * \param roll The euler roll angle in radians.
	 * \param pitch The euler pitch angle in radians.
	 * \param yaw The euler yaw angle in radians.
	 * \param type The euler system. The options can be EULER123, EULER321 or EULER 312
	 */
	void fromEuler(double roll, double pitch, double yaw,
			TransformationTypes::EulerType type) {
		// Assuming the angles are in radians.
		double c1 = (double) (cos(roll * 0.5f));
		double s1 = (double) (sin(roll * 0.5f));
		double c2 = (double) (cos(pitch * 0.5f));
		double s2 = (double) (sin(pitch * 0.5f));
		double c3 = (double) (cos(yaw * 0.5f));
		double s3 = (double) (sin(yaw * 0.5f));
		switch (type) {
		case TransformationTypes::EULER123:

			w = MathHelper::roundDecimal(c1 * c2 * c3 - s1 * s2 * s3, 5);
			x = MathHelper::roundDecimal(s1 * c2 * c3 + c1 * s2 * s3, 5);
			y = MathHelper::roundDecimal(c1 * s2 * c3 - s1 * c2 * s3, 5);
			z = MathHelper::roundDecimal(c1 * c2 * s3 + s1 * s2 * c3, 5);
			break;
		case TransformationTypes::EULER321:

			w = MathHelper::roundDecimal(c3 * c2 * c1 + s3 * s2 * s1, 5);
			x = MathHelper::roundDecimal(c3 * c2 * s1 - s3 * s2 * c1, 5);
			y = MathHelper::roundDecimal(c3 * s2 * c1 + s3 * c2 * s1, 5);
			z = MathHelper::roundDecimal(s3 * c2 * c1 - c3 * s2 * s1, 5);
			break;

		case TransformationTypes::EULER312:

			w = MathHelper::roundDecimal(c3 * c1 * c2 - s3 * s1 * s2, 5);
			x = MathHelper::roundDecimal(s1 * c3 * c2 - s3 * c1 * s2, 5);
			y = MathHelper::roundDecimal(s3 * s1 * c2 + c3 * c1 * s2, 5);
			z = MathHelper::roundDecimal(s3 * c1 * c2 + c3 * s1 * s2, 5);
			break;

		default:
			std::cerr << "This euler type is not implemented." << std::endl;
		//	assert(true);
			break;
		}

	}

	/*!
	 * \brief This method returns a new array with the quaternion data.
	 * The size of the array will be four.
	 * The first element will be the scalar.
	 * the second element will be the x component of the vector.
	 * the third element will be the y component of the vector.
	 * the fourth element will be the z component of the vector.
	 *
	 * ATENTION: The array that this method returns must be deleted for the client.
	 */
	double *toArray() {
		double *quaternionArray = new double[4];
		quaternionArray[0] = MathHelper::roundDecimal(w, 5);
		quaternionArray[1] = MathHelper::roundDecimal(x, 5);
		quaternionArray[2] = MathHelper::roundDecimal(y, 5);
		quaternionArray[3] = MathHelper::roundDecimal(z, 5);
		return quaternionArray;
	}

	friend std::ostream & operator <<(std::ostream & o, Quaternion q) {
		o << "Quaternion: [" << q.w << ", (" << q.x << ", " << q.y << ", "
				<< q.z << ")]";
		return o;
	}

	double getW() const {
		return w;
	}

	double getX() const {
		return x;
	}

	double getY() const {
		return y;
	}

	double getZ() const {
		return z;
	}

};
}
#endif /* QUATERNION_H_ */
