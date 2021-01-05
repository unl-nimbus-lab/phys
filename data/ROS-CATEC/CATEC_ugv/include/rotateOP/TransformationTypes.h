/*
 * TransformationTypes.h
 *
 *  Created on: 11/12/2012
 *      Author: catec
 */

#ifndef TRANSFORMATIONTYPES_H_
#define TRANSFORMATIONTYPES_H_
namespace rotateOp{

/*!
 * \brief This class defines the supported Euler systems.
 */
class TransformationTypes{
public:
	/*!
	 * \brief Enumerate with the supported Euler systems.
	 */
	enum EulerType {
				EULER123, EULER321, EULER312
			};
};
}
#endif /* TRANSFORMATIONTYPES_H_ */
