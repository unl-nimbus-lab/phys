/*
 * Math.h
 *
 *  Created on: 11/12/2012
 *      Author: catec
 */

#ifndef MATH_H_
#define MATH_H_
#include <math.h>

namespace rotateOp {

/*!
 * \brief This class contains useful mathematical helpers needed to perform the rotate and transformed calculations.
 */
class MathHelper {
public:

	static const double PI = 3.14159265358979;

	/*!
	 * \brief This method rounds a double with the indicate number of decimals.
	 * If we have a number like 3.14159 and we want to round to two decimals number, the call will be: roundDecimal(3.14159, 2) and
	 * the result will be 3.14.
	 * \param number Number to be rounded.
	 * \param decimals Number of decimals to round.
	 */
	static double roundDecimal(const double number, const int decimals) {
		int intPart = number;
		double decimalPart = number - intPart;
		double numberDecimals = pow(10, decimals);
		double decimalRound = round(decimalPart * numberDecimals);

		return static_cast<double> (intPart + (decimalRound / numberDecimals));
	}

};
}

#endif /* MATH_H_ */
