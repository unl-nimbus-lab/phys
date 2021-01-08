//
// Created by carlh on 12/7/16.
//

#ifndef RBE_PROJECT_PID_H
#define RBE_PROJECT_PID_H

namespace SparkiControl {
    class PID {
    public:
        // Integrated Error is updated each time class gets an update and error entered is
        // is added to Integrated Error.
        float I_error;
        float error;
        float KP;
        float KI;
        float KD;
        float delta_pos;

        // Initializes the desired value and K values from the arguements.
        PID(float KP, float KI, float KD);

        // Updates the PID with a new error and calculates the pid output
        float update(float error);

        // Sets the I_error to 0
        void reset();
    };
}

#endif //RBE_PROJECT_PID_H
