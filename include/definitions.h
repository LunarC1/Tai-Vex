#include <vector>
#include "main.h"

#define CLAMP "mogo.set_value(true);"
#define UNCLAMP "mogo.set_value(false);"

#define ALLBUTTONS {pros::E_CONTROLLER_DIGITAL_L1, pros::E_CONTROLLER_DIGITAL_L2, pros::E_CONTROLLER_DIGITAL_R1, pros::E_CONTROLLER_DIGITAL_R2, pros::E_CONTROLLER_DIGITAL_UP, pros::E_CONTROLLER_DIGITAL_DOWN, pros::E_CONTROLLER_DIGITAL_LEFT, pros::E_CONTROLLER_DIGITAL_RIGHT, pros::E_CONTROLLER_DIGITAL_X, pros::E_CONTROLLER_DIGITAL_B, pros::E_CONTROLLER_DIGITAL_Y, pros::E_CONTROLLER_DIGITAL_A}
#define PI 3.14159265358979323846

pros::Controller* cont;

enum buttons
{
    L1 = 0,
    NL1 = 1,
    L2 = 2,
    NL2 = 3,
    R1 = 4,
    NR1 = 5,
    R2 = 6,
    NR2 = 7,
    UP = 8,
    NUP = 9,
    DOWN = 10,
    NDOWN = 11,
    LEFT = 12,
    NLEFT = 13,
    RIGHT = 14,
    NRIGHT = 15,
    X = 16,
    NX = 17,
    B = 18,
    NB = 19,
    Y = 20,
    NY = 21,
    A = 22,
    NA = 23 
};

std::vector<bool> getAll(std::vector<pros::controller_digital_e_t> buttons)
        {
            std::vector<bool> out;
            for (pros::controller_digital_e_t button : buttons)
            {
                out.push_back(cont -> get_digital(button));
                out.push_back(cont -> get_digital_new_press(button));
            }
            return(out);
        }