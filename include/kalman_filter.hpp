#include "matplotlibcpp.h"
#include <vector>

std::vector<float> messungen;
std::vector<float> K_list;
std::vector<float> p_hat_list;
std::vector<float> p_check_list;
std::vector<float> x_check_list;
std::vector<float> x_hat_list;

// Declaration
float x_hat;    // geschätzte temperatur
float p_hat;    // schätzungsvarianz; std abweichung von 100°C
float q;        // prozessrauschvarianz
float r;        // messvarianz