#include "kalman_filter.hpp"

namespace plt = matplotlibcpp;

// initialisation
void initialize_variables(float& x_hat, float& p_hat, float& q, float& r)
{
    x_hat = 80.00;      // geschätzte temperatur
    p_hat = 10000.00;   // schätzungsvarianz; std abweichung von 100°C
    q = 0.0001;      // prozessrauschvarianz
    r = 0.01;
}

int main(){
    std::cout << "Starting..." << "\n";
    messungen = {49.986, 49.963, 50.09, 50.001, 50.018, 50.05, 49.938, 49.858, 49.965, 50.114};
    initialize_variables(x_hat, p_hat, q, r);

    for(int i=0; i<10; i++)
    {
        // messen
        float z = messungen[i];

        // aktualisieren
        float x_check = x_hat;
        float p_check = p_hat + q;

        float K = p_check/(p_check + r);
        
        x_hat = x_check + K * (z - x_check);
        p_hat = (1 - K) * p_check;

        std::cout << "iter: " << i << "\n";
        std::cout << "z: " << z << "\n";
        std::cout << "K: " << K << "\n";
        std::cout << "x_hat: " << x_hat << "\n";
        std::cout << "p_hat: " << p_hat << "\n";
        std::cout << "\n";
    
        K_list.push_back(K);
        p_check_list.push_back(p_check);
        x_check_list.push_back(x_check);
        x_hat_list.push_back(x_hat);
    }

    plt::backend("Agg");

    std::vector<float> iterations = {0,1,2,3,4,5,6,7,8,9};

    // figure 1: raw measurements vs kalman estimate
    plt::figure_size(800, 400);
    plt::plot(iterations, messungen,    "r.",  {{"label", "Measurements (z)"}});
    plt::plot(iterations, x_check_list, "b--", {{"label", "Prior estimate (x-)"}});
    plt::plot(iterations, x_hat_list,   "g-",  {{"label", "Posterior estimate (x_hat)"}});
    plt::xlabel("Iteration");
    plt::ylabel("Temperature (C)");
    plt::title("Kalman Filter: Measurement vs Estimate");
    plt::legend();
    plt::grid(true);
    plt::savefig("../kalman_estimates.png");
    plt::clf();

    // figure 2: kalman gain convergence
    plt::figure_size(800, 400);
    plt::plot(iterations, K_list, "m-o", {{"label", "Kalman Gain (K)"}});
    plt::xlabel("Iteration");
    plt::ylabel("Gain");
    plt::title("Kalman Gain Convergence");
    plt::legend();
    plt::grid(true);
    plt::savefig("../kalman_gain.png");

    std::cout << "Plots saved: kalman_estimates.png, kalman_gain.png\n";
    return 0;
}