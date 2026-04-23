import numpy as np
import matplotlib.pyplot as plt

# Initialisierung
x_hat = 80    # geschätzte temperatur
p_hat = 10000   # schätzungsvarianz; std abweichung von 100°C
q = 0.0001  # prozessrauschvarianz
r = 0.01  # messvarianz

messungen = [49.986, 49.963, 50.09, 50.001, 50.018, 50.05, 49.938, 49.858, 49.965, 50.114]
K_list = []
# Schritt 1

for i in range(10):
    # messen
    z = messungen[i]

    # aktualisieren
    x_check = x_hat
    p_check = p_hat + q

    K = p_check/(p_check + r)
    
    x_hat = x_check + K * (z - x_check)
    p_hat = (1 - K) * p_check

    print("iter: ", i)
    print("z: ", z)
    print("K: ", K)
    print("x_hat: ", x_hat)
    print("p_hat: ", p_hat)
    print()
    K_list.append(K)

plt.plot(K_list)
#plt.show()