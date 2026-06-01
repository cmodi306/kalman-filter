# kalman-filter

# Extended Kalman Filter — Car Motion Exercise

A hands-on exercise to implement an EKF for a car moving in a parking lot.
You are given noisy GPS readings and must estimate the car's true state over time.

---

## State vector

The car state at each time step is a 4-element vector:

```
x̂ = [x,  y,  θ,  v]
```

| Symbol | Meaning | Unit |
|--------|---------|------|
| `x` | position along x-axis | m |
| `y` | position along y-axis | m |
| `θ` | heading angle | rad |
| `v` | speed | m/s |

---

## Motion model — f(x, u)

The motion model predicts the next state from the current state and a control input `u = [δ, a]`:

```
x'  =  x  +  v · cos(θ) · dt
y'  =  y  +  v · sin(θ) · dt
θ'  =  θ  +  (v / L) · tan(δ) · dt
v'  =  v  +  a · dt
```

| Symbol | Meaning |
|--------|---------|
| `δ` | steering angle (rad) |
| `a` | acceleration (m/s²) |
| `L` | wheelbase = 2.5 m |
| `dt` | time step = 0.1 s |

---

## Measurement model — h(x)

The sensor is a GPS that returns `[x, y]` position only:

```
h(x) = [x,  y]
```

The measurement Jacobian H is constant:

```
H = [[1, 0, 0, 0],
     [0, 1, 0, 0]]
```

---

## EKF equations

### Predict step

```
x̂⁻  =  f(x̂, u)
P⁻   =  F · P · Fᵀ + Q
```

### Update step

```
K   =  P⁻ · Hᵀ · (H · P⁻ · Hᵀ + R)⁻¹
x̂   =  x̂⁻ + K · (z − h(x̂⁻))
P   =  (I − K · H) · P⁻
```

| Symbol | Meaning |
|--------|---------|
| `F` | Jacobian of f w.r.t. x (see [DERIVATIVES.md](DERIVATIVES.md)) |
| `Q` | process noise covariance (4×4) |
| `R` | measurement noise covariance (2×2) |
| `z` | GPS measurement `[x, y]` |
| `K` | Kalman gain |

---

## Exercise setup

**Initial state and covariance:**

```python
x̂ = [0.0,  0.0,  0.0,  2.0]          # at origin, facing right, 2 m/s
P  = diag([1.0,  1.0,  0.1,  0.5])
```

**Control input (constant throughout):**

```python
u  = [δ=0.2 rad,  a=0.0 m/s²]
```

**Noise matrices:**

```python
Q = diag([0.01,  0.01,  0.001,  0.1])
R = diag([0.5,  0.5])
```

**GPS measurements:**

```python
z1 = [0.21,  0.02]
z2 = [0.40,  0.08]
z3 = [0.58,  0.19]
```

---

## Your task

Run 3 full EKF cycles — one per measurement. For each cycle:

1. Call `f(x̂, u)` to get the predicted state `x̂⁻`
2. Compute `F` evaluated at the current `x̂` (see [DERIVATIVES.md](DERIVATIVES.md))
3. Propagate covariance: `P⁻ = F · P · Fᵀ + Q`
4. Compute Kalman gain `K`
5. Update state and covariance with the GPS measurement

**Checkpoint — after cycle 1:**

```
x̂⁻ ≈ [0.200,  0.040,  0.080,  2.0]   # after predict
x̂  ≈ [0.204,  0.027,  0.080,  2.0]   # after update
```

---

## Stretch goals

- Plot the true path, GPS dots, and EKF estimate on a 2D chart
- Increase `R` (less trust in GPS) and observe the filter smoothing more
- Verify your Jacobian `F` numerically using finite differences:

```python
eps = 1e-5
F_numerical = np.zeros((4, 4))
for i in range(4):
    x_plus        = x.copy(); x_plus[i]  += eps
    x_minus       = x.copy(); x_minus[i] -= eps
    F_numerical[:, i] = (f(x_plus, u) - f(x_minus, u)) / (2 * eps)

print(np.allclose(F_analytical, F_numerical, atol=1e-5))  # should be True
```