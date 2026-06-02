# Kalman Filter — Concepts and Mathematics

A Kalman Filter is an algorithm that estimates the true state of a system
from a sequence of noisy measurements. It works by maintaining a probability
distribution over the possible states, and updating that distribution as new
information arrives from the motion model and sensors.

---

## The core idea

At any point in time, you do not know the exact state of the system.
Instead, you maintain a **belief** — a Gaussian distribution described by:

```
x̂  — the mean (your best estimate of the state)
P   — the covariance (how uncertain you are about that estimate)
```

The filter alternates between two steps every time step:

```
Predict  →  use the motion model to project state forward in time
Update   →  use a sensor measurement to correct the prediction
```

---

## State vector x̂

The state vector holds everything you want to estimate. For a car:

```
x̂ = [x,  y,  θ,  v]
```

For a different system it could be anything — temperature, position in 3D,
orientation of a robot arm. The Kalman Filter is general.

---

## The covariance matrix P

### What it is

`P` is an `n×n` symmetric matrix where `n` is the size of the state vector.
It is a **random covariance matrix** — it describes the uncertainty of the
state estimate `x̂` as a multivariate Gaussian distribution:

```
p(x) = N(x̂, P)
```

This means the true state `x` is assumed to be drawn from a Gaussian
centered at `x̂` with spread described by `P`.

### General form

```
     σ₁²        cov(x₁,x₂)  cov(x₁,x₃)  ···  cov(x₁,xₙ)
     cov(x₂,x₁) σ₂²         cov(x₂,x₃)  ···  cov(x₂,xₙ)
P =  cov(x₃,x₁) cov(x₃,x₂) σ₃²         ···  cov(x₃,xₙ)
     ⋮           ⋮            ⋮            ⋱    ⋮
     cov(xₙ,x₁) cov(xₙ,x₂) cov(xₙ,x₃)  ···  σₙ²
```

### The diagonal — variances

Each diagonal entry `P(i,i) = σᵢ²` is the variance of state variable `xᵢ`.
A large value means high uncertainty; a small value means high confidence.

For the car model:

| Entry | Meaning |
|---|---|
| `P(0,0)` | variance of x position |
| `P(1,1)` | variance of y position |
| `P(2,2)` | variance of heading θ |
| `P(3,3)` | variance of velocity v |

### The off-diagonals — correlations

Each off-diagonal entry `P(i,j) = cov(xᵢ, xⱼ)` describes how much two
state variables are correlated. If `cov(x, θ)` is large, it means: when
you are wrong about heading, you are also wrong about x position.

These correlations start at zero (at initialization, states are assumed
independent) and grow over time as the motion model couples the states
together. For example:

- `cov(x, θ)` grows because x depends on `cos(θ)` in the motion model
- `cov(y, θ)` grows because y depends on `sin(θ)`
- `cov(x, v)` grows because x depends on `v·cos(θ)·dt`

### P is always symmetric

Because `cov(xᵢ, xⱼ) = cov(xⱼ, xᵢ)`, the matrix is symmetric:

```
P = Pᵀ
```

### How P evolves

P is not fixed — it is updated every cycle by the filter:

```
── Predict ──────────────────────────────────────────────
P grows:    P⁻ = F · P · Fᵀ + Q

            Propagating uncertainty through the motion model
            inflates P. Process noise Q adds additional
            uncertainty to account for model imperfection.

── Update ───────────────────────────────────────────────
P shrinks:  P = (I − K·H) · P⁻

            A measurement has arrived. We know more about
            the state, so uncertainty is reduced.
```

P breathes in and out each cycle. Over time it converges to a steady state
where the inflation from Q and the deflation from measurements balance out.

---

## All matrices in the general Kalman Filter

### F — state transition matrix (or Jacobian for EKF)

Describes how the state evolves over one time step.

```
x̂⁻ = F · x̂        (linear KF)
x̂⁻ = f(x̂, u)      (EKF — use nonlinear f directly)
```

In the EKF, `F` is the Jacobian of the motion model `f` evaluated at the
current estimate. It linearizes the nonlinear motion around `x̂`:

```
F = ∂f/∂x |_{x=x̂}
```

It is re-evaluated at every time step because `x̂` changes.

### Q — process noise covariance

Represents how much you distrust your own motion model. No model is perfect —
the car hits bumps, the road is slippery, `dt` is not exact. Q accounts for
this accumulated uncertainty.

```
Q = diag([σ_x², σ_y², σ_θ², σ_v²])
```

It is usually diagonal (noise in each state is assumed independent) and is
tuned manually:

- Q too small → filter trusts model too much, slow to correct errors
- Q too large → filter distrusts model, jumps with every noisy measurement

### H — measurement matrix (or Jacobian for EKF)

Maps the state vector to what the sensor is expected to measure. For a GPS
that reads only `[x, y]`:

```
h(x) = [x, y]

H = [[1, 0, 0, 0],
     [0, 1, 0, 0]]
```

In the EKF, if `h` is nonlinear, `H` is its Jacobian:

```
H = ∂h/∂x |_{x=x̂⁻}
```

### R — measurement noise covariance

Represents how much you distrust your sensor. Comes from the sensor
datasheet or is measured empirically.

```
R = diag([σ_gps_x², σ_gps_y²])
```

- R too small → filter trusts sensor too much, estimate is noisy
- R too large → filter ignores measurements, estimate drifts

### K — Kalman gain

The Kalman gain is computed at every update step. It decides how much weight
to give the measurement versus the model prediction:

```
K = P⁻ · Hᵀ · (H · P⁻ · Hᵀ + R)⁻¹
```

- K close to 0 → trust the model, ignore the measurement
- K close to 1 → trust the measurement, ignore the model

K is not set manually — it is computed automatically from P, H, and R.

### S — innovation covariance

The denominator of the Kalman gain. It represents the total uncertainty
in the measurement space — uncertainty from the predicted state plus
uncertainty from the sensor:

```
S = H · P⁻ · Hᵀ + R
```

### y — innovation (residual)

The difference between the actual measurement and the predicted measurement:

```
y = z − h(x̂⁻)
```

This is the new information the sensor provides. If `y = 0` the measurement
perfectly matches the prediction and the state does not change.

---

## Full EKF cycle — summary

```
── Given ────────────────────────────────────────────────
x̂    current state estimate
P     current covariance
u     control input
z     new sensor measurement

── Predict ──────────────────────────────────────────────
x̂⁻  =  f(x̂, u)                   nonlinear motion model
F    =  ∂f/∂x                      Jacobian (re-evaluated each step)
P⁻  =  F · P · Fᵀ + Q             propagate covariance

── Update ───────────────────────────────────────────────
y    =  z − h(x̂⁻)                 innovation
S    =  H · P⁻ · Hᵀ + R           innovation covariance
K    =  P⁻ · Hᵀ · S⁻¹             Kalman gain
x̂   =  x̂⁻ + K · y                update state
P    =  (I − K · H) · P⁻          update covariance
```

---

## Intuition for tuning

| If you see | Likely cause | Fix |
|---|---|---|
| Estimate lags behind truth | Q too small, R too small | Increase Q |
| Estimate is noisy / jumpy | R too small | Increase R |
| Estimate ignores measurements | R too large | Decrease R |
| P grows without bound | Q too large, no measurements | Decrease Q |
| P converges to zero | Q too small | Increase Q |