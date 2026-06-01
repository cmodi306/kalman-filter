# Derivatives — Jacobian of the Motion Model

This page derives the Jacobian matrix `F = ∂f/∂x` used in the EKF predict step.
Refer to [README.md](README.md) for the motion model equations.

---

## General form

The Jacobian is the matrix of all partial derivatives of `f` with respect to the state `x = [x, y, θ, v]`:

```
         ∂f1/∂x   ∂f1/∂y   ∂f1/∂θ   ∂f1/∂v
         ∂f2/∂x   ∂f2/∂y   ∂f2/∂θ   ∂f2/∂v
F =
         ∂f3/∂x   ∂f3/∂y   ∂f3/∂θ   ∂f3/∂v
         ∂f4/∂x   ∂f4/∂y   ∂f4/∂θ   ∂f4/∂v
```

Each row corresponds to one motion equation. Each column corresponds to one state variable.

---

## Row-by-row derivation

**f1 = x + v · cos(θ) · dt**

```
∂f1/∂x = 1
∂f1/∂y = 0
∂f1/∂θ = -v · sin(θ) · dt
∂f1/∂v =  cos(θ) · dt
```

**f2 = y + v · sin(θ) · dt**

```
∂f2/∂x = 0
∂f2/∂y = 1
∂f2/∂θ =  v · cos(θ) · dt
∂f2/∂v =  sin(θ) · dt
```

**f3 = θ + (v / L) · tan(δ) · dt**

```
∂f3/∂x = 0
∂f3/∂y = 0
∂f3/∂θ = 1
∂f3/∂v = tan(δ) · dt / L
```

**f4 = v + a · dt**

```
∂f4/∂x = 0
∂f4/∂y = 0
∂f4/∂θ = 0
∂f4/∂v = 1
```

---

## Full Jacobian matrix

```
F = [[1,  0,  -v·sin(θ)·dt,   cos(θ)·dt    ],
     [0,  1,   v·cos(θ)·dt,   sin(θ)·dt    ],
     [0,  0,   1,              tan(δ)·dt/L  ],
     [0,  0,   0,              1            ]]
```

---

## Python implementation

```python
def F(x, u, dt=0.1, L=2.5):
    _, _, theta, v = x
    delta, _       = u

    return np.array([
        [1,  0,  -v * np.sin(theta) * dt,   np.cos(theta) * dt    ],
        [0,  1,   v * np.cos(theta) * dt,   np.sin(theta) * dt    ],
        [0,  0,   1,                         np.tan(delta) * dt/L  ],
        [0,  0,   0,                         1                     ]
    ])
```

---

## Notes

- `F` must be re-evaluated at the **current** `x̂` before each predict step — it changes every cycle because `θ` and `v` change.
- `δ` (steering angle) appears only in row 3 column 4. It comes from the control input `u`, not the state.
- Rows 1 and 2 are coupled through `θ` — this is where the nonlinearity lives.
- Row 4 is trivially linear (`v' = v + a·dt`), so its row in `F` is just `[0, 0, 0, 1]`.