# Trajectory and Control Derivation for the 3D Cylindrical Helix Climb

This implementation generates a **three-dimensional cylindrical helix with constant radius that rotates about a vertical axis and climbs continuously**. Its horizontal projection is a circle, its height keeps increasing, and the radius does not grow with time. The trajectory is implemented by `HelixTrajectory` in `isaac_drone/trajectories/helix.py` (a subclass of `SegmentedTrajectory` with two segments, `takeoff` and `helix`), and the matching config is `trajectory: {kind: helix, ...}` (the file name spiral_math is kept for historical reasons).

What is derived here is the control reference. The actual flight path will deviate from it because the real motion is also affected by the full rigid-body dynamics, control error, motor lag, thrust saturation, ground contact and any enabled disturbances. The parameters and examples below are set for this task; they are not calibration results from the real ARL vehicle.

## 1. Four phases starting from the actual center of mass

`reset(initial_state)` records the actual center-of-mass position of the whole vehicle $p_0$, the actual heading and the simulation time $t_0$. Position always means the **center of mass of the whole vehicle**, not the origin of the USD root link. The current entry point places the body using the collision geometry bounds and an explicit ground clearance, and resets the velocity as configured. A ground-settling gate based on sustained contact or rest time is not implemented yet. If the linear or angular speed of the initial state exceeds the configured thresholds, trajectory initialization raises an error.

Let the delay be $T_d$, the takeoff height $h$, the takeoff duration $T_o$, and the duration of the three-dimensional helix phase $T_h$:

| Phase | Interval relative to the reset time | Reference behavior |
|---|---|---|
| `spin_up` (control loop) | $[0,T_d)$ | Hold $p_0$ and the actual yaw at reset; $T_d$ = `vehicle.launch.spin_up_s` |
| `takeoff` | $[T_d,T_d+T_o)$ | Climb vertically by $h$ in place and adjust the yaw smoothly |
| `helix` | $[T_d+T_o,T_d+T_o+T_h)$ | Three-dimensional cylindrical helix climb with constant radius |
| `hold` | $[T_d+T_o+T_h,\infty)$ | Hold the end position and the final yaw indefinitely |

The trajectory itself starts at $T_d$ (`reset(state, start_time_s=T_d)`). `phase()` returns `takeoff` / `helix` / `hold`, and the spin-up phase `spin_up` is labeled by the control loop. `motion_duration_s` returns $T_o+T_h$, which excludes spin-up and the indefinite hold. Reaching the end point only means that the reference enters the hold phase; **it does not shut off the motors or stop the controller**.

The spin-up duration `vehicle.launch.spin_up_s` only shifts the reference in time: the trajectory starts when spin-up ends, the reference stays at the start point during spin-up, and the control loop applies the rotor-speed startup envelope. Contact-settling detection and an automatic takeoff permission gate are not implemented yet. The trajectory module does not operate the motors, advance physics time, or modify the state.

## 2. Where the ninth-degree smooth progress function comes from

Each motion phase uses the normalized time $u=(t-t_{\mathrm{start}})/T$. The progress is 0 for $u\le0$ and 1 for $u\ge1$, and inside the interval it is

$$
S(u)=126u^5-420u^6+540u^7-315u^8+70u^9.
$$

It satisfies ten endpoint conditions:

$$
S(0)=0,\quad S(1)=1,\qquad
S^{(k)}(0)=S^{(k)}(1)=0,\quad k=1,2,3,4.
$$

So position, velocity, acceleration, jerk (the derivative of acceleration) and snap (the derivative of jerk) all join continuously with a segment at rest. One way to construct it is to set

$$
S'(u)=C u^4(1-u)^4.
$$

The fourfold zeros at both ends make the first to fourth derivatives zero, and normalizing the integral gives

$$
1=\int_0^1 S'(u)\,du=C\frac{4!4!}{9!},\qquad C=630.
$$

Expanding and integrating gives the ninth-degree polynomial above. It is also the only polynomial of degree at most nine that satisfies these ten boundary conditions: if there were two solutions, their difference would have a zero of multiplicity at least five at both 0 and 1, so it would have to contain the factor $u^5(u-1)^5$, which contradicts the degree being at most 9.

**This is a ninth-degree Hermite interpolation that meets the given endpoint conditions; it is not optimized as a minimum-snap solution.** The derivation does not solve for a global minimum of any trajectory integral cost.

The first four derivatives of the progress function with respect to normalized time are

$$
\begin{aligned}
S' &=630u^4(1-u)^4,\\
S''&=2520u^3(1-u)^3(1-2u),\\
S'''&=2520u^2(1-u)^2(3-14u+14u^2),\\
S''''&=15120u(1-u)(1-2u)(1-7u+7u^2).
\end{aligned}
$$

When converting to physical time, each derivative is divided by the matching power of the time scale:

$$
s_k(t)=\frac{1}{T^k}S^{(k)}(u),\quad k=1,2,3,4,\qquad s_0(t)=S(u).
$$

The code uses the factored form of the derivatives to keep the endpoint values at zero. For $u>1/2$ the position is evaluated with $S(u)=1-S(1-u)$, which reduces the numerical error from subtracting polynomial terms near the end point.

## 3. Vertical takeoff

The end point of the takeoff phase is

$$
p_o=p_0+h e_z,\qquad e_z=(0,0,1)^\mathsf T.
$$

The position and its derivatives are written with the progress function of this phase:

$$
p(t)=p_0+h s_0(t)e_z,\qquad p^{(k)}(t)=h s_k(t)e_z,\quad k=1,2,3,4.
$$

The horizontal position stays at $p_{0,xy}$ for the whole takeoff phase. The motion derivatives up to fourth order are zero at the start and at the end, and the final position is exactly the start point of the next phase.

## 4. Locating the cylinder axis automatically to remove the lateral jump

Let the radius be $R>0$, the initial phase angle $\theta_0$, the signed number of turns $N\ne0$, and the net climb of this phase $H>0$. A positive number of turns means counterclockwise when looking down from world +Z, a negative number means clockwise, and the number of turns may be a non-integer.

The horizontal position of the cylinder axis is computed from the actual start point:

$$
c_{xy}=p_{0,xy}-R
\begin{bmatrix}\cos\theta_0\\\sin\theta_0\end{bmatrix}.
$$

So $p_0$ is the start point on the circle, not the center, and there is no instant sideways jump of one radius. Let

$$
\Theta=2\pi N,\qquad
\theta(t)=\theta_0+\Theta s_0(t),
$$

then the three-dimensional trajectory is

$$
p(t)=
\begin{bmatrix}
c_x+R\cos\theta(t)\\
c_y+R\sin\theta(t)\\
p_{o,z}+H s_0(t)
\end{bmatrix}.
$$

The radius is constant, and

$$
\frac{dz}{d\theta}=\frac{H}{\Theta}
$$

is constant inside the phase, so the curve is a cylindrical helix with a fixed geometric pitch. Changing the time progress only changes the speed along the curve, not the geometric curve itself.

The end point is

$$
p_f=\begin{bmatrix}
c_x+R\cos(\theta_0+\Theta)\\
c_y+R\sin(\theta_0+\Theta)\\
p_{0,z}+h+H
\end{bmatrix}.
$$

If $N$ is an integer, the horizontal end position returns to the start point; if the number of turns is not an integer, the end point stops at the corresponding phase angle on the circle. The interface `endpoint_position_w` returns a copy of this position.

## 5. Full analytic expressions for velocity, acceleration, jerk and snap

Define the unit radial and unit tangential vectors, which change with the phase angle:

$$
e_r=(\cos\theta,\sin\theta,0)^\mathsf T,\qquad
e_t=(-\sin\theta,\cos\theta,0)^\mathsf T.
$$

Let $\theta_k=\Theta s_k$. Since $\dot e_r=\theta_1e_t$ and $\dot e_t=-\theta_1e_r$, differentiating repeatedly gives

$$
\begin{aligned}
v &= R\theta_1 e_t+Hs_1e_z,\\
a &= R(\theta_2e_t-\theta_1^2e_r)+Hs_2e_z,\\
j &= R\big[(\theta_3-\theta_1^3)e_t-3\theta_1\theta_2e_r\big]+Hs_3e_z,\\
s_{\mathrm{snap}} &= R\big[(\theta_4-6\theta_1^2\theta_2)e_t
 +(\theta_1^4-3\theta_2^2-4\theta_1\theta_3)e_r\big]+Hs_4e_z.
\end{aligned}
$$

These expressions include every chain-rule term from the trigonometric functions: not only the tangential acceleration, but also the centripetal acceleration and the higher-order coupling terms. The code writes them to `velocity_w`, `acceleration_w`, `jerk_w` and `snap_w`, and uses these analytic reference derivatives directly instead of finite differences of discrete positions.

At the start and end of the helix, $\theta_1\ldots\theta_4$ and $s_1\ldots s_4$ are all zero, so the derivatives up to fourth order all vanish. Joined with the takeoff and the final hold, the whole position reference is $C^4$; the fifth derivative can jump at the joins, so the whole reference is not necessarily $C^5$ continuous.

## 6. Why the first half accelerates and the second half decelerates

The total path length of the helix phase is

$$
L=\sqrt{(R\Theta)^2+H^2}.
$$

Because the radial, tangential and vertical directions are mutually orthogonal, and $s_1\ge0$, the speed is

$$
\|v(t)\|=L s_1(t)=\frac{630L}{T_h}u^4(1-u)^4.
$$

The time derivative of the speed is

$$
\frac{d\|v\|}{dt}=\frac{2520L}{T_h^2}u^3(1-u)^3(1-2u).
$$

So the motion strictly accelerates for $0<u<1/2$ and strictly decelerates for $1/2<u<1$, and the peak is at the midpoint:

$$
v_{\max}=\frac{315L}{128T_h}.
$$

This result is about the **speed**. At the midpoint the tangential acceleration is zero, but the centripetal acceleration is still there, so the three-dimensional acceleration is not zero. A longer phase duration lowers the peak speed and lowers the reference derivatives of every order even more, but that alone does not guarantee that the motors are strong enough or that the tracking error is small enough.

## 7. Continuous yaw strategy

The initial yaw $\psi_0$ is taken from the projection of the body X axis of the actual attitude onto the world XY plane. If this projection degenerates to zero, trajectory initialization raises an error.

In `fixed` mode the target yaw is $\psi_h=\psi_0+\psi_{\mathrm{offset}}$. To avoid a jump at initialization caused by a nonzero offset, the delay phase holds $\psi_0$ and the takeoff phase uses

$$
\psi(t)=\psi_0+(\psi_h-\psi_0)s_0(t),\quad
\dot\psi=(\psi_h-\psi_0)s_1,\quad
\ddot\psi=(\psi_h-\psi_0)s_2.
$$

After the helix start point is reached, the yaw is fixed at $\psi_h$. The offset is applied with the signed value given; if a multi-turn angle is entered, the takeoff phase also transitions through that angle.

`tangent` mode adds the yaw offset along the **horizontal tangent direction of the motion** during the helix phase:

$$
\psi(t)\equiv\theta(t)+\operatorname{sgn}(N)\frac{\pi}{2}+\psi_{\mathrm{offset}}
\pmod{2\pi}.
$$

Before takeoff starts, the equivalent tangent angle closest to $\psi_0$ is chosen as $\psi_h$, and the takeoff yaw formula above is used for a smooth transition. The helix phase uses a continuous yaw value that is **not wrapped to $[-\pi,\pi]$**:

$$
\psi(t)=\psi_h+\Theta s_0(t),\quad
\dot\psi=\Theta s_1,\quad \ddot\psi=\Theta s_2.
$$

The end point holds the final accumulated yaw value, and the yaw rate and yaw acceleration are both zero. A zero-speed end point has no instantaneous tangent of motion, but the target heading defined by the left and right limits is continuous.

The peak yaw rate of the trajectory must be checked against the feedforward limit of the controller: the peak in the tangent helix phase is $315|\Theta|/(128T_h)$, and the peak takeoff yaw rate is $315|\psi_h-\psi_0|/(128T_o)$. Simply shortening the time can make the controller limit the command, and then not all of the analytic feedforward is applied as written.

## 8. Relation to the low-level motion control

Let the actual center-of-mass position and velocity be $p,v$, the reference values $p_d,v_d,a_d$, the true mass of the whole vehicle $m$, and the world gravity vector $g$. Without limiting, the position feedback produces the desired force

$$
F_d=m\big[a_d+K_p(p_d-p)+K_v(v_d-v)+K_i\eta-g\big].
$$

Here $K_p,K_v,K_i$ act per axis, and the integral state $\eta$ has an explicit upper limit and anti-windup feedback. The mass is taken from the model actually loaded, and it includes the rotor masses and the actual position of the center of mass.

The controller first applies the configured acceleration and tilt constraints. If nothing is limited and the reference provides jerk/snap, the derivatives of the desired force are split as

$$
\dot F_d=m j_d+\frac{dF_{\mathrm{feedback}}}{dt},\qquad
\ddot F_d=m s_d+\frac{d^2F_{\mathrm{feedback}}}{dt^2},
$$

where $F_{\mathrm{feedback}}=F_d-ma_d$. The reference part is computed analytically and the feedback part is differenced from the state history, so the derivative of the error feedback is included as well. When a constraint limits the command, the numerical derivative of the actual limited total desired force is used instead. On the first computation there is no feedback history yet and this part of the derivative is unknown, so it is initialized to zero; this does not mean that the derivative of the feedback term on a real drone is necessarily zero.

The desired attitude $R_d$ is built from the direction of the desired force and the yaw:

$$
b_{3d}=\frac{F_d}{\|F_d\|},\quad
b_{2d}=\frac{b_{3d}\times(\cos\psi_d,\sin\psi_d,0)^\mathsf T}
{\|b_{3d}\times(\cos\psi_d,\sin\psi_d,0)^\mathsf T\|},\quad
b_{1d}=b_{2d}\times b_{3d},\quad R_d=[b_{1d}\ b_{2d}\ b_{3d}].
$$

Zero thrust and degenerate directions are handled by separate branches in the control module. The yaw in this construction is an auxiliary horizontal heading. When the body tilts, it is not exactly the same as the yaw obtained from the actual attitude under the various Euler angle definitions.

Let the full inertia about the center of mass be $J$ and the actual body-frame angular velocity be $\Omega$:

$$
e_R=\frac12(R_d^\mathsf TR-R^\mathsf TR_d)^\vee,\qquad
e_\Omega=\Omega-R^\mathsf TR_d\Omega_d.
$$

The desired angular velocity and angular acceleration come from the first and second derivatives of the desired rotation matrix, and the control torque is

$$
M_d=-K_Re_R-K_\Omega e_\Omega
+\Omega\times(J\Omega)
+J\left(R^\mathsf TR_d\dot\Omega_d
-\Omega\times(R^\mathsf TR_d\Omega_d)\right).
$$

The attitude gains follow the **direct torque gain** meaning of the local Lee controller and are not multiplied by the inertia again; the full inertia appears in the gyroscopic term and in the angular-acceleration feedforward term. The total positive thrust command is

$$
f_d=\max(0,F_d^\mathsf TRe_3).
$$

The bounded allocator then distributes the desired force/torque into a target for each rotor, and the rotor-speed/thruster layer carries it out with the selected model. With these formulas the motor dynamics stay in the loop and the body position is never written directly. A curve that is geometrically feasible is not automatically a flight task that the dynamics can track.

## 9. Usage example and interface

The example settings are: spin-up 2 s; takeoff in place by 1 m in 4 s; a helix phase with radius 1 m, 2 turns and an extra climb of 3 m in 24 s. The motion reference enters the permanent hold after 30 s, and the end point is 4 m above the initial actual center of mass. The helix path length is about 12.92 m and the peak speed is about 1.325 m/s; in tangent mode the peak helix yaw rate is about 1.289 rad/s. These values still need to be adjusted against the motor capability, the tilt limit and the validation results; they are not a finished calibration on the real vehicle.

```python
from isaac_drone.trajectories import build_trajectory

trajectory = build_trajectory(config["trajectory"])   # {kind: helix, ...}
trajectory.reset(actual_initial_state, start_time_s=spin_up_s)
# A segment whose duration is null must first be planned by min_time.plan_minimum_time, otherwise sample raises an error
reference = trajectory.sample(current_simulation_time_s)
phase = trajectory.phase(current_simulation_time_s)
endpoint = trajectory.endpoint_position_w
```

`sample()` is a pure sampling interface that does not advance internal time: the same time returns the same reference, so records can be looked up again. `time_s` is absolute simulation time and cannot be earlier than the most recent reset; another reset sets a new actual start point and time origin. The parameters are parsed strictly by `HelixParams` (`TRAJECTORIES.parse`). The standalone math regression tests are in `tests/test_helix.py` and cover the analytic derivatives up to fourth order, endpoint continuity, both rotation directions, yaw continuity and peak speed. These tests do not replace actual flight validation in Isaac Sim.

## 10. Feasibility screening before takeoff

`validate_trajectory_feasibility()` takes the trajectory already reset from the actual initial state, the true mass, the full inertia about the center of mass, the actual allocation matrix and the current per-rotor thrust limits, and rejects plans that clearly cannot be executed. It does not change the task parameters automatically, and the check does not assume ideal motors.

The check has three parts:

1. **Static hover condition.** The four thrust axes must be compatible with the body +Z convention of the current controller, and the collective thrust and the three attitude channels must be independent; within the actual per-rotor limits, it must be possible to allocate the force needed for hover with zero net torque. When the center of mass is offset, the hover thrusts of the rotors are not necessarily equal; a lower limit that is too high can also make hover infeasible.
2. **Analytic peak yaw rate.** Using the known exact peak of $S'$, check whether the maximum rate of the takeoff yaw transition and of the tangent helix yaw exceeds the control configuration. This check uses the analytic peak, covers all continuous time in both phases, and does not depend on finite sampling.
3. **Nominal dynamics check on a finite set of samples.** The takeoff phase uses 33 time points and the helix phase uses 65, each including its end points and midpoint; after merging the shared boundary and adding the start of the delay, a normal example has 98 distinct time points. If a phase uses the minimum-time law of Section 16, its acceleration and deceleration segments use 33 points each, and the helix cruise segment uses at least 16 points per turn (at least 33). The check is whether the reference acceleration, the required tilt and the nominal feedforward torque can be realized within the current thrust limits.

At each nominal check point the reference is assumed to be tracked exactly. This assumption is only used to compute the basic rigid-body force/torque that this reference needs:

$$
F=m(a_d-g),\qquad f=\|F\|,\qquad
M=J\dot\Omega_d+\Omega_d\times(J\Omega_d).
$$

$R_d,\Omega_d,\dot\Omega_d$ are computed from the analytic jerk/snap and the yaw derivatives; the allocator checks whether the target body-frame wrench $(0,0,f,M_x,M_y,M_z)$ can be realized within the given bounds. The attitude PD error terms are zero here only because that is **how a nominal trajectory point is defined**. The actual tracking error is not zero, and feedback control is still active in the real simulation.

The outputs keep exact analytic results and sample statistics apart: `exact_*yaw_rate_peak*` is the analytic peak of this time law, while `sampled_max_*` and the per-rotor thrust margins come only from the listed check points. The numerical residual tolerance is for floating-point solver error; it is not an extra dynamic margin added by hand.

**Passing the screening does not guarantee feasibility in continuous time, and it does not guarantee closed-loop stability or collision-free flight.** There can still be extrema between samples that were not sampled; the current screening does not verify motor rise/fall dynamics, command rates, battery transients, sensor noise, disturbances or the margin left for feedback correction. The report returns `continuous_time_certificate: false`, `closed_loop_guarantee: false` and `motor_dynamic_feasibility_checked: false`. The real motor limits still apply in the actuator model, and the tracking performance has to be judged later from the actual simulated trajectory and logs.

## 11. Where the geometric controller comes from and where it applies

The attitude error, the angular-velocity frame transformation and the geometric control structure with the full inertia follow Lee, Leok and McClamroch, [Geometric Tracking Control of a Quadrotor UAV on SE(3), CDC 2010](https://mathweb.ucsd.edu/~mleok/pdf/LeLeMc2010_quadrotor.pdf), and were checked against the local Isaac Lab Lee controller source. In the paper the third body axis points opposite to the thrust; this project takes body +Z as the positive thrust direction, so the translation and thrust formulas here are written in this project's convention, and the symbols in the paper need a coordinate conversion before they can be used.

The stability result of this continuous rigid-body control theory cannot be applied directly to this project. The local implementation has a discrete control period, finite motor thrust, real motor dynamics, tilt and command limiting, numerical feedback derivatives, contact and optional disturbances, and stability under these conditions needs to be verified separately. The endpoint smoothness of the ninth-degree trajectory, the nominal feedforward feasibility screening, the continuous control theory and the actual discrete simulation results are conclusions at different levels.

## 12. Allocating the body control target to the four rotors

Let $r_i$ be the position of the thrust application point of rotor $i$ in the root body frame, $c$ the position of the true center of mass of the whole vehicle in the same frame, and $a_i$ the unit thrust axis of that rotor. Let $f_i\ge0$ be its thrust magnitude, $\sigma_i\in\{-1,+1\}$ the sign of the reaction torque about that axis, and $k_{\tau i}$ the ratio of reaction torque to thrust, in m.

The contribution of this rotor about the center of mass of the whole vehicle is

$$
F_i=f_i a_i,\qquad
M_i=(r_i-c)\times(f_i a_i)+\sigma_i k_{\tau i}f_i a_i.
$$

So column $i$ of the allocation matrix is

$$
A_i=\begin{bmatrix}
a_i\\
(r_i-c)\times a_i+\sigma_i k_{\tau i}a_i
\end{bmatrix},\qquad
w=Af,\quad f=(f_1,f_2,f_3,f_4)^\mathsf T.
$$

All positions, axes and the center of mass here come from the actual loaded result, and $k_{\tau i}$ is read from the corresponding native actuator configuration. The four arm lengths use their actual values and do not have to be equal, and the arm length of the upstream reference matrix in the YAML is not used. $\sigma_i$ is the sign convention of the reaction torque; it is recorded separately from the rotor speed magnitude, and a minus sign by itself does not indicate the wiring direction of the real vehicle.

The allocator solves

$$
\min_{\underline f\le f\le\overline f}
\|W(Af-w_d)\|_2^2+\lambda\|f-f_{\mathrm{prev}}\|_2^2.
$$

$W=\mathrm{diag}(\mathrm{weights})$ scales the force and torque residuals numerically, and $\lambda\ge0$ is a smoothing penalty relative to the previous valid thrust command. The weights are interpreted as residual multipliers, so the coefficients in the quadratic cost are the squares of the weights; mixing N and N·m in one cost requires an explicit choice of scale/priority. A nonzero regularization lets a small wrench residual remain in exchange for a smaller command change, so in that case the allocation is no longer strictly error-free.

Each of the four variables has three active states: at the lower bound, free, or at the upper bound. The program enumerates at most $3^4=81$ combinations, solves a least-squares problem for the free variables in each one, and keeps the solution that satisfies the bounds and has the lowest total cost. This is different from computing a pseudo-inverse first and then roughly clipping all motors. The actual allocation matrix, the per-rotor bounds, the allocation result and the unrealized residual are all kept in the log.

## 13. Units and conversion from thrust to rotor speed

Each rotor uses the $k_{f i}>0$ actually sampled after reset:

$$
f_i=k_{f i}n_i^2,\qquad
n_{i,\mathrm{cmd}}=\sqrt{\frac{f_{i,\mathrm{cmd}}}{k_{f i}}}.
$$

Here $n_i$ is in **rps (revolutions per second)**, not rpm and not the body angular velocity. The conversions are

$$
\mathrm{rpm}_i=60n_i,\qquad \omega_{r,i}=2\pi n_i\quad[\mathrm{rad/s}].
$$

If the rotor angular speed is used instead and $f_i=k_{\omega i}\omega_{r,i}^2$ is written, the coefficient must be changed to

$$
k_{\omega i}=\frac{k_{f i}}{(2\pi)^2}.
$$

A coefficient in rps² cannot be put directly into a formula in rad²/s²; that would give a scale error of $(2\pi)^2$. The rotor quantities $n_i,\omega_{r,i}$ and the attitude angular velocity $\Omega$ of the whole vehicle must also be kept apart.

The operating lower and upper limits convert to

$$
n_{i,\min}=\sqrt{\frac{f_{i,\min}}{k_{f i}}},\qquad
n_{i,\max}=\sqrt{\frac{f_{i,\max}}{k_{f i}}}.
$$

The rotor-speed conversion first keeps the operating lower/upper limits of a native positive command, and a zero command stays zero. When different motors are sampled with different $k_f$, the same thrust needs different rotor speeds, so the coefficient actually sampled for each motor is used here instead of one "typical motor coefficient".

## 14. Asymmetric motor dynamics and discrete integration

`NativeRpsActuator` stores the rotor-speed state $n_i$ directly and reuses the rate function and the RK4/Euler integration functions of the local native thruster. At each physics step it chooses, from the limited target and the current rotor speed,

$$
\tau_i=
\begin{cases}
\tau_{\mathrm{dec},i},&n_{i,\mathrm{target}}<n_i,\\
\tau_{\mathrm{inc},i},&n_{i,\mathrm{target}}\ge n_i.
\end{cases}
$$

The blending coefficient for this step is

$$
\alpha_i=
\begin{cases}
1/(\Delta t+\tau_i),&\texttt{use\_discrete\_approximation=true},\\
1/\tau_i,&\texttt{use\_discrete\_approximation=false}.
\end{cases}
$$

The native rate function corresponds to

$$
G_i(e)=\operatorname{clip}(\alpha_i e,-q_i,q_i),\qquad
\dot n_i=G_i(n_{i,\mathrm{target}}-n_i).
$$

The field `max_thrust_rate` keeps the upstream name, but $q_i$ here actually limits the **rotor-speed rate of change in rps/s**; its unit is not N/s. The blending with $\Delta t$ above is how the current native model does it; it is not an identified electromagnetic motor equation.

The Euler update is $n_i^+=n_i+\Delta t\,G_i(e_i)$. RK4 computes, with the $\tau_i,\alpha_i$ and target selected for this step,

$$
\begin{aligned}
k_1&=G_i(e_i),\\
k_2&=G_i(e_i-\tfrac12\Delta t\,k_1),\\
k_3&=G_i(e_i-\tfrac12\Delta t\,k_2),\\
k_4&=G_i(e_i-\Delta t\,k_3),\\
n_i^+&=n_i+\frac{\Delta t}{6}(k_1+2k_2+2k_3+k_4).
\end{aligned}
$$

The numerical state is then limited to $[0,n_{i,\max}]$, and the **actual state** gives

$$
f_{i,\mathrm{actual}}=k_{f i}(n_i^+)^2.
$$

So the target rotor speed is not the actual rotor speed, and the target thrust is not the current thrust. During startup/stopping the actual rotor speed can be temporarily below the minimum operating speed of a positive command, so a rotor at rest does not get a nonzero thrust because of the operating lower limit. A zero target makes the motor slow down with the fall time constant; the existing rotor speed is not cleared instantly. The native random sampling of $k_f$ and of the rise/fall time constants is kept and recorded.

This layer is an executable empirical model from rotor speed to thrust. It is not CFD, and it is not a complete model that includes the ESC, current, voltage, blade flow field or freely rotating joints; adding aerodynamic and electrical effects still needs the corresponding calibration data and models.

## 15. Startup envelope and the force actually applied by each rotor

The delay phase uses the same ninth-degree function to build the rotor-speed startup envelope before limiting:

$$
\widetilde n_{i,\mathrm{cmd}}(t)=S((t-t_0)/T_d)\,n_{i,\mathrm{allocated}}(t).
$$

It is then processed with the real operating constraints:

$$
n_{i,\mathrm{target}}=
\begin{cases}
0,&\widetilde n_{i,\mathrm{cmd}}=0,\\
\operatorname{clip}(\widetilde n_{i,\mathrm{cmd}},n_{i,\min},n_{i,\max}),
&\widetilde n_{i,\mathrm{cmd}}>0.
\end{cases}
$$

$S$ itself is smooth at the end points, but when there is a positive operating lower limit, the first positive target may land directly on $n_{\min}$; so **the limited rotor-speed target is not necessarily still strictly $C^4$**. The actual rotor speed is still integrated by the motor dynamics of the previous section. The thrust and wrench that can actually be requested after the envelope are fed back to the controller's integral anti-windup logic, so that force not yet applied during startup is not counted as control already achieved.

The backend applies each $f_{i,\mathrm{actual}}a_i$ and its reaction torque at the local thrust origin of that rotor body, and PhysX passes them to the whole vehicle through the fixed joints. The motor output always acts at the rotor application points; it is not applied as a resultant control force on the body root link. The root body only receives the separately configured external disturbances, converted correctly into torque about the center of mass.

In the log,

$$
w_{\mathrm{motor}}=A f_{\mathrm{actual}},\qquad
w_{\mathrm{diagnostic}}=w_{\mathrm{motor}}+w_{\mathrm{external}}
$$

are diagnostic values summed about the center of mass of the whole vehicle and **are not submitted to the root body again as a whole**. Doing so would apply the force already submitted through the four rotors a second time. The true net force also includes terms solved by the physics engine such as gravity, contact and native damping, and should be told apart from the net force/torque derived from motion in the log.

The implementation behind this section is `isaac_drone/sim/actuation.py`, `isaac_drone/sim/isaaclab/backend.py`, `isaac_drone/control/allocation.py` and the local `IsaacLab/source/isaaclab_contrib/isaaclab_contrib/actuators/thruster.py`. These formulas describe the path actually executed now, with the native dynamics in place rather than an ideal instantaneous motor.

## 16. Minimum time: accelerate-cruise-decelerate time law and thrust envelope planning

When `takeoff_duration_s` or `helix_duration_s` is `null`, the geometric path of that phase stays the same (a vertical line segment, or the helix of Section 4). Only the progress $\sigma(t)\in[0,1]$ is replaced by the time law below, and after reset `isaac_drone/trajectories/min_time.py` computes the shortest duration from the actual vehicle parameters. The goal of the planning is to finish this phase in the shortest time; thrust utilization is a constraint, not the goal.

### 16.1 Time law

Let the cruise progress rate be $r$ [1/s], the acceleration segment duration $T_a$ and the deceleration segment duration $T_d$. The rate transition still uses $S(u)$ from Section 2:

$$
\dot\sigma(t)=
\begin{cases}
r\,S(t/T_a), & 0\le t<T_a,\\
r, & T_a\le t<T_a+T_c,\\
r\,\bigl(1-S(t'/T_d)\bigr), & t'=t-T_a-T_c\in[0,T_d].
\end{cases}
$$

Let $I(u)=\int_0^uS=21u^6-60u^7+67.5u^8-35u^9+7u^{10}$, with $I(1)=\tfrac12$ and the symmetry $I(u)=u-\tfrac12+I(1-u)$. Then $\sigma=rT_aI(u)$ in the acceleration segment and $\sigma=1-rT_dI(1-u')$ in the deceleration segment, and total progress equal to 1 gives

$$
T_c=\frac1r-\frac{T_a+T_d}{2}\ \ge 0 .
$$

The derivatives are $\ddot\sigma=rS'(u)/T_a$, $\dddot\sigma=rS''(u)/T_a^2$ and $\sigma^{(4)}=rS'''(u)/T_a^3$ (with a minus sign in the deceleration segment). The first to fourth derivatives of $S$ are zero at the end points, so $\dot\sigma$ is $C^4$ and the position reference is $C^5$, and the joins with takeoff and hover still satisfy $C^4$. The geometric derivatives use the formulas of Section 5, with $s_k$ replaced by the derivatives of $\sigma$.

### 16.2 Constraints

For each sampled state, the total thrust and torque needed for ideal tracking are computed as in Section 10. With four rotors and four channels the allocation is the unique exact solution $f=A_{[z,M]}^{-1}(f_\Sigma,M)$. Let the thrust range of each rotor be $[\underline f_i,\overline f_i]$ and the utilization be $\eta$ (`thrust_utilization`). The requirement is:

$$
\underline f_i+(1-\eta)(\overline f_i-\underline f_i)\ \le\ f_i\ \le\ \overline f_i-(1-\eta)(\overline f_i-\underline f_i).
$$

A margin of $(1-\eta)$ is left at each end for position/attitude feedback. Since thrust $\propto n^2$, $\eta=0.85$ corresponds to a maximum rotor speed of about $\sqrt{0.85}\approx92\%$.

The motor is a first-order lag (Section 14). For the actual thrust to keep up with $f(t)$, the command has to lead by about $\tau\dot f$, so it is also required that

$$
f_i+\tau\,\dot f_i\ \text{is also within the interval above},\qquad
\tau=\begin{cases}\tau_{\mathrm{inc}}^{\max}+\Delta t,&\dot f_i>0\\ \tau_{\mathrm{dec}}^{\max}+\Delta t,&\dot f_i<0\end{cases}
$$

$\tau$ is the upper bound of the configured sampling range (plus one physics step with the discrete approximation), and $\dot f_i$ is computed by differencing adjacent sample points. Without this constraint, the 0.24 s takeoff acceleration segment in the CPU regression makes the motors fall behind the command and saturates the thrust, with a maximum error of 0.5 m. The tilt, `max_acceleration_m_s2` and the yaw rate ($|\Theta| r\le$ `max_yaw_rate_rad_s` with tangent heading) are checked as well.

### 16.3 Search

1. **Steady cruise upper limit**: sample the state with $\dot\sigma\equiv r$ and all other derivatives zero along the whole path (at least 16 points per helix turn), and find the largest $r$ that satisfies the constraints by bisection. Vertical takeoff has no such upper limit.
2. **Shortest transition segments**: for a given $r$, the acceleration segment depends only on $(r,T_a)$ and the deceleration segment only on $(r,T_d)$, so the shortest $T_a,T_d$ that satisfy the constraints are found separately by bisection (33 sample points each).
3. **Total time**: $T(r)=T_a(r)+T_c(r)+T_d(r)$, and $T_c<0$ counts as infeasible. The minimum is found by golden-section search on $(0,\ r_{\max}]$.

The result is only nominally feasible in the sampled sense, and the check of Section 10 still reviews it against the real thrust limits at the end. The planning does not consider disturbances or feedback error. These are covered by the margin left, and whether the margin is enough has to be judged from the actual simulation logs.

### 16.4 Effect of heading

Taking the ARL parameters (1.24 kg, radius 1 m) as an example:

| Heading | Cruise speed | Nominal tilt | Nominal peak thrust of a single rotor |
|---|---|---|---|
| `fixed` | about 4.1 m/s | about 59° | 85% on the busiest rotor, only about 61% on average over the four rotors |
| `tangent` | about 5.0 m/s | about 68° | about 83% (balanced over the four rotors) |

With `fixed` heading the body circles while keeping its heading, so the thrust direction rotates periodically in the body frame and periodically varying roll, pitch and yaw torques are needed (about 0.3–0.4 N·m in this example; the yaw arm is only the reaction-torque ratio of 0.07 m, so yaw is especially costly). The four rotor thrusts differ by about 3 N, and one rotor reaches its upper limit before the total thrust is used up. With `tangent` heading the body rotates about the vertical axis at a constant angular rate, the body-frame angular velocity is constant in steady state, only a small gyroscopic torque remains, the four rotor thrusts are almost equal, and all the thrust can go to centripetal acceleration. The cost is a yaw rate of about 5 rad/s, so `max_yaw_rate_rad_s` has to be relaxed. `initial_phase_rad=-π/2` aligns the tangent with the initial heading, so the heading does not need to turn during takeoff.

For the same thrust the centripetal acceleration limit $a_c$ is fixed and the linear speed is $v=\sqrt{a_cR}$, so a larger radius allows a higher linear speed.

