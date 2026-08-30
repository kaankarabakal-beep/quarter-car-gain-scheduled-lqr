# Sensor-Aware Gain-Scheduled LQR Active Suspension

A MATLAB-based active suspension project developed using a two-degree-of-freedom quarter-car model.

The project progresses from a passive suspension and fixed LQR controller to a response-based gain-scheduled LQR architecture that adapts the control objective according to measured vehicle dynamics rather than prior knowledge of the road profile.

<p align="center">
  <img src="visualization/Model3B_teaser.png" width="800">
</p>

<p align="center">
  <i>Model 3B response-based active suspension simulation on a composite road profile.</i>
</p>

## Control Development

The project was developed progressively through four control stages:

**Model 1 — Passive Suspension**  
A conventional quarter-car suspension model was used to establish the baseline response and illustrate the inherent trade-offs between ride comfort, suspension travel, and tire behavior.

**Model 2 — Fixed LQR Control**  
A fixed Linear Quadratic Regulator was introduced to provide active suspension control. The controller improved body response but retained a single control compromise for all road conditions.

**Model 3A — Manual Gain Scheduling**  
A bank of LQR controllers with different priorities was introduced. Controller selection was manually scheduled according to known road position, demonstrating the potential benefit of changing the control objective with the disturbance condition.

**Model 3B — Response-Based Gain Scheduling**  
The final controller removes the requirement for prior road knowledge. Instead, the active LQR mode is selected online using measured vehicle responses.

<p align="center">
  <b>Passive → Fixed LQR → Manual Gain Scheduling → Response-Based Gain Scheduling</b>
</p>


## Model 3B Architecture

The response-based scheduler uses:

- sprung-mass acceleration
- unsprung-mass acceleration
- suspension travel
- relative suspension velocity

to select between four LQR controllers: **Balanced**, **Comfort**, **Low Force**, and **Protection**.

The scheduler combines slower RMS-based response classification with a faster transient-detection layer for short disturbance events.

## Scheduling Logic

<p align="center">
  <img src="figures/scheduler_logic.png" width="900">
</p>

<p align="center">
  <i>Hierarchical response-based scheduling logic used in Model 3B.</i>
</p>


## Implementation Constraints

To move beyond an idealized controller, the final Model 3B implementation includes:

- actuator force limit: **±750 N**
- actuator force-rate limit: **50,000 N/s**
- sensor sampling frequency: **500 Hz**
- sensor transport delay: **4 ms**
- measurement-noise robustness tests


These constraints were included to evaluate the controller under more realistic implementation conditions rather than under unconstrained ideal actuation and instantaneous measurements.
## Key Results

On the common composite benchmark road, Model 3B was compared with the fixed Balanced LQR controller under the same actuator force and force-rate limits.

| Metric | Fixed LQR | Model 3B | Change |
|---|---:|---:|---:|
| Body acceleration RMS | 1.232 m/s² | 1.219 m/s² | -1.0% |
| Peak body acceleration | 5.976 m/s² | 5.232 m/s² | -12.4% |
| Peak suspension travel | 33.487 mm | 30.714 mm | -8.3% |
| Peak tire deflection | 16.622 mm | 15.288 mm | -8.0% |
| Actuator force RMS | 118.6 N | 128.5 N | +8.3% |
| Peak actuator force | 529.65 N | 750 N | Saturated |

The main benefit of the scheduler is therefore not uniform reduction of every performance metric, but the ability to change the suspension trade-off according to the dominant measured response.


## Robustness Evaluation

The final controller was tested without retuning the controller gains, scheduling thresholds, RMS windows, or transient hold periods.

The robustness studies included:

- bump heights of 15, 30, and 45 mm
- vehicle speeds of 5, 10, and 15 m/s
- previously unseen deterministic multi-sine road profiles
- sensor transport delay
- measurement noise

The scheduler remained operational across all tested conditions. On unseen road profiles, the controller did not uniformly outperform the fixed LQR in every metric; instead, it shifted between control priorities depending on the measured vehicle response.

In the combined sensor-realism tests, the nominal bump was consistently confirmed as a shock under all tested noise levels, while the wheel-hop region was not incorrectly classified as a shock.


## Repository Structure

```text
quarter-car-gain-scheduled-lqr/
├── src/
│   ├── model1_model2_passive_fixed_lqr.m
│   ├── model3a_manual_scheduling.m
│   └── model3b_response_based.m
│
├── report/
│   └── Quarter_Car_Active_Suspension_Report.pdf
│
├── figures/
│   └── scheduler_logic.png
│
└── visualization/
    ├── Model3B_teaser.png
    ├── model3b_animation.m
    └── Model3B_Animation.mp4

## Requirements

- MATLAB
- Control System Toolbox

The project uses MATLAB's state-space modeling, numerical integration, and LQR design tools.


## How to Run

Clone or download the repository and open the project folder in MATLAB.

For the main simulations:

1. Run `src/model1_model2_passive_fixed_lqr.m` for the passive and fixed-LQR stages.
2. Run `src/model3a_manual_scheduling.m` for the manually scheduled reference controller.
3. Run `src/model3b_response_based.m` for the final response-based gain-scheduled controller and robustness studies.

The main Model 3B script contains the controller bank, response-based scheduler, actuator and sensor constraints, benchmark-road evaluation, and robustness tests.


## Full Report

A complete description of the mathematical model, LQR formulation, scheduling logic, implementation constraints, and robustness evaluation is available in the project report:

[**View Full Project Report**](report/Quarter_Car_Active_Suspension_Report.pdf)

## Scope

This project is a simulation-based proof of concept using a linear two-degree-of-freedom quarter-car model. It does not represent a production-ready automotive control system.

