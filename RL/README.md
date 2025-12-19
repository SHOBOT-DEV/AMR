# RL Sensor Fusion + Intent Prediction (Advanced)

## Problem
- Humans are non-deterministic.
- Classical planners react after movement begins.

## What RL learns
- Predict trajectories.
- Decide yield vs go.
- Avoid crossing paths early.

## Inputs
- Camera bounding boxes.
- Depth motion.
- LiDAR clusters.
- IMU dynamics.

## Outputs
- Slow down.
- Reroute.
- Yield.
- Proceed.

## Where used
- Hospitals.
- Airports.
- Human-robot collaboration zones.

---

# Dynamic Speed Modulation (Safety + Efficiency)

## Rule-based safety today
- if distance < X → stop
- Leads to jerky motion, over-conservatism, poor throughput.

## What RL controls
- Only speed, not steering.
- Learns: how fast is safe now, predictive slowing, smooth acceleration.

## Inputs
- Obstacle density.
- Curvature ahead.
- Floor conditions.
- Human proximity.

## Reward
- +Mission speed.
- −Sudden braking.
- −Near collisions.
- +Comfort metrics.

## Outcome
- Faster missions.
- Safer human interaction.
- Reduced emergency stops.
