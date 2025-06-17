# Webots Warehouse Robot Navigation System

A concise Webots project that showcases how **A\*** path-finding, buffer-aware obstacle avoidance, and Bézier-curve smoothing can be combined to let a mobile robot weave through a cluttered warehouse map with human-like fluidity.

## Key features

| Feature | Why it matters |
|---------|----------------|
| **Reduced buffer width** | Fine-tunes the clearance envelope so the robot can slip through narrow aisles without bumping pallets or racks. |
| **Curved path planning** | Converts the jagged A\* output into smooth Bézier curves → lower jerk, more efficient motion, and quieter operation. |

---

## Quick start

```bash
# 1 Clone the repo

cd webots-warehouse-sim

# 2 Launch Webots
webots worlds/warehouse.wbt
