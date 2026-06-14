# PPO-LSTM Multi-UAV Exploration and Coverage

## Overview

This project investigates cooperative exploration and area coverage using multiple UAVs controlled by PPO and LSTM-based policies.

### Key Features
- Multi-agent reinforcement learning
- PPO policy optimization
- LSTM temporal memory
- Grid-world exploration
- A* path planning
- Coverage optimization
- Variable UAV fleet sizes

## Environment

Implemented in `GridWorldTSP60.m`.

- Grid size: 12x12
- Static obstacles
- Shared exploration memory
- Cooperative coverage mission

## Observation Space

Each UAV receives:
1. Obstacle map
2. Self position
3. Other UAV positions
4. Coverage map

## Reward Design

Positive rewards:
- Exploring new cells
- Increasing coverage

Penalties:
- Revisiting explored cells
- Long travel distances
- Inefficient exploration

## Path Planning

Implemented in `a_star.m`.

Capabilities:
- Shortest-path search
- Obstacle avoidance
- Travel-cost estimation

## PPO-LSTM Framework

Experiment files:
- TSP.mlx
- TSPLSTM.mlx

LSTM provides temporal memory for long-horizon exploration tasks.

## Multi-UAV Experiments

Suggested studies:
- Coverage rate
- Mission completion time
- Travel distance
- Scalability with increasing UAV count


## Requirements

- MATLAB
- Reinforcement Learning Toolbox
- Deep Learning Toolbox
- Simulink

## Future Work

- MAPPO
- Transformer policies
- Dynamic obstacles
- Energy-aware planning
- Communication constraints

## License

MIT License
