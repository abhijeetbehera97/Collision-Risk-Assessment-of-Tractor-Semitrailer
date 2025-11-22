# Collision Risk Assessment for Tractor-Semitrailer Vehicles

## Overview
This repository provides the data and code for assessing collision risk for tractor-semitrailer vehicles using two-dimensional time-to-collision.

## Data Description
The `Simulation data` folder contains 30 `.csv` files, with 15 rear-end and 15 sideswipe collision scenarios. The files are named, for example, `vehicle_dataLateral_11_c0`. Here:  
- `Lateral` denotes a sideswipe scenario,  
- `11` is the trailer length in meters,  
- `c0` is the case number.

### Simulator Data Columns
Each simulation file contains the following columns, with each row representing a snapshot at a given time step:

| Column                     | Description                                |
|-----------------------------|--------------------------------------------|
| `x1`, `y1`                 | Global coordinates of the tractor          |
| `x2`, `y2`                 | Global coordinates of the semitrailer      |
| `x3`, `y3`                 | Global coordinates of the car              |
| `vLx1`, `vLy1`             | Local frame velocities of the tractor      |
| `vGx1`, `vGy1`             | Global frame velocities of the tractor     |
| `vLx2`, `vLy2`             | Local frame velocities of the semitrailer  |
| `vGx2`, `vGy2`             | Global frame velocities of the semitrailer |
| `vLx3`, `vLy3`             | Local frame velocities of the car          |
| `vGx3`, `vGy3`             | Global frame velocities of the car         |
| `yaw1`, `pitch1`, `roll1`  | Orientation angles of the tractor          |
| `yaw2`, `pitch2`, `roll2`  | Orientation angles of the semitrailer      |
| `yaw3`, `pitch3`, `roll3`  | Orientation angles of the car              |
| `delta`                     | Steering input of the tractor-semitrailer  |


## Code Description
The files `TTCv2_predict_from_scenarios` and `TTCv3_predict_from_scenarios` read all scenarios and output time-to-collision estimates at 1, 1.38, and 2 seconds for both versions for each scenario.

## Rear-end collision
![RearEnd](RearEnd.gif)

## Citation
If you use this repository in your research, please cite:

```bibtex
@article{behera2025improved,
title={An improved two-dimensional time-to-collision for articulated vehicles: predicting sideswipe and rear-end collisions},
author={Behera, Abhijeet and Kharrazi, Sogol and Frisk, Erik and Aramrattana, Maytheewat},
journal={arXiv preprint arXiv:2507.04184},
year={2025}
}
```


