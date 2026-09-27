# pzb_odometry

&ensp;&ensp;Dead-reckoning odometry for the Puzzlebot, used by every challenge that
needs position feedback.

## Nodes

- **odometry** — integrates the wheel speeds with the differential-drive
  kinematics (`V = r(ω_R + ω_L)/2`, `Ω = r(ω_R − ω_L)/L`) and publishes the pose,
  starting at `(0, 0, 0)`.
  - In: `/VelocityEncR`, `/VelocityEncL` (`std_msgs/Float32`, rad/s)
  - Out: `/odom` (`nav_msgs/Odometry`)

## Structure

```
pzb_odometry/
├── pzb_odometry/
│   └── odometry.py
├── test/
├── package.xml
└── setup.py
```
