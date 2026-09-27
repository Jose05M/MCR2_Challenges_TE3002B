# pzb_tools

&ensp;&ensp;Data logging and analysis, used for the half-term report to measure how
well the robot followed its waypoints.

## Nodes and scripts

- **analysis_node** — writes a CSV row (`time, x, y, v, w, traffic, enc_r, enc_l,
  laser`) for every `/odom` message, with the last command, traffic light state and
  wheel speeds. Started by `traffic_light_nav.launch.py record:=true`.
  - In: `/odom`, `/cmd_vel`, `/traffic_light/state`, `/VelocityEncR`,
    `/VelocityEncL`, `/LaserDistance`
- **scripts/graphs.py** — reads that CSV, prints the metrics (position error,
  distance, time, average speeds, control smoothness) and saves the plots:
  `python3 graphs.py robot_analysis.csv`.

&ensp;&ensp;[results/half_term/](results/half_term/) keeps the CSV and plots of the
half-term run.

## Structure

```
pzb_tools/
├── pzb_tools/
│   └── analysis_node.py
├── scripts/
│   └── graphs.py
├── results/
│   └── half_term/
├── test/
├── package.xml
└── setup.py
```
