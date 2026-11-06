"""Print the 36 rangefinder lines of the LiDAR, one beam every 10 degrees.

The output was pasted into robotis_tb3/turtlebot3_waffle_pi.xml, under the lidar sites.

    python -m part_b_sensing.generatedlidar
"""

N = 36
for i in range(N):
    print(f'<rangefinder name="scan{i}" site="lidar{i:02d}" cutoff="3.5"/>')
