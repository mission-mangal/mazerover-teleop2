# mazerover-teleop2
## **Instruction to follow**
1. Clone the Repository

```bash
git clone https://github.com/mission-mangal/mazerover-teleop2
```
2. Navigate to the workspace
```bash
cd mazerover-teleop2/
```
3. Install ros dependencies
```bash
sudo rosdep init
rosdep update
rosdep install --from-paths src -y --ignore-src -r
```
4. Build the workspace and source it
```bash
colcon build
source install/local_setup.bash
```
5. Run the game! (Wait for 2-3 minutes to load the maze)
```bash
ros2 launch leo_gz_bringup maze_copy.launch.py
```
6. Open another terminal and execute
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
