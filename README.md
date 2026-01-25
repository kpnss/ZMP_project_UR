# Project Development
The following repo was used as the starting point for the development of the project for Underactuated Robots course.

# Description
This is IS-MPC, a framework for humanoid gait generation.

The main reference is:<br />
[N. Scianca, D. De Simone, L. Lanari, G. Oriolo, "MPC for Humanoid Gait Generation: Stability and Feasibility"](https://ieeexplore.ieee.org/document/8955951)<br />
*Transactions on Robotics*, 2020

The extension available in this repository uses the 3D LIP and can also generate vertical motions. Main reference:<br />
[M. Cipriano, P. Ferrari, N. Scianca, L. Lanari, G. Oriolo, "Humanoid motion generation in a world of stairs"](https://www.sciencedirect.com/science/article/pii/S0921889023001343)<br />
*Robotics and Autonomous Systems*, 2023

To this framework, a novel balancing technique based on Capture Points was added. The newest control is comprised of both. Main reference:>br />
[Mitsuharu Morisawa, Shuuji Kajita, Fumio Kanehiro, Kenji Kaneko, Kanako Miura, Kazuhiro Yokoi, "Balance Control based on Capture Point Error Compensation for Biped Walking on Uneven Terrain"](https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=6651601)<br />

# Setup
You need a Python installation and some dependencis. If using pip, you can run the following
```
pip install dartpy casadi scipy matplotlib osqp
```
You need dartpy 0.2, if pip does not allow you to install this version on your system, you probably need to upgrade to Python 3.12 or use conda

To run the simulation
```
python simulation.py
```
then press spacebar to start it
