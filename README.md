# IK_MORF: Inverse kinematic control of MORF

This project contains the tools to perform inverse kinematic control on MORF, either with equations or with neural networks. This includes:
- control code with inverse kinematic equations or with neural networks
    - for simulation
    - for the real robot
- data generation methods for the neural networks
    - with the inverse kinematics equation
    - with the simulation
    - with the real robot
- code for neural network training

**Note:** the executables are in /IK_MORF/catkin_ws/devel/lib/morf_ik

## Dependencies

This code is dependent on:
- fann library ([this page](https://leenissen.dk/fann/wp/help/installing-fann/) has instructions for installation)
- CoppeliaSim (download from [here](https://www.coppeliarobotics.com/downloads))
- ROS ([this page](http://wiki.ros.org/ROS/Installation) has instructions for installation)
- Ansible (install with the command `sudo apt install ansible`)

**Note:** make sure that the CMakeLists.txt (/IK_MORF/catkin_ws/src/morf_ik/CMakeLists.txt) is according to your machine. This means that you should change the path in line 11 to your installation of the fann library and the path in line 12 to you installation of CoppeliaSim.


## Control code in simulation

- Start a ROS core with the command `roscore`
- Open CoppeliaSim
- Open the simulation scene (File > Open scene... > MORF.ttt)
- execute `./main <TYPE> <N_TRIALS>` in the terminal
    - TYPE: 0 is using the equations and 1 is using the neural networks
    - N_TRIALS: number of trials to be executed

## Control code for the real robot

- Connect to morf's network
- Transfer the code by executing `ansible-playbook -i inventory morf_transfer_controller.yml`
- If using the neural networks, execute also `ansible-playbook -i inventory morf_transfer_nn.yml`
- SSH into MORF and execute the command `cd /home/morf-one/workspace/gorobots-mthor/projects/morf/real/catkin_ws/src/morf_controller/bin`
- Execute `./morf_controller_real <TYPE>`
    - TYPE: 0 is using the equations and 1 is using the neural networks

## Data generation for neural networks

### Inverse kinematics equations
- Execute `./gen_data_ik`

### Simulation
- Start a ROS core with the command `roscore`
- Open CoppeliaSim
- Open the simulation scene (File > Open scene... > MORF_gen_data.ttt)
- Execute `./gen_data_sim`
- When that is done, execute `./sort_data`

**Note:** go to line 100 of the file **sort_data.cpp** to change the number of divisions of the workspace (variable named **div**)

<!-- ### Real robot
- Connect to morf's network
- Transfer the code by executing `ansible-playbook -i inventory morf_transfer_genData.yml`
- If using the neural networks, execute also `ansible-playbook -i inventory morf_transfer_nn.yml`
- SSH into MORF and execute the command `cd /home/morf-one/workspace/gorobots-mthor/projects/morf/real/catkin_ws/src/morf_controller/bin`
- Execute -->