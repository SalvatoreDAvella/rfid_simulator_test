# rfid_simulator_test

This is an example repo to show how to use the RFID Gazebo plugin implemented [here](https://github.com/SalvatoreDAvella/rfid_simulator). The package allows simulating different settings: a single reader antenna moving in the plane, a single reader antenna also moving along Z, and two antennas, one that can move and the other that is fixed. Ten tags have been placed on the racks of a warehouse. The number of tags and reader antennas can be chosen as needed. 

## Usage
To run the simulations you have to
- clone the warehouse scenario in your ROS workspace

    ``git clone https://github.com/aws-robotics/aws-robomaker-small-warehouse-world.git``
    
- make sure you have installed gmapping, map server, amcl, and move base on your ROS environment

```
    sudo apt-get install ros-noetic-gmapping
    sudo apt-get install ros-noetic-map-server
    sudo apt-get install ros-noetic-amcl
    sudo apt-get install ros-noetic-move-base
```

- if you want to move the mobile robot with the keyboard, install teleop twist keyboard

    ``sudo apt-get install ros-noetic-teleop-twist-keyboard``
    
otherwise, you can choose your preferred method to move the robot.
If you want just to set up the Gazebo environment and move the robot with the keyboard, run

    roslaunch rfid_simulator_test simulation.launch
    
on a terminal, and 

    rosrun teleop_twist_keyboard teleop_twist_keyboard.py
    
on another terminal.
If you want to create the map of the environment, run 

    roslaunch rfid_simulator_test navigation_with_gmapping.launch
    
in place of ``simulation.launch`` and move around the mobile robot with the keyboard.
Once you are satisfied with the map you can save it using the map server.
Having the map you can navigate in the environment using amcl, by running

    roslaunch rfid_simulator_test navigation_with_amcl.launch
    
If you want to collect the data acquired by the reader antenna while the mobile robot moves, you have to run on another terminal

    rosrun rfid_simulator_test collect_data /topic1 /topic2 
    
Substitute ``/topic1 /topic2`` with the name of the topic advertised by your antenna(s).
When you decide to end the simulation, a csv file will be generated in your ROS HOME for each of the topic name having several rows containing ``time``, ``posx``, ``posy``, ``posz``, ``q1``, ``q2``, ``q3``, ``q4``, ``name``, ``dist``, ``phase``, ``rssi``, where ``time`` is the time of the acquisition, the other seven data are the position and orientation (quaternion) of the reader antenna, ``name`` is the ID of the RFID tag, ``dist`` is the ground truth distance between the reader and the tag, ``phase`` and ``rssi`` are the phase and RSSI signals.
By reading such csv file you can use your preferred tag localization method.

## Citing

If you use this tool in a research project, please check the [paper](https://doi.org/10.1109/ACCESS.2022.3152199) and cite it as follows:
```
@article{davella2022RFID,
  author={D’Avella, Salvatore and Unetti, Matteo and Tripicchio, Paolo},
  journal={IEEE Access}, 
  title={RFID Gazebo-Based Simulator With RSSI and Phase Signals for UHF Tags Localization and Tracking}, 
  year={2022},
  volume={10},
  number={},
  pages={22150-22160},
  publisher={IEEE},
  doi={10.1109/ACCESS.2022.3152199},
  issn={2169-3536}
}
```

## License
Distributed under the GNU License. See LICENSE for more information.
