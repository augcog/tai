import string
import tiktoken
from dotenv import load_dotenv
load_dotenv()

encoding = tiktoken.encoding_for_model("gpt-3.5-turbo")


print(encoding.encode("tiktoken is great!"))


def token_size(sentence):
    return len(encoding.encode(sentence))




print("++++++++++++")
def rfind_punctuation(s, start, end):
    for i in range(end-1, start-1, -1):  # end-1 because Python slices are exclusive at the end
        if s[i] in string.punctuation:
            return i
    return -1  # If no punctuation is found


def send_split_message_user(response, token_limit=300):
    msg_list = []
    tokens = token_size(response)

    if tokens > token_limit:
        start = 0
        while start < len(response):
            end = start
            while end < len(response) and token_size(response[start:end]) < token_limit:
                end += 1

            if end < len(response):
                # Look for a suitable split position
                split_pos = response.rfind('\n\n', start, end)
                if split_pos == -1:
                    split_pos = response.rfind('\n', start, end)
                if split_pos == -1:
                    split_pos = rfind_punctuation(response, start, end)
                if split_pos == -1:
                    split_pos = response.rfind(' ', start, end)
                if split_pos == -1 or split_pos <= start:
                    split_pos = end - 1

                msg_list.append(response[start:split_pos].strip())
                start = split_pos + 1
            else:
                # Add the last chunk
                msg_list.append(response[start:end].strip())
                break
    else:
        msg_list.append(response)

    return msg_list


message="""
# OPW Kinematics Solver for Industrial Manipulators

## Intro

Taken from [opw_kinematics](https://github.com/Jmeyer1292/opw_kinematics):
A simple, analytical inverse kinematic library for industrial robots with parallel bases and spherical wrists.
Based on the paper *An Analytical Solution of the Inverse Kinematics Problem of Industrial Serial Manipulators
with an Ortho-parallel Basis and a Spherical Wrist* by Mathias Brandstötter, Arthur Angerer, and Michael Hofbaur.

## Purpose

This package is meant to provide a simpler alternative to IK-Fast based solutions in situations where one has an
industrial robot with a parallel base and spherical wrist. This configuration is extremely common in industrial robots.

The kinematics are parameterized by 7 primary values taken directly from the robot's spec sheet and a set of
joint-zero offsets. Given this structure, no other setup is required.

## Installation

The [opw_kinematics_plugin](https://github.com/JeroenDM/moveit_opw_kinematics_plugin) can be installed using `apt` on Ubuntu and Debian:

```
sudo apt install ros-noetic-moveit-opw-kinematics-plugin
```

## Usage

- Find the MoveIt [kinematics.yaml](../kinematics_configuration/kinematics_configuration_tutorial.html) file created for your robot.
- Replace `kinematics_solver: kdl_kinematics_plugin/KDLKinematicsPlugin` (or similar) with `kinematics_solver: moveit_opw_kinematics_plugin/MoveItOPWKinematicsPlugin`.
- Set parameters to describe the geometry of your manipulator.

The following is an example of the parameters needed for the KUKA KR 6 R700:

```yaml
manipulator:
  kinematics_solver: moveit_opw_kinematics_plugin/MoveItOPWKinematicsPlugin
  opw_kinematics_geometric_parameters:
    a1:  0.025
    a2: -0.035
    b:   0.000
    c1:  0.400
    c2:  0.315
    c3:  0.365
    c4:  0.080
  opw_kinematics_joint_offsets: [0.0, -1.57079632679, 0, 0, 0, 0]
  opw_kinematics_joint_sign_corrections: [-1, 1, 1, -1, 1, -1]
```

The meaning of the parameters can best be understood with a sketch and some tinkering. The plugin will print a `ROS_ERROR` on startup if they
do not match your URDF, so you can safely guess and test if needed:

```{image} images/opw.png
```

## Sharing OPW descriptions

We plan to collect OPW parameter sets as part of the ROS-Industrial robot support packages. This has already started for FANUC.
Taking the [M-10iA](https://github.com/ros-industrial/fanuc/blob/3ea2842baca3184cc621071b785cbf0c588a4046/fanuc_m10ia_support/config/opw_parameters_m10ia.yaml) as
an example you can reduce your `kinematics.yaml` to the following:

```yaml
manipulator:
  kinematics_solver: moveit_opw_kinematics_plugin/MoveItOPWKinematicsPlugin
```

and then add a `rosparam` `load` line to your `launch/planning_context.launch` which causes the parameters in that file to be loaded onto the parameter server:

```xml
<!-- Load default settings for kinematics; these settings are overridden by settings in a node's namespace -->
<group ns="$(arg robot_description)_kinematics">
  <rosparam command="load" file="$(find opw_tutorial)/config/kinematics.yaml"/>
  <rosparam command="load" ns="manipulator" file="$(find fanuc_lrmate200ib_support)/config/opw_parameters_lrmate200ib.yaml"/>
</group>
```

Note that the `ns` parameter has to match the name you gave your planning group during the setup.

The MoveIt Setup Assistant can automatically insert that line, removing the need to manually edit the `planning_context.launch` file. On the *Define Planning Group* page, select the *opw_parameters*
file for your robot in the *Kin. parameters file* field

```{image} images/assistant.png
```

"""

for n in [900,800,700,600,500,400,300,200,100]:
    list = send_split_message_user(message, n)
    # print(list)
    for i, split in enumerate(list):
        print(f"|{[split]}|")
        print(f"|{token_size(split)}|")
    print("++++++++++++")
