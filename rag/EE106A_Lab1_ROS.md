------------------------------------------------------------------------

Goals {#goals .unnumbered}
=====

By the end of this lab you should be able to:

-   Set up a new ROS environment, including creating a new workspace and
    creating a package with the appropriate dependencies specified

-   Use the `catkin` tool to build the packages contained in a ROS
    workspace

-   Run nodes using `rosrun`

-   Use ROS's built-in tools to examine the topics and services used by
    a given node

-   Run a simple Gazebo simulation

------------------------------------------------------------------------

If you get stuck at any point in the lab you may submit a help request
during your lab section at <https://tinyurl.com/106alabs20>.\
*A quick note:* Most of this lab is borrowed from the official ROS
tutorials at <http://www.ros.org/wiki/ROS/Tutorials>. We've tried to
pick out the material you'll find most useful later in the semester, but
feel free to explore the other tutorials too if you're interested in
learning more.\
**Note:** For all labs this semester you may collaborate with a lab
partner but we expect everyone to do every part of the labs themselves.
You should work closely with your partner to overcome obstacles in the
labs but each member of the team must do the lab themselves.

What is ROS?
============

The ROS website says:

> ROS is an open-source, meta-operating system for your robot. It
> provides the services you would expect from an operating system,
> including hardware abstraction, low-level device control,
> implementation of commonly-used functionality, message-passing between
> processes, and package management. It also provides tools and
> libraries for obtaining, building, writing, and running code across
> multiple computers.
>
> The ROS runtime "graph" is a peer-to-peer network of processes that
> are loosely coupled using the ROS communication infrastructure. ROS
> implements several different styles of communication, including
> synchronous RPC-style communication over services, asynchronous
> streaming of data over topics, and storage of data on a Parameter
> Server.

This isn't terribly enlightening to a new user, so we'll simplify a
little bit. For the purposes of this class, we'll be concerned with two
big pieces of ROS's functionality: the *computation graph* and the *file
system*.

Computation graph
-----------------

A typical robotic system has numerous sensing, actuation, and computing
components. Consider a two-joint manipulator arm for a pick-and-place
task. This system might have:

-   Two motors, each connected to a revolute joint

-   A motorized gripper on the end of the arm

-   A stationary camera that observes the robot's workspace

-   An infrared distance sensor next to the gripper on the manipulator
    arm

To pick up an object on a table, the robot might first use the camera to
measure the position of the object, then command the arm to move toward
the object's position. As the arm nears the object, the robot could use
the IR distance sensor to detect when the object is properly positioned
in the gripper, at which point it will command the gripper to close
around the object. Given this sequence of tasks, how should we structure
the robot's control software?

A useful abstraction for many robotic systems is to divide the control
software into various low-level, independent control loops that each
manage a single task on the robot, then couple these low level loops
together with higher-level logic. In our example system above, we might
divide the control software into:

-   Two control loops (one for each joint) that, given a position or
    velocity command, control the power applied to the joint motor based
    on position sensor measurements at the joint

-   Another control loop that receives commands to open or close the
    gripper, then switches the gripper motor on and off while
    controlling the power applied to it to avoid crushing objects

-   A sensing loop that reads individual images from the camera at 30 Hz

-   A sensing loop that reads the output of the IR distance sensor at
    100 Hz

-   A single high-level module that performs the supervisory control of
    the whole system

Given this structure for the robot's software, the control flow for the
pick-and-place task could be the following: The high-level supervisor
queries the camera sensing loop for a single image. It uses a vision
algorithm to compute the location of the object to grasp, then computes
the joint angles necessary to move the manipulator arm to this location
and sends position commands to each of the joint control loops telling
them to move to this position. As the arm nears the target, the
supervisor queries the IR sensor loop for the distance to the object at
a rate of 5 Hz and issues several more fine motion commands to the joint
control loops to position the arm correctly. Finally, the supervisor
signals the gripper control loop to close the gripper.

An important feature of this design is that the supervisor need not know
the implementation details of any of the low-level control loops. It
interacts with each only through simple control messages. This
encapsulation of functionality within each individual control loop makes
the system modular, and makes it easier to reuse the same code across
many robotic platforms.

The ROS computation graph lets us build this style of software easily.
In ROS, each individual control loop is a *node* within the computation
graph. A node is simply an executable file that performs some task.
Nodes exchange control messages, sensor readings, and other data by
publishing or subscribing to *topics* or by sending requests to
*services* offered by other nodes (these concepts will be discussed in
detail later in the lab).

Nodes can be written in a variety of languages, including Python and
C++, and ROS transparently handles the details of converting between
different datatypes, exchanging messages between nodes, etc.

File system
-----------

As you might imagine, large software systems written using this model
can become quite complex (nodes written in different languages, nodes
that depend on third-party libraries and drivers, nodes that depend on
other nodes, etc.). To help with this situation, ROS provides a system
for organizing your ROS code into logical units and managing
dependencies between these units. Specifically, ROS code is contained in
*packages*. ROS provides a collection of tools to manage packages that
will be discussed in more detail in the following sections.

Setting Up and Using Your Virtual Machine
=========================================

Your virtual machine should be set up and ready to use after completing
Lab 0. It should be done *before* beginning this lab. If you have not
completed Lab 0 yet, you must go back and finish it first.

Please note that if you delete your container your files saved in the VM
will also be deleted so we **highly recommend you use GitHub** or some
other form of version control just in case. This will also make it
easier for collaboration between you and your lab partner. Just don't
forget to make your repo **private**!

Gradescope Submission
=====================

In this class we use an autograder on Gradescope in order to
automatically grade everyone's checkoffs. Please create a `.txt` file
containing **only your SID** and nothing else (the file should only be 8
or 10 bytes in size) and upload it to the Lab Checkoffs assignment on
Gradescope. The autograder will automatically run and you should see a
score of 0 points. You **must** do this in order to get credit for the
labs. You may name your file whatever you would like as long as it is a
`.txt` file.\
Throughout the semester we will automatically be rerunning the
autograder at the end of each lab module (more about the modules
[here](https://ucb-ee106.github.io/106a-fa20site/policies/#labs)) to
update your lab score. Please be aware that it is your responsibility to
check your score periodically and make sure there isn't a mistake. If
you believe there is a mistake please email Tiffany as soon as possible
so that it can be corrected.

Initial configuration
=====================

The virtual machines you're using already have ROS installed from Lab 0.
Open the `.bashrc` file, located in your root directory (denoted " `~`
"), in a text editor (If you don't have a preferred editor, we recommend
either [Sublime Text](https://www.sublimetext.com/) or vim). Then add
the following line to the end of the file if it is not already there:

``` {frame="single"}
source /opt/ros/kinetic/setup.bash
```

Save and close the file when you're done editing, then execute the
command "`source ~/.bashrc`" to update your environment with the new
settings. This line tells Ubuntu to run a ROS-specific configuration
script every time you open a new terminal window. This script sets
several environment variables that tell the system where the ROS
installation is located.

Navigating the ROS file system
==============================

The basic unit of software organization in ROS is the *package*. A
package can contain executables, source code, libraries, and other
resources. A `package.xml` file is included in the root directory of
each package. The `package.xml` contains metadata about the package
contents, and most importantly, about which other packages this package
depends on. Let's examine a package within the Baxter robot SDK as an
example.

File system tools
-----------------

ROS provides a collection of tools to create, edit, and manage packages.
One of the most useful is `rospack`, which returns information about a
specific package. For example, you can run the command

``` {frame="single"}
rospack find baxter_examples
```

in order to find the filepath for where `baxter_examples` is located.

*Note*: To get info on the options and functionality of many ROS command
line utilities, run the utility plus "`help`" (e.g., just run
"`rospack help`").

Anatomy of a package
--------------------

In a terminal window, try running

``` {frame="single"}
roscd baxter_examples
```

What happened? what do you think the command `roscd` does? What do you
think `rosls` does? Try it out!\
The `baxter_examples` package contains several example nodes which
demonstrate the motion control features of Baxter. The folder contains
several items:

-   `\src` - source code for nodes

-   `package.xml` - the package's configuration and dependencies

-   `\launch` - launch files that start ROS and relevant packages all at
    once

-   `\scripts` - another folder to store nodes

Other packages might contain some additional items:

-   `\lib` - extra libraries used in the package

-   `\msg` and `\srv` - message and service definitions which define the
    protocols nodes use to exchange data

If you open the `package.xml` it should look something like this:

``` {frame="single"}
<?xml version="1.0"?>
<package>
  <name>baxter_examples</name>
  <version>1.2.0</version>
  <description>
    Example programs for Baxter SDK usage.
  </description>

  <maintainer email="rsdk.support@rethinkrobotics.com">
    Rethink Robotics Inc.
  </maintainer>
  <license>BSD</license>
  <url type="website">http://sdk.rethinkrobotics.com</url>
  <url type="repository">
    https://github.com/RethinkRobotics/baxter_examples
  </url>
  <url type="bugtracker">
    https://github.com/RethinkRobotics/baxter_examples/issues
  </url>
  <author>Rethink Robotics Inc.</author>

  <buildtool_depend>catkin</buildtool_depend>

  <build_depend>rospy</build_depend>
  <build_depend>xacro</build_depend>
  <build_depend>actionlib</build_depend>
  <build_depend>sensor_msgs</build_depend>
  <build_depend>control_msgs</build_depend>
  <build_depend>trajectory_msgs</build_depend>
  <build_depend>cv_bridge</build_depend>
  <build_depend>dynamic_reconfigure</build_depend>
  <build_depend>baxter_core_msgs</build_depend>
  <build_depend>baxter_interface</build_depend>

  <run_depend>rospy</run_depend>
  <run_depend>xacro</run_depend>
  <run_depend>actionlib</run_depend>
  <run_depend>sensor_msgs</run_depend>
  <run_depend>control_msgs</run_depend>
  <run_depend>trajectory_msgs</run_depend>
  <run_depend>cv_bridge</run_depend>
  <run_depend>dynamic_reconfigure</run_depend>
  <run_depend>baxter_core_msgs</run_depend>
  <run_depend>baxter_interface</run_depend>

</package>
```

Along with some metadata about the package, the `package.xml` specifies
11 packages on which `baxter_examples` depends. The packages with
`<build_depend>` are the packages used during the build phase and the
ones with `<run_depend>` are used during the run phase. The `rospy`
dependency is important - `rospy` is the ROS library that Python nodes
use to communicate with other nodes in the computation graph. The
corresponding library for C++ nodes is `roscpp`. The `build_depend` tags
indicate packages used during the build phase. The `run_depend` tags
indicate packages used during runtime.

Creating ROS Workspaces and Packages {#sec:ws_and_pack}
====================================

You're now ready to create your own ROS package. To do this, we also
need to create a catkin workspace. Since all ROS code must be contained
within a package in a workspace, this is something you'll do frequently.
Don't forget to do all this *inside* of your VM.

Creating a workspace
--------------------

A workspace is a collection of packages that are built together. ROS
uses the `catkin` tool to build all code in a workspace, and do some
bookkeeping to easily run code in packages. Each time you start a new
project (i.e. lab or final project) you will want to create and
initialize a new catkin workspace.\
For this lab, begin by creating a directory for all of your lab
workspaces for the semester.

``` {frame="single"}
mkdir ros_workspaces
```

Then create a directory with

``` {frame="single"}
mkdir ros_workspaces/lab1
```

for Lab 1's workspace. The directory "`ros_workspaces`" will eventually
conatin several lab-specific workspaces (named `lab1`, `lab2`, etc.)\
Next, create a folder `src` in your new workspace directory (`lab1`).
From inside the new `src` folder, run:

``` {frame="single"}
catkin_init_workspace
```

It should create a single file called `CMakeLists.txt`\
After you fill `/src` with packages, you can build them by running
"`catkin_make`" from the workspace directory (`lab1` in this case). Try
running this command now, just to make sure the build system works. You
should notice two new directories alongside `src`: `build` and `devel`.
ROS uses these directories to store information related to building your
packages (in `build`) as well as automatically generated files, like
binary executables and header files (in `devel`).

Two other useful commands to know are `rmdir` to remove an empty
directory and `rm -r` to remove a non-empty directory.

Creating a new package
----------------------

Now you're now ready to create a package. From inside the `src`
directory, run

``` {frame="single"}
catkin_create_pkg foo
```

Examine the contents of your newly created package, and open its
`package.xml` file. By default, you will see that the only dependency
created is for the catkin tool itself:

``` {frame="single"}
<buildtool_depend>catkin</buildtool_depend>
```

Next, we'll try the same command, but we'll specify a few dependencies
for our new package. Return to the `src` directory and run the following
command:

``` {frame="single"}
catkin_create_pkg bar rospy roscpp std_msgs geometry_msgs turtlesim gazebo_ros
```

Examine the `package.xml` file for the new package and verify that the
dependencies have been added. You're now ready to add source code,
message and service definitions, and other resources to your project.

Building a package
------------------

Now imagine you've added all your resources to the new package. The last
step before you can use the package with ROS is to *build* it. This is
accomplished with

``` {frame="single"}
catkin_make
```

You need to run it from the root of your workspace (`lab1` in this
case).\
`catkin_make` builds all the packages and their dependencies in the
correct order. If everything worked, `catkin_make` should print a bunch
of configuration and build information for your new packages "`foo`" and
"`bar`", with no errors.\
You should also notice that the `devel` directory contains a script
called "`setup.bash`." "Sourcing" this script will prepare your ROS
environment for using the packages contained in this workspace (among
other functions, it adds "`~/ros_workspaces/lab1/src`" to the
`$ROS_PACKAGE_PATH`). Run the commands

``` {frame="single"}
echo $ROS_PACKAGE_PATH
source devel/setup.bash
echo $ROS_PACKAGE_PATH
```

and note the difference between the output of the first and second
`echo`.

------------------------------------------------------------------------

**Note:** *Anytime that you want to use a non-built-in package, such as
one that you have created, you will need to source the*
`devel/setup.bash` *file for that package's workspace.*

------------------------------------------------------------------------

To summarize what we've done, here's what your directory structure
should look like:

``` {frame="single" samepage="true"}
ros_workspaces
  lab1
    build
    devel
      setup.bash
    src
      CMakeLists.txt
      foo
        CMakeLists.txt
        package.xml
      bar
        CMakeLists.txt
        package.xml
        include
        src
```

------------------------------------------------------------------------

Checkpoint 1 {#checkpoint-1 .unnumbered}
------------

Submit a checkoff request at <https://tinyurl.com/106alabs20> for a TA
to come and check your work. You should be able to:

-   Show your TA that you have submitted a `.txt` file to the Lab
    Checkoffs assignment on Gradescope

-   Explain the contents of your `~/ros_workspaces` directory

-   Demonstrate the use of the `catkin_make` command

-   Explain the contents of a `package.xml` file

-   Use ROS's utility functions to get data about packages

------------------------------------------------------------------------

Understanding ROS nodes
=======================

We're now ready to test out some actual software running on ROS. First,
a quick review of some computation graph concepts:

-   *Node*: an executable that uses ROS to communicate with other nodes

-   *Message*: a ROS datatype used to exchange data between nodes

-   *Topic*: nodes can *publish* messages to a topic as well as
    *subscribe* to a topic to receive messages

Now let's test out some built-in examples of ROS nodes.

Running roscore
---------------

First, run the command

``` {frame="single"}
roscore
```

This starts a server that all other ROS nodes use to communicate. Leave
`roscore` running and open a second terminal window (`Ctrl+Shift+T` or
`Ctrl+Shift+N`).

As with packages, ROS provides a collection of tools we can use to get
information about the nodes and topics that make up the current
computation graph. Try running

``` {frame="single"}
rosnode list
```

This tells us that the only node currently running is `/rosout`, which
listens for debugging and error messages published by other nodes and
logs them to a file. We can get more information on the `/rosout` node
by running

``` {frame="single"}
rosnode info /rosout
```

whose output shows that `/rosout` publishes the `/rosout_agg` topic,
subscribes to the `/rosout` topic, and offers the `/set_logger_level`
and `/get_loggers` services.

The `/rosout` node isn't very exciting. Let's look at some other
built-in ROS nodes that have more interesting behavior.

Running turtlesim
-----------------

To start additional nodes, we use the `rosrun` command. The syntax is

``` {frame="single"}
rosrun [package_name] [executable_name]
```

The ROS equivalent of a "hello world" program is turtlesim. To run
turtlesim, we first want to start the `turtlesim_node` executable, which
is located in the `turtlesim` package, so we open a new terminal window
and run

``` {frame="single"}
rosrun turtlesim turtlesim_node
```

A turtlesim window should appear. Repeat the two `rosnode` commands from
above and compare the results. You should see a new node called
`/turtlesim` that publishes and subscribes to a number of additional
topics.

Understanding ROS topics
========================

Now we're ready to make our turtle do something. Leave the `roscore` and
`turtlesim_node` windows open from the previous section. In a yet
another new terminal window, use `rosrun` to start the
`turtle_teleop_key` executable in the `turtlesim` package:

``` {frame="single"}
rosrun turtlesim turtle_teleop_key
```

You should now be able to drive your turtle around the screen with the
arrow keys when in this terminal window.

Using rqt\_graph
----------------

Let's take a closer look at what's going on here. We'll use a tool
called `rqt_graph` to visulize the current computation graph. Open a new
terminal window and run

``` {frame="single"}
rosrun rqt_graph rqt_graph
```

This should produce an illustration similar to Figure
[1](#rqt){reference-type="ref" reference="rqt"} (it might not look
exactly the same but that's fine). In this example, the `teleop_turtle`
node is capturing your keystrokes and publishing them as control
messages on the `/turtle1/cmd_vel` topic. The `/turtlesim` node then
subscibes to this same topic to receive the control messages.

![Output of rqt\_plot when running
turtlesim.[]{label="rqt"}](img/rqt_graph.png){#rqt width="6.5in"}

Using rostopic
--------------

Let's take a closer look at the `/turtle1/cmd_vel` topic. We can use the
`rostopic` tool. First, let's look at individual messages that
`/teleop_turtle` is publishing to the topic. We will use
"`rostopic echo`" to echo those messages. Open a new terminal window and
run

``` {frame="single"}
rostopic echo /turtle1/cmd_vel
```

Now move the turtle with the arrow keys and observe the messages
published on the topic. Return to your `rqt_graph` window, and click the
refresh button (blue circle arrow icon in the top left corner). You
should now see that a second node (the `rostopic` node) has subscribed
to the `/turtle1/cmd_vel` topic, as shown in Figure
[2](#rqt2){reference-type="ref" reference="rqt2"}.

![Output of rqt\_graph when running turtlesim and viewing a topic using
rostopic echo.[]{label="rqt2"}](img/rqt2.png){#rqt2 width="6.5in"}

While `rqt_graph` only shows topics with at least one publisher and
subscriber, we can view all the topics published or subscribed to by all
nodes by running

``` {frame="single"}
rostopic list
```

For even more information, including the message type used for each
topic, we can use the verbose option:

``` {frame="single"}
rostopic list -v
```

Keep the turtlesim running for use in the next section.

Understanding ROS services
==========================

*Services* are another method nodes can use to pass data between each
other. While topics are typically used to exchange a continuous stream
of data, a service allows one node to *request* data from another node,
and receive a *response*. Requests and responses are to services as
messages are to topics: that is, they are containers of relevant
information for their associated service or topic.

Using rosservice
----------------

The `rosservice` tool is analogous to `rostopic`, but for services
rather than topics. We can call

``` {frame="single"}
rosservice list
```

to show all the services offered by currently running nodes.

We can also see what type of data is included in a request/response for
a service. Check the service type for the `/clear` service by running

``` {frame="single"}
rosservice type /clear
```

This tells us that the service is of type `std_srvs/Empty`, which means
that the service does not require any data as part of its request, and
does not return any data in its response.

Calling services
----------------

Let's try calling the the `/clear` service. While this would usually be
done programmatically from inside a node, we can do it manually using
the `rosservice call` command. The syntax is

``` {frame="single"}
rosservice call [service] [arguments]
```

Because the `/clear` service does not take any input data, we can call
it without arguments

``` {frame="single"}
rosservice call /clear
```

If we look back at the `turtlesim` window, we see that our call has
cleared the background.

We can also call services that require arguments. Use `rosservice type`
to find the datatype for the `/spawn` service. The query should return
`turtlesim/Spawn`, which tells us that the service is of type `Spawn`,
and that this service type is defined in the `turtlesim` package. Use
`rospack find turtlesim` to get the location of the `turtlesim` package
(hint: you could also use "`roscd`" to navigate to the `turtlesim`
package), then open the `Spawn.srv` service definition, located in the
package's `/srv` subfolder. The file should look like

``` {frame="single"}
float32 x
float32 y
float32 theta
string name
---
string name
```

This definition tells us that the `/spawn` service takes four arguments
in its request: three decimal numbers giving the position and
orientation of the new turtle, and a single string specifying the new
turtle's name. The second portion of the definition tells us that the
service returns one data item: a string with the new name we specified
in the request.

Now let's call the `/spawn` service to create a new turtle, specifying
the values for each of the four arguments, in order:

``` {frame="single"}
rosservice call /spawn 2.0 2.0 1.2 "newturtle"
```

The service call returns the name of the newly created turtle, and you
should see the second turtle appear in the `turtlesim` window.

A Quick Introduction to Gazebo
==============================

In many of the upcoming labs, we will be using the Gazebo simulation
environment. To start Gazebo, make sure `roscore` is running and run

``` {frame="single"}
rosrun gazebo_ros gazebo
```

The Gazebo GUI should then pop up. To verify it is properly publishing
to ROS topics, open another terminal window and run

``` {frame="single"}
rostopic list
```

You should see a list like this:

``` {frame="single"}
/gazebo/link_states
/gazebo/model_states
/gazebo/parameter_descriptions
/gazebo/parameter_updates
/gazebo/set_link_state
/gazebo/set_model_state
```

------------------------------------------------------------------------

Checkpoint 2 {#checkpoint-2 .unnumbered}
------------

Submit a checkoff request at <https://tinyurl.com/106alabs20>. You
should be able to:

-   Explain what a *node*, *topic*, and *message* are

-   Drive your turtle around the screen using arrow keys

-   Use ROS's utility functions to view data on topics and messages

-   Show your Gazebo GUI and explain what you think the topics being
    published are for.

------------------------------------------------------------------------

[^1]: Developed by Amay Saxena and Tiffany Cappellari, Fall 2020 (the
    year of the plague).
