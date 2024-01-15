

 var \_gaq = \_gaq || [];
 \_gaq.push(['\_setAccount', 'UA-17821189-2']);
 \_gaq.push(['\_setDomainName', 'wiki.ros.org']);
 \_gaq.push(['\_trackPageview']);

 (function() {
 var ga = document.createElement('script'); ga.type = 'text/javascript'; ga.async = true;
 ga.src = ('https:' == document.location.protocol ? 'https://ssl' : 'http://www') + '.google-analytics.com/ga.js';
 var s = document.getElementsByTagName('script')[0]; s.parentNode.insertBefore(ga, s);
 })();

ROS/Tutorials/UnderstandingTopics - ROS Wiki

<!--
var search\_hint = "Search";
//-->

 window.dataLayer = window.dataLayer || [];
 function gtag(){dataLayer.push(arguments);}
 gtag('js', new Date());

 gtag('config', 'G-EVD5Z6G6NH');

<!--// Initialize search form
var f = document.getElementById('searchform');
if(f) f.getElementsByTagName('label')[0].style.display = 'none';
var e = document.getElementById('searchinput');
if(e) {
 searchChange(e);
 searchBlur(e);
}

function handleSubmit() {
 var f = document.getElementById('searchform');
 var t = document.getElementById('searchinput');
 var r = document.getElementById('real\_searchinput');

 //alert("handleSubmit "+ t.value);
 if(t.value.match(/review/)) {
 r.value = t.value;
 } else {
 //r.value = t.value + " -PackageReviewCategory -StackReviewCategory -M3Review -DocReview -ApiReview -HelpOn -BadContent -LocalSpellingWords";
 r.value = t.value + " -PackageReviewCategory -StackReviewCategory -DocReview -ApiReview";
 }
 //return validate(f);
}
//-->

|  |  |
| --- | --- |
| [ros.org](/ "/") | [About](http://www.ros.org/about-ros "http://www.ros.org/about-ros")
 |
 [Support](/Support "/Support")
 |
 [Discussion Forum](http://discourse.ros.org/ "http://discourse.ros.org/")
 |
 [Index](http://index.ros.org/ "http://index.ros.org/")
 |
 [Service Status](http://status.ros.org/ "http://status.ros.org/")
 |
 [Q&A answers.ros.org](http://answers.ros.org/ "http://answers.ros.org/") |
| [Documentation](/ "/")[Browse Software](https://index.ros.org/packages "https://index.ros.org/packages")[News](https://discourse.ros.org/c/general "https://discourse.ros.org/c/general")[Download](/ROS/Installation "/ROS/Installation") |

* [ROS](/ROS "/ROS")
* [Tutorials](/ROS/Tutorials "/ROS/Tutorials")
* [UnderstandingTopics](/ROS/Tutorials/UnderstandingTopics "/ROS/Tutorials/UnderstandingTopics")

#### ROS 2 Documentation

The ROS Wiki is for ROS 1. Are you using ROS 2 ([Humble](http://docs.ros.org/en/humble/ "http://docs.ros.org/en/humble/"), [Iron](http://docs.ros.org/en/iron/ "http://docs.ros.org/en/iron/"), or [Rolling](http://docs.ros.org/en/rolling/ "http://docs.ros.org/en/rolling/"))?   
[Check out the ROS 2 Project Documentation](http://docs.ros.org "http://docs.ros.org")  
Package specific documentation can be found on [index.ros.org](https://index.ros.org "https://index.ros.org")

# Wiki

* [Distributions](/Distributions "/Distributions")
* [ROS/Installation](/ROS/Installation "/ROS/Installation")
* [ROS/Tutorials](/ROS/Tutorials "/ROS/Tutorials")
* [RecentChanges](/RecentChanges "/RecentChanges")
* [UnderstandingTopics](/ROS/Tutorials/UnderstandingTopics "/ROS/Tutorials/UnderstandingTopics")

# Page

* Immutable Page
* [Comments](# "#")
* [Info](/action/info/ROS/Tutorials/UnderstandingTopics?action=info "/action/info/ROS/Tutorials/UnderstandingTopics?action=info")
* [Attachments](/action/AttachFile/ROS/Tutorials/UnderstandingTopics?action=AttachFile "/action/AttachFile/ROS/Tutorials/UnderstandingTopics?action=AttachFile")
* More Actions:

Raw Text
Print View
Render as Docbook
Delete Cache
------------------------
Check Spelling
Like Pages
Local Site Map
------------------------
Rename Page
Copy Page
Delete Page
------------------------
My Pages
Subscribe User
------------------------
Remove Spam
Revert to this revision
Package Pages
Sync Pages
------------------------
CreatePdfDocument
Load
RawFile
Save
SlideShow

<!--// Init menu
actionsMenuInit('More Actions:');
//-->

# User

* [Login](/action/login/ROS/Tutorials/UnderstandingTopics?action=login "/action/login/ROS/Tutorials/UnderstandingTopics?action=login")

|  |
| --- |
| **Note:** This tutorial assumes that you have completed the previous tutorials: [understanding ROS nodes](/ROS/Tutorials/UnderstandingNodes "/ROS/Tutorials/UnderstandingNodes").  |

|  |
| --- |
| (!) Please ask about problems and questions regarding this tutorial on [answers.ros.org](http://answers.ros.org "http://answers.ros.org"). Don't forget to include in your question the link to this page, the versions of your OS & ROS, and also add appropriate tags. |

# Understanding ROS Topics

**Description:** This tutorial introduces ROS topics as well as using the [rostopic](/rostopic "/rostopic") and [rqt\_plot](/rqt_plot "/rqt_plot") commandline tools.  

**Tutorial Level:** BEGINNER  

**Next Tutorial:** [Understanding ROS services and parameters](/ROS/Tutorials/UnderstandingServicesParams "/ROS/Tutorials/UnderstandingServicesParams")  

 Contents1. [Setup](#Setup "#Setup")
	1. [roscore](#roscore "#roscore")
	2. [turtlesim](#turtlesim "#turtlesim")
	3. [turtle keyboard teleoperation](#turtle_keyboard_teleoperation "#turtle_keyboard_teleoperation")
2. [ROS Topics](#ROS_Topics "#ROS_Topics")
	1. [Using rqt\_graph](#Using_rqt_graph "#Using_rqt_graph")
	2. [Introducing rostopic](#Introducing_rostopic "#Introducing_rostopic")
	3. [Using rostopic echo](#Using_rostopic_echo "#Using_rostopic_echo")
	4. [Using rostopic list](#Using_rostopic_list "#Using_rostopic_list")
3. [ROS Messages](#ROS_Messages "#ROS_Messages")
	1. [Using rostopic type](#Using_rostopic_type "#Using_rostopic_type")
4. [rostopic continued](#rostopic_continued "#rostopic_continued")
	1. [Using rostopic pub](#Using_rostopic_pub "#Using_rostopic_pub")
	2. [Using rostopic hz](#Using_rostopic_hz "#Using_rostopic_hz")
5. [Using rqt\_plot](#Using_rqt_plot "#Using_rqt_plot")
6. [Video Tutorial](#Video_Tutorial "#Video_Tutorial")

## Setup

### roscore

Let's start by making sure that we have roscore running, **in a new terminal**: 
```
$ roscore
```
If you left roscore running from the last tutorial, you may get the error message: * 
```
roscore cannot run as another roscore/master is already running. 
Please kill other roscore/master processes before relaunching
```

This is fine. Only one roscore needs to be running. 
### turtlesim

For this tutorial we will also use turtlesim. Please run **in a new terminal**: 
```
$ rosrun turtlesim turtlesim_node
```

### turtle keyboard teleoperation

We'll also need something to drive the turtle around with. Please run **in a new terminal**: 
```
$ rosrun turtlesim turtle_teleop_key
```
* 
```
[ INFO] 1254264546.878445000: Started node [/teleop_turtle], pid [5528], bound on [aqy], xmlrpc port [43918], tcpros port [55936], logging to [~/ros/ros/log/teleop_turtle_5528.log], using [real] time
Reading from keyboard
---------------------------
Use arrow keys to move the turtle.
```

Now you can use the arrow keys of the keyboard to drive the turtle around. If you can not drive the turtle **select the terminal window of the turtle\_teleop\_key** to make sure that the keys that you type are recorded. * ![turtle_key.png](/ROS/Tutorials/UnderstandingTopics?action=AttachFile&do=get&target=turtle_key.png "turtle_key.png")

Now that you can drive your turtle around, let's look at what's going on behind the scenes. 
## ROS Topics

The turtlesim\_node and the turtle\_teleop\_key node are communicating with each other over a ROS **Topic**. turtle\_teleop\_key is **publishing** the key strokes on a topic, while turtlesim **subscribes** to the same topic to receive the key strokes. Let's use [rqt\_graph](/rqt_graph "/rqt_graph") which shows the nodes and topics currently running. Note: If you're using electric or earlier, rqt is not available. Use rxgraph instead. 
### Using rqt\_graph

rqt\_graph creates a dynamic graph of what's going on in the system. rqt\_graph is part of the rqt package. Unless you already have it installed, run: * 
```
$ sudo apt-get install ros-<distro>-rqt
$ sudo apt-get install ros-<distro>-rqt-common-plugins
```

replacing <distro> with the name of your [ROS distribution](/Distributions "/Distributions") (e.g. indigo, jade, kinetic, lunar ...) **In a new terminal**: 
```
$ rosrun rqt_graph rqt_graph
```
You will see something similar to: ![rqt_graph_turtle_key.png](/ROS/Tutorials/UnderstandingTopics?action=AttachFile&do=get&target=rqt_graph_turtle_key.png "rqt_graph_turtle_key.png") If you place your mouse over /turtle1/command\_velocity it will highlight the ROS nodes (here blue and green) and topics (here red). As you can see, the turtlesim\_node and the turtle\_teleop\_key nodes are communicating on the topic named /turtle1/command\_velocity. ![rqt_graph_turtle_key2.png](/ROS/Tutorials/UnderstandingTopics?action=AttachFile&do=get&target=rqt_graph_turtle_key2.png "rqt_graph_turtle_key2.png") 
### Introducing rostopic

The rostopic tool allows you to get information about ROS **topics**. You can use the help option to get the available sub-commands for rostopic 
```
$ rostopic -h
```
* 
```
rostopic bw     display bandwidth used by topic
rostopic echo   print messages to screen
rostopic hz     display publishing rate of topic    
rostopic list   print information about active topics
rostopic pub    publish data to topic
rostopic type   print topic type
```

Or pressing tab key after rostopic prints the possible sub-commands: 
```
$ rostopic 
bw    echo  find  hz    info  list  pub   type 
```
Let's use some of these topic sub-commands to examine turtlesim. 
### Using rostopic echo

rostopic echo shows the data published on a topic. Usage: 
```
rostopic echo [topic]
```
Let's look at the command velocity data published by the turtle\_teleop\_key node. *For ROS Hydro and later,* this data is published on the /turtle1/cmd\_vel topic. **In a new terminal, run:** 
```
$ rostopic echo /turtle1/cmd_vel
```
*For ROS Groovy and earlier,* this data is published on the /turtle1/command\_velocity topic. **In a new terminal, run:** 
```
$ rostopic echo /turtle1/command_velocity
```
You probably won't see anything happen because no data is being published on the topic. Let's make turtle\_teleop\_key publish data by pressing the arrow keys. **Remember if the turtle isn't moving you need to select the turtle\_teleop\_key terminal again.** *For ROS Hydro and later,* you should now see the following when you press the up key: 
```
linear: 
  x: 2.0
  y: 0.0
  z: 0.0
angular: 
  x: 0.0
  y: 0.0
  z: 0.0
---
linear: 
  x: 2.0
  y: 0.0
  z: 0.0
angular: 
  x: 0.0
  y: 0.0
  z: 0.0
---
```
*For ROS Groovy and earlier,* you should now see the following when you press the up key: 
```
---
linear: 2.0
angular: 0.0
---
linear: 2.0
angular: 0.0
---
linear: 2.0
angular: 0.0
---
linear: 2.0
angular: 0.0
---
linear: 2.0
angular: 0.0
```
Now let's look at rqt\_graph again. Press the refresh button in the upper-left to show the new node. As you can see rostopic echo, shown here in red, is now also **subscribed** to the turtle1/command\_velocity topic. ![rqt_graph_echo.png](/ROS/Tutorials/UnderstandingTopics?action=AttachFile&do=get&target=rqt_graph_echo.png "rqt_graph_echo.png") 
### Using rostopic list

rostopic list returns a list of all topics currently subscribed to and published. Let's figure out what argument the list sub-command needs. In a **new terminal** run: 
```
$ rostopic list -h
```
* 
```
Usage: rostopic list [/topic]

Options:
  -h, --help            show this help message and exit
  -b BAGFILE, --bag=BAGFILE
                        list topics in .bag file
  -v, --verbose         list full details about each topic
  -p                    list only publishers
  -s                    list only subscribers
```

For rostopic list use the **verbose** option: 
```
$ rostopic list -v
```
This displays a verbose list of topics to publish to and subscribe to and their type. *For ROS Hydro and later,* * 
```
Published topics:
 * /turtle1/color_sensor [turtlesim/Color] 1 publisher
 * /turtle1/cmd_vel [geometry_msgs/Twist] 1 publisher
 * /rosout [rosgraph_msgs/Log] 2 publishers
 * /rosout_agg [rosgraph_msgs/Log] 1 publisher
 * /turtle1/pose [turtlesim/Pose] 1 publisher

Subscribed topics:
 * /turtle1/cmd_vel [geometry_msgs/Twist] 1 subscriber
 * /rosout [rosgraph_msgs/Log] 1 subscriber
```

*For ROS Groovy and earlier,* * 
```
Published topics:
 * /turtle1/color_sensor [turtlesim/Color] 1 publisher
 * /turtle1/command_velocity [turtlesim/Velocity] 1 publisher
 * /rosout [roslib/Log] 2 publishers
 * /rosout_agg [roslib/Log] 1 publisher
 * /turtle1/pose [turtlesim/Pose] 1 publisher

Subscribed topics:
 * /turtle1/command_velocity [turtlesim/Velocity] 1 subscriber
 * /rosout [roslib/Log] 1 subscriber
```

## ROS Messages

Communication on topics happens by sending ROS **messages** between nodes. For the publisher (turtle\_teleop\_key) and subscriber (turtlesim\_node) to communicate, the publisher and subscriber must send and receive the same **type** of message. This means that a topic **type** is defined by the message **type** published on it. The **type** of the message sent on a topic can be determined using rostopic type. 
### Using rostopic type

rostopic type returns the message type of any topic being published. Usage: 
```
rostopic type [topic]
```
*For ROS Hydro and later,* * Try: 
```
$ rostopic type /turtle1/cmd_vel
```

	+ You should get: 

```
	geometry_msgs/Twist

```We can look at the details of the message using rosmsg: 
```
$ rosmsg show geometry_msgs/Twist
```

	+ 
```
	geometry_msgs/Vector3 linear
	  float64 x
	  float64 y
	  float64 z
	geometry_msgs/Vector3 angular
	  float64 x
	  float64 y
	  float64 z

```

*For ROS Groovy and earlier,* * Try: 
```
$ rostopic type /turtle1/command_velocity
```

	+ You should get: 

```
	turtlesim/Velocity

```We can look at the details of the message using rosmsg: 
```
$ rosmsg show turtlesim/Velocity
```

	+ 
```
	float32 linear
	float32 angular

```

Now that we know what type of message turtlesim expects, we can publish commands to our turtle. 
## rostopic continued

Now that we have learned about ROS **messages**, let's use rostopic with messages. 
### Using rostopic pub

rostopic pub publishes data on to a topic currently advertised. Usage: 
```
rostopic pub [topic] [msg_type] [args]
```
*For ROS Hydro and later,* example: 
```
$ rostopic pub -1 /turtle1/cmd_vel geometry_msgs/Twist -- '[2.0, 0.0, 0.0]' '[0.0, 0.0, 1.8]'
```
*For ROS Groovy and earlier,* example: 
```
$ rostopic pub -1 /turtle1/command_velocity turtlesim/Velocity  -- 2.0  1.8
```
The previous command will send a single message to turtlesim telling it to move with a linear velocity of 2.0, and an angular velocity of 1.8 . * ![turtle(rostopicpub).png](/ROS/Tutorials/UnderstandingTopics?action=AttachFile&do=get&target=turtle%28rostopicpub%29.png "turtle(rostopicpub).png")

This is a pretty complicated example, so lets look at each argument in detail. *For ROS Hydro and later,* * This command will publish messages to a given topic: 
```
rostopic pub
```
* This option (dash-one) causes rostopic to only publish one message then exit: 
```
 -1 
```
* This is the name of the topic to publish to: 
```
/turtle1/cmd_vel
```
* This is the message type to use when publishing to the topic: 
```
geometry_msgs/Twist
```
* This option (double-dash) tells the option parser that none of the following arguments is an option. This is required in cases where your arguments have a leading dash -, like negative numbers.
```
--
```
* As noted before, a geometry\_msgs/Twist msg has two vectors of three floating point elements each: linear and angular. In this case, '[2.0, 0.0, 0.0]' becomes the linear value with x=2.0, y=0.0, and z=0.0, and '[0.0, 0.0, 1.8]' is the angular value with x=0.0, y=0.0, and z=1.8. These arguments are actually in YAML syntax, which is described more in the [YAML command line documentation](/ROS/YAMLCommandLine "/ROS/YAMLCommandLine"). 
```
'[2.0, 0.0, 0.0]' '[0.0, 0.0, 1.8]' 
```

*For ROS Groovy and earlier,* * This command will publish messages to a given topic: 
```
rostopic pub
```
* This option (dash-one) causes rostopic to only publish one message then exit: 
```
 -1 
```
* This is the name of the topic to publish to: 
```
/turtle1/command_velocity
```
* This is the message type to use when publishing to the topic: 
```
turtlesim/Velocity
```
* This option (double-dash) tells the option parser that none of the following arguments is an option. This is required in cases where your arguments have a leading dash -, like negative numbers.
```
--
```
* As noted before, a turtlesim/Velocity msg has two floating point elements : linear and angular. In this case, 2.0 becomes the linear value, and 1.8 is the angular value. These arguments are actually in YAML syntax, which is described more in the [YAML command line documentation](/ROS/YAMLCommandLine "/ROS/YAMLCommandLine"). 
```
2.0 1.8 
```

You may have noticed that the turtle has stopped moving; this is because the turtle requires a steady stream of commands at 1 Hz to keep moving. We can publish a steady stream of commands using rostopic pub -r command: *For ROS Hydro and later,* * 
```
$ rostopic pub /turtle1/cmd_vel geometry_msgs/Twist -r 1 -- '[2.0, 0.0, 0.0]' '[0.0, 0.0, -1.8]'
```

*For ROS Groovy and earlier,* * 
```
$ rostopic pub /turtle1/command_velocity turtlesim/Velocity -r 1 -- 2.0  -1.8
```

This publishes the velocity commands at a rate of 1 Hz on the velocity topic. * ![turtle(rostopicpub)2.png](/ROS/Tutorials/UnderstandingTopics?action=AttachFile&do=get&target=turtle%28rostopicpub%292.png "turtle(rostopicpub)2.png")

We can also look at what is happening in rqt\_graph. Press the refresh button in the upper-left. The rostopic pub node (here in red) is communicating with the rostopic echo node (here in green): ![rqt_graph_pub.png](/ROS/Tutorials/UnderstandingTopics?action=AttachFile&do=get&target=rqt_graph_pub.png "rqt_graph_pub.png") As you can see the turtle is running in a continuous circle. In a **new terminal**, we can use rostopic echo to see the data published by our turtlesim: 
```
rostopic echo /turtle1/pose
```

### Using rostopic hz

rostopic hz reports the rate at which data is published. Usage: 
```
rostopic hz [topic]
```
Let's see how fast the turtlesim\_node is publishing /turtle1/pose: 
```
$ rostopic hz /turtle1/pose
```
You will see: * 
```
subscribed to [/turtle1/pose]
average rate: 59.354
        min: 0.005s max: 0.027s std dev: 0.00284s window: 58
average rate: 59.459
        min: 0.005s max: 0.027s std dev: 0.00271s window: 118
average rate: 59.539
        min: 0.004s max: 0.030s std dev: 0.00339s window: 177
average rate: 59.492
        min: 0.004s max: 0.030s std dev: 0.00380s window: 237
average rate: 59.463
        min: 0.004s max: 0.030s std dev: 0.00380s window: 290
```

Now we can tell that the turtlesim is publishing data about our turtle at the rate of 60 Hz. We can also use rostopic type in conjunction with rosmsg show to get in depth information about a topic: *For ROS Hydro and later,* * 
```
$ rostopic type /turtle1/cmd_vel | rosmsg show
```

*For ROS Groovy and earlier,* * 
```
$ rostopic type /turtle1/command_velocity | rosmsg show
```

Now that we've examined the topics using rostopic let's use another tool to look at the data published by our turtlesim: 
## Using rqt\_plot

Note: If you're using electric or earlier, rqt is not available. Use rxplot instead. rqt\_plot displays a scrolling time plot of the data published on topics. Here we'll use rqt\_plot to plot the data being published on the /turtle1/pose topic. First, start rqt\_plot by typing 
```
$ rosrun rqt_plot rqt_plot
```
in a new terminal. In the new window that should pop up, a text box in the upper left corner gives you the ability to add any topic to the plot. Typing /turtle1/pose/x will highlight the plus button, previously disabled. Press it and repeat the same procedure with the topic /turtle1/pose/y. You will now see the turtle's x-y location plotted in the graph. ![rqt_plot.png](/ROS/Tutorials/UnderstandingTopics?action=AttachFile&do=get&target=rqt_plot.png "rqt_plot.png") Pressing the minus button shows a menu that allows you to hide the specified topic from the plot. Hiding both the topics you just added and adding /turtle1/pose/theta will result in the plot shown in the next figure. ![rqt_plot2.png](/ROS/Tutorials/UnderstandingTopics?action=AttachFile&do=get&target=rqt_plot2.png "rqt_plot2.png") That's it for this section, use Ctrl-C to kill the rostopic terminals but keep your turtlesim running. Now that you understand how ROS topics work, let's look at how [services and parameters work](/ROS/Tutorials/UnderstandingServicesParams "/ROS/Tutorials/UnderstandingServicesParams"). 
## Video Tutorial

The following video presents a small tutorial using turtlesim on ROS nodes and ROS topics  

Wiki: ROS/Tutorials/UnderstandingTopics (last edited 2022-10-18 16:18:07 by [Muhammad Luqman](/Muhammad%20Luqman "Muhammad Luqman @ 103.138.11.4[103.138.11.4]"))

Except where otherwise noted, the ROS wiki is licensed under the   

