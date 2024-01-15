

 var \_gaq = \_gaq || [];
 \_gaq.push(['\_setAccount', 'UA-17821189-2']);
 \_gaq.push(['\_setDomainName', 'wiki.ros.org']);
 \_gaq.push(['\_trackPageview']);

 (function() {
 var ga = document.createElement('script'); ga.type = 'text/javascript'; ga.async = true;
 ga.src = ('https:' == document.location.protocol ? 'https://ssl' : 'http://www') + '.google-analytics.com/ga.js';
 var s = document.getElementsByTagName('script')[0]; s.parentNode.insertBefore(ga, s);
 })();

ROS/Tutorials/WhereNext - ROS Wiki

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
* [WhereNext](/ROS/Tutorials/WhereNext "/ROS/Tutorials/WhereNext")

#### ROS 2 Documentation

The ROS Wiki is for ROS 1. Are you using ROS 2 ([Humble](http://docs.ros.org/en/humble/ "http://docs.ros.org/en/humble/"), [Iron](http://docs.ros.org/en/iron/ "http://docs.ros.org/en/iron/"), or [Rolling](http://docs.ros.org/en/rolling/ "http://docs.ros.org/en/rolling/"))?   
[Check out the ROS 2 Project Documentation](http://docs.ros.org "http://docs.ros.org")  
Package specific documentation can be found on [index.ros.org](https://index.ros.org "https://index.ros.org")

# Wiki

* [Distributions](/Distributions "/Distributions")
* [ROS/Installation](/ROS/Installation "/ROS/Installation")
* [ROS/Tutorials](/ROS/Tutorials "/ROS/Tutorials")
* [RecentChanges](/RecentChanges "/RecentChanges")
* [ROS/Tutorials/WhereNext](/ROS/Tutorials/WhereNext "/ROS/Tutorials/WhereNext")

# Page

* Immutable Page
* [Comments](# "#")
* [Info](/action/info/ROS/Tutorials/WhereNext?action=info "/action/info/ROS/Tutorials/WhereNext?action=info")
* [Attachments](/action/AttachFile/ROS/Tutorials/WhereNext?action=AttachFile "/action/AttachFile/ROS/Tutorials/WhereNext?action=AttachFile")
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

* [Login](/action/login/ROS/Tutorials/WhereNext?action=login "/action/login/ROS/Tutorials/WhereNext?action=login")

|  |
| --- |
| **Note:** This tutorial assumes that you have completed the previous tutorials: [Navigating the Wiki](/ROS/Tutorials/NavigatingTheWiki "/ROS/Tutorials/NavigatingTheWiki").  |

|  |
| --- |
| (!) Please ask about problems and questions regarding this tutorial on [answers.ros.org](http://answers.ros.org "http://answers.ros.org"). Don't forget to include in your question the link to this page, the versions of your OS & ROS, and also add appropriate tags. |

# Where Next?

**Description:** This tutorial discusses options for getting to know more about using ROS on real or simulated robots.  

**Tutorial Level:** BEGINNER  

 Contents1. [Launching a Simulator](#Launching_a_Simulator "#Launching_a_Simulator")
2. [Exploring RViz](#Exploring_RViz "#Exploring_RViz")
3. [Understanding TF](#Understanding_TF "#Understanding_TF")
4. [Going Deeper](#Going_Deeper "#Going_Deeper")

 At this point in the beginner's tutorials you should have an understanding of the core concepts of ROS. Given a robot that runs ROS, you could use this understanding to list topics published and subscribed by the robot, to identify the messages consumed by these topics and then write your own nodes that process sensor data and act in the world. The real attraction of ROS is not the publish/subscribe middle-ware itself but that ROS provides a standard mechanism for developers around the world to share their code. The best "feature" of ROS is its enormous community. The number of packages available can be overwhelming. This tutorial attempts to give you an idea of what to explore next. 
## Launching a Simulator

Even if you have a real robot, it is good to get started using a simulator so that if something goes wrong you don't injure yourself or damage an expensive robot. You can get started with the [PR2 Simulator](/pr2_simulator/Tutorials "/pr2_simulator/Tutorials") or the [Turtlebot Simulator](/turtlebot_simulator/Tutorials "/turtlebot_simulator/Tutorials"). Alternately, you might [search for your robot](/Robots "/Robots") and check whether it has a simulator of its own. At this point, you might try to control the simulated robot using a 'teleop' package (e.g., [turtlebot\_teleop](/turtlebot_teleop/Tutorials/Teleoperation "/turtlebot_teleop/Tutorials/Teleoperation")) or use your understanding of ROS to find a topic and write code that sends an appropriate message to drive your robot. 
## Exploring RViz

[RViz](/rviz "/rviz") is a powerful visualization tool that allows you to view the robot's sensors and internal state. The [user guide](/rviz/UserGuide "/rviz/UserGuide") will help you get started. 
## Understanding TF

The [TF](/tf "/tf") package transforms between different coordinate frames used by your robot and keeps track of these transforms over time. A good understanding of TF is essential when working with any real robot. It is worthwhile to work through the tutorials. If you're building your own robot, you might at this point consider constructing a [URDF model](/urdf/Tutorials "/urdf/Tutorials") for your robot. If you're using a "standard" robot then one has probably already being built for you. Nevertheless, it may be worthwhile to briefly familiarize yourself with the [URDF](/urdf "/urdf") package. 
## Going Deeper

At this point, you're probably ready to start getting your robot to perform more sophisticated tasks. The following pages may help you: 1. [actionlib](/actionlib "/actionlib") - The actionlib package provides a standardized interface for interfacing with preemptible tasks. This is widely used by "higher-level" packages in ROS.
2. [navigation](/navigation "/navigation") - 2D navigation: map-building and path planning.
3. [MoveIt](http://moveit.ros.org/ "http://moveit.ros.org/") - To control the arms of your robot.

Wiki: ROS/Tutorials/WhereNext (last edited 2016-09-08 00:59:36 by [DanielHeater](/DanielHeater "DanielHeater @ dynamic-75-76-201-28.knology.net[75.76.201.28]"))

Except where otherwise noted, the ROS wiki is licensed under the   

