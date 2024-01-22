

|  |
| --- |
| (!) Please ask about problems and questions regarding this tutorial on [answers.ros.org](http://answers.ros.org "http://answers.ros.org"). Don't forget to include in your question the link to this page, the versions of your OS & ROS, and also add appropriate tags. |

# Getting started with roswtf

**Description:** Basic introduction to the [roswtf](/roswtf "/roswtf") tool.  

**Keywords:** roswtf  

**Tutorial Level:** BEGINNER  

**Next Tutorial:** [Navigating the wiki](/ROS/Tutorials/NavigatingTheWiki "/ROS/Tutorials/NavigatingTheWiki")  

 Contents1. [Checking your installation](#Checking_your_installation "#Checking_your_installation")
2. [Trying it online](#Trying_it_online "#Trying_it_online")
3. [Errors](#Errors "#Errors")

 **Before you start this tutorial, please make sure your roscore is NOT running**. * On Linux, you can check if roscore is still running or not by something like this (if you see a line like this that includes rosmaster, which starts as part of roscore, roscore is running): 
```
$ ps -ef | grep -i rosmaster
00:00:00 /usr/bin/python /opt/ros/kinetic/bin/rosmaster 
```

## Checking your installation

[roswtf](/roswtf "/roswtf") examines your system to try and find problems. Let's try it out:
```
$ roscd rosmaster
$ roswtf
```
You should see (detail of the output varies): 
```
Package: rosmaster
================================================================================
Static checks summary:

No errors or warnings
================================================================================

ROS Master does not appear to be running.
Online graph checks will not be run.
ROS_MASTER_URI is [http://localhost:11311]
```
If your installation ran correctly, you should see an output similar to the above. The output is telling you: * Package: rosmaster: [roswtf](/roswtf "/roswtf") uses whatever your current directory is to determine what checks it does. This output is telling us that you started roswtf in the rosmaster package directory.
* Static checks summary: this is a report on any filesystem or any non-runtime (i.e. no roscore required to run) issues. It's telling us that there were no errors.
* ROS Master does not appear to be running.: the roscore isn't running. roswtf didn't do any online checks.

## Trying it online

For this next step, we want a [Master](/Master "/Master") to be up, so go ahead and start a roscore in another terminal. Now, try running the same sequence again: 
```
$ roscd
$ roswtf
```
You should see: 
```
No package or stack in context
======================================================
Static checks summary:

No errors or warnings
======================================================
Beginning tests of your ROS graph. These may take awhile...
analyzing graph...
... done analyzing graph
running graph rules...
... done running graph rules

Online checks summary:

Found 1 warning(s).
Warnings are things that may be just fine, but are sometimes at fault

WARNING The following node subscriptions are unconnected:
 * /rosout:
   * /rosout
```
roswtf did some online examination of your graph now that your roscore is running. Depending on how many ROS nodes you have running, this can take a long time to complete. As you can see, this time it produced a warning: 
```
WARNING The following node subscriptions are unconnected:
 * /rosout:
   * /rosout
```
This time, roscd ran without argument, which likely takes you to a directory there's no ROS package, so we see a message No package or stack in context. roswtf is warning you that the [rosout](/rosout "/rosout") node is subscribed to a topic that no one is publishing to. In this case, this is expected because nothing else is running, so we can ignore it. 
## Errors

roswtf will warn you about things that look suspicious but may be normal in your system. It can also report errors for problems that it knows are wrong. For this part, we are going to set your ROS\_PACKAGE\_PATH to a *bad* value. We're also going to stop our roscore to simplify the output that you see. 
```
$ roscd
$ ROS_PACKAGE_PATH=bad:$ROS_PACKAGE_PATH roswtf
```
This time we see: 
```
Stack: ros
======================================================
Static checks summary:

Found 1 error(s).

ERROR Not all paths in ROS_PACKAGE_PATH [bad] point to an existing directory: 
 * bad

======================================================

Cannot communicate with master, ignoring graph checks
```
As you can see, roswtf now gives us an error about the ROS\_PACKAGE\_PATH setting. There are many other types of problems that roswtf can find. If you find yourself stumped by a build or communication issue, try running it and seeing if it can point you in the right direction. Now that you know how to use roswtf, take sometime to learn more about how ros.org is structured and [navigating the wiki](/ROS/Tutorials/NavigatingTheWiki "/ROS/Tutorials/NavigatingTheWiki"). 

