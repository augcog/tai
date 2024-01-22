

 var \_gaq = \_gaq || [];
 \_gaq.push(['\_setAccount', 'UA-17821189-2']);
 \_gaq.push(['\_setDomainName', 'wiki.ros.org']);
 \_gaq.push(['\_trackPageview']);

 (function() {
 var ga = document.createElement('script'); ga.type = 'text/javascript'; ga.async = true;
 ga.src = ('https:' == document.location.protocol ? 'https://ssl' : 'http://www') + '.google-analytics.com/ga.js';
 var s = document.getElementsByTagName('script')[0]; s.parentNode.insertBefore(ga, s);
 })();

ROS/Tutorials/rosbuild/CreatingPackage - ROS Wiki

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
* [rosbuild](/ROS/Tutorials/rosbuild "/ROS/Tutorials/rosbuild")
* [CreatingPackage](/ROS/Tutorials/rosbuild/CreatingPackage "/ROS/Tutorials/rosbuild/CreatingPackage")

#### ROS 2 Documentation

The ROS Wiki is for ROS 1. Are you using ROS 2 ([Humble](http://docs.ros.org/en/humble/ "http://docs.ros.org/en/humble/"), [Iron](http://docs.ros.org/en/iron/ "http://docs.ros.org/en/iron/"), or [Rolling](http://docs.ros.org/en/rolling/ "http://docs.ros.org/en/rolling/"))?   
[Check out the ROS 2 Project Documentation](http://docs.ros.org "http://docs.ros.org")  
Package specific documentation can be found on [index.ros.org](https://index.ros.org "https://index.ros.org")

# Wiki

* [Distributions](/Distributions "/Distributions")
* [ROS/Installation](/ROS/Installation "/ROS/Installation")
* [ROS/Tutorials](/ROS/Tutorials "/ROS/Tutorials")
* [RecentChanges](/RecentChanges "/RecentChanges")
* [CreatingPackage](/ROS/Tutorials/rosbuild/CreatingPackage "/ROS/Tutorials/rosbuild/CreatingPackage")

# Page

* Immutable Page
* [Comments](# "#")
* [Info](/action/info/ROS/Tutorials/rosbuild/CreatingPackage?action=info "/action/info/ROS/Tutorials/rosbuild/CreatingPackage?action=info")
* [Attachments](/action/AttachFile/ROS/Tutorials/rosbuild/CreatingPackage?action=AttachFile "/action/AttachFile/ROS/Tutorials/rosbuild/CreatingPackage?action=AttachFile")
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

* [Login](/action/login/ROS/Tutorials/rosbuild/CreatingPackage?action=login "/action/login/ROS/Tutorials/rosbuild/CreatingPackage?action=login")

Contents1. [Using roscreate](#Using_roscreate "#Using_roscreate")
2. [Creating a New ROS Package](#Creating_a_New_ROS_Package "#Creating_a_New_ROS_Package")
3. [First-order package dependencies](#First-order_package_dependencies "#First-order_package_dependencies")
4. [Indirect package dependencies](#Indirect_package_dependencies "#Indirect_package_dependencies")
5. [ROS Client Libraries](#ROS_Client_Libraries "#ROS_Client_Libraries")
6. [Review](#Review "#Review")

## Using roscreate

Before we create a package, let's see how the roscreate-pkg command-line tool works. This creates a new ROS [package](/Packages "/Packages"). All ROS packages consist of the many similar files : [manifests](/Manifest "/Manifest"), [CMakeLists.txt](/CMakeLists "/CMakeLists"), mainpage.dox, and Makefiles. roscreate-pkg eliminates many tedious tasks of creating a new package by hand, and eliminates common errors caused by hand-typing build files and manifests. To create a new package in the current directory: 
```
# roscreate-pkg [package_name]
```
You can also specify dependencies of that package: 
```
# roscreate-pkg [package_name] [depend1] [depend2] [depend3]
```

## Creating a New ROS Package

Now we're going to go into your home or project directory and create our beginner\_tutorials package. We are going to make it depend on [std\_msgs](/std_msgs "/std_msgs"), [roscpp](/roscpp "/roscpp"), and [rospy](/rospy "/rospy"), which are common ROS packages. Now go into the ~/fuerte\_workspace/sandbox directory: 
```
$ cd ~/fuerte_workspace/sandbox
```
Alternatively, if you use Fuerte or later release, you can simply do: 
```
$ roscd
$ cd sandbox
```
Then create your package: 
```
$ roscreate-pkg beginner_tutorials std_msgs rospy roscpp
```
You will see something similar to: * 
```
Creating package directory ~/fuerte_workspace/sandbox/beginner_tutorials
Creating include directory ~/fuerte_workspace/sandbox/beginner_tutorials/include/beginner_tutorials
Creating cpp source directory ~/ros/ros_tutorials/beginner_tutorials/src
Creating python source directory ~/fuerte_workspace/sandbox/beginner_tutorials/src/beginner_tutorials
Creating package file ~/fuerte_workspace/sandbox/beginner_tutorials/Makefile
Creating package file ~/fuerte_workspace/sandbox/beginner_tutorials/manifest.xml
Creating package file ~/fuerte_workspace/sandbox/beginner_tutorials/CMakeLists.txt
Creating package file ~/fuerte_workspace/sandbox/beginner_tutorials/mainpage.dox

Please edit beginner_tutorials/manifest.xml and mainpage.dox to finish creating your package
```

You're going to want to spend some time looking at beginner\_tutorials/manifest.xml. [manifests](/Manifest "/Manifest") play an important role in ROS as they define how Packages are built, run, and documented. Now lets make sure that ROS can find your new package. It is often useful to call *rospack profile* after making changes to your path so that new directories will be found: 
```
$ rospack profile
$ rospack find beginner_tutorials 
```
* 
```
YOUR_PACKAGE_PATH/beginner_tutorials
```

If this fails, it means ROS can't find your new package, which may be an issue with your ROS\_PACKAGE\_PATH. Please consult the installation instructions for setup from SVN or from binaries, depending how you installed ROS. If you've created or added a package that's outside of the existing package paths, you will need to amend your [ROS\_PACKAGE\_PATH](/ROS/EnvironmentVariables#ROS_PACKAGE_PATH "/ROS/EnvironmentVariables#ROS_PACKAGE_PATH") environment variable to include that new location. Try re-sourcing your setup.sh in your fuerte\_workspace. Try moving to the directory for the package. 
```
$ roscd beginner_tutorials 
$ pwd
```
* 
```
YOUR_PACKAGE_PATH/beginner_tutorials
```

## First-order package dependencies

When using roscreate-pkg earlier, a few package dependencies were provided. These **first-order** dependencies can now be reviewed with the rospack tool. (Jan 9, 2013) There is [a bug](https://github.com/ros/rospack/issues/4 "https://github.com/ros/rospack/issues/4") reported and already fixed in [rospack](/rospack "/rospack") in groovy; it may take some time to be reflected in the packages. If you see [an issue similar to this](http://answers.ros.org/question/51555/beginner-tutorials-segmentation-fault-with-rospack-depends1/?comment=51762#comment-51762 "http://answers.ros.org/question/51555/beginner-tutorials-segmentation-fault-with-rospack-depends1/?comment=51762#comment-51762") with the next command, you can skip to the following command. 

```
$ rospack depends1 beginner_tutorials 
```
* 
```
std_msgs
rospy
roscpp
```

As you can see, rospack lists the same dependencies that were used as arguments when running roscreate-pkg. These dependencies for a package are stored in the **manifest** file. Take a look at the manifest file. 
```
$ roscd beginner_tutorials
$ cat manifest.xml
```
* 
```
<package>

...

  <depend package="std_msgs"/>
  <depend package="rospy"/>
  <depend package="roscpp"/>

</package>
```

## Indirect package dependencies

In many cases, a dependency will also have its own dependencies. For instance, rospy has other dependencies. (Jan 9, 2013) There is [a bug](https://github.com/ros/rospack/issues/4 "https://github.com/ros/rospack/issues/4") reported and already fixed in [rospack](/rospack "/rospack") in groovy; it may take some time to be reflected in the packages. If you see [an issue similar to this](http://answers.ros.org/question/51555/beginner-tutorials-segmentation-fault-with-rospack-depends1/?comment=51762#comment-51762 "http://answers.ros.org/question/51555/beginner-tutorials-segmentation-fault-with-rospack-depends1/?comment=51762#comment-51762") with the next command, you can skip to the following command. 

```
$ rospack depends1 rospy
```
* 
```
roslib
roslang
```

A package can have quite a few indirect dependencies. Luckily rospack can recursively determine all nested dependencies. 
```
$ rospack depends beginner_tutorials
```
* 
```
rospack
roslib
std_msgs
rosgraph_msgs
rosbuild
roslang
rospy
cpp_common
roscpp_traits
rostime
roscpp_serialization
xmlrpcpp
rosconsole
roscpp
```

Note: in Fuerte, the list is much shorter: * 
```
std_msgs
roslang
rospy
roscpp
```

## ROS Client Libraries

You may be wondering what rospy and roscpp dependencies are from the previous examples. rospy and roscpp are [Client Libraries](/Client%20Libraries "/Client%20Libraries"). The client libraries allow different programming languages to communicate through ROS. rospy is the client library for Python. roscpp is the client library for C++. 
## Review

Lets just list some of the commands we've used so far: * roscreate-pkg = ros+create-pkg : generates all the files needed to create a ROS package
* rospack = ros+pack(age) : provides information related to ROS packages
* rosstack = ros+stack : provides information related to ROS stacks

Wiki: ROS/Tutorials/rosbuild/CreatingPackage (last edited 2016-09-27 13:27:24 by [yoyekw](/yoyekw "yoyekw @ robot-NAT.elka.pw.edu.pl[194.29.160.190]"))

Except where otherwise noted, the ROS wiki is licensed under the   

