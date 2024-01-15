

 var \_gaq = \_gaq || [];
 \_gaq.push(['\_setAccount', 'UA-17821189-2']);
 \_gaq.push(['\_setDomainName', 'wiki.ros.org']);
 \_gaq.push(['\_trackPageview']);

 (function() {
 var ga = document.createElement('script'); ga.type = 'text/javascript'; ga.async = true;
 ga.src = ('https:' == document.location.protocol ? 'https://ssl' : 'http://www') + '.google-analytics.com/ga.js';
 var s = document.getElementsByTagName('script')[0]; s.parentNode.insertBefore(ga, s);
 })();

ROS/Tutorials/CreatingPackage - ROS Wiki

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
* [CreatingPackage](/ROS/Tutorials/CreatingPackage "/ROS/Tutorials/CreatingPackage")

#### ROS 2 Documentation

The ROS Wiki is for ROS 1. Are you using ROS 2 ([Humble](http://docs.ros.org/en/humble/ "http://docs.ros.org/en/humble/"), [Iron](http://docs.ros.org/en/iron/ "http://docs.ros.org/en/iron/"), or [Rolling](http://docs.ros.org/en/rolling/ "http://docs.ros.org/en/rolling/"))?   
[Check out the ROS 2 Project Documentation](http://docs.ros.org "http://docs.ros.org")  
Package specific documentation can be found on [index.ros.org](https://index.ros.org "https://index.ros.org")

# Wiki

* [Distributions](/Distributions "/Distributions")
* [ROS/Installation](/ROS/Installation "/ROS/Installation")
* [ROS/Tutorials](/ROS/Tutorials "/ROS/Tutorials")
* [RecentChanges](/RecentChanges "/RecentChanges")
* [CreatingPackage](/ROS/Tutorials/CreatingPackage "/ROS/Tutorials/CreatingPackage")

# Page

* Immutable Page
* [Comments](# "#")
* [Info](/action/info/ROS/Tutorials/CreatingPackage?action=info "/action/info/ROS/Tutorials/CreatingPackage?action=info")
* [Attachments](/action/AttachFile/ROS/Tutorials/CreatingPackage?action=AttachFile "/action/AttachFile/ROS/Tutorials/CreatingPackage?action=AttachFile")
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

* [Login](/action/login/ROS/Tutorials/CreatingPackage?action=login "/action/login/ROS/Tutorials/CreatingPackage?action=login")

|  |
| --- |
| **Note:** This tutorial assumes that you have completed the previous tutorials: [navigating the ROS filesystem](/ROS/Tutorials/NavigatingTheFilesystem "/ROS/Tutorials/NavigatingTheFilesystem").  |

|  |
| --- |
| (!) Please ask about problems and questions regarding this tutorial on [answers.ros.org](http://answers.ros.org "http://answers.ros.org"). Don't forget to include in your question the link to this page, the versions of your OS & ROS, and also add appropriate tags. |

# Creating a ROS Package

**Description:** This tutorial covers using [roscreate-pkg](/roscreate "/roscreate") or [catkin](/catkin "/catkin") to create a new package, and [rospack](/rospack "/rospack") to list package dependencies.  

**Tutorial Level:** BEGINNER  

**Next Tutorial:** [Building a ROS package](/ROS/Tutorials/BuildingPackages "/ROS/Tutorials/BuildingPackages")  

<!--
// @@ Buildsystem macro
function Buildsystem(sections) {
 var dotversion = ".buildsystem."

 // Tag shows unless already tagged
 $.each(sections.show,
 function() {
 $("div" + dotversion + this).not(".versionshow,.versionhide").addClass("versionshow")
 }
 )

 // Tag hides unless already tagged
 $.each(sections.hide,
 function() {
 $("div" + dotversion + this).not(".versionshow,.versionhide").addClass("versionhide")
 }
 )

 // Show or hide according to tag
 $(".versionshow").removeClass("versionshow").filter("div").show()
 $(".versionhide").removeClass("versionhide").filter("div").hide()
}

function getURLParameter(name) {
 return decodeURIComponent(
 (
 new RegExp(
 '[?|&]' + name + '=' + '([^&;]+?)(&|#|;|$)'
 ).exec(location.search) || [,""]
 )[1].replace(/\+/g, '%20')
 ) || null;
}

$(document).ready(function() {
 var activesystem = "catkin";
 var url\_distro = getURLParameter('buildsystem');
 if (url\_distro)
 {
 activesystem = url\_distro;
 }
 $("div.buildsystem").not("."+activesystem).hide();
 $("#"+activesystem).click();
 $("input.version:hidden").each(function() {
 var bg = $(this).attr("value").split(":");
 $("div.version." + bg[0]).css("background-color", bg[1]).removeClass(bg[0])
 });
})
 // -->

 catkin 
 rosbuild 

Contents1. [Using roscreate](#ROS.2FTutorials.2Frosbuild.2FCreatingPackage.Using_roscreate "#ROS.2FTutorials.2Frosbuild.2FCreatingPackage.Using_roscreate")
2. [Creating a New ROS Package](#ROS.2FTutorials.2Frosbuild.2FCreatingPackage.Creating_a_New_ROS_Package "#ROS.2FTutorials.2Frosbuild.2FCreatingPackage.Creating_a_New_ROS_Package")
3. [First-order package dependencies](#ROS.2FTutorials.2Frosbuild.2FCreatingPackage.First-order_package_dependencies "#ROS.2FTutorials.2Frosbuild.2FCreatingPackage.First-order_package_dependencies")
4. [Indirect package dependencies](#ROS.2FTutorials.2Frosbuild.2FCreatingPackage.Indirect_package_dependencies "#ROS.2FTutorials.2Frosbuild.2FCreatingPackage.Indirect_package_dependencies")
5. [ROS Client Libraries](#ROS.2FTutorials.2Frosbuild.2FCreatingPackage.ROS_Client_Libraries "#ROS.2FTutorials.2Frosbuild.2FCreatingPackage.ROS_Client_Libraries")
6. [Review](#ROS.2FTutorials.2Frosbuild.2FCreatingPackage.Review "#ROS.2FTutorials.2Frosbuild.2FCreatingPackage.Review")

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

Contents1. [What makes up a catkin Package?](#ROS.2FTutorials.2Fcatkin.2FCreatingPackage.What_makes_up_a_catkin_Package.3F "#ROS.2FTutorials.2Fcatkin.2FCreatingPackage.What_makes_up_a_catkin_Package.3F")
2. [Packages in a catkin Workspace](#ROS.2FTutorials.2Fcatkin.2FCreatingPackage.Packages_in_a_catkin_Workspace "#ROS.2FTutorials.2Fcatkin.2FCreatingPackage.Packages_in_a_catkin_Workspace")
3. [Creating a catkin Package](#ROS.2FTutorials.2Fcatkin.2FCreatingPackage.Creating_a_catkin_Package "#ROS.2FTutorials.2Fcatkin.2FCreatingPackage.Creating_a_catkin_Package")
4. [Building a catkin workspace and sourcing the setup file](#ROS.2FTutorials.2Fcatkin.2FCreatingPackage.Building_a_catkin_workspace_and_sourcing_the_setup_file "#ROS.2FTutorials.2Fcatkin.2FCreatingPackage.Building_a_catkin_workspace_and_sourcing_the_setup_file")
5. [package dependencies](#ROS.2FTutorials.2Fcatkin.2FCreatingPackage.package_dependencies "#ROS.2FTutorials.2Fcatkin.2FCreatingPackage.package_dependencies")
	1. [First-order dependencies](#ROS.2FTutorials.2Fcatkin.2FCreatingPackage.First-order_dependencies "#ROS.2FTutorials.2Fcatkin.2FCreatingPackage.First-order_dependencies")
	2. [Indirect dependencies](#ROS.2FTutorials.2Fcatkin.2FCreatingPackage.Indirect_dependencies "#ROS.2FTutorials.2Fcatkin.2FCreatingPackage.Indirect_dependencies")
6. [Customizing Your Package](#ROS.2FTutorials.2Fcatkin.2FCreatingPackage.Customizing_Your_Package "#ROS.2FTutorials.2Fcatkin.2FCreatingPackage.Customizing_Your_Package")
	1. [Customizing the package.xml](#ROS.2FTutorials.2Fcatkin.2FCreatingPackage.Customizing_the_package.xml "#ROS.2FTutorials.2Fcatkin.2FCreatingPackage.Customizing_the_package.xml")
		1. [description tag](#ROS.2FTutorials.2Fcatkin.2FCreatingPackage.description_tag "#ROS.2FTutorials.2Fcatkin.2FCreatingPackage.description_tag")
		2. [maintainer tags](#ROS.2FTutorials.2Fcatkin.2FCreatingPackage.maintainer_tags "#ROS.2FTutorials.2Fcatkin.2FCreatingPackage.maintainer_tags")
		3. [license tags](#ROS.2FTutorials.2Fcatkin.2FCreatingPackage.license_tags "#ROS.2FTutorials.2Fcatkin.2FCreatingPackage.license_tags")
		4. [dependencies tags](#ROS.2FTutorials.2Fcatkin.2FCreatingPackage.dependencies_tags "#ROS.2FTutorials.2Fcatkin.2FCreatingPackage.dependencies_tags")
		5. [Final package.xml](#ROS.2FTutorials.2Fcatkin.2FCreatingPackage.Final_package.xml "#ROS.2FTutorials.2Fcatkin.2FCreatingPackage.Final_package.xml")
	2. [Customizing the CMakeLists.txt](#ROS.2FTutorials.2Fcatkin.2FCreatingPackage.Customizing_the_CMakeLists.txt "#ROS.2FTutorials.2Fcatkin.2FCreatingPackage.Customizing_the_CMakeLists.txt")

## What makes up a catkin Package?

For a package to be considered a catkin package it must meet a few requirements: * The package must contain a [catkin compliant package.xml](/catkin/package.xml "/catkin/package.xml") file. 
	+ That package.xml file provides meta information about the package.
* The package must contain a [CMakeLists.txt which uses catkin](/catkin/CMakeLists.txt "/catkin/CMakeLists.txt"). 
	+ If it is a [catkin metapackage](/catkin/package.xml#Metapackages "/catkin/package.xml#Metapackages") it must have the relevant boilerplate CMakeLists.txt file.
* Each package must have its own folder 
	+ This means no nested packages nor multiple packages sharing the same directory.

The simplest possible package might have a structure which looks like this: * 
```
my_package/
  CMakeLists.txt
  package.xml
```

## Packages in a catkin Workspace

The recommended method of working with catkin packages is using a [catkin workspace](/catkin/workspaces "/catkin/workspaces"), but you can also build catkin packages standalone. A trivial workspace might look like this: * 
```
workspace_folder/        -- WORKSPACE
  src/                   -- SOURCE SPACE
    CMakeLists.txt       -- 'Toplevel' CMake file, provided by catkin
    package_1/
      CMakeLists.txt     -- CMakeLists.txt file for package_1
      package.xml        -- Package manifest for package_1
    ...
    package_n/
      CMakeLists.txt     -- CMakeLists.txt file for package_n
      package.xml        -- Package manifest for package_n
```

Before continuing with this tutorial create an empty catkin workspace by following the [Creating a workspace for catkin](/catkin/Tutorials/create_a_workspace "/catkin/Tutorials/create_a_workspace") tutorial. 
## Creating a catkin Package

This tutorial will demonstrate how to use the [catkin\_create\_pkg](/catkin/commands/catkin_create_pkg "/catkin/commands/catkin_create_pkg") script to create a new catkin package, and what you can do with it after it has been created. First change to the source space directory of the catkin workspace you created in the [Creating a Workspace for catkin tutorial](/catkin/Tutorials/create_a_workspace "/catkin/Tutorials/create_a_workspace"): 
```
# You should have created this in the Creating a Workspace Tutorial
$ cd ~/catkin_ws/src
```
Now use the catkin\_create\_pkg script to create a new package called 'beginner\_tutorials' which depends on std\_msgs, roscpp, and rospy: 
```
$ catkin_create_pkg beginner_tutorials std_msgs rospy roscpp
```
This will create a beginner\_tutorials folder which contains a [package.xml](/catkin/package.xml "/catkin/package.xml") and a [CMakeLists.txt](/catkin/CMakeLists.txt "/catkin/CMakeLists.txt"), which have been partially filled out with the information you gave catkin\_create\_pkg. catkin\_create\_pkg requires that you give it a package\_name and optionally a list of dependencies on which that package depends: 
```
# This is an example, do not try to run this
# catkin_create_pkg <package_name> [depend1] [depend2] [depend3]
```
catkin\_create\_pkg also has more advanced functionalities which are described in [catkin/commands/catkin\_create\_pkg](/catkin/commands/catkin_create_pkg "/catkin/commands/catkin_create_pkg"). 
## Building a catkin workspace and sourcing the setup file

Now you need to build the packages in the catkin workspace: 
```
$ cd ~/catkin_ws
$ catkin_make
```
After the workspace has been built it has created a similar structure in the devel subfolder as you usually find under /opt/ros/$ROSDISTRO\_NAME. To add the workspace to your ROS environment you need to source the generated setup file: 
```
$ . ~/catkin_ws/devel/setup.bash
```

## package dependencies

### First-order dependencies

When using [catkin\_create\_pkg](/catkin/commands/catkin_create_pkg "/catkin/commands/catkin_create_pkg") earlier, a few package dependencies were provided. These **first-order** dependencies can now be reviewed with the rospack tool. 
```
$ rospack depends1 beginner_tutorials 
```
* 
```
roscpp
rospy
std_msgs
```

As you can see, rospack lists the same dependencies that were used as arguments when running catkin\_create\_pkg. These dependencies for a package are stored in the **package.xml** file: 
```
$ roscd beginner_tutorials
$ cat package.xml
```
* 
```
<package format="2">
...
  <buildtool_depend>catkin</buildtool_depend>
  <build_depend>roscpp</build_depend>
  <build_depend>rospy</build_depend>
  <build_depend>std_msgs</build_depend>
...
</package>
```

### Indirect dependencies

In many cases, a dependency will also have its own dependencies. For instance, rospy has other dependencies. 
```
$ rospack depends1 rospy
```
* 
```
genpy
roscpp
rosgraph
rosgraph_msgs
roslib
std_msgs
```

A package can have quite a few indirect dependencies. Luckily rospack can recursively determine all nested dependencies. 
```
$ rospack depends beginner_tutorials
cpp_common
rostime
roscpp_traits
roscpp_serialization
catkin
genmsg
genpy
message_runtime
gencpp
geneus
gennodejs
genlisp
message_generation
rosbuild
rosconsole
std_msgs
rosgraph_msgs
xmlrpcpp
roscpp
rosgraph
ros_environment
rospack
roslib
rospy
```

## Customizing Your Package

This part of the tutorial will look at each file generated by [catkin\_create\_pkg](/catkin/commands/catkin_create_pkg "/catkin/commands/catkin_create_pkg") and describe, line by line, each component of those files and how you can customize them for your package. 
### Customizing the package.xml

The generated [package.xml](/catkin/package.xml "/catkin/package.xml") should be in your new package. Now lets go through the new [package.xml](/catkin/package.xml "/catkin/package.xml") and touch up any elements that need your attention. 
#### description tag

First update the description tag: 

function isnumbered(obj) {
 return obj.childNodes.length && obj.firstChild.childNodes.length && obj.firstChild.firstChild.className == 'LineNumber';
}
function nformat(num,chrs,add) {
 var nlen = Math.max(0,chrs-(''+num).length), res = '';
 while (nlen>0) { res += ' '; nlen-- }
 return res+num+add;
}
function addnumber(did, nstart, nstep) {
 var c = document.getElementById(did), l = c.firstChild, n = 1;
 if (!isnumbered(c)) {
 if (typeof nstart == 'undefined') nstart = 1;
 if (typeof nstep == 'undefined') nstep = 1;
 var n = nstart;
 while (l != null) {
 if (l.tagName == 'SPAN') {
 var s = document.createElement('SPAN');
 var a = document.createElement('A');
 s.className = 'LineNumber';
 a.appendChild(document.createTextNode(nformat(n,4,'')));
 a.href = '#' + did + '\_' + n;
 s.appendChild(a);
 s.appendChild(document.createTextNode(' '));
 n += nstep;
 if (l.childNodes.length) {
 l.insertBefore(s, l.firstChild);
 }
 else {
 l.appendChild(s);
 }
 }
 l = l.nextSibling;
 }
 }
 return false;
}
function remnumber(did) {
 var c = document.getElementById(did), l = c.firstChild;
 if (isnumbered(c)) {
 while (l != null) {
 if (l.tagName == 'SPAN' && l.firstChild.className == 'LineNumber') l.removeChild(l.firstChild);
 l = l.nextSibling;
 }
 }
 return false;
}
function togglenumber(did, nstart, nstep) {
 var c = document.getElementById(did);
 if (isnumbered(c)) {
 remnumber(did);
 } else {
 addnumber(did,nstart,nstep);
 }
 return false;
}

document.write('<a href="#" onclick="return togglenumber(\'ROS.2FTutorials.2Fcatkin.2FCreatingPackage.CA-fb97e97d8b791b4f6b911c62d34fe5604a07675c\', 5, 1);" \
 class="codenumbers">Toggle line numbers<\/a>');

```
 [5](#ROS.2FTutorials.2Fcatkin.2FCreatingPackage.CA-fb97e97d8b791b4f6b911c62d34fe5604a07675c_5 "#ROS.2FTutorials.2Fcatkin.2FCreatingPackage.CA-fb97e97d8b791b4f6b911c62d34fe5604a07675c_5")  <description>The beginner\_tutorials package</description>

```
 Change the description to anything you like, but by convention the first sentence should be short while covering the scope of the package. If it is hard to describe the package in a single sentence then it might need to be broken up. 
#### maintainer tags

Next comes the maintainer tag: 

document.write('<a href="#" onclick="return togglenumber(\'ROS.2FTutorials.2Fcatkin.2FCreatingPackage.CA-2f2f1839d8bd274cd8f99fd961a225fe80fcc1d2\', 7, 1);" \
 class="codenumbers">Toggle line numbers<\/a>');

```
 [7](#ROS.2FTutorials.2Fcatkin.2FCreatingPackage.CA-2f2f1839d8bd274cd8f99fd961a225fe80fcc1d2_7 "#ROS.2FTutorials.2Fcatkin.2FCreatingPackage.CA-2f2f1839d8bd274cd8f99fd961a225fe80fcc1d2_7")  <!-- One maintainer tag required, multiple allowed, one person per tag --> 
 [8](#ROS.2FTutorials.2Fcatkin.2FCreatingPackage.CA-2f2f1839d8bd274cd8f99fd961a225fe80fcc1d2_8 "#ROS.2FTutorials.2Fcatkin.2FCreatingPackage.CA-2f2f1839d8bd274cd8f99fd961a225fe80fcc1d2_8")  <!-- Example: -->
 [9](#ROS.2FTutorials.2Fcatkin.2FCreatingPackage.CA-2f2f1839d8bd274cd8f99fd961a225fe80fcc1d2_9 "#ROS.2FTutorials.2Fcatkin.2FCreatingPackage.CA-2f2f1839d8bd274cd8f99fd961a225fe80fcc1d2_9")  <!-- <maintainer email="jane.doe@example.com">Jane Doe</maintainer> -->
 [10](#ROS.2FTutorials.2Fcatkin.2FCreatingPackage.CA-2f2f1839d8bd274cd8f99fd961a225fe80fcc1d2_10 "#ROS.2FTutorials.2Fcatkin.2FCreatingPackage.CA-2f2f1839d8bd274cd8f99fd961a225fe80fcc1d2_10")  <maintainer email="user@todo.todo">user</maintainer>

```
 This is a required and important tag for the [package.xml](/catkin/package.xml "/catkin/package.xml") because it lets others know who to contact about the package. At least one maintainer is required, but you can have many if you like. The name of the maintainer goes into the body of the tag, but there is also an email attribute that should be filled out: 

document.write('<a href="#" onclick="return togglenumber(\'ROS.2FTutorials.2Fcatkin.2FCreatingPackage.CA-30d7f8b2d6d329b74df4869bade8fdfa316bb17c\', 7, 1);" \
 class="codenumbers">Toggle line numbers<\/a>');

```
 [7](#ROS.2FTutorials.2Fcatkin.2FCreatingPackage.CA-30d7f8b2d6d329b74df4869bade8fdfa316bb17c_7 "#ROS.2FTutorials.2Fcatkin.2FCreatingPackage.CA-30d7f8b2d6d329b74df4869bade8fdfa316bb17c_7")  <maintainer email="you@yourdomain.tld">Your Name</maintainer>

```

#### license tags

Next is the license tag, which is also required: 

document.write('<a href="#" onclick="return togglenumber(\'ROS.2FTutorials.2Fcatkin.2FCreatingPackage.CA-0e6145b2e070238d1afa432d8b9a6a2e65d073cf\', 12, 1);" \
 class="codenumbers">Toggle line numbers<\/a>');

```
 [12](#ROS.2FTutorials.2Fcatkin.2FCreatingPackage.CA-0e6145b2e070238d1afa432d8b9a6a2e65d073cf_12 "#ROS.2FTutorials.2Fcatkin.2FCreatingPackage.CA-0e6145b2e070238d1afa432d8b9a6a2e65d073cf_12")  <!-- One license tag required, multiple allowed, one license per tag -->
 [13](#ROS.2FTutorials.2Fcatkin.2FCreatingPackage.CA-0e6145b2e070238d1afa432d8b9a6a2e65d073cf_13 "#ROS.2FTutorials.2Fcatkin.2FCreatingPackage.CA-0e6145b2e070238d1afa432d8b9a6a2e65d073cf_13")  <!-- Commonly used license strings: -->
 [14](#ROS.2FTutorials.2Fcatkin.2FCreatingPackage.CA-0e6145b2e070238d1afa432d8b9a6a2e65d073cf_14 "#ROS.2FTutorials.2Fcatkin.2FCreatingPackage.CA-0e6145b2e070238d1afa432d8b9a6a2e65d073cf_14")  <!-- BSD, MIT, Boost Software License, GPLv2, GPLv3, LGPLv2.1, LGPLv3 -->
 [15](#ROS.2FTutorials.2Fcatkin.2FCreatingPackage.CA-0e6145b2e070238d1afa432d8b9a6a2e65d073cf_15 "#ROS.2FTutorials.2Fcatkin.2FCreatingPackage.CA-0e6145b2e070238d1afa432d8b9a6a2e65d073cf_15")  <license>TODO</license>

```
 You should choose a license and fill it in here. Some common open source licenses are BSD, MIT, Boost Software License, GPLv2, GPLv3, LGPLv2.1, and LGPLv3. You can read about several of these at the [Open Source Initiative](http://opensource.org/licenses/alphabetical "http://opensource.org/licenses/alphabetical"). For this tutorial we'll use the BSD license because the rest of the core ROS components use it already: 

document.write('<a href="#" onclick="return togglenumber(\'ROS.2FTutorials.2Fcatkin.2FCreatingPackage.CA-c2f485d5ba9efdc616470c2d71df9cf14a55e882\', 8, 1);" \
 class="codenumbers">Toggle line numbers<\/a>');

```
 [8](#ROS.2FTutorials.2Fcatkin.2FCreatingPackage.CA-c2f485d5ba9efdc616470c2d71df9cf14a55e882_8 "#ROS.2FTutorials.2Fcatkin.2FCreatingPackage.CA-c2f485d5ba9efdc616470c2d71df9cf14a55e882_8")  <license>BSD</license>

```

#### dependencies tags

The next set of tags describe the dependencies of your package. The dependencies are split into build\_depend, buildtool\_depend, exec\_depend, test\_depend. For a more detailed explanation of these tags see the documentation about [Catkin Dependencies](/catkin/package.xml#Build.2C_Run.2C_and_Test_Dependencies "/catkin/package.xml#Build.2C_Run.2C_and_Test_Dependencies"). Since we passed std\_msgs, roscpp, and rospy as arguments to [catkin\_create\_pkg](/catkin/commands/catkin_create_pkg "/catkin/commands/catkin_create_pkg"), the dependencies will look like this: 

document.write('<a href="#" onclick="return togglenumber(\'ROS.2FTutorials.2Fcatkin.2FCreatingPackage.CA-004f90df6bf9db2229e4f6943fb865f2dd8e8fc0\', 27, 1);" \
 class="codenumbers">Toggle line numbers<\/a>');

```
 [27](#ROS.2FTutorials.2Fcatkin.2FCreatingPackage.CA-004f90df6bf9db2229e4f6943fb865f2dd8e8fc0_27 "#ROS.2FTutorials.2Fcatkin.2FCreatingPackage.CA-004f90df6bf9db2229e4f6943fb865f2dd8e8fc0_27")  <!-- The \*\_depend tags are used to specify dependencies -->
 [28](#ROS.2FTutorials.2Fcatkin.2FCreatingPackage.CA-004f90df6bf9db2229e4f6943fb865f2dd8e8fc0_28 "#ROS.2FTutorials.2Fcatkin.2FCreatingPackage.CA-004f90df6bf9db2229e4f6943fb865f2dd8e8fc0_28")  <!-- Dependencies can be catkin packages or system dependencies -->
 [29](#ROS.2FTutorials.2Fcatkin.2FCreatingPackage.CA-004f90df6bf9db2229e4f6943fb865f2dd8e8fc0_29 "#ROS.2FTutorials.2Fcatkin.2FCreatingPackage.CA-004f90df6bf9db2229e4f6943fb865f2dd8e8fc0_29")  <!-- Examples: -->
 [30](#ROS.2FTutorials.2Fcatkin.2FCreatingPackage.CA-004f90df6bf9db2229e4f6943fb865f2dd8e8fc0_30 "#ROS.2FTutorials.2Fcatkin.2FCreatingPackage.CA-004f90df6bf9db2229e4f6943fb865f2dd8e8fc0_30")  <!-- Use build\_depend for packages you need at compile time: -->
 [31](#ROS.2FTutorials.2Fcatkin.2FCreatingPackage.CA-004f90df6bf9db2229e4f6943fb865f2dd8e8fc0_31 "#ROS.2FTutorials.2Fcatkin.2FCreatingPackage.CA-004f90df6bf9db2229e4f6943fb865f2dd8e8fc0_31")  <!-- <build\_depend>genmsg</build\_depend> -->
 [32](#ROS.2FTutorials.2Fcatkin.2FCreatingPackage.CA-004f90df6bf9db2229e4f6943fb865f2dd8e8fc0_32 "#ROS.2FTutorials.2Fcatkin.2FCreatingPackage.CA-004f90df6bf9db2229e4f6943fb865f2dd8e8fc0_32")  <!-- Use buildtool\_depend for build tool packages: -->
 [33](#ROS.2FTutorials.2Fcatkin.2FCreatingPackage.CA-004f90df6bf9db2229e4f6943fb865f2dd8e8fc0_33 "#ROS.2FTutorials.2Fcatkin.2FCreatingPackage.CA-004f90df6bf9db2229e4f6943fb865f2dd8e8fc0_33")  <!-- <buildtool\_depend>catkin</buildtool\_depend> -->
 [34](#ROS.2FTutorials.2Fcatkin.2FCreatingPackage.CA-004f90df6bf9db2229e4f6943fb865f2dd8e8fc0_34 "#ROS.2FTutorials.2Fcatkin.2FCreatingPackage.CA-004f90df6bf9db2229e4f6943fb865f2dd8e8fc0_34")  <!-- Use exec\_depend for packages you need at runtime: -->
 [35](#ROS.2FTutorials.2Fcatkin.2FCreatingPackage.CA-004f90df6bf9db2229e4f6943fb865f2dd8e8fc0_35 "#ROS.2FTutorials.2Fcatkin.2FCreatingPackage.CA-004f90df6bf9db2229e4f6943fb865f2dd8e8fc0_35")  <!-- <exec\_depend>python-yaml</exec\_depend> -->
 [36](#ROS.2FTutorials.2Fcatkin.2FCreatingPackage.CA-004f90df6bf9db2229e4f6943fb865f2dd8e8fc0_36 "#ROS.2FTutorials.2Fcatkin.2FCreatingPackage.CA-004f90df6bf9db2229e4f6943fb865f2dd8e8fc0_36")  <!-- Use test\_depend for packages you need only for testing: -->
 [37](#ROS.2FTutorials.2Fcatkin.2FCreatingPackage.CA-004f90df6bf9db2229e4f6943fb865f2dd8e8fc0_37 "#ROS.2FTutorials.2Fcatkin.2FCreatingPackage.CA-004f90df6bf9db2229e4f6943fb865f2dd8e8fc0_37")  <!-- <test\_depend>gtest</test\_depend> -->
 [38](#ROS.2FTutorials.2Fcatkin.2FCreatingPackage.CA-004f90df6bf9db2229e4f6943fb865f2dd8e8fc0_38 "#ROS.2FTutorials.2Fcatkin.2FCreatingPackage.CA-004f90df6bf9db2229e4f6943fb865f2dd8e8fc0_38")  <buildtool\_depend>catkin</buildtool\_depend>
 [39](#ROS.2FTutorials.2Fcatkin.2FCreatingPackage.CA-004f90df6bf9db2229e4f6943fb865f2dd8e8fc0_39 "#ROS.2FTutorials.2Fcatkin.2FCreatingPackage.CA-004f90df6bf9db2229e4f6943fb865f2dd8e8fc0_39")  <build\_depend>roscpp</build\_depend>
 [40](#ROS.2FTutorials.2Fcatkin.2FCreatingPackage.CA-004f90df6bf9db2229e4f6943fb865f2dd8e8fc0_40 "#ROS.2FTutorials.2Fcatkin.2FCreatingPackage.CA-004f90df6bf9db2229e4f6943fb865f2dd8e8fc0_40")  <build\_depend>rospy</build\_depend>
 [41](#ROS.2FTutorials.2Fcatkin.2FCreatingPackage.CA-004f90df6bf9db2229e4f6943fb865f2dd8e8fc0_41 "#ROS.2FTutorials.2Fcatkin.2FCreatingPackage.CA-004f90df6bf9db2229e4f6943fb865f2dd8e8fc0_41")  <build\_depend>std\_msgs</build\_depend>

```
 All of our listed dependencies have been added as a build\_depend for us, in addition to the default buildtool\_depend on catkin. In this case we want all of our specified dependencies to be available at build and run time, so we'll add a exec\_depend tag for each of them as well: 

document.write('<a href="#" onclick="return togglenumber(\'ROS.2FTutorials.2Fcatkin.2FCreatingPackage.CA-9ff0b8e163f24ad004671579a176a140ede469f6\', 12, 1);" \
 class="codenumbers">Toggle line numbers<\/a>');

```
 [12](#ROS.2FTutorials.2Fcatkin.2FCreatingPackage.CA-9ff0b8e163f24ad004671579a176a140ede469f6_12 "#ROS.2FTutorials.2Fcatkin.2FCreatingPackage.CA-9ff0b8e163f24ad004671579a176a140ede469f6_12")  <buildtool\_depend>catkin</buildtool\_depend>
 [13](#ROS.2FTutorials.2Fcatkin.2FCreatingPackage.CA-9ff0b8e163f24ad004671579a176a140ede469f6_13 "#ROS.2FTutorials.2Fcatkin.2FCreatingPackage.CA-9ff0b8e163f24ad004671579a176a140ede469f6_13") 
 [14](#ROS.2FTutorials.2Fcatkin.2FCreatingPackage.CA-9ff0b8e163f24ad004671579a176a140ede469f6_14 "#ROS.2FTutorials.2Fcatkin.2FCreatingPackage.CA-9ff0b8e163f24ad004671579a176a140ede469f6_14")  <build\_depend>roscpp</build\_depend>
 [15](#ROS.2FTutorials.2Fcatkin.2FCreatingPackage.CA-9ff0b8e163f24ad004671579a176a140ede469f6_15 "#ROS.2FTutorials.2Fcatkin.2FCreatingPackage.CA-9ff0b8e163f24ad004671579a176a140ede469f6_15")  <build\_depend>rospy</build\_depend>
 [16](#ROS.2FTutorials.2Fcatkin.2FCreatingPackage.CA-9ff0b8e163f24ad004671579a176a140ede469f6_16 "#ROS.2FTutorials.2Fcatkin.2FCreatingPackage.CA-9ff0b8e163f24ad004671579a176a140ede469f6_16")  <build\_depend>std\_msgs</build\_depend>
 [17](#ROS.2FTutorials.2Fcatkin.2FCreatingPackage.CA-9ff0b8e163f24ad004671579a176a140ede469f6_17 "#ROS.2FTutorials.2Fcatkin.2FCreatingPackage.CA-9ff0b8e163f24ad004671579a176a140ede469f6_17") 
 [18](#ROS.2FTutorials.2Fcatkin.2FCreatingPackage.CA-9ff0b8e163f24ad004671579a176a140ede469f6_18 "#ROS.2FTutorials.2Fcatkin.2FCreatingPackage.CA-9ff0b8e163f24ad004671579a176a140ede469f6_18")  <exec\_depend>roscpp</exec\_depend>
 [19](#ROS.2FTutorials.2Fcatkin.2FCreatingPackage.CA-9ff0b8e163f24ad004671579a176a140ede469f6_19 "#ROS.2FTutorials.2Fcatkin.2FCreatingPackage.CA-9ff0b8e163f24ad004671579a176a140ede469f6_19")  <exec\_depend>rospy</exec\_depend>
 [20](#ROS.2FTutorials.2Fcatkin.2FCreatingPackage.CA-9ff0b8e163f24ad004671579a176a140ede469f6_20 "#ROS.2FTutorials.2Fcatkin.2FCreatingPackage.CA-9ff0b8e163f24ad004671579a176a140ede469f6_20")  <exec\_depend>std\_msgs</exec\_depend>

```

#### Final package.xml

As you can see the final [package.xml](/catkin/package.xml "/catkin/package.xml"), without comments and unused tags, is much more concise: 

document.write('<a href="#" onclick="return togglenumber(\'ROS.2FTutorials.2Fcatkin.2FCreatingPackage.CA-503bfcd09daf7b64b4d684d92f02fa06898fc402\', 1, 1);" \
 class="codenumbers">Toggle line numbers<\/a>');

```
 [1](#ROS.2FTutorials.2Fcatkin.2FCreatingPackage.CA-503bfcd09daf7b64b4d684d92f02fa06898fc402_1 "#ROS.2FTutorials.2Fcatkin.2FCreatingPackage.CA-503bfcd09daf7b64b4d684d92f02fa06898fc402_1") <?xml version="1.0"?>
 [2](#ROS.2FTutorials.2Fcatkin.2FCreatingPackage.CA-503bfcd09daf7b64b4d684d92f02fa06898fc402_2 "#ROS.2FTutorials.2Fcatkin.2FCreatingPackage.CA-503bfcd09daf7b64b4d684d92f02fa06898fc402_2") <package format="2">
 [3](#ROS.2FTutorials.2Fcatkin.2FCreatingPackage.CA-503bfcd09daf7b64b4d684d92f02fa06898fc402_3 "#ROS.2FTutorials.2Fcatkin.2FCreatingPackage.CA-503bfcd09daf7b64b4d684d92f02fa06898fc402_3")  <name>beginner\_tutorials</name>
 [4](#ROS.2FTutorials.2Fcatkin.2FCreatingPackage.CA-503bfcd09daf7b64b4d684d92f02fa06898fc402_4 "#ROS.2FTutorials.2Fcatkin.2FCreatingPackage.CA-503bfcd09daf7b64b4d684d92f02fa06898fc402_4")  <version>0.1.0</version>
 [5](#ROS.2FTutorials.2Fcatkin.2FCreatingPackage.CA-503bfcd09daf7b64b4d684d92f02fa06898fc402_5 "#ROS.2FTutorials.2Fcatkin.2FCreatingPackage.CA-503bfcd09daf7b64b4d684d92f02fa06898fc402_5")  <description>The beginner\_tutorials package</description>
 [6](#ROS.2FTutorials.2Fcatkin.2FCreatingPackage.CA-503bfcd09daf7b64b4d684d92f02fa06898fc402_6 "#ROS.2FTutorials.2Fcatkin.2FCreatingPackage.CA-503bfcd09daf7b64b4d684d92f02fa06898fc402_6") 
 [7](#ROS.2FTutorials.2Fcatkin.2FCreatingPackage.CA-503bfcd09daf7b64b4d684d92f02fa06898fc402_7 "#ROS.2FTutorials.2Fcatkin.2FCreatingPackage.CA-503bfcd09daf7b64b4d684d92f02fa06898fc402_7")  <maintainer email="you@yourdomain.tld">Your Name</maintainer>
 [8](#ROS.2FTutorials.2Fcatkin.2FCreatingPackage.CA-503bfcd09daf7b64b4d684d92f02fa06898fc402_8 "#ROS.2FTutorials.2Fcatkin.2FCreatingPackage.CA-503bfcd09daf7b64b4d684d92f02fa06898fc402_8")  <license>BSD</license>
 [9](#ROS.2FTutorials.2Fcatkin.2FCreatingPackage.CA-503bfcd09daf7b64b4d684d92f02fa06898fc402_9 "#ROS.2FTutorials.2Fcatkin.2FCreatingPackage.CA-503bfcd09daf7b64b4d684d92f02fa06898fc402_9")  <url type="website">http://wiki.ros.org/beginner\_tutorials</url>
 [10](#ROS.2FTutorials.2Fcatkin.2FCreatingPackage.CA-503bfcd09daf7b64b4d684d92f02fa06898fc402_10 "#ROS.2FTutorials.2Fcatkin.2FCreatingPackage.CA-503bfcd09daf7b64b4d684d92f02fa06898fc402_10")  <author email="you@yourdomain.tld">Jane Doe</author>
 [11](#ROS.2FTutorials.2Fcatkin.2FCreatingPackage.CA-503bfcd09daf7b64b4d684d92f02fa06898fc402_11 "#ROS.2FTutorials.2Fcatkin.2FCreatingPackage.CA-503bfcd09daf7b64b4d684d92f02fa06898fc402_11") 
 [12](#ROS.2FTutorials.2Fcatkin.2FCreatingPackage.CA-503bfcd09daf7b64b4d684d92f02fa06898fc402_12 "#ROS.2FTutorials.2Fcatkin.2FCreatingPackage.CA-503bfcd09daf7b64b4d684d92f02fa06898fc402_12")  <buildtool\_depend>catkin</buildtool\_depend>
 [13](#ROS.2FTutorials.2Fcatkin.2FCreatingPackage.CA-503bfcd09daf7b64b4d684d92f02fa06898fc402_13 "#ROS.2FTutorials.2Fcatkin.2FCreatingPackage.CA-503bfcd09daf7b64b4d684d92f02fa06898fc402_13") 
 [14](#ROS.2FTutorials.2Fcatkin.2FCreatingPackage.CA-503bfcd09daf7b64b4d684d92f02fa06898fc402_14 "#ROS.2FTutorials.2Fcatkin.2FCreatingPackage.CA-503bfcd09daf7b64b4d684d92f02fa06898fc402_14")  <build\_depend>roscpp</build\_depend>
 [15](#ROS.2FTutorials.2Fcatkin.2FCreatingPackage.CA-503bfcd09daf7b64b4d684d92f02fa06898fc402_15 "#ROS.2FTutorials.2Fcatkin.2FCreatingPackage.CA-503bfcd09daf7b64b4d684d92f02fa06898fc402_15")  <build\_depend>rospy</build\_depend>
 [16](#ROS.2FTutorials.2Fcatkin.2FCreatingPackage.CA-503bfcd09daf7b64b4d684d92f02fa06898fc402_16 "#ROS.2FTutorials.2Fcatkin.2FCreatingPackage.CA-503bfcd09daf7b64b4d684d92f02fa06898fc402_16")  <build\_depend>std\_msgs</build\_depend>
 [17](#ROS.2FTutorials.2Fcatkin.2FCreatingPackage.CA-503bfcd09daf7b64b4d684d92f02fa06898fc402_17 "#ROS.2FTutorials.2Fcatkin.2FCreatingPackage.CA-503bfcd09daf7b64b4d684d92f02fa06898fc402_17") 
 [18](#ROS.2FTutorials.2Fcatkin.2FCreatingPackage.CA-503bfcd09daf7b64b4d684d92f02fa06898fc402_18 "#ROS.2FTutorials.2Fcatkin.2FCreatingPackage.CA-503bfcd09daf7b64b4d684d92f02fa06898fc402_18")  <exec\_depend>roscpp</exec\_depend>
 [19](#ROS.2FTutorials.2Fcatkin.2FCreatingPackage.CA-503bfcd09daf7b64b4d684d92f02fa06898fc402_19 "#ROS.2FTutorials.2Fcatkin.2FCreatingPackage.CA-503bfcd09daf7b64b4d684d92f02fa06898fc402_19")  <exec\_depend>rospy</exec\_depend>
 [20](#ROS.2FTutorials.2Fcatkin.2FCreatingPackage.CA-503bfcd09daf7b64b4d684d92f02fa06898fc402_20 "#ROS.2FTutorials.2Fcatkin.2FCreatingPackage.CA-503bfcd09daf7b64b4d684d92f02fa06898fc402_20")  <exec\_depend>std\_msgs</exec\_depend>
 [21](#ROS.2FTutorials.2Fcatkin.2FCreatingPackage.CA-503bfcd09daf7b64b4d684d92f02fa06898fc402_21 "#ROS.2FTutorials.2Fcatkin.2FCreatingPackage.CA-503bfcd09daf7b64b4d684d92f02fa06898fc402_21") 
 [22](#ROS.2FTutorials.2Fcatkin.2FCreatingPackage.CA-503bfcd09daf7b64b4d684d92f02fa06898fc402_22 "#ROS.2FTutorials.2Fcatkin.2FCreatingPackage.CA-503bfcd09daf7b64b4d684d92f02fa06898fc402_22") </package>

```

### Customizing the CMakeLists.txt

Now that the [package.xml](/catkin/package.xml "/catkin/package.xml"), which contains meta information, has been tailored to your package, you are ready to move on in the tutorials. The [CMakeLists.txt](/catkin/CMakeLists.txt "/catkin/CMakeLists.txt") file created by [catkin\_create\_pkg](/catkin/commands/catkin_create_pkg "/catkin/commands/catkin_create_pkg") will be covered in the later tutorials about building ROS code. 

Now that you've made a new ROS package, let's [build our ROS package](/ROS/Tutorials/BuildingPackages "/ROS/Tutorials/BuildingPackages"). 
## Video Demonstration

Watch the video below to have more explanation on Custom Workspace and Package Creation with step by step guide .  

Wiki: ROS/Tutorials/CreatingPackage (last edited 2022-10-18 15:57:46 by [Muhammad Luqman](/Muhammad%20Luqman "Muhammad Luqman @ 103.138.11.4[103.138.11.4]"))

Except where otherwise noted, the ROS wiki is licensed under the   

