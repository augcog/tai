

 var \_gaq = \_gaq || [];
 \_gaq.push(['\_setAccount', 'UA-17821189-2']);
 \_gaq.push(['\_setDomainName', 'wiki.ros.org']);
 \_gaq.push(['\_trackPageview']);

 (function() {
 var ga = document.createElement('script'); ga.type = 'text/javascript'; ga.async = true;
 ga.src = ('https:' == document.location.protocol ? 'https://ssl' : 'http://www') + '.google-analytics.com/ga.js';
 var s = document.getElementsByTagName('script')[0]; s.parentNode.insertBefore(ga, s);
 })();

ROS/Tutorials/Creating a Package by Hand - ROS Wiki

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
* [Creating a Package by Hand](/action/fullsearch/ROS/Tutorials/Creating%20a%20Package%20by%20Hand?action=fullsearch&context=180&value=linkto%3A%22ROS%2FTutorials%2FCreating+a+Package+by+Hand%22 "Click to do a full-text search for this title")

#### ROS 2 Documentation

The ROS Wiki is for ROS 1. Are you using ROS 2 ([Humble](http://docs.ros.org/en/humble/ "http://docs.ros.org/en/humble/"), [Iron](http://docs.ros.org/en/iron/ "http://docs.ros.org/en/iron/"), or [Rolling](http://docs.ros.org/en/rolling/ "http://docs.ros.org/en/rolling/"))?   
[Check out the ROS 2 Project Documentation](http://docs.ros.org "http://docs.ros.org")  
Package specific documentation can be found on [index.ros.org](https://index.ros.org "https://index.ros.org")

# Wiki

* [Distributions](/Distributions "/Distributions")
* [ROS/Installation](/ROS/Installation "/ROS/Installation")
* [ROS/Tutorials](/ROS/Tutorials "/ROS/Tutorials")
* [RecentChanges](/RecentChanges "/RecentChanges")
* [Creating a ...age by Hand](/ROS/Tutorials/Creating%20a%20Package%20by%20Hand "/ROS/Tutorials/Creating%20a%20Package%20by%20Hand")

# Page

* Immutable Page
* [Comments](# "#")
* [Info](/action/info/ROS/Tutorials/Creating%20a%20Package%20by%20Hand?action=info "/action/info/ROS/Tutorials/Creating%20a%20Package%20by%20Hand?action=info")
* [Attachments](/action/AttachFile/ROS/Tutorials/Creating%20a%20Package%20by%20Hand?action=AttachFile "/action/AttachFile/ROS/Tutorials/Creating%20a%20Package%20by%20Hand?action=AttachFile")
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

* [Login](/action/login/ROS/Tutorials/Creating%20a%20Package%20by%20Hand?action=login "/action/login/ROS/Tutorials/Creating%20a%20Package%20by%20Hand?action=login")

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

|  |
| --- |
| (!) Please ask about problems and questions regarding this tutorial on [answers.ros.org](http://answers.ros.org "http://answers.ros.org"). Don't forget to include in your question the link to this page, the versions of your OS & ROS, and also add appropriate tags. |

# Creating a ROS package by hand.

**Description:** This tutorial explains how to manually create a ROS package.  

**Tutorial Level:** INTERMEDIATE   

**Next Tutorial:** [Managing System Dependencies](/ROS/Tutorials/rosdep "/ROS/Tutorials/rosdep")   

 There is a tool for creating ROS [Packages](/Packages "/Packages") ([roscreate-pkg](/roscreate "/roscreate")), but, as you will see, there is nothing actually difficult here. roscreate-pkg prevents mistakes and saves effort, but packages are just a directory and a simple XML file. Now we'll create a new foobar package. This tutorial assumes that we're working in the directory pkgs on your [ROS\_PACKAGE\_PATH](/ROS/EnvironmentVariables "/ROS/EnvironmentVariables"). 
```
pkgs$ mkdir foobar
pkgs$ cd foobar
```
The very first thing we'll do is add our [manifest](/Manifest "/Manifest") file. The manifest.xml file allows tools like [rospack](/rospack "/rospack") to determine information about what your package depends upon. Inside of foobar/manifest.xml put the following: 
```
<package>
  <description brief="example package tutorial">A simple tutorial package</description>
  <author>Your Name Here</author>
  <license>BSD</license>
  <depend package="roscpp" />
  <depend package="std_msgs" />
</package>
```
Now that your package has a manifest, ROS can find it. Try executing the command: 
```
rospack find foobar
```
If ROS is set up correctly you should see something like: /home/user/ros/pkgs/foobar. This is how ROS finds packages behind the scenes. Note that this package now also has dependencies on [roscpp](/roscpp "/roscpp") and [std\_msgs](/std_msgs "/std_msgs"). To see one example of why specifying these dependencies is useful, try executing the following commands [rospack](/rospack "/rospack") commands: 
```
rospack export --lang=cpp --attrib=cflags foobar
rospack export --lang=cpp --attrib=lflags foobar
```
When you run these, rospack looks up the dependencies of foobar and generates the necessary list of includes or linking statements to compile and link the executable. These commands are used by the ROS build system to correctly compile and link your packages despite the modular nature of ROS. You'll probably never have to use these directly since our build system takes care of it for you. However, as you can see, they are reasonably easy use if you want to use a different build system. In order to take advantage of this, we need to make two build files: a Makefile and [CMakeLists.txt](/CMakeLists "/CMakeLists") file. Inside of foobar/Makefile, put: 
```
include $(shell rospack find mk)/cmake.mk
```
This tells make that we're going to use CMake instead of Make to build this package. Now we need the [CMakeLists.txt](/CMakeLists "/CMakeLists") file so that we can use CMake instead. ROS uses CMake for its more powerful flexibility when building across multiple platforms. In foobar/CMakeLists.txt put: 
```
cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rosbuild_init()
```
That's all you need to start building a package in ROS. Of course, if you want it to actually start building something, you're going to need to learn a couple more CMake macros. See our [CMakeLists](/CMakeLists "/CMakeLists") guide for more information. 

There is a tool for creating ROS [Packages](/Packages "/Packages") ([catkin\_create\_pkg](/catkin/commands/catkin_create_pkg "/catkin/commands/catkin_create_pkg")), but, as you will see, there is nothing actually difficult here. catkin\_create\_pkg prevents mistakes and saves effort, but packages are just a directory and a simple XML file. Now we'll create a new foobar package. This tutorial assumes that we're working your catkin workspace and sourcing of the setup file is already done. 
```
catkin_ws_top $ mkdir -p src/foobar
catkin_ws_top $ cd src/foobar
```
The very first thing we'll do is add our [manifest](/catkin/package.xml "/catkin/package.xml") file. The package.xml file allows tools like [rospack](/rospack "/rospack") to determine information about what your package depends upon. Inside of foobar/package.xml put the following: 
```
<package format="2">
  <name>foobar</name>
  <version>1.2.4</version>
  <description>
  This package provides foo capability.
  </description>
  <maintainer email="foobar@foo.bar.willowgarage.com">PR-foobar</maintainer>
  <license>BSD</license>

  <buildtool_depend>catkin</buildtool_depend>

  <build_depend>roscpp</build_depend>
  <build_depend>std_msgs</build_depend>

  <exec_depend>roscpp</exec_depend>
  <exec_depend>std_msgs</exec_depend>
</package>
```
See also [this page from catkin tutorial](/catkin/Tutorials/CreatingPackage#ROS.2BAC8-Tutorials.2BAC8-catkin.2BAC8-CreatingPackage.Customizing_the_package.xml "/catkin/Tutorials/CreatingPackage#ROS.2BAC8-Tutorials.2BAC8-catkin.2BAC8-CreatingPackage.Customizing_the_package.xml") for further information on [catkin/package.xml](/catkin/package.xml "/catkin/package.xml"). Now that your package has a manifest, ROS can find it. Try executing the command: 
```
rospack find foobar
```
If ROS is set up correctly you should see something like: /home/user/ros/catkin\_ws\_top/src/foobar. This is how ROS finds packages behind the scenes. Note that this package now also has dependencies on [roscpp](/roscpp "/roscpp") and [std\_msgs](/std_msgs "/std_msgs"). Such dependencies are used by catkin to configure packages in the right order. Now we need the [CMakeLists.txt](/CMakeLists "/CMakeLists") file so that [catkin\_make](/catkin_make "/catkin_make"), which uses CMake for its more powerful flexibility when building across multiple platforms, builds the package. In foobar/CMakeLists.txt put: 
```
cmake_minimum_required(VERSION 2.8.3)
project(foobar)
find_package(catkin REQUIRED roscpp std_msgs)
catkin_package()
```
That's all you need to start building a package in ROS using catkin. Of course, if you want it to actually start building something, you're going to need to learn a couple more CMake macros. See our [CMakeLists.txt](/catkin/CMakeLists.txt "/catkin/CMakeLists.txt") guide for more information. Also always go back to beginner level tutorial ([CreatingPackage](/ROS/Tutorials/CreatingPackage "/ROS/Tutorials/CreatingPackage") and so on) to customize your package.xml and CMakeLists.txt. 

Wiki: ROS/Tutorials/Creating a Package by Hand (last edited 2018-05-21 18:40:34 by [ChrisLalancette](/ChrisLalancette "ChrisLalancette @ 70-35-50-58.static.wiline.com[70.35.50.58]"))

Except where otherwise noted, the ROS wiki is licensed under the   

