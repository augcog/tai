

 var \_gaq = \_gaq || [];
 \_gaq.push(['\_setAccount', 'UA-17821189-2']);
 \_gaq.push(['\_setDomainName', 'wiki.ros.org']);
 \_gaq.push(['\_trackPageview']);

 (function() {
 var ga = document.createElement('script'); ga.type = 'text/javascript'; ga.async = true;
 ga.src = ('https:' == document.location.protocol ? 'https://ssl' : 'http://www') + '.google-analytics.com/ga.js';
 var s = document.getElementsByTagName('script')[0]; s.parentNode.insertBefore(ga, s);
 })();

ROS/Tutorials/InstallingandConfiguringROSEnvironment - ROS Wiki

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
* [InstallingandConfiguringROSEnvironment](/ROS/Tutorials/InstallingandConfiguringROSEnvironment "/ROS/Tutorials/InstallingandConfiguringROSEnvironment")

#### ROS 2 Documentation

The ROS Wiki is for ROS 1. Are you using ROS 2 ([Humble](http://docs.ros.org/en/humble/ "http://docs.ros.org/en/humble/"), [Iron](http://docs.ros.org/en/iron/ "http://docs.ros.org/en/iron/"), or [Rolling](http://docs.ros.org/en/rolling/ "http://docs.ros.org/en/rolling/"))?   
[Check out the ROS 2 Project Documentation](http://docs.ros.org "http://docs.ros.org")  
Package specific documentation can be found on [index.ros.org](https://index.ros.org "https://index.ros.org")

# Wiki

* [Distributions](/Distributions "/Distributions")
* [ROS/Installation](/ROS/Installation "/ROS/Installation")
* [ROS/Tutorials](/ROS/Tutorials "/ROS/Tutorials")
* [RecentChanges](/RecentChanges "/RecentChanges")
* [Installinga...Environment](/ROS/Tutorials/InstallingandConfiguringROSEnvironment "/ROS/Tutorials/InstallingandConfiguringROSEnvironment")

# Page

* Immutable Page
* [Comments](# "#")
* [Info](/action/info/ROS/Tutorials/InstallingandConfiguringROSEnvironment?action=info "/action/info/ROS/Tutorials/InstallingandConfiguringROSEnvironment?action=info")
* [Attachments](/action/AttachFile/ROS/Tutorials/InstallingandConfiguringROSEnvironment?action=AttachFile "/action/AttachFile/ROS/Tutorials/InstallingandConfiguringROSEnvironment?action=AttachFile")
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

* [Login](/action/login/ROS/Tutorials/InstallingandConfiguringROSEnvironment?action=login "/action/login/ROS/Tutorials/InstallingandConfiguringROSEnvironment?action=login")

|  |
| --- |
| (!) Please ask about problems and questions regarding this tutorial on [answers.ros.org](http://answers.ros.org "http://answers.ros.org"). Don't forget to include in your question the link to this page, the versions of your OS & ROS, and also add appropriate tags. |

# Installing and Configuring Your ROS Environment

**Description:** This tutorial walks you through installing ROS and setting up the ROS environment on your computer.  

**Tutorial Level:** BEGINNER  

**Next Tutorial:** [Navigating the ROS Filesystem](/ROS/Tutorials/NavigatingTheFilesystem "/ROS/Tutorials/NavigatingTheFilesystem")   

 Contents1. [Install ROS](#Install_ROS "#Install_ROS")
2. [Managing Your Environment](#Managing_Your_Environment "#Managing_Your_Environment")
3. [Create a ROS Workspace](#Create_a_ROS_Workspace "#Create_a_ROS_Workspace")
4. [Video Demonstration](#Video_Demonstration "#Video_Demonstration")

 **Note:** The ROS Wiki is generally for ROS 1 only! If you have installed ROS 2 please use the [ROS 2 documentation website](https://index.ros.org/doc/ros2/Contributing/Developer-Guide/ "https://index.ros.org/doc/ros2/Contributing/Developer-Guide/"). 

## Install ROS

Before starting these tutorials please complete installation as described in the [ROS installation instructions](/ROS/Installation "/ROS/Installation"). **Note:** If you installed ROS from a package manager like apt, then those packages will not be write accessible and should not be edited by you the user. When working with ROS packages from source or when creating a new ROS package, you should always work in a directory that you have access to, like your home folder. 

## Managing Your Environment

During the installation of ROS, you will see that you are prompted to source one of several setup.\*sh files, or even add this 'sourcing' to your shell startup script. This is required because ROS relies on the notion of combining spaces using the shell environment. This makes developing against different versions of ROS or against different sets of packages easier. If you are ever having problems finding or using your ROS packages make sure that you have your environment properly setup. A good way to check is to ensure that [environment variables](/ROS/EnvironmentVariables "/ROS/EnvironmentVariables") like [ROS\_ROOT](/ROS/EnvironmentVariables#ROS_ROOT "/ROS/EnvironmentVariables#ROS_ROOT") and [ROS\_PACKAGE\_PATH](/ROS/EnvironmentVariables#ROS_PACKAGE_PATH "/ROS/EnvironmentVariables#ROS_PACKAGE_PATH") are set: 
```
$ printenv | grep ROS
```
If they are not then you might need to 'source' some setup.\*sh files. Environment setup files are generated for you, but can come from different places: * ROS packages installed with package managers provide setup.\*sh files
* [rosbuild workspaces](http://www.ros.org/wiki/fuerte/Installation/Overlays "http://www.ros.org/wiki/fuerte/Installation/Overlays") provide setup.\*sh files using tools like [rosws](/rosws "/rosws")
* Setup.\*sh files are created as a by-product of [building](/catkin/workspaces#Building_Packages_with_catkin "/catkin/workspaces#Building_Packages_with_catkin") or [installing](/catkin/workspaces#Installing_Packages_with_Catkin "/catkin/workspaces#Installing_Packages_with_Catkin") catkin packages

**Note:** Throughout the tutorials you will see references to [rosbuild](/rosbuild "/rosbuild") and [catkin](/catkin "/catkin"). These are the two available methods for organizing and building your ROS code. rosbuild is not recommended or maintained anymore but kept for legacy. catkin is the recommended way to organise your code, it uses more standard CMake conventions and provides more flexibility especially for people wanting to integrate external code bases or who want to release their software. For a full break down visit [catkin or rosbuild](/catkin_or_rosbuild "/catkin_or_rosbuild"). 

If you just installed ROS from apt on Ubuntu then you will have setup.\*sh files in '/opt/ros/<distro>/', and you could source them like so: 
```
$ source /opt/ros/<distro>/setup.bash
```
Using the short name of your ROS distribution instead of <distro> If you installed ROS Kinetic, that would be: 
```
$ source /opt/ros/kinetic/setup.bash
```
You will need to run this command on every new shell you open to have access to the ROS commands, unless you add this line to your .bashrc. This process allows you to install several ROS distributions (e.g. indigo and kinetic) on the same computer and switch between them. On other platforms you will find these setup.\*sh files wherever you installed ROS. 
## Create a ROS Workspace

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
 These instructions are for ROS Groovy and later. For ROS Fuerte and earlier, select rosbuild. 

Let's create and build a [catkin workspace](/catkin/workspaces "/catkin/workspaces"): 
```
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/
$ catkin_make
```
The [catkin\_make](/catkin/commands/catkin_make "/catkin/commands/catkin_make") command is a convenience tool for working with [catkin workspaces](/catkin/workspaces "/catkin/workspaces"). Running it the first time in your workspace, it will create a CMakeLists.txt link in your 'src' folder. **Python 3 users in ROS Melodic and earlier**: note, if you are building ROS from source to achieve Python 3 compatibility, and have setup your system appropriately (ie: have the Python 3 versions of all the required ROS Python packages installed, such as catkin) the first [catkin\_make](/catkin/commands/catkin_make "/catkin/commands/catkin_make") command in a clean [catkin workspace](/catkin/workspaces "/catkin/workspaces") must be: 
```
$ catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3
```
This will configure [catkin\_make](/catkin/commands/catkin_make "/catkin/commands/catkin_make") with Python 3. You may then proceed to use just catkin\_make for subsequent builds. 

Additionally, if you look in your current directory you should now have a 'build' and 'devel' folder. Inside the 'devel' folder you can see that there are now several setup.\*sh files. Sourcing any of these files will overlay this workspace on top of your environment. To understand more about this see the general catkin documentation: [catkin](/catkin "/catkin"). Before continuing source your new setup.\*sh file: 
```
$ source devel/setup.bash
```
To make sure your workspace is properly overlayed by the setup script, make sure ROS\_PACKAGE\_PATH environment variable includes the directory you're in. 
```
$ echo $ROS_PACKAGE_PATH
/home/youruser/catkin_ws/src:/opt/ros/kinetic/share
```

When working with ROS source code, it is often useful to do so in a "workspace". For the following ROS tutorials you will need an area for working on tutorials and creating new ROS stacks and packages. [rosws](/rosws "/rosws") is a tool that provides a uniform interface to various version control systems such as SVN, Git and Mercurial and for managing all packages installed in a ROS overlay. An extensive tutorial on rosws can be found [here](http://www.ros.org/doc/api/rosinstall/html/rosws_tutorial.html "http://www.ros.org/doc/api/rosinstall/html/rosws_tutorial.html"). 
### Creating a new workspace

The following command creates a new workspace in ~/fuerte\_workspace which extends the set of packages installed in /opt/ros/fuerte: 
```
rosws init ~/fuerte_workspace /opt/ros/fuerte
```
**Note:** rosws is part of the [rosinstall package](/rosinstall "/rosinstall"), which is not installed by default. The following command downloads it using the Ubuntu package manager: 
```
sudo apt-get install python-rosinstall
```

### Creating a sandbox directory for new packages

New packages need to be put in a path that is in the variable ROS\_PACKAGE\_PATH. All directories that are managed by rosws, i.e. that have been added using rosws are automatically added to the ROS\_PACKAGE\_PATH when the file setup.bash of the corresponding workspace is sourced. Although new packages should always be put in repositories that have been installed using rosws, it can be very convenient to have a sandbox directory where for instance packages created during the tutorials can be put without requiring any additional rosws commands. For that we create a new directory sandbox and add it to the hidden .rosinstall file with rosws: 
```
mkdir ~/fuerte_workspace/sandbox
rosws set ~/fuerte_workspace/sandbox
```
Every time the entries in the workspace change, it is necessary to re-source ~/fuerte\_workspace/setup.bash to make sure that the updated ROS\_PACKAGE\_PATH is used. 
```
source ~/fuerte_workspace/setup.bash
```
It is very common to replace the line source /opt/ros/fuerte/setup.bash to source the setup.bash in ~/fuerte\_workspace or whichever workspace you use most often. A more complete ROS Workspace tutorial can be found [here](/fuerte/Installation/Overlays "/fuerte/Installation/Overlays"). 
### Confirmation

To confirm that your package path has been set, echo the ROS\_PACKAGE\_PATH variable. 
```
$ echo $ROS_PACKAGE_PATH
```
You should see something similar to: 
```
/home/your_user_name/fuerte_workspace/sandbox:/opt/ros/fuerte/share:/opt/ros/fuerte/stacks
```

Now that your environment is setup, continue with the [ROS file system tutorial](/ROS/Tutorials/NavigatingTheFilesystem "/ROS/Tutorials/NavigatingTheFilesystem"). 
## Video Demonstration

Watch the video below to have more explanation on Custom Workspace and Package Creation with step by step guide .  

Wiki: ROS/Tutorials/InstallingandConfiguringROSEnvironment (last edited 2022-10-18 15:59:49 by [Muhammad Luqman](/Muhammad%20Luqman "Muhammad Luqman @ 103.138.11.4[103.138.11.4]"))

Except where otherwise noted, the ROS wiki is licensed under the   

