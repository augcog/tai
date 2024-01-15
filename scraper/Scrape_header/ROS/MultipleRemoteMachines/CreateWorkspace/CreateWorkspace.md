

 var \_gaq = \_gaq || [];
 \_gaq.push(['\_setAccount', 'UA-17821189-2']);
 \_gaq.push(['\_setDomainName', 'wiki.ros.org']);
 \_gaq.push(['\_trackPageview']);

 (function() {
 var ga = document.createElement('script'); ga.type = 'text/javascript'; ga.async = true;
 ga.src = ('https:' == document.location.protocol ? 'https://ssl' : 'http://www') + '.google-analytics.com/ga.js';
 var s = document.getElementsByTagName('script')[0]; s.parentNode.insertBefore(ga, s);
 })();

ROS/Tutorials/rosbuild/CreateWorkspace - ROS Wiki

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
* [CreateWorkspace](/ROS/Tutorials/rosbuild/CreateWorkspace "/ROS/Tutorials/rosbuild/CreateWorkspace")

#### ROS 2 Documentation

The ROS Wiki is for ROS 1. Are you using ROS 2 ([Humble](http://docs.ros.org/en/humble/ "http://docs.ros.org/en/humble/"), [Iron](http://docs.ros.org/en/iron/ "http://docs.ros.org/en/iron/"), or [Rolling](http://docs.ros.org/en/rolling/ "http://docs.ros.org/en/rolling/"))?   
[Check out the ROS 2 Project Documentation](http://docs.ros.org "http://docs.ros.org")  
Package specific documentation can be found on [index.ros.org](https://index.ros.org "https://index.ros.org")

# Wiki

* [Distributions](/Distributions "/Distributions")
* [ROS/Installation](/ROS/Installation "/ROS/Installation")
* [ROS/Tutorials](/ROS/Tutorials "/ROS/Tutorials")
* [RecentChanges](/RecentChanges "/RecentChanges")
* [CreateWorkspace](/ROS/Tutorials/rosbuild/CreateWorkspace "/ROS/Tutorials/rosbuild/CreateWorkspace")

# Page

* Immutable Page
* [Comments](# "#")
* [Info](/action/info/ROS/Tutorials/rosbuild/CreateWorkspace?action=info "/action/info/ROS/Tutorials/rosbuild/CreateWorkspace?action=info")
* [Attachments](/action/AttachFile/ROS/Tutorials/rosbuild/CreateWorkspace?action=AttachFile "/action/AttachFile/ROS/Tutorials/rosbuild/CreateWorkspace?action=AttachFile")
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

* [Login](/action/login/ROS/Tutorials/rosbuild/CreateWorkspace?action=login "/action/login/ROS/Tutorials/rosbuild/CreateWorkspace?action=login")

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

Wiki: ROS/Tutorials/rosbuild/CreateWorkspace (last edited 2013-01-30 08:39:50 by [JonathanBohren](/JonathanBohren "JonathanBohren @ 128.220.159.9[128.220.159.9]"))

Except where otherwise noted, the ROS wiki is licensed under the   

