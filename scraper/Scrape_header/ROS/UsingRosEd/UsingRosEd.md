

 var \_gaq = \_gaq || [];
 \_gaq.push(['\_setAccount', 'UA-17821189-2']);
 \_gaq.push(['\_setDomainName', 'wiki.ros.org']);
 \_gaq.push(['\_trackPageview']);

 (function() {
 var ga = document.createElement('script'); ga.type = 'text/javascript'; ga.async = true;
 ga.src = ('https:' == document.location.protocol ? 'https://ssl' : 'http://www') + '.google-analytics.com/ga.js';
 var s = document.getElementsByTagName('script')[0]; s.parentNode.insertBefore(ga, s);
 })();

ROS/Tutorials/UsingRosEd - ROS Wiki

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
* [UsingRosEd](/action/fullsearch/ROS/Tutorials/UsingRosEd?action=fullsearch&context=180&value=linkto%3A%22ROS%2FTutorials%2FUsingRosEd%22 "Click to do a full-text search for this title")

#### ROS 2 Documentation

The ROS Wiki is for ROS 1. Are you using ROS 2 ([Humble](http://docs.ros.org/en/humble/ "http://docs.ros.org/en/humble/"), [Iron](http://docs.ros.org/en/iron/ "http://docs.ros.org/en/iron/"), or [Rolling](http://docs.ros.org/en/rolling/ "http://docs.ros.org/en/rolling/"))?   
[Check out the ROS 2 Project Documentation](http://docs.ros.org "http://docs.ros.org")  
Package specific documentation can be found on [index.ros.org](https://index.ros.org "https://index.ros.org")

# Wiki

* [Distributions](/Distributions "/Distributions")
* [ROS/Installation](/ROS/Installation "/ROS/Installation")
* [ROS/Tutorials](/ROS/Tutorials "/ROS/Tutorials")
* [RecentChanges](/RecentChanges "/RecentChanges")
* [ROS/Tutorials/UsingRosEd](/ROS/Tutorials/UsingRosEd "/ROS/Tutorials/UsingRosEd")

# Page

* Immutable Page
* [Comments](# "#")
* [Info](/action/info/ROS/Tutorials/UsingRosEd?action=info "/action/info/ROS/Tutorials/UsingRosEd?action=info")
* [Attachments](/action/AttachFile/ROS/Tutorials/UsingRosEd?action=AttachFile "/action/AttachFile/ROS/Tutorials/UsingRosEd?action=AttachFile")
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

* [Login](/action/login/ROS/Tutorials/UsingRosEd?action=login "/action/login/ROS/Tutorials/UsingRosEd?action=login")

|  |
| --- |
| **Note:** This tutorial assumes that you have completed the previous tutorials: [Using rqt\_console and roslaunch](/ROS/Tutorials/UsingRqtconsoleRoslaunch "/ROS/Tutorials/UsingRqtconsoleRoslaunch").  |

|  |
| --- |
| (!) Please ask about problems and questions regarding this tutorial on [answers.ros.org](http://answers.ros.org "http://answers.ros.org"). Don't forget to include in your question the link to this page, the versions of your OS & ROS, and also add appropriate tags. |

# Using rosed to edit files in ROS

**Description:** This tutorial shows how to use [rosed](/rosbash "/rosbash") to make editing easier.  

**Tutorial Level:** BEGINNER  

**Next Tutorial:** [Creating a Msg and Srv](/ROS/Tutorials/CreatingMsgAndSrv "/ROS/Tutorials/CreatingMsgAndSrv")   

 Contents1. [Using rosed](#Using_rosed "#Using_rosed")
2. [Using rosed with tab completion](#Using_rosed_with_tab_completion "#Using_rosed_with_tab_completion")
3. [Editor](#Editor "#Editor")

## Using rosed

rosed is part of the [rosbash](/rosbash "/rosbash") suite. It allows you to directly edit a file within a package by using the package name rather than having to type the entire path to the package. Usage: 
```
$ rosed [package_name] [filename]
```
Example: 
```
$ rosed roscpp Logger.msg
```
This example demonstrates how you would edit the Logger.msg file within the roscpp package. If this example doesn't work it's probably because you don't have the vim editor installed. Please refer to [Editor](/ROS/Tutorials/UsingRosEd#Editor "/ROS/Tutorials/UsingRosEd#Editor") section. If you don't know how to get out of vim, [click here](http://kb.iu.edu/data/afcz.html "http://kb.iu.edu/data/afcz.html"). If the filename is not uniquely defined within the package, a menu will prompt you to choose which of the possible files you want to edit. 
## Using rosed with tab completion

This way you can easily see and optionally edit all files from a package without knowing its exact name. Usage: 
```
$ rosed [package_name] <tab><tab>
```
Example: 
```
$ rosed roscpp <tab><tab>
```
* ```
Empty.srv                   package.xml
GetLoggers.srv              roscpp-msg-extras.cmake
Logger.msg                  roscpp-msg-paths.cmake
SetLoggerLevel.srv          roscpp.cmake
genmsg_cpp.py               roscppConfig-version.cmake
gensrv_cpp.py               roscppConfig.cmake
msg_gen.py                  
```

## Editor

The default editor for rosed is vim. The more beginner-friendly editor nano is included with the default Ubuntu install. You can use it by editing your ~/.bashrc file to include: 
```
export EDITOR='nano -w'
```
To set the default editor to emacs you can edit your ~/.bashrc file to include: 
```
export EDITOR='emacs -nw'
```
***NOTE:*** *changes in .bashrc will only take effect for new terminals. Terminals that are already open will not see the new environmental variable.* Open a new terminal and see if EDITOR is defined: 
```
$ echo $EDITOR
```
* ```
nano -w
```
or 
```
emacs -nw
```

Now that you have successfully configured and used rosed, let's [create a Msg and Srv](/ROS/Tutorials/CreatingMsgAndSrv "/ROS/Tutorials/CreatingMsgAndSrv"). 

Wiki: ROS/Tutorials/UsingRosEd (last edited 2020-04-21 16:08:23 by [chapulina](/chapulina "chapulina @ c-24-5-73-27.hsd1.ca.comcast.net[24.5.73.27]"))

Except where otherwise noted, the ROS wiki is licensed under the   

