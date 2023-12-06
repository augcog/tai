

 var \_gaq = \_gaq || [];
 \_gaq.push(['\_setAccount', 'UA-17821189-2']);
 \_gaq.push(['\_setDomainName', 'wiki.ros.org']);
 \_gaq.push(['\_trackPageview']);

 (function() {
 var ga = document.createElement('script'); ga.type = 'text/javascript'; ga.async = true;
 ga.src = ('https:' == document.location.protocol ? 'https://ssl' : 'http://www') + '.google-analytics.com/ga.js';
 var s = document.getElementsByTagName('script')[0]; s.parentNode.insertBefore(ga, s);
 })();

ROS/Tutorials/Getting started with roswtf - ROS Wiki

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
* [Getting started with roswtf](/action/fullsearch/ROS/Tutorials/Getting%20started%20with%20roswtf?action=fullsearch&context=180&value=linkto%3A%22ROS%2FTutorials%2FGetting+started+with+roswtf%22 "Click to do a full-text search for this title")

#### ROS 2 Documentation

The ROS Wiki is for ROS 1. Are you using ROS 2 ([Humble](http://docs.ros.org/en/humble/ "http://docs.ros.org/en/humble/"), [Iron](http://docs.ros.org/en/iron/ "http://docs.ros.org/en/iron/"), or [Rolling](http://docs.ros.org/en/rolling/ "http://docs.ros.org/en/rolling/"))?   
[Check out the ROS 2 Project Documentation](http://docs.ros.org "http://docs.ros.org")  
Package specific documentation can be found on [index.ros.org](https://index.ros.org "https://index.ros.org")

# Wiki

* [Distributions](/Distributions "/Distributions")
* [ROS/Installation](/ROS/Installation "/ROS/Installation")
* [ROS/Tutorials](/ROS/Tutorials "/ROS/Tutorials")
* [RecentChanges](/RecentChanges "/RecentChanges")
* [Getting sta...with roswtf](/ROS/Tutorials/Getting%20started%20with%20roswtf "/ROS/Tutorials/Getting%20started%20with%20roswtf")

# Page

* Immutable Page
* [Comments](# "#")
* [Info](/action/info/ROS/Tutorials/Getting%20started%20with%20roswtf?action=info "/action/info/ROS/Tutorials/Getting%20started%20with%20roswtf?action=info")
* [Attachments](/action/AttachFile/ROS/Tutorials/Getting%20started%20with%20roswtf?action=AttachFile "/action/AttachFile/ROS/Tutorials/Getting%20started%20with%20roswtf?action=AttachFile")
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

* [Login](/action/login/ROS/Tutorials/Getting%20started%20with%20roswtf?action=login "/action/login/ROS/Tutorials/Getting%20started%20with%20roswtf?action=login")

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

Wiki: ROS/Tutorials/Getting started with roswtf (last edited 2018-12-16 14:47:24 by [HabibOladepo](/HabibOladepo "HabibOladepo @ 197.210.227.200[197.210.227.200]"))

Except where otherwise noted, the ROS wiki is licensed under the   

