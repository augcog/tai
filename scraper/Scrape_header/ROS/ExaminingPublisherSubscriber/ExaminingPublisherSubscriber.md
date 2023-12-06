

 var \_gaq = \_gaq || [];
 \_gaq.push(['\_setAccount', 'UA-17821189-2']);
 \_gaq.push(['\_setDomainName', 'wiki.ros.org']);
 \_gaq.push(['\_trackPageview']);

 (function() {
 var ga = document.createElement('script'); ga.type = 'text/javascript'; ga.async = true;
 ga.src = ('https:' == document.location.protocol ? 'https://ssl' : 'http://www') + '.google-analytics.com/ga.js';
 var s = document.getElementsByTagName('script')[0]; s.parentNode.insertBefore(ga, s);
 })();

ROS/Tutorials/ExaminingPublisherSubscriber - ROS Wiki

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
* [ExaminingPublisherSubscriber](/action/fullsearch/ROS/Tutorials/ExaminingPublisherSubscriber?action=fullsearch&context=180&value=linkto%3A%22ROS%2FTutorials%2FExaminingPublisherSubscriber%22 "Click to do a full-text search for this title")

#### ROS 2 Documentation

The ROS Wiki is for ROS 1. Are you using ROS 2 ([Humble](http://docs.ros.org/en/humble/ "http://docs.ros.org/en/humble/"), [Iron](http://docs.ros.org/en/iron/ "http://docs.ros.org/en/iron/"), or [Rolling](http://docs.ros.org/en/rolling/ "http://docs.ros.org/en/rolling/"))?   
[Check out the ROS 2 Project Documentation](http://docs.ros.org "http://docs.ros.org")  
Package specific documentation can be found on [index.ros.org](https://index.ros.org "https://index.ros.org")

# Wiki

* [Distributions](/Distributions "/Distributions")
* [ROS/Installation](/ROS/Installation "/ROS/Installation")
* [ROS/Tutorials](/ROS/Tutorials "/ROS/Tutorials")
* [RecentChanges](/RecentChanges "/RecentChanges")
* [ExaminingPu...rSubscriber](/ROS/Tutorials/ExaminingPublisherSubscriber "/ROS/Tutorials/ExaminingPublisherSubscriber")

# Page

* Immutable Page
* [Comments](# "#")
* [Info](/action/info/ROS/Tutorials/ExaminingPublisherSubscriber?action=info "/action/info/ROS/Tutorials/ExaminingPublisherSubscriber?action=info")
* [Attachments](/action/AttachFile/ROS/Tutorials/ExaminingPublisherSubscriber?action=AttachFile "/action/AttachFile/ROS/Tutorials/ExaminingPublisherSubscriber?action=AttachFile")
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

* [Login](/action/login/ROS/Tutorials/ExaminingPublisherSubscriber?action=login "/action/login/ROS/Tutorials/ExaminingPublisherSubscriber?action=login")

|  |
| --- |
| **Note:** This tutorial assumes that you have completed the previous tutorials: writing a simple publisher and subscriber [(python)](/ROS/Tutorials/WritingPublisherSubscriber%28python%29 "/ROS/Tutorials/WritingPublisherSubscriber%28python%29") [(c++)](/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29 "/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29").  |

|  |
| --- |
| (!) Please ask about problems and questions regarding this tutorial on [answers.ros.org](http://answers.ros.org "http://answers.ros.org"). Don't forget to include in your question the link to this page, the versions of your OS & ROS, and also add appropriate tags. |

# Examining the Simple Publisher and Subscriber

**Description:** This tutorial examines running the simple publisher and subscriber.  

**Tutorial Level:** BEGINNER  

**Next Tutorial:** Writing a simple service and client [(python)](/ROS/Tutorials/WritingServiceClient%28python%29 "/ROS/Tutorials/WritingServiceClient%28python%29") [(c++)](/ROS/Tutorials/WritingServiceClient%28c%2B%2B%29 "/ROS/Tutorials/WritingServiceClient%28c%2B%2B%29")  

 Contents1. [Running the Publisher](#Running_the_Publisher "#Running_the_Publisher")
2. [Running the Subscriber](#Running_the_Subscriber "#Running_the_Subscriber")
3. [Video Demonstration](#Video_Demonstration "#Video_Demonstration")

## Running the Publisher

Make sure that a roscore is up and running: 
```
$ roscore
```
catkin specific If you are using catkin, make sure you have sourced your workspace's setup.sh file after calling catkin\_make but before trying to use your applications: 
```
# In your catkin workspace
$ cd ~/catkin_ws
$ source ./devel/setup.bash
```
In the last tutorial we made a publisher called "talker". Let's run it: 
```
$ rosrun beginner_tutorials talker      (C++)
$ rosrun beginner_tutorials talker.py   (Python) 
```
You will see something similar to: * ```
[INFO] [WallTime: 1314931831.774057] hello world 1314931831.77
[INFO] [WallTime: 1314931832.775497] hello world 1314931832.77
[INFO] [WallTime: 1314931833.778937] hello world 1314931833.78
[INFO] [WallTime: 1314931834.782059] hello world 1314931834.78
[INFO] [WallTime: 1314931835.784853] hello world 1314931835.78
[INFO] [WallTime: 1314931836.788106] hello world 1314931836.79
```

The publisher node is up and running. Now we need a subscriber to receive messages from the publisher. 
## Running the Subscriber

In the last tutorial we made a subscriber called "listener". Let's run it: 
```
$ rosrun beginner_tutorials listener     (C++)
$ rosrun beginner_tutorials listener.py  (Python) 
```
You will see something similar to: * ```
[INFO] [WallTime: 1314931969.258941] /listener_17657_1314931968795I heard hello world 1314931969.26
[INFO] [WallTime: 1314931970.262246] /listener_17657_1314931968795I heard hello world 1314931970.26
[INFO] [WallTime: 1314931971.266348] /listener_17657_1314931968795I heard hello world 1314931971.26
[INFO] [WallTime: 1314931972.270429] /listener_17657_1314931968795I heard hello world 1314931972.27
[INFO] [WallTime: 1314931973.274382] /listener_17657_1314931968795I heard hello world 1314931973.27
[INFO] [WallTime: 1314931974.277694] /listener_17657_1314931968795I heard hello world 1314931974.28
[INFO] [WallTime: 1314931975.283708] /listener_17657_1314931968795I heard hello world 1314931975.28
```

When you are done, press Ctrl-C to terminate both the listener and the talker. Now that you have examined the simple publisher and subscriber, let's write a simple service and client [(python)](/ROS/Tutorials/WritingServiceClient%28python%29 "/ROS/Tutorials/WritingServiceClient%28python%29") [(c++)](/ROS/Tutorials/WritingServiceClient%28c%2B%2B%29 "/ROS/Tutorials/WritingServiceClient%28c%2B%2B%29"). 
## Video Demonstration

Watch the video below to have more explanation on Nodes Communication and step by step guide .  

Wiki: ROS/Tutorials/ExaminingPublisherSubscriber (last edited 2022-10-18 16:28:44 by [Muhammad Luqman](/Muhammad%20Luqman "Muhammad Luqman @ 103.138.11.4[103.138.11.4]"))

Except where otherwise noted, the ROS wiki is licensed under the   

