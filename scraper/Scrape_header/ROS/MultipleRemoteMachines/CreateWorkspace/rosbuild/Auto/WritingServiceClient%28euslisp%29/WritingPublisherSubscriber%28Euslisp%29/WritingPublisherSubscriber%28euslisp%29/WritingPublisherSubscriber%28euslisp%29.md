

 var \_gaq = \_gaq || [];
 \_gaq.push(['\_setAccount', 'UA-17821189-2']);
 \_gaq.push(['\_setDomainName', 'wiki.ros.org']);
 \_gaq.push(['\_trackPageview']);

 (function() {
 var ga = document.createElement('script'); ga.type = 'text/javascript'; ga.async = true;
 ga.src = ('https:' == document.location.protocol ? 'https://ssl' : 'http://www') + '.google-analytics.com/ga.js';
 var s = document.getElementsByTagName('script')[0]; s.parentNode.insertBefore(ga, s);
 })();

ROS/Tutorials/WritingPublisherSubscriber(euslisp) - ROS Wiki

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
* [WritingPublisherSubscriber(euslisp)](/ROS/Tutorials/WritingPublisherSubscriber%28euslisp%29 "/ROS/Tutorials/WritingPublisherSubscriber%28euslisp%29")

#### ROS 2 Documentation

The ROS Wiki is for ROS 1. Are you using ROS 2 ([Humble](http://docs.ros.org/en/humble/ "http://docs.ros.org/en/humble/"), [Iron](http://docs.ros.org/en/iron/ "http://docs.ros.org/en/iron/"), or [Rolling](http://docs.ros.org/en/rolling/ "http://docs.ros.org/en/rolling/"))?   
[Check out the ROS 2 Project Documentation](http://docs.ros.org "http://docs.ros.org")  
Package specific documentation can be found on [index.ros.org](https://index.ros.org "https://index.ros.org")

# Wiki

* [Distributions](/Distributions "/Distributions")
* [ROS/Installation](/ROS/Installation "/ROS/Installation")
* [ROS/Tutorials](/ROS/Tutorials "/ROS/Tutorials")
* [RecentChanges](/RecentChanges "/RecentChanges")
* [WritingPubl...er(euslisp)](/ROS/Tutorials/WritingPublisherSubscriber%28euslisp%29 "/ROS/Tutorials/WritingPublisherSubscriber%28euslisp%29")

# Page

* Immutable Page
* [Comments](# "#")
* [Info](/action/info/ROS/Tutorials/WritingPublisherSubscriber%28euslisp%29?action=info "/action/info/ROS/Tutorials/WritingPublisherSubscriber%28euslisp%29?action=info")
* [Attachments](/action/AttachFile/ROS/Tutorials/WritingPublisherSubscriber%28euslisp%29?action=AttachFile "/action/AttachFile/ROS/Tutorials/WritingPublisherSubscriber%28euslisp%29?action=AttachFile")
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

* [Login](/action/login/ROS/Tutorials/WritingPublisherSubscriber%28euslisp%29?action=login "/action/login/ROS/Tutorials/WritingPublisherSubscriber%28euslisp%29?action=login")

|  |
| --- |
| **Note:** This tutorial assumes that you have completed the previous tutorials: [creating a ROS msg and srv](/ROS/Tutorials/CreatingMsgAndSrv "/ROS/Tutorials/CreatingMsgAndSrv").  |

|  |
| --- |
| (!) Please ask about problems and questions regarding this tutorial on [answers.ros.org](http://answers.ros.org "http://answers.ros.org"). Don't forget to include in your question the link to this page, the versions of your OS & ROS, and also add appropriate tags. |

# Writing a Simple Publisher and Subscriber (Euslisp)

**Description:** This tutorial covers how to write a publisher and subscriber node in euslisp.  

**Tutorial Level:** BEGINNER  

**Next Tutorial:** [Writing a Simple Service and Client (Euslisp)](/ROS/Tutorials/WritingServiceClient%28euslisp%29 "/ROS/Tutorials/WritingServiceClient%28euslisp%29")   

 Contents1. [Writing a Simple Publisher and Subscriber (Euslisp)](#Writing_a_Simple_Publisher_and_Subscriber_.28Euslisp.29 "#Writing_a_Simple_Publisher_and_Subscriber_.28Euslisp.29")
	1. 1. [The Code](#The_Code "#The_Code")
		2. [The Code Explained](#The_Code_Explained "#The_Code_Explained")
	2. [Writing the Subscriber Node](#Writing_the_Subscriber_Node "#Writing_the_Subscriber_Node")
		1. [The Code](#The_Code-1 "#The_Code-1")
		2. [The Code Explained](#The_Code_Explained-1 "#The_Code_Explained-1")
	3. [Building your nodes](#Building_your_nodes "#Building_your_nodes")
	4. [Running Nodes](#Running_Nodes "#Running_Nodes")

## Writing a Simple Publisher and Subscriber (Euslisp)

"Node" is the ROS term for an executable that is connected to the ROS network. Here we'll create the publisher ("talker") node which will continually broadcast a message. Change directory into the beginner\_tutorials package, you created in the earlier tutorial, creating a package: 
```
roscd beginner_tutorials
```

#### The Code

First lets create a 'euslisp' folder to store our [EusLisp](/EusLisp "/EusLisp") scripts in: 
```
$ mkdir euslisp
```
Then, create talker.l file under your new euslisp directly. 
```
#!/usr/bin/env roseus
;;;
;;; euslisp version of ros_tutorials/rospy_tutorials/001_talker_listener
;;;

(ros::load-ros-manifest "roseus")

;;;
(ros::roseus "talker")
(ros::advertise "chatter" std_msgs::string 1)
(ros::rate 100)
(while (ros::ok)
  (setq msg (instance std_msgs::string :init))
  (send msg :data (format nil "hello world ~a" (send (ros::time-now) :sec-nsec)))
  (ros::ros-info "msg [~A]" (send msg :data))
  (ros::publish "chatter" msg)
  (ros::sleep)
  )
(ros::roseus "shutdown")
(exit)
```
Don't forget to make the node executable: 
```
$ chmod +x euslisp/talker.l
```

#### The Code Explained

Now, let's break the code down. 
```
#!/usr/bin/env roseus
```
 Every ROS[Node](/Nodes "/Nodes")Node will have this declaration at the top. The first line makes sure your script is executed as a roseus script. 
```
(ros::load-ros-manifest "roseus")
```
 Thisline reads roseus's [manifest.xml](/Manifest "/Manifest") to import all dependent files. You need to import roseus if you are writing a ROS Node. This will enable users to use std\_msgs.msg so that we can reuse the std\_msgs/String message type (a simple string container) for publishing. 
```
(ros::roseus "talker")
```
 The next line, (ros::roseus "talker"), is very important as it tells roseus the name of your node -- until roseus has this information, it cannot start communicating with the ROS [Master](/Master "/Master"). In this case, your node will take on the name talker. NOTE: the name must be a [base name](/Names "/Names"), i.e. it cannot contain any slashes "/" 
```
(ros::advertise "chatter" std_msgs::string 1)
```
 This section of code defines the talker's interface to the rest of ROS. (ros::advertise "chatter" std\_msgs::string 1) declares that your node is publishing to the chatter topic using the message type std\_msgs::string. 
```
(ros::rate 100)
```
 This line creates a Rate object r. With the help of its method sleep(), it offers a convenient way for looping at the desired rate. With its argument of 100, we should expect to go through the loop 100 times per second (as long as our processing time does not exceed 1/100th of a second!) 
```
(while (ros::ok)
  (setq msg (instance std_msgs::string :init))
  (send msg :data (format nil "hello world ~a" (send (ros::time-now) :sec-nsec)))
  (ros::ros-info "msg [~A]" (send msg :data))
  (ros::publish "chatter" msg)
  (ros::sleep)
  )
```
 This loop is a fairly standard roseus construct: checking the (ros::ok) flag and then doing work. You have to check (ros::ok) to check if your program should exit (e.g. if there is a Ctrl-C or otherwise). In this case, the "work" is a call to   (ros::publish "chatter" msg) that publishes to our chatter topic using a newly created String message. The loop calls time.sleep(), which sleeps just long enough to maintain the desired rate through the loop. This loop also calls   (ros::ros-info "msg [~A]" (send msg :data)), which performs triple-duty: the messages get printed to screen, it gets written to the Node's log file, and it gets written to rosout. [[rosout](/%5Brosout "/%5Brosout")] is a handy for debugging: you can pull up messages using [[rqt\_console](/%5Brqt_console "/%5Brqt_console")] instead of having to find the console window with your Node's output. 
```
(ros::roseus "shutdown")
(exit)
```
 Finally we'll stop the node. 
### Writing the Subscriber Node

#### The Code

Next, let's create listener.l file under your new euslisp directly. 

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

document.write('<a href="#" onclick="return togglenumber(\'CA-50a497b9421dd2ae5c537338b7af9b2f859ea6aa\', 1, 1);" \
 class="codenumbers">Toggle line numbers<\/a>');

```
 [1](#CA-50a497b9421dd2ae5c537338b7af9b2f859ea6aa_1 "#CA-50a497b9421dd2ae5c537338b7af9b2f859ea6aa_1") #!/usr/bin/env roseus
 [2](#CA-50a497b9421dd2ae5c537338b7af9b2f859ea6aa_2 "#CA-50a497b9421dd2ae5c537338b7af9b2f859ea6aa_2") ;;;
 [3](#CA-50a497b9421dd2ae5c537338b7af9b2f859ea6aa_3 "#CA-50a497b9421dd2ae5c537338b7af9b2f859ea6aa_3") ;;; euslisp version of ros\_tutorials/rospy\_tutorials/001\_talker\_listener
 [4](#CA-50a497b9421dd2ae5c537338b7af9b2f859ea6aa_4 "#CA-50a497b9421dd2ae5c537338b7af9b2f859ea6aa_4") ;;;
 [5](#CA-50a497b9421dd2ae5c537338b7af9b2f859ea6aa_5 "#CA-50a497b9421dd2ae5c537338b7af9b2f859ea6aa_5") 
 [6](#CA-50a497b9421dd2ae5c537338b7af9b2f859ea6aa_6 "#CA-50a497b9421dd2ae5c537338b7af9b2f859ea6aa_6") (ros::load-ros-manifest "roseus")
 [7](#CA-50a497b9421dd2ae5c537338b7af9b2f859ea6aa_7 "#CA-50a497b9421dd2ae5c537338b7af9b2f859ea6aa_7") ;;;
 [8](#CA-50a497b9421dd2ae5c537338b7af9b2f859ea6aa_8 "#CA-50a497b9421dd2ae5c537338b7af9b2f859ea6aa_8") 
 [9](#CA-50a497b9421dd2ae5c537338b7af9b2f859ea6aa_9 "#CA-50a497b9421dd2ae5c537338b7af9b2f859ea6aa_9") ;;;
 [10](#CA-50a497b9421dd2ae5c537338b7af9b2f859ea6aa_10 "#CA-50a497b9421dd2ae5c537338b7af9b2f859ea6aa_10") ;;;
 [11](#CA-50a497b9421dd2ae5c537338b7af9b2f859ea6aa_11 "#CA-50a497b9421dd2ae5c537338b7af9b2f859ea6aa_11") (ros::roseus "listener" :anonymous t)
 [12](#CA-50a497b9421dd2ae5c537338b7af9b2f859ea6aa_12 "#CA-50a497b9421dd2ae5c537338b7af9b2f859ea6aa_12") ;;(setq sys::\*gc-hook\* #'(lambda (a b) (format t ";; gc ~A ~A~%" a b)))
 [13](#CA-50a497b9421dd2ae5c537338b7af9b2f859ea6aa_13 "#CA-50a497b9421dd2ae5c537338b7af9b2f859ea6aa_13") 
 [14](#CA-50a497b9421dd2ae5c537338b7af9b2f859ea6aa_14 "#CA-50a497b9421dd2ae5c537338b7af9b2f859ea6aa_14") ;; callback function
 [15](#CA-50a497b9421dd2ae5c537338b7af9b2f859ea6aa_15 "#CA-50a497b9421dd2ae5c537338b7af9b2f859ea6aa_15") ;(defun string-cb (msg) (print (list 'cb (sys::thread-self) (send msg :data))))
 [16](#CA-50a497b9421dd2ae5c537338b7af9b2f859ea6aa_16 "#CA-50a497b9421dd2ae5c537338b7af9b2f859ea6aa_16") ;(ros::subscribe "chatter" std\_msgs::string #'string-cb)
 [17](#CA-50a497b9421dd2ae5c537338b7af9b2f859ea6aa_17 "#CA-50a497b9421dd2ae5c537338b7af9b2f859ea6aa_17") 
 [18](#CA-50a497b9421dd2ae5c537338b7af9b2f859ea6aa_18 "#CA-50a497b9421dd2ae5c537338b7af9b2f859ea6aa_18") ; lambda function
 [19](#CA-50a497b9421dd2ae5c537338b7af9b2f859ea6aa_19 "#CA-50a497b9421dd2ae5c537338b7af9b2f859ea6aa_19") ;(ros::subscribe "chatter" std\_msgs::string
 [20](#CA-50a497b9421dd2ae5c537338b7af9b2f859ea6aa_20 "#CA-50a497b9421dd2ae5c537338b7af9b2f859ea6aa_20") ; #'(lambda (msg) (ros::rosinfo 
 [21](#CA-50a497b9421dd2ae5c537338b7af9b2f859ea6aa_21 "#CA-50a497b9421dd2ae5c537338b7af9b2f859ea6aa_21") ; (format nil "I heard ~A" (send msg :data)))))
 [22](#CA-50a497b9421dd2ae5c537338b7af9b2f859ea6aa_22 "#CA-50a497b9421dd2ae5c537338b7af9b2f859ea6aa_22") 
 [23](#CA-50a497b9421dd2ae5c537338b7af9b2f859ea6aa_23 "#CA-50a497b9421dd2ae5c537338b7af9b2f859ea6aa_23") ;; method call
 [24](#CA-50a497b9421dd2ae5c537338b7af9b2f859ea6aa_24 "#CA-50a497b9421dd2ae5c537338b7af9b2f859ea6aa_24") (defclass string-cb-class
 [25](#CA-50a497b9421dd2ae5c537338b7af9b2f859ea6aa_25 "#CA-50a497b9421dd2ae5c537338b7af9b2f859ea6aa_25")  :super propertied-object
 [26](#CA-50a497b9421dd2ae5c537338b7af9b2f859ea6aa_26 "#CA-50a497b9421dd2ae5c537338b7af9b2f859ea6aa_26")  :slots ())
 [27](#CA-50a497b9421dd2ae5c537338b7af9b2f859ea6aa_27 "#CA-50a497b9421dd2ae5c537338b7af9b2f859ea6aa_27") (defmethod string-cb-class
 [28](#CA-50a497b9421dd2ae5c537338b7af9b2f859ea6aa_28 "#CA-50a497b9421dd2ae5c537338b7af9b2f859ea6aa_28")  (:init () (ros::subscribe "chatter" std\_msgs::string #'send self :string-cb))
 [29](#CA-50a497b9421dd2ae5c537338b7af9b2f859ea6aa_29 "#CA-50a497b9421dd2ae5c537338b7af9b2f859ea6aa_29")  (:string-cb (msg) (print (list 'cb self (send msg :data)))))
 [30](#CA-50a497b9421dd2ae5c537338b7af9b2f859ea6aa_30 "#CA-50a497b9421dd2ae5c537338b7af9b2f859ea6aa_30") (setq m (instance string-cb-class :init))
 [31](#CA-50a497b9421dd2ae5c537338b7af9b2f859ea6aa_31 "#CA-50a497b9421dd2ae5c537338b7af9b2f859ea6aa_31") 
 [32](#CA-50a497b9421dd2ae5c537338b7af9b2f859ea6aa_32 "#CA-50a497b9421dd2ae5c537338b7af9b2f859ea6aa_32") (do-until-key
 [33](#CA-50a497b9421dd2ae5c537338b7af9b2f859ea6aa_33 "#CA-50a497b9421dd2ae5c537338b7af9b2f859ea6aa_33")  (ros::spin-once)
 [34](#CA-50a497b9421dd2ae5c537338b7af9b2f859ea6aa_34 "#CA-50a497b9421dd2ae5c537338b7af9b2f859ea6aa_34")  ;;(sys::gc)
 [35](#CA-50a497b9421dd2ae5c537338b7af9b2f859ea6aa_35 "#CA-50a497b9421dd2ae5c537338b7af9b2f859ea6aa_35") )
 [36](#CA-50a497b9421dd2ae5c537338b7af9b2f859ea6aa_36 "#CA-50a497b9421dd2ae5c537338b7af9b2f859ea6aa_36") ;(ros::spin)

```
Don't forget to make the node executable: 
```
$ chmod +x euslisp/listener.l
```

#### The Code Explained

The code for listener.l is similar to talker.l, except we've introduced a new callback-based mechanism for subscribing to messages. 

document.write('<a href="#" onclick="return togglenumber(\'CA-c9e23aed46354eac0ea6e53f9f73881f7cb8c63b\', 12, 1);" \
 class="codenumbers">Toggle line numbers<\/a>');

```
 [12](#CA-c9e23aed46354eac0ea6e53f9f73881f7cb8c63b_12 "#CA-c9e23aed46354eac0ea6e53f9f73881f7cb8c63b_12") ;;(setq sys::\*gc-hook\* #'(lambda (a b) (format t ";; gc ~A ~A~%" a b)))
 [13](#CA-c9e23aed46354eac0ea6e53f9f73881f7cb8c63b_13 "#CA-c9e23aed46354eac0ea6e53f9f73881f7cb8c63b_13") 
 [14](#CA-c9e23aed46354eac0ea6e53f9f73881f7cb8c63b_14 "#CA-c9e23aed46354eac0ea6e53f9f73881f7cb8c63b_14") ;; callback function

```
 This declares that your node subscribes to the chatter topic which is of type std\_msgs::String. When new messages are received, callback is invoked with the message as the first argument. We also changed up the call to (ros::roseus ) somewhat. We've added the :anonymous t keyword argument. ROS requires that each node have a unique name. If a node with the same name comes up, it bumps the previous one. This is so that malfunctioning nodes can easily be kicked off the network. The :anonymous t flag tells roseus to generate a unique name for the node so that you can have multiple listener.l nodes run easily. The final addition, (ros::spin) simply keeps your node from exiting until the node has been shutdown. 
### Building your nodes

We use CMake as our build system and, yes, you have to use it even for [EusLisp](/EusLisp "/EusLisp") nodes. This is to make sure that the autogenerated [EusLisp](/EusLisp "/EusLisp") code for messages and services is created. Go to your catkin workspace and run catkin\_make: 
```
$ cd ~/catkin_ws
$ catkin_make
```

### Running Nodes

Make sure that a roscore is up and running: 
```
$ roscore
```
You will see something similar to: 
```
... logging to /u/nkoenig/ros-jaunty/ros/log/d92b213a-90d4-11de-9344-00301b8246bf/roslaunch-ncq-11315.log
... loading XML file [/u/nkoenig/ros-jaunty/ros/tools/roslaunch/roscore.xml]
Added core node of type [rosout/rosout] in namespace [/]
started roslaunch server http://ncq:60287/

SUMMARY
========

NODES

starting new master (master configured for auto start)
process[master]: started with pid [11338]
ROS_MASTER_URI=http://ncq:11311/
setting /run_id to d92b213a-90d4-11de-9344-00301b8246bf
+PARAM [/run_id] by /roslaunch
+PARAM [/roslaunch/uris/ncq:60287] by /roslaunch
process[rosout-1]: started with pid [11353]
started core service [/rosout]
+SERVICE [/rosout/get_loggers] /rosout http://ncq:36277/
+SERVICE [/rosout/set_logger_level] /rosout http://ncq:36277/
+SUB [/time] /rosout http://ncq:36277/
+PUB [/rosout_agg] /rosout http://ncq:36277/
+SUB [/rosout] /rosout http://ncq:36277/
```
Now, we're ready to run talker/listener node. Open a new terminal and type follwings: 
```
$ rosrun beginner_tutorials talker.l
```
Then, open another terminal and type: 
```
$ rosrun beginner_tutorials listener.l
```
rosrun is a simple script to run any node under pacakge, you can directly run the node by typing ./talker.l. Talker will outputs something like: 
```
Registered [/talker-9224-1233892469.83] with master node http://localhost:11311
hello world 1233892469.86
hello world 1233892470.86
hello world 1233892471.86
hello world 1233892472.86
hello world 1233892473.86
...
```
And listener will show: 
```
/listener-7457-1233891102.92: I heard hello world 1233892470.86
/listener-7457-1233891102.92: I heard hello world 1233892471.86
/listener-7457-1233891102.92: I heard hello world 1233892472.86
/listener-7457-1233891102.92: I heard hello world 1233892473.86
/listener-7457-1233891102.92: I heard hello world 1233892474.86
...
```
Now you write your first listener node, you also able to use [rostopic](/rostopic "/rostopic") that listens any type rof topic. If you run rostopic echo topic\_name, you'll see outputs limilar to listener.l. 
```
$ rostopic echo chatter
```

```
rostopic: topic is [/chatter]
topic type is [std_msgs/String]
---
data: hello world 1241463489.67
---
data: hello world 1241463490.67
---
```
Congratulations! You now use an [EusLisp](/EusLisp "/EusLisp") communicating with your ROS nodes! For more depth information, please check [roseus](/roseus "/roseus") package or move to a next tutorial. 

---

Wiki: ROS/Tutorials/WritingPublisherSubscriber(euslisp) (last edited 2014-12-17 05:28:20 by [Kei Okada](/Kei%20Okada "Kei Okada @ kokada-t440s.jsk.imi.i.u-tokyo.ac.jp[133.11.216.162]"))

Except where otherwise noted, the ROS wiki is licensed under the   

