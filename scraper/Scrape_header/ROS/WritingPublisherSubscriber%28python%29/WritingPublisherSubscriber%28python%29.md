

 var \_gaq = \_gaq || [];
 \_gaq.push(['\_setAccount', 'UA-17821189-2']);
 \_gaq.push(['\_setDomainName', 'wiki.ros.org']);
 \_gaq.push(['\_trackPageview']);

 (function() {
 var ga = document.createElement('script'); ga.type = 'text/javascript'; ga.async = true;
 ga.src = ('https:' == document.location.protocol ? 'https://ssl' : 'http://www') + '.google-analytics.com/ga.js';
 var s = document.getElementsByTagName('script')[0]; s.parentNode.insertBefore(ga, s);
 })();

ROS/Tutorials/WritingPublisherSubscriber(python) - ROS Wiki

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
* [WritingPublisherSubscriber(python)](/ROS/Tutorials/WritingPublisherSubscriber%28python%29 "/ROS/Tutorials/WritingPublisherSubscriber%28python%29")

#### ROS 2 Documentation

The ROS Wiki is for ROS 1. Are you using ROS 2 ([Humble](http://docs.ros.org/en/humble/ "http://docs.ros.org/en/humble/"), [Iron](http://docs.ros.org/en/iron/ "http://docs.ros.org/en/iron/"), or [Rolling](http://docs.ros.org/en/rolling/ "http://docs.ros.org/en/rolling/"))?   
[Check out the ROS 2 Project Documentation](http://docs.ros.org "http://docs.ros.org")  
Package specific documentation can be found on [index.ros.org](https://index.ros.org "https://index.ros.org")

# Wiki

* [Distributions](/Distributions "/Distributions")
* [ROS/Installation](/ROS/Installation "/ROS/Installation")
* [ROS/Tutorials](/ROS/Tutorials "/ROS/Tutorials")
* [RecentChanges](/RecentChanges "/RecentChanges")
* [WritingPubl...ber(python)](/ROS/Tutorials/WritingPublisherSubscriber%28python%29 "/ROS/Tutorials/WritingPublisherSubscriber%28python%29")

# Page

* Immutable Page
* [Comments](# "#")
* [Info](/action/info/ROS/Tutorials/WritingPublisherSubscriber%28python%29?action=info "/action/info/ROS/Tutorials/WritingPublisherSubscriber%28python%29?action=info")
* [Attachments](/action/AttachFile/ROS/Tutorials/WritingPublisherSubscriber%28python%29?action=AttachFile "/action/AttachFile/ROS/Tutorials/WritingPublisherSubscriber%28python%29?action=AttachFile")
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

* [Login](/action/login/ROS/Tutorials/WritingPublisherSubscriber%28python%29?action=login "/action/login/ROS/Tutorials/WritingPublisherSubscriber%28python%29?action=login")

|  |
| --- |
| **Note:** This tutorial assumes that you have completed the previous tutorials: [creating a ROS msg and srv](/ROS/Tutorials/CreatingMsgAndSrv "/ROS/Tutorials/CreatingMsgAndSrv").  |

|  |
| --- |
| (!) Please ask about problems and questions regarding this tutorial on [answers.ros.org](http://answers.ros.org "http://answers.ros.org"). Don't forget to include in your question the link to this page, the versions of your OS & ROS, and also add appropriate tags. |

# Writing a Simple Publisher and Subscriber (Python)

**Description:** This tutorial covers how to write a publisher and subscriber node in python.  

**Tutorial Level:** BEGINNER  

**Next Tutorial:** [Examining the simple publisher and subscriber](/ROS/Tutorials/ExaminingPublisherSubscriber "/ROS/Tutorials/ExaminingPublisherSubscriber")   

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
 Contents1. [Writing the Publisher Node](#rospy_tutorials.2FTutorials.2FWritingPublisherSubscriber.Writing_the_Publisher_Node "#rospy_tutorials.2FTutorials.2FWritingPublisherSubscriber.Writing_the_Publisher_Node")
	1. [The Code](#rospy_tutorials.2FTutorials.2FWritingPublisherSubscriber.The_Code "#rospy_tutorials.2FTutorials.2FWritingPublisherSubscriber.The_Code")
	2. [The Code Explained](#rospy_tutorials.2FTutorials.2FWritingPublisherSubscriber.The_Code_Explained "#rospy_tutorials.2FTutorials.2FWritingPublisherSubscriber.The_Code_Explained")
2. [Writing the Subscriber Node](#rospy_tutorials.2FTutorials.2FWritingPublisherSubscriber.Writing_the_Subscriber_Node "#rospy_tutorials.2FTutorials.2FWritingPublisherSubscriber.Writing_the_Subscriber_Node")
	1. [The Code](#rospy_tutorials.2FTutorials.2FWritingPublisherSubscriber.The_Code-1 "#rospy_tutorials.2FTutorials.2FWritingPublisherSubscriber.The_Code-1")
	2. [The Code Explained](#rospy_tutorials.2FTutorials.2FWritingPublisherSubscriber.The_Code_Explained-1 "#rospy_tutorials.2FTutorials.2FWritingPublisherSubscriber.The_Code_Explained-1")
3. [Building your nodes](#rospy_tutorials.2FTutorials.2FWritingPublisherSubscriber.Building_your_nodes "#rospy_tutorials.2FTutorials.2FWritingPublisherSubscriber.Building_your_nodes")
	1. [Additional Resources](#Additional_Resources "#Additional_Resources")
4. [Video Demonstration](#Video_Demonstration "#Video_Demonstration")

## Writing the Publisher Node

"Node" is the ROS term for an executable that is connected to the ROS network. Here we'll create the publisher ("talker") node which will continually broadcast a message. Change directory into the beginner\_tutorials package, you created in the earlier tutorial, [creating a package](/ROS/Tutorials/CreatingPackage "/ROS/Tutorials/CreatingPackage"): 
```
$ roscd beginner_tutorials
```

### The Code

First lets create a 'scripts' folder to store our Python scripts in: 
```
$ mkdir scripts
$ cd scripts
```
Then download the example script [talker.py](https://raw.github.com/ros/ros_tutorials/kinetic-devel/rospy_tutorials/001_talker_listener/talker.py "https://raw.github.com/ros/ros_tutorials/kinetic-devel/rospy_tutorials/001_talker_listener/talker.py") to your new scripts directory and make it executable: 
```
$ wget https://raw.github.com/ros/ros_tutorials/kinetic-devel/rospy_tutorials/001_talker_listener/talker.py
$ chmod +x talker.py
```
We will not run it yet. You can view and edit the file with  $ rosed beginner\_tutorials talker.py  or just look below. 

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

document.write('<a href="#" onclick="return togglenumber(\'rospy\_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-c82832e0d612370fe9886563f0b7f5433f6caee1\', 1, 1);" \
 class="codenumbers">Toggle line numbers<\/a>');

```
 [1](#rospy_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-c82832e0d612370fe9886563f0b7f5433f6caee1_1 "#rospy_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-c82832e0d612370fe9886563f0b7f5433f6caee1_1") #!/usr/bin/env python
 [2](#rospy_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-c82832e0d612370fe9886563f0b7f5433f6caee1_2 "#rospy_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-c82832e0d612370fe9886563f0b7f5433f6caee1_2") # license removed for brevity
 [3](#rospy_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-c82832e0d612370fe9886563f0b7f5433f6caee1_3 "#rospy_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-c82832e0d612370fe9886563f0b7f5433f6caee1_3") import rospy
 [4](#rospy_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-c82832e0d612370fe9886563f0b7f5433f6caee1_4 "#rospy_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-c82832e0d612370fe9886563f0b7f5433f6caee1_4") from std\_msgs.msg import String
 [5](#rospy_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-c82832e0d612370fe9886563f0b7f5433f6caee1_5 "#rospy_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-c82832e0d612370fe9886563f0b7f5433f6caee1_5") 
 [6](#rospy_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-c82832e0d612370fe9886563f0b7f5433f6caee1_6 "#rospy_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-c82832e0d612370fe9886563f0b7f5433f6caee1_6") def talker():
 [7](#rospy_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-c82832e0d612370fe9886563f0b7f5433f6caee1_7 "#rospy_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-c82832e0d612370fe9886563f0b7f5433f6caee1_7")  pub = rospy.Publisher('chatter', String, queue\_size=10)
 [8](#rospy_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-c82832e0d612370fe9886563f0b7f5433f6caee1_8 "#rospy_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-c82832e0d612370fe9886563f0b7f5433f6caee1_8")  rospy.init\_node('talker', anonymous=True)
 [9](#rospy_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-c82832e0d612370fe9886563f0b7f5433f6caee1_9 "#rospy_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-c82832e0d612370fe9886563f0b7f5433f6caee1_9")  rate = rospy.Rate(10) # 10hz
 [10](#rospy_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-c82832e0d612370fe9886563f0b7f5433f6caee1_10 "#rospy_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-c82832e0d612370fe9886563f0b7f5433f6caee1_10")  while not rospy.is\_shutdown():
 [11](#rospy_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-c82832e0d612370fe9886563f0b7f5433f6caee1_11 "#rospy_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-c82832e0d612370fe9886563f0b7f5433f6caee1_11")  hello\_str = "hello world %s" % rospy.get\_time()
 [12](#rospy_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-c82832e0d612370fe9886563f0b7f5433f6caee1_12 "#rospy_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-c82832e0d612370fe9886563f0b7f5433f6caee1_12")  rospy.loginfo(hello\_str)
 [13](#rospy_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-c82832e0d612370fe9886563f0b7f5433f6caee1_13 "#rospy_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-c82832e0d612370fe9886563f0b7f5433f6caee1_13")  pub.publish(hello\_str)
 [14](#rospy_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-c82832e0d612370fe9886563f0b7f5433f6caee1_14 "#rospy_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-c82832e0d612370fe9886563f0b7f5433f6caee1_14")  rate.sleep()
 [15](#rospy_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-c82832e0d612370fe9886563f0b7f5433f6caee1_15 "#rospy_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-c82832e0d612370fe9886563f0b7f5433f6caee1_15") 
 [16](#rospy_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-c82832e0d612370fe9886563f0b7f5433f6caee1_16 "#rospy_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-c82832e0d612370fe9886563f0b7f5433f6caee1_16") if \_\_name\_\_ == '\_\_main\_\_':
 [17](#rospy_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-c82832e0d612370fe9886563f0b7f5433f6caee1_17 "#rospy_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-c82832e0d612370fe9886563f0b7f5433f6caee1_17")  try:
 [18](#rospy_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-c82832e0d612370fe9886563f0b7f5433f6caee1_18 "#rospy_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-c82832e0d612370fe9886563f0b7f5433f6caee1_18")  talker()
 [19](#rospy_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-c82832e0d612370fe9886563f0b7f5433f6caee1_19 "#rospy_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-c82832e0d612370fe9886563f0b7f5433f6caee1_19")  except rospy.ROSInterruptException:
 [20](#rospy_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-c82832e0d612370fe9886563f0b7f5433f6caee1_20 "#rospy_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-c82832e0d612370fe9886563f0b7f5433f6caee1_20")  pass

```
Add the following to your CMakeLists.txt. This makes sure the python script gets installed properly, and uses the right python interpreter. 
```
catkin_install_python(PROGRAMS scripts/talker.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
```

### The Code Explained

Now, let's break the code down. 

document.write('<a href="#" onclick="return togglenumber(\'rospy\_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-da6097954e7fee459e60cb816897ac01d266cf48\', 1, 1);" \
 class="codenumbers">Toggle line numbers<\/a>');

```
 [1](#rospy_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-da6097954e7fee459e60cb816897ac01d266cf48_1 "#rospy_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-da6097954e7fee459e60cb816897ac01d266cf48_1") #!/usr/bin/env python

```
 Every Python ROS [Node](/Nodes "/Nodes") will have this declaration at the top. The first line makes sure your script is executed as a Python script. 

document.write('<a href="#" onclick="return togglenumber(\'rospy\_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-fb2d85bed2cdd0892041958808af40a4b2e2619f\', 3, 1);" \
 class="codenumbers">Toggle line numbers<\/a>');

```
 [3](#rospy_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-fb2d85bed2cdd0892041958808af40a4b2e2619f_3 "#rospy_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-fb2d85bed2cdd0892041958808af40a4b2e2619f_3") import rospy
 [4](#rospy_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-fb2d85bed2cdd0892041958808af40a4b2e2619f_4 "#rospy_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-fb2d85bed2cdd0892041958808af40a4b2e2619f_4") from std\_msgs.msg import String

```
 You need to import rospy if you are writing a ROS [Node](/Nodes "/Nodes"). The std\_msgs.msg import is so that we can reuse the std\_msgs/String message type (a simple string container) for publishing. 

document.write('<a href="#" onclick="return togglenumber(\'rospy\_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-0d784dd4f9d2b50d6bfc6dbddf4a45b58f61bbe1\', 7, 1);" \
 class="codenumbers">Toggle line numbers<\/a>');

```
 [7](#rospy_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-0d784dd4f9d2b50d6bfc6dbddf4a45b58f61bbe1_7 "#rospy_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-0d784dd4f9d2b50d6bfc6dbddf4a45b58f61bbe1_7")  pub = rospy.Publisher('chatter', String, queue\_size=10)
 [8](#rospy_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-0d784dd4f9d2b50d6bfc6dbddf4a45b58f61bbe1_8 "#rospy_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-0d784dd4f9d2b50d6bfc6dbddf4a45b58f61bbe1_8")  rospy.init\_node('talker', anonymous=True)

```
 This section of code defines the talker's interface to the rest of ROS. pub = rospy.Publisher("chatter", String, queue\_size=10) declares that your node is publishing to the chatter topic using the message type String. String here is actually the class std\_msgs.msg.String. The queue\_size argument is New in ROS hydro and limits the amount of queued messages if any subscriber is not receiving them fast enough. In older ROS distributions just omit the argument. The next line, rospy.init\_node(NAME, ...), is very important as it tells rospy the name of your node -- until rospy has this information, it cannot start communicating with the ROS [Master](/Master "/Master"). In this case, your node will take on the name talker. NOTE: the name must be a [base name](/Names "/Names"), i.e. it cannot contain any slashes "/". anonymous = True ensures that your node has a unique name by adding random numbers to the end of NAME. Refer to [Initialization and Shutdown - Initializing your ROS Node](/rospy/Overview/Initialization%20and%20Shutdown#Initializing_your_ROS_Node "/rospy/Overview/Initialization%20and%20Shutdown#Initializing_your_ROS_Node") in the rospy documentation for more information about node initialization options. 

document.write('<a href="#" onclick="return togglenumber(\'rospy\_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-deeb77cb436e1a6b1a92349b80ed4634d8d9587b\', 9, 1);" \
 class="codenumbers">Toggle line numbers<\/a>');

```
 [9](#rospy_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-deeb77cb436e1a6b1a92349b80ed4634d8d9587b_9 "#rospy_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-deeb77cb436e1a6b1a92349b80ed4634d8d9587b_9")  rate = rospy.Rate(10) # 10hz

```
 This line creates a Rate object rate. With the help of its method sleep(), it offers a convenient way for looping at the desired rate. With its argument of 10, we should expect to go through the loop 10 times per second (as long as our processing time does not exceed 1/10th of a second!) 

document.write('<a href="#" onclick="return togglenumber(\'rospy\_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-c119a947845452aa2f89a85d4fb37e402941b0e1\', 10, 1);" \
 class="codenumbers">Toggle line numbers<\/a>');

```
 [10](#rospy_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-c119a947845452aa2f89a85d4fb37e402941b0e1_10 "#rospy_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-c119a947845452aa2f89a85d4fb37e402941b0e1_10")  while not rospy.is\_shutdown():
 [11](#rospy_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-c119a947845452aa2f89a85d4fb37e402941b0e1_11 "#rospy_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-c119a947845452aa2f89a85d4fb37e402941b0e1_11")  hello\_str = "hello world %s" % rospy.get\_time()
 [12](#rospy_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-c119a947845452aa2f89a85d4fb37e402941b0e1_12 "#rospy_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-c119a947845452aa2f89a85d4fb37e402941b0e1_12")  rospy.loginfo(hello\_str)
 [13](#rospy_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-c119a947845452aa2f89a85d4fb37e402941b0e1_13 "#rospy_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-c119a947845452aa2f89a85d4fb37e402941b0e1_13")  pub.publish(hello\_str)
 [14](#rospy_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-c119a947845452aa2f89a85d4fb37e402941b0e1_14 "#rospy_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-c119a947845452aa2f89a85d4fb37e402941b0e1_14")  rate.sleep()

```
 This loop is a fairly standard rospy construct: checking the rospy.is\_shutdown() flag and then doing work. You have to check is\_shutdown() to check if your program should exit (e.g. if there is a Ctrl-C or otherwise). In this case, the "work" is a call to pub.publish(hello\_str) that publishes a string to our chatter topic. The loop calls rate.sleep(), which sleeps just long enough to maintain the desired rate through the loop. (You may also run across rospy.sleep() which is similar to time.sleep() except that it works with simulated time as well (see [Clock](/Clock "/Clock")).) This loop also calls rospy.loginfo(str), which performs triple-duty: the messages get printed to screen, it gets written to the Node's log file, and it gets written to [rosout](/rosout "/rosout"). [rosout](/rosout "/rosout") is a handy tool for debugging: you can pull up messages using [rqt\_console](/rqt_console "/rqt_console") instead of having to find the console window with your Node's output. std\_msgs.msg.String is a very simple message type, so you may be wondering what it looks like to publish more complicated types. The general rule of thumb is that *constructor args are in the same order as in the .msg file*. You can also pass in no arguments and initialize the fields directly, e.g. 
```
msg = String()
msg.data = str
```
or you can initialize some of the fields and leave the rest with default values: 
```
String(data=str)
```
You may be wondering about the last little bit: 

document.write('<a href="#" onclick="return togglenumber(\'rospy\_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-03913aa01eb4dc084a8831e8982cf5d7c49e193f\', 17, 1);" \
 class="codenumbers">Toggle line numbers<\/a>');

```
 [17](#rospy_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-03913aa01eb4dc084a8831e8982cf5d7c49e193f_17 "#rospy_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-03913aa01eb4dc084a8831e8982cf5d7c49e193f_17")  try:
 [18](#rospy_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-03913aa01eb4dc084a8831e8982cf5d7c49e193f_18 "#rospy_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-03913aa01eb4dc084a8831e8982cf5d7c49e193f_18")  talker()
 [19](#rospy_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-03913aa01eb4dc084a8831e8982cf5d7c49e193f_19 "#rospy_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-03913aa01eb4dc084a8831e8982cf5d7c49e193f_19")  except rospy.ROSInterruptException:
 [20](#rospy_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-03913aa01eb4dc084a8831e8982cf5d7c49e193f_20 "#rospy_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-03913aa01eb4dc084a8831e8982cf5d7c49e193f_20")  pass

```
 In addition to the standard Python \_\_main\_\_ check, this catches a rospy.ROSInterruptException exception, which can be thrown by rospy.sleep() and rospy.Rate.sleep() methods when Ctrl-C is pressed or your Node is otherwise shutdown. The reason this exception is raised is so that you don't accidentally continue executing code after the sleep(). Now we need to write a node to receive the messages. 
## Writing the Subscriber Node

### The Code

Download the [listener.py](https://raw.github.com/ros/ros_tutorials/kinetic-devel/rospy_tutorials/001_talker_listener/listener.py "https://raw.github.com/ros/ros_tutorials/kinetic-devel/rospy_tutorials/001_talker_listener/listener.py") file into your scripts directory: 
```
$ roscd beginner_tutorials/scripts/
$ wget https://raw.github.com/ros/ros_tutorials/kinetic-devel/rospy_tutorials/001_talker_listener/listener.py
$ chmod +x listener.py
```
The file contents look close to: 

document.write('<a href="#" onclick="return togglenumber(\'rospy\_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-56fb82d4681d7880a3d3a97f7af70cfe17618f86\', 1, 1);" \
 class="codenumbers">Toggle line numbers<\/a>');

```
 [1](#rospy_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-56fb82d4681d7880a3d3a97f7af70cfe17618f86_1 "#rospy_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-56fb82d4681d7880a3d3a97f7af70cfe17618f86_1") #!/usr/bin/env python
 [2](#rospy_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-56fb82d4681d7880a3d3a97f7af70cfe17618f86_2 "#rospy_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-56fb82d4681d7880a3d3a97f7af70cfe17618f86_2") import rospy
 [3](#rospy_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-56fb82d4681d7880a3d3a97f7af70cfe17618f86_3 "#rospy_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-56fb82d4681d7880a3d3a97f7af70cfe17618f86_3") from std\_msgs.msg import String
 [4](#rospy_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-56fb82d4681d7880a3d3a97f7af70cfe17618f86_4 "#rospy_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-56fb82d4681d7880a3d3a97f7af70cfe17618f86_4") 
 [5](#rospy_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-56fb82d4681d7880a3d3a97f7af70cfe17618f86_5 "#rospy_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-56fb82d4681d7880a3d3a97f7af70cfe17618f86_5") def callback(data):
 [6](#rospy_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-56fb82d4681d7880a3d3a97f7af70cfe17618f86_6 "#rospy_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-56fb82d4681d7880a3d3a97f7af70cfe17618f86_6")  rospy.loginfo(rospy.get\_caller\_id() + "I heard %s", data.data)
 [7](#rospy_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-56fb82d4681d7880a3d3a97f7af70cfe17618f86_7 "#rospy_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-56fb82d4681d7880a3d3a97f7af70cfe17618f86_7")  
 [8](#rospy_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-56fb82d4681d7880a3d3a97f7af70cfe17618f86_8 "#rospy_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-56fb82d4681d7880a3d3a97f7af70cfe17618f86_8") def listener():
 [9](#rospy_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-56fb82d4681d7880a3d3a97f7af70cfe17618f86_9 "#rospy_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-56fb82d4681d7880a3d3a97f7af70cfe17618f86_9") 
 [10](#rospy_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-56fb82d4681d7880a3d3a97f7af70cfe17618f86_10 "#rospy_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-56fb82d4681d7880a3d3a97f7af70cfe17618f86_10")  # In ROS, nodes are uniquely named. If two nodes with the same
 [11](#rospy_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-56fb82d4681d7880a3d3a97f7af70cfe17618f86_11 "#rospy_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-56fb82d4681d7880a3d3a97f7af70cfe17618f86_11")  # name are launched, the previous one is kicked off. The
 [12](#rospy_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-56fb82d4681d7880a3d3a97f7af70cfe17618f86_12 "#rospy_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-56fb82d4681d7880a3d3a97f7af70cfe17618f86_12")  # anonymous=True flag means that rospy will choose a unique
 [13](#rospy_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-56fb82d4681d7880a3d3a97f7af70cfe17618f86_13 "#rospy_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-56fb82d4681d7880a3d3a97f7af70cfe17618f86_13")  # name for our 'listener' node so that multiple listeners can
 [14](#rospy_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-56fb82d4681d7880a3d3a97f7af70cfe17618f86_14 "#rospy_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-56fb82d4681d7880a3d3a97f7af70cfe17618f86_14")  # run simultaneously.
 [15](#rospy_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-56fb82d4681d7880a3d3a97f7af70cfe17618f86_15 "#rospy_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-56fb82d4681d7880a3d3a97f7af70cfe17618f86_15")  rospy.init\_node('listener', anonymous=True)
 [16](#rospy_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-56fb82d4681d7880a3d3a97f7af70cfe17618f86_16 "#rospy_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-56fb82d4681d7880a3d3a97f7af70cfe17618f86_16") 
 [17](#rospy_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-56fb82d4681d7880a3d3a97f7af70cfe17618f86_17 "#rospy_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-56fb82d4681d7880a3d3a97f7af70cfe17618f86_17")  rospy.Subscriber("chatter", String, callback)
 [18](#rospy_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-56fb82d4681d7880a3d3a97f7af70cfe17618f86_18 "#rospy_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-56fb82d4681d7880a3d3a97f7af70cfe17618f86_18") 
 [19](#rospy_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-56fb82d4681d7880a3d3a97f7af70cfe17618f86_19 "#rospy_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-56fb82d4681d7880a3d3a97f7af70cfe17618f86_19")  # spin() simply keeps python from exiting until this node is stopped
 [20](#rospy_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-56fb82d4681d7880a3d3a97f7af70cfe17618f86_20 "#rospy_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-56fb82d4681d7880a3d3a97f7af70cfe17618f86_20")  rospy.spin()
 [21](#rospy_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-56fb82d4681d7880a3d3a97f7af70cfe17618f86_21 "#rospy_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-56fb82d4681d7880a3d3a97f7af70cfe17618f86_21") 
 [22](#rospy_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-56fb82d4681d7880a3d3a97f7af70cfe17618f86_22 "#rospy_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-56fb82d4681d7880a3d3a97f7af70cfe17618f86_22") if \_\_name\_\_ == '\_\_main\_\_':
 [23](#rospy_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-56fb82d4681d7880a3d3a97f7af70cfe17618f86_23 "#rospy_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-56fb82d4681d7880a3d3a97f7af70cfe17618f86_23")  listener()

```
Then, edit the catkin\_install\_python() call in your CMakeLists.txt so it looks like the following: 
```
catkin_install_python(PROGRAMS scripts/talker.py scripts/listener.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
```

### The Code Explained

The code for listener.py is similar to talker.py, except we've introduced a new callback-based mechanism for subscribing to messages. 

document.write('<a href="#" onclick="return togglenumber(\'rospy\_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-c54953afb3f7ac20abfe7460208d0cd6c36c1d62\', 15, 1);" \
 class="codenumbers">Toggle line numbers<\/a>');

```
 [15](#rospy_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-c54953afb3f7ac20abfe7460208d0cd6c36c1d62_15 "#rospy_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-c54953afb3f7ac20abfe7460208d0cd6c36c1d62_15")  rospy.init\_node('listener', anonymous=True)
 [16](#rospy_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-c54953afb3f7ac20abfe7460208d0cd6c36c1d62_16 "#rospy_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-c54953afb3f7ac20abfe7460208d0cd6c36c1d62_16") 
 [17](#rospy_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-c54953afb3f7ac20abfe7460208d0cd6c36c1d62_17 "#rospy_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-c54953afb3f7ac20abfe7460208d0cd6c36c1d62_17")  rospy.Subscriber("chatter", String, callback)
 [18](#rospy_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-c54953afb3f7ac20abfe7460208d0cd6c36c1d62_18 "#rospy_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-c54953afb3f7ac20abfe7460208d0cd6c36c1d62_18") 
 [19](#rospy_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-c54953afb3f7ac20abfe7460208d0cd6c36c1d62_19 "#rospy_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-c54953afb3f7ac20abfe7460208d0cd6c36c1d62_19")  # spin() simply keeps python from exiting until this node is stopped
 [20](#rospy_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-c54953afb3f7ac20abfe7460208d0cd6c36c1d62_20 "#rospy_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-c54953afb3f7ac20abfe7460208d0cd6c36c1d62_20")  rospy.spin()

```
 This declares that your node subscribes to the chatter topic which is of type std\_msgs.msgs.String. When new messages are received, callback is invoked with the message as the first argument. We also changed up the call to rospy.init\_node() somewhat. We've added the anonymous=True keyword argument. ROS requires that each node have a unique name. If a node with the same name comes up, it bumps the previous one. This is so that malfunctioning nodes can easily be kicked off the network. The anonymous=True flag tells rospy to generate a unique name for the node so that you can have multiple listener.py nodes run easily. The final addition, rospy.spin() simply keeps your node from exiting until the node has been shutdown. Unlike roscpp, rospy.spin() does not affect the subscriber callback functions, as those have their own threads. 
## Building your nodes

We use CMake as our build system and, yes, you have to use it even for Python nodes. This is to make sure that the autogenerated Python code for messages and services is created. We also use a Makefile for a bit of convenience. roscreate-pkg automatically created a Makefile, so you don't have to edit it. Now run make: 
```
$ make
```

Go to your catkin workspace and run catkin\_make: 
```
$ cd ~/catkin_ws
$ catkin_make
```

 Now that you have written a simple publisher and subscriber, let's [examine the simple publisher and subscriber](/ROS/Tutorials/ExaminingPublisherSubscriber "/ROS/Tutorials/ExaminingPublisherSubscriber"). 
### Additional Resources

Here are some additional resources contributed by the community. 
## Video Demonstration

Watch the video below to have more explanation on Python Nodes Communication and step by step guide .  

Wiki: ROS/Tutorials/WritingPublisherSubscriber(python) (last edited 2022-10-18 16:06:01 by [Muhammad Luqman](/Muhammad%20Luqman "Muhammad Luqman @ 103.138.11.4[103.138.11.4]"))

Except where otherwise noted, the ROS wiki is licensed under the   

