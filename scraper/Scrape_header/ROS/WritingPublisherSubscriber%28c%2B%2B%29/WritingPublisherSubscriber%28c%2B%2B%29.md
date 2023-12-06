

 var \_gaq = \_gaq || [];
 \_gaq.push(['\_setAccount', 'UA-17821189-2']);
 \_gaq.push(['\_setDomainName', 'wiki.ros.org']);
 \_gaq.push(['\_trackPageview']);

 (function() {
 var ga = document.createElement('script'); ga.type = 'text/javascript'; ga.async = true;
 ga.src = ('https:' == document.location.protocol ? 'https://ssl' : 'http://www') + '.google-analytics.com/ga.js';
 var s = document.getElementsByTagName('script')[0]; s.parentNode.insertBefore(ga, s);
 })();

ROS/Tutorials/WritingPublisherSubscriber(c++) - ROS Wiki

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
* [WritingPublisherSubscriber(c++)](/action/fullsearch/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29?action=fullsearch&context=180&value=linkto%3A%22ROS%2FTutorials%2FWritingPublisherSubscriber%28c%2B%2B%29%22 "Click to do a full-text search for this title")

#### ROS 2 Documentation

The ROS Wiki is for ROS 1. Are you using ROS 2 ([Humble](http://docs.ros.org/en/humble/ "http://docs.ros.org/en/humble/"), [Iron](http://docs.ros.org/en/iron/ "http://docs.ros.org/en/iron/"), or [Rolling](http://docs.ros.org/en/rolling/ "http://docs.ros.org/en/rolling/"))?   
[Check out the ROS 2 Project Documentation](http://docs.ros.org "http://docs.ros.org")  
Package specific documentation can be found on [index.ros.org](https://index.ros.org "https://index.ros.org")

# Wiki

* [Distributions](/Distributions "/Distributions")
* [ROS/Installation](/ROS/Installation "/ROS/Installation")
* [ROS/Tutorials](/ROS/Tutorials "/ROS/Tutorials")
* [RecentChanges](/RecentChanges "/RecentChanges")
* [WritingPubl...criber(c++)](/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29 "/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29")

# Page

* Immutable Page
* [Comments](# "#")
* [Info](/action/info/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29?action=info "/action/info/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29?action=info")
* [Attachments](/action/AttachFile/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29?action=AttachFile "/action/AttachFile/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29?action=AttachFile")
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

* [Login](/action/login/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29?action=login "/action/login/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29?action=login")

|  |
| --- |
| **Note:** This tutorial assumes that you have completed the previous tutorials: [understanding ROS services and parameters](/ROS/Tutorials/UnderstandingServicesParams "/ROS/Tutorials/UnderstandingServicesParams").  |

|  |
| --- |
| (!) Please ask about problems and questions regarding this tutorial on [answers.ros.org](http://answers.ros.org "http://answers.ros.org"). Don't forget to include in your question the link to this page, the versions of your OS & ROS, and also add appropriate tags. |

# Writing a Simple Publisher and Subscriber (C++)

**Description:** This tutorial covers how to write a publisher and subscriber node in C++.  

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
 Contents1. [Writing the Publisher Node](#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.Writing_the_Publisher_Node "#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.Writing_the_Publisher_Node")
	1. [The Code](#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.The_Code "#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.The_Code")
	2. [The Code Explained](#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.The_Code_Explained "#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.The_Code_Explained")
2. [Writing the Subscriber Node](#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.Writing_the_Subscriber_Node "#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.Writing_the_Subscriber_Node")
	1. [The Code](#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.The_Code-1 "#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.The_Code-1")
	2. [The Code Explained](#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.The_Code_Explained-1 "#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.The_Code_Explained-1")
3. [Building your nodes](#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.Building_your_nodes "#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.Building_your_nodes")
4. [Building your nodes](#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.Building_your_nodes-1 "#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.Building_your_nodes-1")
5. [Additional Resources](#Additional_Resources "#Additional_Resources")
	1. [Video Tutorial](#Video_Tutorial "#Video_Tutorial")

## Writing the Publisher Node

"Node" is the ROS term for an executable that is connected to the ROS network. Here we'll create a publisher ("talker") node which will continually broadcast a message. Change directory into the beginner\_tutorials package, you created previously in the [creating a rosbuild package](/ROS/Tutorials/CreatingPackage "/ROS/Tutorials/CreatingPackage") tutorial: 
```
roscd beginner_tutorials
```

Change directories to your beginner\_tutorials package you created in your catkin workspace previous tutorials: 
```
roscd beginner_tutorials
```

### The Code

Create a src directory in the beginner\_tutorials package directory: 
```
mkdir -p src
```
This directory will contain any source files for our beginner\_tutorials package. 

Create the src/talker.cpp file within the beginner\_tutorials package and paste the following inside it: *[https://raw.github.com/ros/ros\_tutorials/kinetic-devel/roscpp\_tutorials/talker/talker.cpp](https://raw.github.com/ros/ros_tutorials/kinetic-devel/roscpp_tutorials/talker/talker.cpp "https://raw.github.com/ros/ros_tutorials/kinetic-devel/roscpp_tutorials/talker/talker.cpp")* 

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

document.write('<a href="#" onclick="return togglenumber(\'roscpp\_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-7875e5af8250b639212f51b1a24ec13990321f5b\', 27, 1);" \
 class="codenumbers">Toggle line numbers<\/a>');

```
 [27](#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-7875e5af8250b639212f51b1a24ec13990321f5b_27 "#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-7875e5af8250b639212f51b1a24ec13990321f5b_27") #include "ros/ros.h"
 [28](#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-7875e5af8250b639212f51b1a24ec13990321f5b_28 "#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-7875e5af8250b639212f51b1a24ec13990321f5b_28") #include "std\_msgs/String.h"
 [29](#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-7875e5af8250b639212f51b1a24ec13990321f5b_29 "#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-7875e5af8250b639212f51b1a24ec13990321f5b_29") 
 [30](#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-7875e5af8250b639212f51b1a24ec13990321f5b_30 "#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-7875e5af8250b639212f51b1a24ec13990321f5b_30") #include <sstream>
 [31](#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-7875e5af8250b639212f51b1a24ec13990321f5b_31 "#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-7875e5af8250b639212f51b1a24ec13990321f5b_31") 
 [32](#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-7875e5af8250b639212f51b1a24ec13990321f5b_32 "#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-7875e5af8250b639212f51b1a24ec13990321f5b_32") /\*\*
 [33](#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-7875e5af8250b639212f51b1a24ec13990321f5b_33 "#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-7875e5af8250b639212f51b1a24ec13990321f5b_33")  \* This tutorial demonstrates simple sending of messages over the ROS system.
 [34](#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-7875e5af8250b639212f51b1a24ec13990321f5b_34 "#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-7875e5af8250b639212f51b1a24ec13990321f5b_34")  \*/
 [35](#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-7875e5af8250b639212f51b1a24ec13990321f5b_35 "#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-7875e5af8250b639212f51b1a24ec13990321f5b_35") int main(int argc, char \*\*argv)
 [36](#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-7875e5af8250b639212f51b1a24ec13990321f5b_36 "#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-7875e5af8250b639212f51b1a24ec13990321f5b_36") {
 [37](#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-7875e5af8250b639212f51b1a24ec13990321f5b_37 "#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-7875e5af8250b639212f51b1a24ec13990321f5b_37")  /\*\*
 [38](#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-7875e5af8250b639212f51b1a24ec13990321f5b_38 "#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-7875e5af8250b639212f51b1a24ec13990321f5b_38")  \* The ros::init() function needs to see argc and argv so that it can perform
 [39](#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-7875e5af8250b639212f51b1a24ec13990321f5b_39 "#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-7875e5af8250b639212f51b1a24ec13990321f5b_39")  \* any ROS arguments and name remapping that were provided at the command line.
 [40](#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-7875e5af8250b639212f51b1a24ec13990321f5b_40 "#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-7875e5af8250b639212f51b1a24ec13990321f5b_40")  \* For programmatic remappings you can use a different version of init() which takes
 [41](#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-7875e5af8250b639212f51b1a24ec13990321f5b_41 "#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-7875e5af8250b639212f51b1a24ec13990321f5b_41")  \* remappings directly, but for most command-line programs, passing argc and argv is
 [42](#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-7875e5af8250b639212f51b1a24ec13990321f5b_42 "#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-7875e5af8250b639212f51b1a24ec13990321f5b_42")  \* the easiest way to do it. The third argument to init() is the name of the node.
 [43](#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-7875e5af8250b639212f51b1a24ec13990321f5b_43 "#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-7875e5af8250b639212f51b1a24ec13990321f5b_43")  \*
 [44](#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-7875e5af8250b639212f51b1a24ec13990321f5b_44 "#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-7875e5af8250b639212f51b1a24ec13990321f5b_44")  \* You must call one of the versions of ros::init() before using any other
 [45](#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-7875e5af8250b639212f51b1a24ec13990321f5b_45 "#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-7875e5af8250b639212f51b1a24ec13990321f5b_45")  \* part of the ROS system.
 [46](#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-7875e5af8250b639212f51b1a24ec13990321f5b_46 "#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-7875e5af8250b639212f51b1a24ec13990321f5b_46")  \*/
 [47](#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-7875e5af8250b639212f51b1a24ec13990321f5b_47 "#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-7875e5af8250b639212f51b1a24ec13990321f5b_47")  ros::init(argc, argv, "talker");
 [48](#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-7875e5af8250b639212f51b1a24ec13990321f5b_48 "#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-7875e5af8250b639212f51b1a24ec13990321f5b_48") 
 [49](#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-7875e5af8250b639212f51b1a24ec13990321f5b_49 "#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-7875e5af8250b639212f51b1a24ec13990321f5b_49")  /\*\*
 [50](#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-7875e5af8250b639212f51b1a24ec13990321f5b_50 "#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-7875e5af8250b639212f51b1a24ec13990321f5b_50")  \* NodeHandle is the main access point to communications with the ROS system.
 [51](#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-7875e5af8250b639212f51b1a24ec13990321f5b_51 "#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-7875e5af8250b639212f51b1a24ec13990321f5b_51")  \* The first NodeHandle constructed will fully initialize this node, and the last
 [52](#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-7875e5af8250b639212f51b1a24ec13990321f5b_52 "#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-7875e5af8250b639212f51b1a24ec13990321f5b_52")  \* NodeHandle destructed will close down the node.
 [53](#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-7875e5af8250b639212f51b1a24ec13990321f5b_53 "#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-7875e5af8250b639212f51b1a24ec13990321f5b_53")  \*/
 [54](#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-7875e5af8250b639212f51b1a24ec13990321f5b_54 "#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-7875e5af8250b639212f51b1a24ec13990321f5b_54")  ros::NodeHandle n;
 [55](#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-7875e5af8250b639212f51b1a24ec13990321f5b_55 "#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-7875e5af8250b639212f51b1a24ec13990321f5b_55") 
 [56](#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-7875e5af8250b639212f51b1a24ec13990321f5b_56 "#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-7875e5af8250b639212f51b1a24ec13990321f5b_56")  /\*\*
 [57](#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-7875e5af8250b639212f51b1a24ec13990321f5b_57 "#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-7875e5af8250b639212f51b1a24ec13990321f5b_57")  \* The advertise() function is how you tell ROS that you want to
 [58](#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-7875e5af8250b639212f51b1a24ec13990321f5b_58 "#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-7875e5af8250b639212f51b1a24ec13990321f5b_58")  \* publish on a given topic name. This invokes a call to the ROS
 [59](#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-7875e5af8250b639212f51b1a24ec13990321f5b_59 "#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-7875e5af8250b639212f51b1a24ec13990321f5b_59")  \* master node, which keeps a registry of who is publishing and who
 [60](#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-7875e5af8250b639212f51b1a24ec13990321f5b_60 "#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-7875e5af8250b639212f51b1a24ec13990321f5b_60")  \* is subscribing. After this advertise() call is made, the master
 [61](#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-7875e5af8250b639212f51b1a24ec13990321f5b_61 "#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-7875e5af8250b639212f51b1a24ec13990321f5b_61")  \* node will notify anyone who is trying to subscribe to this topic name,
 [62](#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-7875e5af8250b639212f51b1a24ec13990321f5b_62 "#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-7875e5af8250b639212f51b1a24ec13990321f5b_62")  \* and they will in turn negotiate a peer-to-peer connection with this
 [63](#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-7875e5af8250b639212f51b1a24ec13990321f5b_63 "#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-7875e5af8250b639212f51b1a24ec13990321f5b_63")  \* node. advertise() returns a Publisher object which allows you to
 [64](#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-7875e5af8250b639212f51b1a24ec13990321f5b_64 "#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-7875e5af8250b639212f51b1a24ec13990321f5b_64")  \* publish messages on that topic through a call to publish(). Once
 [65](#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-7875e5af8250b639212f51b1a24ec13990321f5b_65 "#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-7875e5af8250b639212f51b1a24ec13990321f5b_65")  \* all copies of the returned Publisher object are destroyed, the topic
 [66](#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-7875e5af8250b639212f51b1a24ec13990321f5b_66 "#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-7875e5af8250b639212f51b1a24ec13990321f5b_66")  \* will be automatically unadvertised.
 [67](#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-7875e5af8250b639212f51b1a24ec13990321f5b_67 "#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-7875e5af8250b639212f51b1a24ec13990321f5b_67")  \*
 [68](#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-7875e5af8250b639212f51b1a24ec13990321f5b_68 "#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-7875e5af8250b639212f51b1a24ec13990321f5b_68")  \* The second parameter to advertise() is the size of the message queue
 [69](#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-7875e5af8250b639212f51b1a24ec13990321f5b_69 "#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-7875e5af8250b639212f51b1a24ec13990321f5b_69")  \* used for publishing messages. If messages are published more quickly
 [70](#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-7875e5af8250b639212f51b1a24ec13990321f5b_70 "#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-7875e5af8250b639212f51b1a24ec13990321f5b_70")  \* than we can send them, the number here specifies how many messages to
 [71](#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-7875e5af8250b639212f51b1a24ec13990321f5b_71 "#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-7875e5af8250b639212f51b1a24ec13990321f5b_71")  \* buffer up before throwing some away.
 [72](#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-7875e5af8250b639212f51b1a24ec13990321f5b_72 "#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-7875e5af8250b639212f51b1a24ec13990321f5b_72")  \*/
 [73](#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-7875e5af8250b639212f51b1a24ec13990321f5b_73 "#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-7875e5af8250b639212f51b1a24ec13990321f5b_73")  ros::Publisher chatter\_pub = n.advertise<std\_msgs::String>("chatter", 1000);
 [74](#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-7875e5af8250b639212f51b1a24ec13990321f5b_74 "#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-7875e5af8250b639212f51b1a24ec13990321f5b_74") 
 [75](#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-7875e5af8250b639212f51b1a24ec13990321f5b_75 "#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-7875e5af8250b639212f51b1a24ec13990321f5b_75")  ros::Rate loop\_rate(10);
 [76](#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-7875e5af8250b639212f51b1a24ec13990321f5b_76 "#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-7875e5af8250b639212f51b1a24ec13990321f5b_76") 
 [77](#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-7875e5af8250b639212f51b1a24ec13990321f5b_77 "#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-7875e5af8250b639212f51b1a24ec13990321f5b_77")  /\*\*
 [78](#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-7875e5af8250b639212f51b1a24ec13990321f5b_78 "#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-7875e5af8250b639212f51b1a24ec13990321f5b_78")  \* A count of how many messages we have sent. This is used to create
 [79](#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-7875e5af8250b639212f51b1a24ec13990321f5b_79 "#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-7875e5af8250b639212f51b1a24ec13990321f5b_79")  \* a unique string for each message.
 [80](#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-7875e5af8250b639212f51b1a24ec13990321f5b_80 "#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-7875e5af8250b639212f51b1a24ec13990321f5b_80")  \*/
 [81](#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-7875e5af8250b639212f51b1a24ec13990321f5b_81 "#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-7875e5af8250b639212f51b1a24ec13990321f5b_81")  int count = 0;
 [82](#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-7875e5af8250b639212f51b1a24ec13990321f5b_82 "#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-7875e5af8250b639212f51b1a24ec13990321f5b_82")  while (ros::ok())
 [83](#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-7875e5af8250b639212f51b1a24ec13990321f5b_83 "#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-7875e5af8250b639212f51b1a24ec13990321f5b_83")  {
 [84](#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-7875e5af8250b639212f51b1a24ec13990321f5b_84 "#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-7875e5af8250b639212f51b1a24ec13990321f5b_84")  /\*\*
 [85](#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-7875e5af8250b639212f51b1a24ec13990321f5b_85 "#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-7875e5af8250b639212f51b1a24ec13990321f5b_85")  \* This is a message object. You stuff it with data, and then publish it.
 [86](#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-7875e5af8250b639212f51b1a24ec13990321f5b_86 "#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-7875e5af8250b639212f51b1a24ec13990321f5b_86")  \*/
 [87](#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-7875e5af8250b639212f51b1a24ec13990321f5b_87 "#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-7875e5af8250b639212f51b1a24ec13990321f5b_87")  std\_msgs::String msg;
 [88](#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-7875e5af8250b639212f51b1a24ec13990321f5b_88 "#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-7875e5af8250b639212f51b1a24ec13990321f5b_88") 
 [89](#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-7875e5af8250b639212f51b1a24ec13990321f5b_89 "#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-7875e5af8250b639212f51b1a24ec13990321f5b_89")  std::stringstream ss;
 [90](#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-7875e5af8250b639212f51b1a24ec13990321f5b_90 "#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-7875e5af8250b639212f51b1a24ec13990321f5b_90")  ss << "hello world " << count;
 [91](#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-7875e5af8250b639212f51b1a24ec13990321f5b_91 "#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-7875e5af8250b639212f51b1a24ec13990321f5b_91")  msg.data = ss.str();
 [92](#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-7875e5af8250b639212f51b1a24ec13990321f5b_92 "#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-7875e5af8250b639212f51b1a24ec13990321f5b_92") 
 [93](#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-7875e5af8250b639212f51b1a24ec13990321f5b_93 "#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-7875e5af8250b639212f51b1a24ec13990321f5b_93")  ROS\_INFO("%s", msg.data.c\_str());
 [94](#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-7875e5af8250b639212f51b1a24ec13990321f5b_94 "#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-7875e5af8250b639212f51b1a24ec13990321f5b_94") 
 [95](#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-7875e5af8250b639212f51b1a24ec13990321f5b_95 "#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-7875e5af8250b639212f51b1a24ec13990321f5b_95")  /\*\*
 [96](#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-7875e5af8250b639212f51b1a24ec13990321f5b_96 "#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-7875e5af8250b639212f51b1a24ec13990321f5b_96")  \* The publish() function is how you send messages. The parameter
 [97](#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-7875e5af8250b639212f51b1a24ec13990321f5b_97 "#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-7875e5af8250b639212f51b1a24ec13990321f5b_97")  \* is the message object. The type of this object must agree with the type
 [98](#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-7875e5af8250b639212f51b1a24ec13990321f5b_98 "#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-7875e5af8250b639212f51b1a24ec13990321f5b_98")  \* given as a template parameter to the advertise<>() call, as was done
 [99](#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-7875e5af8250b639212f51b1a24ec13990321f5b_99 "#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-7875e5af8250b639212f51b1a24ec13990321f5b_99")  \* in the constructor above.
 [100](#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-7875e5af8250b639212f51b1a24ec13990321f5b_100 "#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-7875e5af8250b639212f51b1a24ec13990321f5b_100")  \*/
 [101](#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-7875e5af8250b639212f51b1a24ec13990321f5b_101 "#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-7875e5af8250b639212f51b1a24ec13990321f5b_101")  chatter\_pub.publish(msg);
 [102](#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-7875e5af8250b639212f51b1a24ec13990321f5b_102 "#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-7875e5af8250b639212f51b1a24ec13990321f5b_102") 
 [103](#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-7875e5af8250b639212f51b1a24ec13990321f5b_103 "#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-7875e5af8250b639212f51b1a24ec13990321f5b_103")  ros::spinOnce();
 [104](#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-7875e5af8250b639212f51b1a24ec13990321f5b_104 "#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-7875e5af8250b639212f51b1a24ec13990321f5b_104") 
 [105](#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-7875e5af8250b639212f51b1a24ec13990321f5b_105 "#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-7875e5af8250b639212f51b1a24ec13990321f5b_105")  loop\_rate.sleep();
 [106](#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-7875e5af8250b639212f51b1a24ec13990321f5b_106 "#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-7875e5af8250b639212f51b1a24ec13990321f5b_106")  ++count;
 [107](#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-7875e5af8250b639212f51b1a24ec13990321f5b_107 "#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-7875e5af8250b639212f51b1a24ec13990321f5b_107")  }
 [108](#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-7875e5af8250b639212f51b1a24ec13990321f5b_108 "#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-7875e5af8250b639212f51b1a24ec13990321f5b_108") 
 [109](#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-7875e5af8250b639212f51b1a24ec13990321f5b_109 "#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-7875e5af8250b639212f51b1a24ec13990321f5b_109") 
 [110](#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-7875e5af8250b639212f51b1a24ec13990321f5b_110 "#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-7875e5af8250b639212f51b1a24ec13990321f5b_110")  return 0;
 [111](#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-7875e5af8250b639212f51b1a24ec13990321f5b_111 "#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-7875e5af8250b639212f51b1a24ec13990321f5b_111") }

```

### The Code Explained

Now, let's break the code down. 

document.write('<a href="#" onclick="return togglenumber(\'roscpp\_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-8c1080856928989e4b4a21f76a6dbdb47f49196f\', 27, 1);" \
 class="codenumbers">Toggle line numbers<\/a>');

```
 [27](#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-8c1080856928989e4b4a21f76a6dbdb47f49196f_27 "#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-8c1080856928989e4b4a21f76a6dbdb47f49196f_27") #include "ros/ros.h"
 [28](#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-8c1080856928989e4b4a21f76a6dbdb47f49196f_28 "#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-8c1080856928989e4b4a21f76a6dbdb47f49196f_28") 

```
 ros/ros.h is a convenience include that includes all the headers necessary to use the most common public pieces of the ROS system. 

document.write('<a href="#" onclick="return togglenumber(\'roscpp\_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-dcabc2a2d315305bc1e9eb823364cbdade421375\', 28, 1);" \
 class="codenumbers">Toggle line numbers<\/a>');

```
 [28](#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-dcabc2a2d315305bc1e9eb823364cbdade421375_28 "#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-dcabc2a2d315305bc1e9eb823364cbdade421375_28") #include "std\_msgs/String.h"
 [29](#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-dcabc2a2d315305bc1e9eb823364cbdade421375_29 "#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-dcabc2a2d315305bc1e9eb823364cbdade421375_29") 

```
 This includes the [std\_msgs/String](http://docs.ros.org/en/api/std_msgs/html/msg/String.html "http://docs.ros.org/en/api/std_msgs/html/msg/String.html") message, which resides in the [std\_msgs](/std_msgs "/std_msgs") package. This is a header generated automatically from the String.msg file in that package. For more information on message definitions, see the [msg](/msg "/msg") page. 

document.write('<a href="#" onclick="return togglenumber(\'roscpp\_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-bbd7c4e67305a8926e488e6a5fa5695747e525f9\', 47, 1);" \
 class="codenumbers">Toggle line numbers<\/a>');

```
 [47](#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-bbd7c4e67305a8926e488e6a5fa5695747e525f9_47 "#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-bbd7c4e67305a8926e488e6a5fa5695747e525f9_47")  ros::init(argc, argv, "talker");

```
 Initialize ROS. This allows ROS to do name remapping through the command line -- not important for now. This is also where we specify the name of our node. Node names must be unique in a running system. The name used here must be a [base name](/Names#Graph "/Names#Graph"), ie. it cannot have a / in it. 

document.write('<a href="#" onclick="return togglenumber(\'roscpp\_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-1079aa1401a6c4cbfa4d833c391f1312fc5f3ba3\', 54, 1);" \
 class="codenumbers">Toggle line numbers<\/a>');

```
 [54](#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-1079aa1401a6c4cbfa4d833c391f1312fc5f3ba3_54 "#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-1079aa1401a6c4cbfa4d833c391f1312fc5f3ba3_54")  ros::NodeHandle n;

```
 Create a handle to this process' node. The first NodeHandle created will actually do the initialization of the node, and the last one destructed will cleanup any resources the node was using. 

document.write('<a href="#" onclick="return togglenumber(\'roscpp\_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-48c016747d035bfd32771b254802cfbd5feafe7d\', 73, 1);" \
 class="codenumbers">Toggle line numbers<\/a>');

```
 [73](#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-48c016747d035bfd32771b254802cfbd5feafe7d_73 "#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-48c016747d035bfd32771b254802cfbd5feafe7d_73")  ros::Publisher chatter\_pub = n.advertise<std\_msgs::String>("chatter", 1000);

```
 Tell the master that we are going to be publishing a message of type [std\_msgs/String](http://docs.ros.org/en/api/std_msgs/html/msg/String.html "http://docs.ros.org/en/api/std_msgs/html/msg/String.html") on the topic chatter. This lets the master tell any nodes listening on chatter that we are going to publish data on that topic. The second argument is the size of our publishing queue. In this case if we are publishing too quickly it will buffer up a maximum of 1000 messages before beginning to throw away old ones. NodeHandle::advertise() returns a ros::Publisher object, which serves two purposes: 1) it contains a publish() method that lets you publish messages onto the topic it was created with, and 2) when it goes out of scope, it will automatically unadvertise. 

document.write('<a href="#" onclick="return togglenumber(\'roscpp\_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-436a142c5bd0d3fab1d54147bea31450c8941b19\', 75, 1);" \
 class="codenumbers">Toggle line numbers<\/a>');

```
 [75](#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-436a142c5bd0d3fab1d54147bea31450c8941b19_75 "#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-436a142c5bd0d3fab1d54147bea31450c8941b19_75")  ros::Rate loop\_rate(10);

```
 A ros::Rate object allows you to specify a frequency that you would like to loop at. It will keep track of how long it has been since the last call to Rate::sleep(), and sleep for the correct amount of time. In this case we tell it we want to run at 10Hz. 

document.write('<a href="#" onclick="return togglenumber(\'roscpp\_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-0dd0ef99791fdf3e69587d021e2663b3d8181408\', 81, 1);" \
 class="codenumbers">Toggle line numbers<\/a>');

```
 [81](#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-0dd0ef99791fdf3e69587d021e2663b3d8181408_81 "#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-0dd0ef99791fdf3e69587d021e2663b3d8181408_81")  int count = 0;
 [82](#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-0dd0ef99791fdf3e69587d021e2663b3d8181408_82 "#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-0dd0ef99791fdf3e69587d021e2663b3d8181408_82")  while (ros::ok())
 [83](#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-0dd0ef99791fdf3e69587d021e2663b3d8181408_83 "#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-0dd0ef99791fdf3e69587d021e2663b3d8181408_83")  {

```
 By default roscpp will install a SIGINT handler which provides Ctrl-C handling which will cause ros::ok() to return false if that happens. ros::ok() will return false if: * a SIGINT is received (Ctrl-C)
* we have been kicked off the network by another node with the same name
* ros::shutdown() has been called by another part of the application.
* all ros::[NodeHandles](/NodeHandles "/NodeHandles") have been destroyed

Once ros::ok() returns false, all ROS calls will fail. 

document.write('<a href="#" onclick="return togglenumber(\'roscpp\_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-8456e0bb7d9b8e4831fbf057e53685ec7270519e\', 87, 1);" \
 class="codenumbers">Toggle line numbers<\/a>');

```
 [87](#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-8456e0bb7d9b8e4831fbf057e53685ec7270519e_87 "#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-8456e0bb7d9b8e4831fbf057e53685ec7270519e_87")  std\_msgs::String msg;
 [88](#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-8456e0bb7d9b8e4831fbf057e53685ec7270519e_88 "#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-8456e0bb7d9b8e4831fbf057e53685ec7270519e_88") 
 [89](#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-8456e0bb7d9b8e4831fbf057e53685ec7270519e_89 "#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-8456e0bb7d9b8e4831fbf057e53685ec7270519e_89")  std::stringstream ss;
 [90](#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-8456e0bb7d9b8e4831fbf057e53685ec7270519e_90 "#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-8456e0bb7d9b8e4831fbf057e53685ec7270519e_90")  ss << "hello world " << count;
 [91](#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-8456e0bb7d9b8e4831fbf057e53685ec7270519e_91 "#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-8456e0bb7d9b8e4831fbf057e53685ec7270519e_91")  msg.data = ss.str();

```
 We broadcast a message on ROS using a message-adapted class, generally generated from a [msg file](/msg "/msg"). More complicated datatypes are possible, but for now we're going to use the standard String message, which has one member: "data". 

document.write('<a href="#" onclick="return togglenumber(\'roscpp\_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-aa202e8d4afb2f5fef5365e963060fa589e97965\', 101, 1);" \
 class="codenumbers">Toggle line numbers<\/a>');

```
 [101](#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-aa202e8d4afb2f5fef5365e963060fa589e97965_101 "#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-aa202e8d4afb2f5fef5365e963060fa589e97965_101")  chatter\_pub.publish(msg);

```
 Now we actually broadcast the message to anyone who is connected. 

document.write('<a href="#" onclick="return togglenumber(\'roscpp\_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-ff7616a808b0d827bbb7a305f4b0a9b18a7d4309\', 93, 1);" \
 class="codenumbers">Toggle line numbers<\/a>');

```
 [93](#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-ff7616a808b0d827bbb7a305f4b0a9b18a7d4309_93 "#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-ff7616a808b0d827bbb7a305f4b0a9b18a7d4309_93")  ROS\_INFO("%s", msg.data.c\_str());

```
 ROS\_INFO and friends are our replacement for printf/cout. See the [rosconsole documentation](/rosconsole "/rosconsole") for more information. 

document.write('<a href="#" onclick="return togglenumber(\'roscpp\_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-77f59f9b7961ee481f7596923b938df8c19148f6\', 103, 1);" \
 class="codenumbers">Toggle line numbers<\/a>');

```
 [103](#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-77f59f9b7961ee481f7596923b938df8c19148f6_103 "#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-77f59f9b7961ee481f7596923b938df8c19148f6_103")  ros::spinOnce();

```
 Calling ros::spinOnce() here is not necessary for this simple program, because we are not receiving any callbacks. However, if you were to add a subscription into this application, and did not have ros::spinOnce() here, your callbacks would never get called. So, add it for good measure. 

document.write('<a href="#" onclick="return togglenumber(\'roscpp\_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-abbdd3c55b5ae72f624a06429120470b29dffbf7\', 105, 1);" \
 class="codenumbers">Toggle line numbers<\/a>');

```
 [105](#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-abbdd3c55b5ae72f624a06429120470b29dffbf7_105 "#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-abbdd3c55b5ae72f624a06429120470b29dffbf7_105")  loop\_rate.sleep();

```
 Now we use the ros::Rate object to sleep for the time remaining to let us hit our 10Hz publish rate. Here's the condensed version of what's going on: * Initialize the ROS system
* Advertise that we are going to be publishing [std\_msgs/String](http://docs.ros.org/en/api/std_msgs/html/msg/String.html "http://docs.ros.org/en/api/std_msgs/html/msg/String.html") messages on the chatter topic to the master
* Loop while publishing messages to chatter 10 times a second

Now we need to write a node to receive the messsages. 
## Writing the Subscriber Node

### The Code

Create the src/listener.cpp file within the beginner\_tutorials package and paste the following inside it: *[https://raw.github.com/ros/ros\_tutorials/kinetic-devel/roscpp\_tutorials/listener/listener.cpp](https://raw.github.com/ros/ros_tutorials/kinetic-devel/roscpp_tutorials/listener/listener.cpp "https://raw.github.com/ros/ros_tutorials/kinetic-devel/roscpp_tutorials/listener/listener.cpp")* 

document.write('<a href="#" onclick="return togglenumber(\'roscpp\_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-b00794bcdcacff4e742e844261078a50000d0a69\', 28, 1);" \
 class="codenumbers">Toggle line numbers<\/a>');

```
 [28](#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-b00794bcdcacff4e742e844261078a50000d0a69_28 "#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-b00794bcdcacff4e742e844261078a50000d0a69_28") #include "ros/ros.h"
 [29](#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-b00794bcdcacff4e742e844261078a50000d0a69_29 "#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-b00794bcdcacff4e742e844261078a50000d0a69_29") #include "std\_msgs/String.h"
 [30](#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-b00794bcdcacff4e742e844261078a50000d0a69_30 "#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-b00794bcdcacff4e742e844261078a50000d0a69_30") 
 [31](#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-b00794bcdcacff4e742e844261078a50000d0a69_31 "#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-b00794bcdcacff4e742e844261078a50000d0a69_31") /\*\*
 [32](#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-b00794bcdcacff4e742e844261078a50000d0a69_32 "#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-b00794bcdcacff4e742e844261078a50000d0a69_32")  \* This tutorial demonstrates simple receipt of messages over the ROS system.
 [33](#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-b00794bcdcacff4e742e844261078a50000d0a69_33 "#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-b00794bcdcacff4e742e844261078a50000d0a69_33")  \*/
 [34](#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-b00794bcdcacff4e742e844261078a50000d0a69_34 "#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-b00794bcdcacff4e742e844261078a50000d0a69_34") void chatterCallback(const std\_msgs::String::ConstPtr& msg)
 [35](#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-b00794bcdcacff4e742e844261078a50000d0a69_35 "#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-b00794bcdcacff4e742e844261078a50000d0a69_35") {
 [36](#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-b00794bcdcacff4e742e844261078a50000d0a69_36 "#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-b00794bcdcacff4e742e844261078a50000d0a69_36")  ROS\_INFO("I heard: [%s]", msg->data.c\_str());
 [37](#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-b00794bcdcacff4e742e844261078a50000d0a69_37 "#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-b00794bcdcacff4e742e844261078a50000d0a69_37") }
 [38](#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-b00794bcdcacff4e742e844261078a50000d0a69_38 "#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-b00794bcdcacff4e742e844261078a50000d0a69_38") 
 [39](#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-b00794bcdcacff4e742e844261078a50000d0a69_39 "#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-b00794bcdcacff4e742e844261078a50000d0a69_39") int main(int argc, char \*\*argv)
 [40](#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-b00794bcdcacff4e742e844261078a50000d0a69_40 "#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-b00794bcdcacff4e742e844261078a50000d0a69_40") {
 [41](#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-b00794bcdcacff4e742e844261078a50000d0a69_41 "#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-b00794bcdcacff4e742e844261078a50000d0a69_41")  /\*\*
 [42](#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-b00794bcdcacff4e742e844261078a50000d0a69_42 "#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-b00794bcdcacff4e742e844261078a50000d0a69_42")  \* The ros::init() function needs to see argc and argv so that it can perform
 [43](#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-b00794bcdcacff4e742e844261078a50000d0a69_43 "#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-b00794bcdcacff4e742e844261078a50000d0a69_43")  \* any ROS arguments and name remapping that were provided at the command line.
 [44](#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-b00794bcdcacff4e742e844261078a50000d0a69_44 "#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-b00794bcdcacff4e742e844261078a50000d0a69_44")  \* For programmatic remappings you can use a different version of init() which takes
 [45](#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-b00794bcdcacff4e742e844261078a50000d0a69_45 "#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-b00794bcdcacff4e742e844261078a50000d0a69_45")  \* remappings directly, but for most command-line programs, passing argc and argv is
 [46](#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-b00794bcdcacff4e742e844261078a50000d0a69_46 "#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-b00794bcdcacff4e742e844261078a50000d0a69_46")  \* the easiest way to do it. The third argument to init() is the name of the node.
 [47](#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-b00794bcdcacff4e742e844261078a50000d0a69_47 "#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-b00794bcdcacff4e742e844261078a50000d0a69_47")  \*
 [48](#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-b00794bcdcacff4e742e844261078a50000d0a69_48 "#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-b00794bcdcacff4e742e844261078a50000d0a69_48")  \* You must call one of the versions of ros::init() before using any other
 [49](#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-b00794bcdcacff4e742e844261078a50000d0a69_49 "#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-b00794bcdcacff4e742e844261078a50000d0a69_49")  \* part of the ROS system.
 [50](#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-b00794bcdcacff4e742e844261078a50000d0a69_50 "#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-b00794bcdcacff4e742e844261078a50000d0a69_50")  \*/
 [51](#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-b00794bcdcacff4e742e844261078a50000d0a69_51 "#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-b00794bcdcacff4e742e844261078a50000d0a69_51")  ros::init(argc, argv, "listener");
 [52](#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-b00794bcdcacff4e742e844261078a50000d0a69_52 "#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-b00794bcdcacff4e742e844261078a50000d0a69_52") 
 [53](#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-b00794bcdcacff4e742e844261078a50000d0a69_53 "#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-b00794bcdcacff4e742e844261078a50000d0a69_53")  /\*\*
 [54](#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-b00794bcdcacff4e742e844261078a50000d0a69_54 "#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-b00794bcdcacff4e742e844261078a50000d0a69_54")  \* NodeHandle is the main access point to communications with the ROS system.
 [55](#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-b00794bcdcacff4e742e844261078a50000d0a69_55 "#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-b00794bcdcacff4e742e844261078a50000d0a69_55")  \* The first NodeHandle constructed will fully initialize this node, and the last
 [56](#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-b00794bcdcacff4e742e844261078a50000d0a69_56 "#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-b00794bcdcacff4e742e844261078a50000d0a69_56")  \* NodeHandle destructed will close down the node.
 [57](#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-b00794bcdcacff4e742e844261078a50000d0a69_57 "#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-b00794bcdcacff4e742e844261078a50000d0a69_57")  \*/
 [58](#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-b00794bcdcacff4e742e844261078a50000d0a69_58 "#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-b00794bcdcacff4e742e844261078a50000d0a69_58")  ros::NodeHandle n;
 [59](#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-b00794bcdcacff4e742e844261078a50000d0a69_59 "#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-b00794bcdcacff4e742e844261078a50000d0a69_59") 
 [60](#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-b00794bcdcacff4e742e844261078a50000d0a69_60 "#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-b00794bcdcacff4e742e844261078a50000d0a69_60")  /\*\*
 [61](#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-b00794bcdcacff4e742e844261078a50000d0a69_61 "#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-b00794bcdcacff4e742e844261078a50000d0a69_61")  \* The subscribe() call is how you tell ROS that you want to receive messages
 [62](#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-b00794bcdcacff4e742e844261078a50000d0a69_62 "#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-b00794bcdcacff4e742e844261078a50000d0a69_62")  \* on a given topic. This invokes a call to the ROS
 [63](#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-b00794bcdcacff4e742e844261078a50000d0a69_63 "#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-b00794bcdcacff4e742e844261078a50000d0a69_63")  \* master node, which keeps a registry of who is publishing and who
 [64](#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-b00794bcdcacff4e742e844261078a50000d0a69_64 "#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-b00794bcdcacff4e742e844261078a50000d0a69_64")  \* is subscribing. Messages are passed to a callback function, here
 [65](#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-b00794bcdcacff4e742e844261078a50000d0a69_65 "#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-b00794bcdcacff4e742e844261078a50000d0a69_65")  \* called chatterCallback. subscribe() returns a Subscriber object that you
 [66](#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-b00794bcdcacff4e742e844261078a50000d0a69_66 "#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-b00794bcdcacff4e742e844261078a50000d0a69_66")  \* must hold on to until you want to unsubscribe. When all copies of the Subscriber
 [67](#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-b00794bcdcacff4e742e844261078a50000d0a69_67 "#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-b00794bcdcacff4e742e844261078a50000d0a69_67")  \* object go out of scope, this callback will automatically be unsubscribed from
 [68](#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-b00794bcdcacff4e742e844261078a50000d0a69_68 "#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-b00794bcdcacff4e742e844261078a50000d0a69_68")  \* this topic.
 [69](#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-b00794bcdcacff4e742e844261078a50000d0a69_69 "#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-b00794bcdcacff4e742e844261078a50000d0a69_69")  \*
 [70](#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-b00794bcdcacff4e742e844261078a50000d0a69_70 "#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-b00794bcdcacff4e742e844261078a50000d0a69_70")  \* The second parameter to the subscribe() function is the size of the message
 [71](#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-b00794bcdcacff4e742e844261078a50000d0a69_71 "#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-b00794bcdcacff4e742e844261078a50000d0a69_71")  \* queue. If messages are arriving faster than they are being processed, this
 [72](#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-b00794bcdcacff4e742e844261078a50000d0a69_72 "#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-b00794bcdcacff4e742e844261078a50000d0a69_72")  \* is the number of messages that will be buffered up before beginning to throw
 [73](#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-b00794bcdcacff4e742e844261078a50000d0a69_73 "#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-b00794bcdcacff4e742e844261078a50000d0a69_73")  \* away the oldest ones.
 [74](#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-b00794bcdcacff4e742e844261078a50000d0a69_74 "#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-b00794bcdcacff4e742e844261078a50000d0a69_74")  \*/
 [75](#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-b00794bcdcacff4e742e844261078a50000d0a69_75 "#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-b00794bcdcacff4e742e844261078a50000d0a69_75")  ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);
 [76](#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-b00794bcdcacff4e742e844261078a50000d0a69_76 "#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-b00794bcdcacff4e742e844261078a50000d0a69_76") 
 [77](#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-b00794bcdcacff4e742e844261078a50000d0a69_77 "#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-b00794bcdcacff4e742e844261078a50000d0a69_77")  /\*\*
 [78](#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-b00794bcdcacff4e742e844261078a50000d0a69_78 "#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-b00794bcdcacff4e742e844261078a50000d0a69_78")  \* ros::spin() will enter a loop, pumping callbacks. With this version, all
 [79](#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-b00794bcdcacff4e742e844261078a50000d0a69_79 "#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-b00794bcdcacff4e742e844261078a50000d0a69_79")  \* callbacks will be called from within this thread (the main one). ros::spin()
 [80](#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-b00794bcdcacff4e742e844261078a50000d0a69_80 "#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-b00794bcdcacff4e742e844261078a50000d0a69_80")  \* will exit when Ctrl-C is pressed, or the node is shutdown by the master.
 [81](#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-b00794bcdcacff4e742e844261078a50000d0a69_81 "#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-b00794bcdcacff4e742e844261078a50000d0a69_81")  \*/
 [82](#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-b00794bcdcacff4e742e844261078a50000d0a69_82 "#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-b00794bcdcacff4e742e844261078a50000d0a69_82")  ros::spin();
 [83](#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-b00794bcdcacff4e742e844261078a50000d0a69_83 "#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-b00794bcdcacff4e742e844261078a50000d0a69_83") 
 [84](#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-b00794bcdcacff4e742e844261078a50000d0a69_84 "#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-b00794bcdcacff4e742e844261078a50000d0a69_84")  return 0;
 [85](#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-b00794bcdcacff4e742e844261078a50000d0a69_85 "#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-b00794bcdcacff4e742e844261078a50000d0a69_85") }

```

### The Code Explained

Now, let's break it down piece by piece, ignoring some pieces that have already been explained above. 

document.write('<a href="#" onclick="return togglenumber(\'roscpp\_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-6d9609d1db3a7bdca0302c1203d82078afb15323\', 34, 1);" \
 class="codenumbers">Toggle line numbers<\/a>');

```
 [34](#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-6d9609d1db3a7bdca0302c1203d82078afb15323_34 "#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-6d9609d1db3a7bdca0302c1203d82078afb15323_34") void chatterCallback(const std\_msgs::String::ConstPtr& msg)
 [35](#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-6d9609d1db3a7bdca0302c1203d82078afb15323_35 "#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-6d9609d1db3a7bdca0302c1203d82078afb15323_35") {
 [36](#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-6d9609d1db3a7bdca0302c1203d82078afb15323_36 "#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-6d9609d1db3a7bdca0302c1203d82078afb15323_36")  ROS\_INFO("I heard: [%s]", msg->data.c\_str());
 [37](#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-6d9609d1db3a7bdca0302c1203d82078afb15323_37 "#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-6d9609d1db3a7bdca0302c1203d82078afb15323_37") }

```
 This is the callback function that will get called when a new message has arrived on the chatter topic. The message is passed in a [boost shared\_ptr](http://www.boost.org/doc/libs/1_37_0/libs/smart_ptr/shared_ptr.htm "http://www.boost.org/doc/libs/1_37_0/libs/smart_ptr/shared_ptr.htm"), which means you can store it off if you want, without worrying about it getting deleted underneath you, and without copying the underlying data. 

document.write('<a href="#" onclick="return togglenumber(\'roscpp\_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-5796660589beb27dca39a7b53df11813783e220d\', 75, 1);" \
 class="codenumbers">Toggle line numbers<\/a>');

```
 [75](#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-5796660589beb27dca39a7b53df11813783e220d_75 "#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-5796660589beb27dca39a7b53df11813783e220d_75")  ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);

```
 Subscribe to the chatter topic with the master. ROS will call the chatterCallback() function whenever a new message arrives. The 2nd argument is the queue size, in case we are not able to process messages fast enough. In this case, if the queue reaches 1000 messages, we will start throwing away old messages as new ones arrive. NodeHandle::subscribe() returns a ros::Subscriber object, that you must hold on to until you want to unsubscribe. When the Subscriber object is destructed, it will automatically unsubscribe from the chatter topic. There are versions of the NodeHandle::subscribe() function which allow you to specify a class member function, or even anything callable by a Boost.Function object. The [roscpp overview](/roscpp/Overview "/roscpp/Overview") contains more information. 

document.write('<a href="#" onclick="return togglenumber(\'roscpp\_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-6424988503448f0a5323c4531325345a7f643fe3\', 82, 1);" \
 class="codenumbers">Toggle line numbers<\/a>');

```
 [82](#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-6424988503448f0a5323c4531325345a7f643fe3_82 "#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-6424988503448f0a5323c4531325345a7f643fe3_82")  ros::spin();

```
 ros::spin() enters a loop, calling message callbacks as fast as possible. Don't worry though, if there's nothing for it to do it won't use much CPU. ros::spin() will exit once ros::ok() returns false, which means ros::shutdown() has been called, either by the default Ctrl-C handler, the master telling us to shutdown, or it being called manually. There are other ways of pumping callbacks, but we won't worry about those here. The [roscpp\_tutorials](/roscpp_tutorials "/roscpp_tutorials") package has some demo applications which demonstrate this. The [roscpp overview](/roscpp/Overview "/roscpp/Overview") also contains more information. Again, here's a condensed version of what's going on: * Initialize the ROS system
* Subscribe to the chatter topic
* Spin, waiting for messages to arrive
* When a message arrives, the chatterCallback() function is called

## Building your nodes

[roscreate-pkg](/roscreate "/roscreate") will create a default Makefile and CMakeLists.txt for your package. 
```
$ rosed beginner_tutorials CMakeLists.txt 
```
It should look something like this: * ```
cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})
```

Adding the following at the bottom: 
```
rosbuild_add_executable(talker src/talker.cpp)
rosbuild_add_executable(listener src/listener.cpp)
```
This will create two executables, talker and listener, which by default will go into the "bin" directory. For more information on using CMake with ROS, see [CMakeLists](/CMakeLists "/CMakeLists") Now run make: 
```
$ make
```

## Building your nodes

You used [catkin\_create\_pkg](/catkin/commands/catkin_create_pkg "/catkin/commands/catkin_create_pkg") in a previous tutorial which created a [package.xml](/catkin/package_manifest "/catkin/package_manifest") and a [CMakeLists.txt](/catkin/CMakeLists.txt "/catkin/CMakeLists.txt") file for you. The generated [CMakeLists.txt](/catkin/CMakeLists.txt "/catkin/CMakeLists.txt") should look like this (with modifications from the [Creating Msgs and Srvs](/ROS/Tutorials/CreatingMsgAndSrv "/ROS/Tutorials/CreatingMsgAndSrv") tutorial and unused comments and examples removed): *[https://raw.github.com/ros/catkin\_tutorials/master/create\_package\_modified/catkin\_ws/src/beginner\_tutorials/CMakeLists.txt](https://raw.github.com/ros/catkin_tutorials/master/create_package_modified/catkin_ws/src/beginner_tutorials/CMakeLists.txt "https://raw.github.com/ros/catkin_tutorials/master/create_package_modified/catkin_ws/src/beginner_tutorials/CMakeLists.txt")* 

document.write('<a href="#" onclick="return togglenumber(\'roscpp\_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-613be11e3a734306735eeef98b24f783cdadf938\', 1, 1);" \
 class="codenumbers">Toggle line numbers<\/a>');

```
 [1](#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-613be11e3a734306735eeef98b24f783cdadf938_1 "#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-613be11e3a734306735eeef98b24f783cdadf938_1") cmake\_minimum\_required(VERSION 2.8.3)
 [2](#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-613be11e3a734306735eeef98b24f783cdadf938_2 "#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-613be11e3a734306735eeef98b24f783cdadf938_2") project(beginner\_tutorials)
 [3](#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-613be11e3a734306735eeef98b24f783cdadf938_3 "#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-613be11e3a734306735eeef98b24f783cdadf938_3") 
 [4](#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-613be11e3a734306735eeef98b24f783cdadf938_4 "#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-613be11e3a734306735eeef98b24f783cdadf938_4") ## Find catkin and any catkin packages
 [5](#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-613be11e3a734306735eeef98b24f783cdadf938_5 "#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-613be11e3a734306735eeef98b24f783cdadf938_5") find\_package(catkin REQUIRED COMPONENTS roscpp rospy std\_msgs genmsg)
 [6](#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-613be11e3a734306735eeef98b24f783cdadf938_6 "#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-613be11e3a734306735eeef98b24f783cdadf938_6") 
 [7](#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-613be11e3a734306735eeef98b24f783cdadf938_7 "#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-613be11e3a734306735eeef98b24f783cdadf938_7") ## Declare ROS messages and services
 [8](#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-613be11e3a734306735eeef98b24f783cdadf938_8 "#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-613be11e3a734306735eeef98b24f783cdadf938_8") add\_message\_files(DIRECTORY msg FILES Num.msg)
 [9](#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-613be11e3a734306735eeef98b24f783cdadf938_9 "#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-613be11e3a734306735eeef98b24f783cdadf938_9") add\_service\_files(DIRECTORY srv FILES AddTwoInts.srv)
 [10](#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-613be11e3a734306735eeef98b24f783cdadf938_10 "#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-613be11e3a734306735eeef98b24f783cdadf938_10") 
 [11](#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-613be11e3a734306735eeef98b24f783cdadf938_11 "#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-613be11e3a734306735eeef98b24f783cdadf938_11") ## Generate added messages and services
 [12](#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-613be11e3a734306735eeef98b24f783cdadf938_12 "#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-613be11e3a734306735eeef98b24f783cdadf938_12") generate\_messages(DEPENDENCIES std\_msgs)
 [13](#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-613be11e3a734306735eeef98b24f783cdadf938_13 "#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-613be11e3a734306735eeef98b24f783cdadf938_13") 
 [14](#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-613be11e3a734306735eeef98b24f783cdadf938_14 "#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-613be11e3a734306735eeef98b24f783cdadf938_14") ## Declare a catkin package
 [15](#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-613be11e3a734306735eeef98b24f783cdadf938_15 "#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-613be11e3a734306735eeef98b24f783cdadf938_15") catkin\_package()

```
 Don't worry about modifying the commented (#) examples, simply add these few lines to the bottom of your [CMakeLists.txt](/catkin/CMakeLists.txt "/catkin/CMakeLists.txt"): 
```
add_executable(talker src/talker.cpp)
target_link_libraries(talker ${catkin_LIBRARIES})
add_dependencies(talker beginner_tutorials_generate_messages_cpp)

add_executable(listener src/listener.cpp)
target_link_libraries(listener ${catkin_LIBRARIES})
add_dependencies(listener beginner_tutorials_generate_messages_cpp)
```
Your resulting [CMakeLists.txt](/catkin/CMakeLists.txt "/catkin/CMakeLists.txt") file should look like this: *[https://raw.github.com/ros/catkin\_tutorials/master/create\_package\_pubsub/catkin\_ws/src/beginner\_tutorials/CMakeLists.txt](https://raw.github.com/ros/catkin_tutorials/master/create_package_pubsub/catkin_ws/src/beginner_tutorials/CMakeLists.txt "https://raw.github.com/ros/catkin_tutorials/master/create_package_pubsub/catkin_ws/src/beginner_tutorials/CMakeLists.txt")* 

document.write('<a href="#" onclick="return togglenumber(\'roscpp\_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-56e552a9f29027036adb74aeea6375a15de4f70e\', 1, 1);" \
 class="codenumbers">Toggle line numbers<\/a>');

```
 [1](#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-56e552a9f29027036adb74aeea6375a15de4f70e_1 "#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-56e552a9f29027036adb74aeea6375a15de4f70e_1") cmake\_minimum\_required(VERSION 2.8.3)
 [2](#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-56e552a9f29027036adb74aeea6375a15de4f70e_2 "#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-56e552a9f29027036adb74aeea6375a15de4f70e_2") project(beginner\_tutorials)
 [3](#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-56e552a9f29027036adb74aeea6375a15de4f70e_3 "#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-56e552a9f29027036adb74aeea6375a15de4f70e_3") 
 [4](#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-56e552a9f29027036adb74aeea6375a15de4f70e_4 "#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-56e552a9f29027036adb74aeea6375a15de4f70e_4") ## Find catkin and any catkin packages
 [5](#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-56e552a9f29027036adb74aeea6375a15de4f70e_5 "#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-56e552a9f29027036adb74aeea6375a15de4f70e_5") find\_package(catkin REQUIRED COMPONENTS roscpp rospy std\_msgs genmsg)
 [6](#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-56e552a9f29027036adb74aeea6375a15de4f70e_6 "#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-56e552a9f29027036adb74aeea6375a15de4f70e_6") 
 [7](#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-56e552a9f29027036adb74aeea6375a15de4f70e_7 "#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-56e552a9f29027036adb74aeea6375a15de4f70e_7") ## Declare ROS messages and services
 [8](#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-56e552a9f29027036adb74aeea6375a15de4f70e_8 "#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-56e552a9f29027036adb74aeea6375a15de4f70e_8") add\_message\_files(FILES Num.msg)
 [9](#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-56e552a9f29027036adb74aeea6375a15de4f70e_9 "#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-56e552a9f29027036adb74aeea6375a15de4f70e_9") add\_service\_files(FILES AddTwoInts.srv)
 [10](#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-56e552a9f29027036adb74aeea6375a15de4f70e_10 "#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-56e552a9f29027036adb74aeea6375a15de4f70e_10") 
 [11](#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-56e552a9f29027036adb74aeea6375a15de4f70e_11 "#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-56e552a9f29027036adb74aeea6375a15de4f70e_11") ## Generate added messages and services
 [12](#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-56e552a9f29027036adb74aeea6375a15de4f70e_12 "#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-56e552a9f29027036adb74aeea6375a15de4f70e_12") generate\_messages(DEPENDENCIES std\_msgs)
 [13](#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-56e552a9f29027036adb74aeea6375a15de4f70e_13 "#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-56e552a9f29027036adb74aeea6375a15de4f70e_13") 
 [14](#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-56e552a9f29027036adb74aeea6375a15de4f70e_14 "#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-56e552a9f29027036adb74aeea6375a15de4f70e_14") ## Declare a catkin package
 [15](#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-56e552a9f29027036adb74aeea6375a15de4f70e_15 "#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-56e552a9f29027036adb74aeea6375a15de4f70e_15") catkin\_package()
 [16](#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-56e552a9f29027036adb74aeea6375a15de4f70e_16 "#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-56e552a9f29027036adb74aeea6375a15de4f70e_16") 
 [17](#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-56e552a9f29027036adb74aeea6375a15de4f70e_17 "#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-56e552a9f29027036adb74aeea6375a15de4f70e_17") ## Build talker and listener
 [18](#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-56e552a9f29027036adb74aeea6375a15de4f70e_18 "#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-56e552a9f29027036adb74aeea6375a15de4f70e_18") include\_directories(include ${catkin\_INCLUDE\_DIRS})
 [19](#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-56e552a9f29027036adb74aeea6375a15de4f70e_19 "#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-56e552a9f29027036adb74aeea6375a15de4f70e_19") 
 [20](#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-56e552a9f29027036adb74aeea6375a15de4f70e_20 "#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-56e552a9f29027036adb74aeea6375a15de4f70e_20") add\_executable(talker src/talker.cpp)
 [21](#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-56e552a9f29027036adb74aeea6375a15de4f70e_21 "#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-56e552a9f29027036adb74aeea6375a15de4f70e_21") target\_link\_libraries(talker ${catkin\_LIBRARIES})
 [22](#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-56e552a9f29027036adb74aeea6375a15de4f70e_22 "#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-56e552a9f29027036adb74aeea6375a15de4f70e_22") add\_dependencies(talker beginner\_tutorials\_generate\_messages\_cpp)
 [23](#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-56e552a9f29027036adb74aeea6375a15de4f70e_23 "#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-56e552a9f29027036adb74aeea6375a15de4f70e_23") 
 [24](#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-56e552a9f29027036adb74aeea6375a15de4f70e_24 "#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-56e552a9f29027036adb74aeea6375a15de4f70e_24") add\_executable(listener src/listener.cpp)
 [25](#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-56e552a9f29027036adb74aeea6375a15de4f70e_25 "#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-56e552a9f29027036adb74aeea6375a15de4f70e_25") target\_link\_libraries(listener ${catkin\_LIBRARIES})
 [26](#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-56e552a9f29027036adb74aeea6375a15de4f70e_26 "#roscpp_tutorials.2FTutorials.2FWritingPublisherSubscriber.CA-56e552a9f29027036adb74aeea6375a15de4f70e_26") add\_dependencies(listener beginner\_tutorials\_generate\_messages\_cpp)

```
 This will create two executables, talker and listener, which by default will go into package directory of your [devel space](/catkin/workspaces#Development_.28Devel.29_Space "/catkin/workspaces#Development_.28Devel.29_Space"), located by default at ~/catkin\_ws/devel/lib/<package name>. Note that you have to add dependencies for the executable targets to message generation targets: 
```
add_dependencies(talker beginner_tutorials_generate_messages_cpp)
```
This makes sure message headers of this package are generated before being used. If you use messages from other packages inside your catkin workspace, you need to add dependencies to their respective generation targets as well, because catkin builds all projects in parallel. As of \*Groovy\* you can use the following variable to depend on all necessary targets: 
```
target_link_libraries(talker ${catkin_LIBRARIES})
```
You can invoke executables directly or you can use rosrun to invoke them. They are not placed in '<prefix>/bin' because that would pollute the PATH when installing your package to the system. If you wish for your executable to be on the PATH at installation time, you can setup an install target, see: [catkin/CMakeLists.txt](/catkin/CMakeLists.txt "/catkin/CMakeLists.txt") For more detailed discription of the [CMakeLists.txt](/catkin/CMakeLists.txt "/catkin/CMakeLists.txt") file see: [catkin/CMakeLists.txt](/catkin/CMakeLists.txt "/catkin/CMakeLists.txt") Now run catkin\_make: 
```
# In your catkin workspace
$ cd ~/catkin_ws
$ catkin_make  
```
Note: Or if you're adding as new pkg, you may need to tell catkin to force making by --force-cmake option. See [catkin/Tutorials/using\_a\_workspace#With\_catkin\_make](/catkin/Tutorials/using_a_workspace#With_catkin_make "/catkin/Tutorials/using_a_workspace#With_catkin_make"). 

 Now that you have written a simple publisher and subscriber, let's [examine the simple publisher and subscriber](/ROS/Tutorials/ExaminingPublisherSubscriber "/ROS/Tutorials/ExaminingPublisherSubscriber"). 
## Additional Resources

Here are some additional resources contributed by the community: 
### Video Tutorial

The following video presents a small tutorial explaining how to write and test a publisher and subscriber in ROS with C++ and Python based on the talker/listener example above  

Wiki: ROS/Tutorials/WritingPublisherSubscriber(c++) (last edited 2019-07-18 19:12:37 by [AnisKoubaa](/AnisKoubaa "AnisKoubaa @ 102.172.113.40[102.172.113.40]"))

Except where otherwise noted, the ROS wiki is licensed under the   

