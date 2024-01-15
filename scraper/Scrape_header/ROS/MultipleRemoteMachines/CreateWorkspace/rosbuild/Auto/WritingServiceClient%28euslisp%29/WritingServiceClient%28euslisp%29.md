

 var \_gaq = \_gaq || [];
 \_gaq.push(['\_setAccount', 'UA-17821189-2']);
 \_gaq.push(['\_setDomainName', 'wiki.ros.org']);
 \_gaq.push(['\_trackPageview']);

 (function() {
 var ga = document.createElement('script'); ga.type = 'text/javascript'; ga.async = true;
 ga.src = ('https:' == document.location.protocol ? 'https://ssl' : 'http://www') + '.google-analytics.com/ga.js';
 var s = document.getElementsByTagName('script')[0]; s.parentNode.insertBefore(ga, s);
 })();

ROS/Tutorials/WritingServiceClient(euslisp) - ROS Wiki

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
* [WritingServiceClient(euslisp)](/ROS/Tutorials/WritingServiceClient%28euslisp%29 "/ROS/Tutorials/WritingServiceClient%28euslisp%29")

#### ROS 2 Documentation

The ROS Wiki is for ROS 1. Are you using ROS 2 ([Humble](http://docs.ros.org/en/humble/ "http://docs.ros.org/en/humble/"), [Iron](http://docs.ros.org/en/iron/ "http://docs.ros.org/en/iron/"), or [Rolling](http://docs.ros.org/en/rolling/ "http://docs.ros.org/en/rolling/"))?   
[Check out the ROS 2 Project Documentation](http://docs.ros.org "http://docs.ros.org")  
Package specific documentation can be found on [index.ros.org](https://index.ros.org "https://index.ros.org")

# Wiki

* [Distributions](/Distributions "/Distributions")
* [ROS/Installation](/ROS/Installation "/ROS/Installation")
* [ROS/Tutorials](/ROS/Tutorials "/ROS/Tutorials")
* [RecentChanges](/RecentChanges "/RecentChanges")
* [WritingServ...nt(euslisp)](/ROS/Tutorials/WritingServiceClient%28euslisp%29 "/ROS/Tutorials/WritingServiceClient%28euslisp%29")

# Page

* Immutable Page
* [Comments](# "#")
* [Info](/action/info/ROS/Tutorials/WritingServiceClient%28euslisp%29?action=info "/action/info/ROS/Tutorials/WritingServiceClient%28euslisp%29?action=info")
* [Attachments](/action/AttachFile/ROS/Tutorials/WritingServiceClient%28euslisp%29?action=AttachFile "/action/AttachFile/ROS/Tutorials/WritingServiceClient%28euslisp%29?action=AttachFile")
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

* [Login](/action/login/ROS/Tutorials/WritingServiceClient%28euslisp%29?action=login "/action/login/ROS/Tutorials/WritingServiceClient%28euslisp%29?action=login")

|  |
| --- |
| **Note:** This tutorial assumes that you have completed the previous tutorials: [Writing a Simple Publisher and Subscriber (euslisp)](/ROS/Tutorials/WritingPublisherSubscriber%28Euslisp%29 "/ROS/Tutorials/WritingPublisherSubscriber%28Euslisp%29").  |

|  |
| --- |
| (!) Please ask about problems and questions regarding this tutorial on [answers.ros.org](http://answers.ros.org "http://answers.ros.org"). Don't forget to include in your question the link to this page, the versions of your OS & ROS, and also add appropriate tags. |

# Writing Simple Service and Client (EusLisp)

**Description:** This tutorial covers how to write a service and client node in euslisp.  

**Tutorial Level:** BEGINNER  

**Next Tutorial:** [Using EusLisp (roseus) to control rtmtros robots](/rtmros_common/Tutorials/WorkingWithEusLisp "/rtmros_common/Tutorials/WorkingWithEusLisp")   

 Contents1. [Writing a Service Node](#Writing_a_Service_Node "#Writing_a_Service_Node")
	1. [The Code](#The_Code "#The_Code")
	2. [The Code Explained](#The_Code_Explained "#The_Code_Explained")
2. [Writing the Client Node](#Writing_the_Client_Node "#Writing_the_Client_Node")
	1. [The Code](#The_Code-1 "#The_Code-1")
	2. [The Code Explained](#The_Code_Explained-1 "#The_Code_Explained-1")
3. [Building your nodes](#Building_your_nodes "#Building_your_nodes")
4. [Try it out!](#Try_it_out.21 "#Try_it_out.21")

### Writing a Service Node

Here we'll create the service ("add\_two\_ints\_server") node which will receive two ints and return the sum. Change directory into the beginner\_tutorials package, you created in the earlier tutorial, * [creating a package](http://wiki.ros.org/ROS/Tutorials/CreatingPackage?buildsystem=catkin "http://wiki.ros.org/ROS/Tutorials/CreatingPackage?buildsystem=catkin"):

```
$ roscd beginner_tutorials
```
Please make sure you have followed the directions in [the previous tutorial for creating the service](/ROS/Tutorials/CreatingMsgAndSr "/ROS/Tutorials/CreatingMsgAndSr") needed in this tutorial, creating the [AddTwoInts](/AddTwoInts "/AddTwoInts").srv (be sure to choose the right version of build tool you're using at the top of wiki page in the link). 
#### The Code

Create the **euslsp/add-two-ints-server.py** file within the beginner\_tutorials package and paste the following inside it: 

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

document.write('<a href="#" onclick="return togglenumber(\'CA-8a25119b00ebcc85b1daf308d9e0700412f3bcef\', 1, 1);" \
 class="codenumbers">Toggle line numbers<\/a>');

```
 [1](#CA-8a25119b00ebcc85b1daf308d9e0700412f3bcef_1 "#CA-8a25119b00ebcc85b1daf308d9e0700412f3bcef_1") #!/usr/bin/env roseus
 [2](#CA-8a25119b00ebcc85b1daf308d9e0700412f3bcef_2 "#CA-8a25119b00ebcc85b1daf308d9e0700412f3bcef_2") ;;;
 [3](#CA-8a25119b00ebcc85b1daf308d9e0700412f3bcef_3 "#CA-8a25119b00ebcc85b1daf308d9e0700412f3bcef_3") ;;; euslisp version of ros\_tutorials/rospy\_tutorials/005\_add\_two\_ints
 [4](#CA-8a25119b00ebcc85b1daf308d9e0700412f3bcef_4 "#CA-8a25119b00ebcc85b1daf308d9e0700412f3bcef_4") ;;;
 [5](#CA-8a25119b00ebcc85b1daf308d9e0700412f3bcef_5 "#CA-8a25119b00ebcc85b1daf308d9e0700412f3bcef_5") (ros::load-ros-manifest "roseus")
 [6](#CA-8a25119b00ebcc85b1daf308d9e0700412f3bcef_6 "#CA-8a25119b00ebcc85b1daf308d9e0700412f3bcef_6") 
 [7](#CA-8a25119b00ebcc85b1daf308d9e0700412f3bcef_7 "#CA-8a25119b00ebcc85b1daf308d9e0700412f3bcef_7") ;;;
 [8](#CA-8a25119b00ebcc85b1daf308d9e0700412f3bcef_8 "#CA-8a25119b00ebcc85b1daf308d9e0700412f3bcef_8") (defun add-two-ints (req)
 [9](#CA-8a25119b00ebcc85b1daf308d9e0700412f3bcef_9 "#CA-8a25119b00ebcc85b1daf308d9e0700412f3bcef_9")  (let ((m (send req :response)))
 [10](#CA-8a25119b00ebcc85b1daf308d9e0700412f3bcef_10 "#CA-8a25119b00ebcc85b1daf308d9e0700412f3bcef_10")  (format \*error-output\* "Returning [~d + ~d = ~d]~%"
 [11](#CA-8a25119b00ebcc85b1daf308d9e0700412f3bcef_11 "#CA-8a25119b00ebcc85b1daf308d9e0700412f3bcef_11")  (send req :a) (send req :b)
 [12](#CA-8a25119b00ebcc85b1daf308d9e0700412f3bcef_12 "#CA-8a25119b00ebcc85b1daf308d9e0700412f3bcef_12")  (+ (send req :a) (send req :b)))
 [13](#CA-8a25119b00ebcc85b1daf308d9e0700412f3bcef_13 "#CA-8a25119b00ebcc85b1daf308d9e0700412f3bcef_13")  (send m :sum (+ (send req :a) (send req :b)))
 [14](#CA-8a25119b00ebcc85b1daf308d9e0700412f3bcef_14 "#CA-8a25119b00ebcc85b1daf308d9e0700412f3bcef_14")  m))
 [15](#CA-8a25119b00ebcc85b1daf308d9e0700412f3bcef_15 "#CA-8a25119b00ebcc85b1daf308d9e0700412f3bcef_15") ;;;
 [16](#CA-8a25119b00ebcc85b1daf308d9e0700412f3bcef_16 "#CA-8a25119b00ebcc85b1daf308d9e0700412f3bcef_16") ;;;
 [17](#CA-8a25119b00ebcc85b1daf308d9e0700412f3bcef_17 "#CA-8a25119b00ebcc85b1daf308d9e0700412f3bcef_17") (ros::roseus "add\_two\_ints\_server")
 [18](#CA-8a25119b00ebcc85b1daf308d9e0700412f3bcef_18 "#CA-8a25119b00ebcc85b1daf308d9e0700412f3bcef_18") (ros::advertise-service "add\_two\_ints" roseus::AddTwoInts #'add-two-ints)
 [19](#CA-8a25119b00ebcc85b1daf308d9e0700412f3bcef_19 "#CA-8a25119b00ebcc85b1daf308d9e0700412f3bcef_19") (do-until-key
 [20](#CA-8a25119b00ebcc85b1daf308d9e0700412f3bcef_20 "#CA-8a25119b00ebcc85b1daf308d9e0700412f3bcef_20")  (ros::spin-once))

```
Don't forget to make the node executable: 
```
chmod +x euslisp/add-two-ints-server.l
```

#### The Code Explained

Now, let's break the code down. There's very little to writing a service using [roseus](/roseus "/roseus"). We declare our node using (ros::roseus "add\_two\_ints\_server") and then declare our service: 

document.write('<a href="#" onclick="return togglenumber(\'CA-15c01ab9c6f15fb79ad17bcc45b375cf4baf64aa\', 18, 1);" \
 class="codenumbers">Toggle line numbers<\/a>');

```
 [18](#CA-15c01ab9c6f15fb79ad17bcc45b375cf4baf64aa_18 "#CA-15c01ab9c6f15fb79ad17bcc45b375cf4baf64aa_18") (ros::advertise-service "add\_two\_ints" roseus::AddTwoInts #'add-two-ints)

```
 This declares a new service named add\_two\_ints with the [AddTwoInts](/AddTwoInts "/AddTwoInts") service type. All requests are passed to handle\_add\_two\_ints function. handle\_add\_two\_ints is called with instances of [AddTwoIntsRequest](/AddTwoIntsRequest "/AddTwoIntsRequest") and returns instances of [AddTwoIntsResponse](/AddTwoIntsResponse "/AddTwoIntsResponse"). Just like with the subscriber example, 

document.write('<a href="#" onclick="return togglenumber(\'CA-65b946734ee5db7881b362084550549f7727ea33\', 19, 1);" \
 class="codenumbers">Toggle line numbers<\/a>');

```
 [19](#CA-65b946734ee5db7881b362084550549f7727ea33_19 "#CA-65b946734ee5db7881b362084550549f7727ea33_19") (do-until-key
 [20](#CA-65b946734ee5db7881b362084550549f7727ea33_20 "#CA-65b946734ee5db7881b362084550549f7727ea33_20")  (ros::spin-once))

```
 keeps your code from exiting until the service is shutdown. 
### Writing the Client Node

#### The Code

Create the euslsp/add-two-ints-client.lfile within the beginner\_tutorials package and paste the following inside it: 

document.write('<a href="#" onclick="return togglenumber(\'CA-f165cc848319568c2fcdc43d37abe95057e5f4e7\', 1, 1);" \
 class="codenumbers">Toggle line numbers<\/a>');

```
 [1](#CA-f165cc848319568c2fcdc43d37abe95057e5f4e7_1 "#CA-f165cc848319568c2fcdc43d37abe95057e5f4e7_1") #!/usr/bin/env roseus
 [2](#CA-f165cc848319568c2fcdc43d37abe95057e5f4e7_2 "#CA-f165cc848319568c2fcdc43d37abe95057e5f4e7_2") ;;;
 [3](#CA-f165cc848319568c2fcdc43d37abe95057e5f4e7_3 "#CA-f165cc848319568c2fcdc43d37abe95057e5f4e7_3") ;;; euslisp version of ros\_tutorials/rospy\_tutorials/005\_add\_two\_ints
 [4](#CA-f165cc848319568c2fcdc43d37abe95057e5f4e7_4 "#CA-f165cc848319568c2fcdc43d37abe95057e5f4e7_4") ;;;
 [5](#CA-f165cc848319568c2fcdc43d37abe95057e5f4e7_5 "#CA-f165cc848319568c2fcdc43d37abe95057e5f4e7_5") (ros::load-ros-manifest "roseus")
 [6](#CA-f165cc848319568c2fcdc43d37abe95057e5f4e7_6 "#CA-f165cc848319568c2fcdc43d37abe95057e5f4e7_6") 
 [7](#CA-f165cc848319568c2fcdc43d37abe95057e5f4e7_7 "#CA-f165cc848319568c2fcdc43d37abe95057e5f4e7_7") ;;;
 [8](#CA-f165cc848319568c2fcdc43d37abe95057e5f4e7_8 "#CA-f165cc848319568c2fcdc43d37abe95057e5f4e7_8") ;;;
 [9](#CA-f165cc848319568c2fcdc43d37abe95057e5f4e7_9 "#CA-f165cc848319568c2fcdc43d37abe95057e5f4e7_9") (ros::roseus "add\_two\_ints\_client")
 [10](#CA-f165cc848319568c2fcdc43d37abe95057e5f4e7_10 "#CA-f165cc848319568c2fcdc43d37abe95057e5f4e7_10") 
 [11](#CA-f165cc848319568c2fcdc43d37abe95057e5f4e7_11 "#CA-f165cc848319568c2fcdc43d37abe95057e5f4e7_11") (when (setq \*arguments\* (member "add-two-ints-client.l" lisp::\*eustop-argument\*
 [12](#CA-f165cc848319568c2fcdc43d37abe95057e5f4e7_12 "#CA-f165cc848319568c2fcdc43d37abe95057e5f4e7_12")  :test #'substringp))
 [13](#CA-f165cc848319568c2fcdc43d37abe95057e5f4e7_13 "#CA-f165cc848319568c2fcdc43d37abe95057e5f4e7_13")  (cond ((= (length \*arguments\*) 1)
 [14](#CA-f165cc848319568c2fcdc43d37abe95057e5f4e7_14 "#CA-f165cc848319568c2fcdc43d37abe95057e5f4e7_14")  (setq x (random 10)
 [15](#CA-f165cc848319568c2fcdc43d37abe95057e5f4e7_15 "#CA-f165cc848319568c2fcdc43d37abe95057e5f4e7_15")  y (random 20)))
 [16](#CA-f165cc848319568c2fcdc43d37abe95057e5f4e7_16 "#CA-f165cc848319568c2fcdc43d37abe95057e5f4e7_16")  ((= (length \*arguments\*) 3)
 [17](#CA-f165cc848319568c2fcdc43d37abe95057e5f4e7_17 "#CA-f165cc848319568c2fcdc43d37abe95057e5f4e7_17")  (setq x (read-from-string (elt \*arguments\* 1))
 [18](#CA-f165cc848319568c2fcdc43d37abe95057e5f4e7_18 "#CA-f165cc848319568c2fcdc43d37abe95057e5f4e7_18")  y (read-from-string (elt \*arguments\* 2))))
 [19](#CA-f165cc848319568c2fcdc43d37abe95057e5f4e7_19 "#CA-f165cc848319568c2fcdc43d37abe95057e5f4e7_19")  (t
 [20](#CA-f165cc848319568c2fcdc43d37abe95057e5f4e7_20 "#CA-f165cc848319568c2fcdc43d37abe95057e5f4e7_20")  (ros::ros-error "Usage: ~A [x y]~%" (elt \*arguments\* 0))
 [21](#CA-f165cc848319568c2fcdc43d37abe95057e5f4e7_21 "#CA-f165cc848319568c2fcdc43d37abe95057e5f4e7_21")  (exit 1))))
 [22](#CA-f165cc848319568c2fcdc43d37abe95057e5f4e7_22 "#CA-f165cc848319568c2fcdc43d37abe95057e5f4e7_22") 
 [23](#CA-f165cc848319568c2fcdc43d37abe95057e5f4e7_23 "#CA-f165cc848319568c2fcdc43d37abe95057e5f4e7_23") (ros::wait-for-service "add\_two\_ints")
 [24](#CA-f165cc848319568c2fcdc43d37abe95057e5f4e7_24 "#CA-f165cc848319568c2fcdc43d37abe95057e5f4e7_24") (dotimes (i 100)
 [25](#CA-f165cc848319568c2fcdc43d37abe95057e5f4e7_25 "#CA-f165cc848319568c2fcdc43d37abe95057e5f4e7_25")  (setq req (instance roseus::AddTwoIntsRequest :init))
 [26](#CA-f165cc848319568c2fcdc43d37abe95057e5f4e7_26 "#CA-f165cc848319568c2fcdc43d37abe95057e5f4e7_26")  (send req :a x)
 [27](#CA-f165cc848319568c2fcdc43d37abe95057e5f4e7_27 "#CA-f165cc848319568c2fcdc43d37abe95057e5f4e7_27")  (send req :b y)
 [28](#CA-f165cc848319568c2fcdc43d37abe95057e5f4e7_28 "#CA-f165cc848319568c2fcdc43d37abe95057e5f4e7_28")  (setq before (ros::time-now))
 [29](#CA-f165cc848319568c2fcdc43d37abe95057e5f4e7_29 "#CA-f165cc848319568c2fcdc43d37abe95057e5f4e7_29")  (case (mod i 3)
 [30](#CA-f165cc848319568c2fcdc43d37abe95057e5f4e7_30 "#CA-f165cc848319568c2fcdc43d37abe95057e5f4e7_30")  (0 (setq res (ros::service-call "add\_two\_ints" req t)))
 [31](#CA-f165cc848319568c2fcdc43d37abe95057e5f4e7_31 "#CA-f165cc848319568c2fcdc43d37abe95057e5f4e7_31")  (1 (setq res (ros::service-call "add\_two\_ints" req nil)))
 [32](#CA-f165cc848319568c2fcdc43d37abe95057e5f4e7_32 "#CA-f165cc848319568c2fcdc43d37abe95057e5f4e7_32")  (2 (setq res (ros::service-call "add\_two\_ints" req))))
 [33](#CA-f165cc848319568c2fcdc43d37abe95057e5f4e7_33 "#CA-f165cc848319568c2fcdc43d37abe95057e5f4e7_33")  (setq after (ros::time-now))
 [34](#CA-f165cc848319568c2fcdc43d37abe95057e5f4e7_34 "#CA-f165cc848319568c2fcdc43d37abe95057e5f4e7_34")  (format t "~d + ~d = ~d~ (~A sec)~%" (send req :a) (send req :b) (send res :sum) (send (ros::time- after before) :to-sec))
 [35](#CA-f165cc848319568c2fcdc43d37abe95057e5f4e7_35 "#CA-f165cc848319568c2fcdc43d37abe95057e5f4e7_35")  (unix:sleep 1))

```
Don't forget to make the node executable: 
```
$ chmod +x euslisp/add-two-ints-client.l
```

#### The Code Explained

Now, let's break the code down. The client code for calling services is also simple. For clients, we first call: 

document.write('<a href="#" onclick="return togglenumber(\'CA-3839d1c0e21e2fab2ec88b9fc88d95b5da1096f8\', 23, 1);" \
 class="codenumbers">Toggle line numbers<\/a>');

```
 [23](#CA-3839d1c0e21e2fab2ec88b9fc88d95b5da1096f8_23 "#CA-3839d1c0e21e2fab2ec88b9fc88d95b5da1096f8_23") (ros::wait-for-service "add\_two\_ints")

```
 This is a convenience method that blocks until the service named add\_two\_ints is available. Next we create a handle for calling the service: 

document.write('<a href="#" onclick="return togglenumber(\'CA-3ce579860e66a72abdc8af64686a7168060e4dfb\', 25, 1);" \
 class="codenumbers">Toggle line numbers<\/a>');

```
 [25](#CA-3ce579860e66a72abdc8af64686a7168060e4dfb_25 "#CA-3ce579860e66a72abdc8af64686a7168060e4dfb_25")  (setq req (instance roseus::AddTwoIntsRequest :init))
 [26](#CA-3ce579860e66a72abdc8af64686a7168060e4dfb_26 "#CA-3ce579860e66a72abdc8af64686a7168060e4dfb_26")  (send req :a x)
 [27](#CA-3ce579860e66a72abdc8af64686a7168060e4dfb_27 "#CA-3ce579860e66a72abdc8af64686a7168060e4dfb_27")  (send req :b y)

```
 Because we've declared the type of the service to be AddTwoInts, it does the work of generating the AddTwoIntsRequest object for you (you're free to pass in your own instead). 

document.write('<a href="#" onclick="return togglenumber(\'CA-0bc905cac499861a63d9885e6cb3f702a3df8be4\', 30, 1);" \
 class="codenumbers">Toggle line numbers<\/a>');

```
 [30](#CA-0bc905cac499861a63d9885e6cb3f702a3df8be4_30 "#CA-0bc905cac499861a63d9885e6cb3f702a3df8be4_30")  (0 (setq res (ros::service-call "add\_two\_ints" req t)))
 [31](#CA-0bc905cac499861a63d9885e6cb3f702a3df8be4_31 "#CA-0bc905cac499861a63d9885e6cb3f702a3df8be4_31")  (1 (setq res (ros::service-call "add\_two\_ints" req nil)))
 [32](#CA-0bc905cac499861a63d9885e6cb3f702a3df8be4_32 "#CA-0bc905cac499861a63d9885e6cb3f702a3df8be4_32")  (2 (setq res (ros::service-call "add\_two\_ints" req))))

```
 call (ros::service-call) function with argument of "add\_two\_ints" and req. The return value is an AddTwoIntsResponse object. 
### Building your nodes

We use CMake as our build system and, yes, you have to use it even for [EusLisp](/EusLisp "/EusLisp") nodes. This is to make sure that [the autogenerated EusLisp code for messages and services](/ROS/Tutorials/CreatingMsgAndSrv "/ROS/Tutorials/CreatingMsgAndSrv") is created. Go to your catkin workspace and run catkin\_make. 
```
# In the catkin workspace
$ cd ~/catkin_ws
$ catkin_make
```

### Try it out!

In a **new terminal**, run 
```
$ rosrun beginner_tutorials add_two_ints_server.l
```
In a **new terminal**, run 
```
$ rosrun beginner_tutorials add_two_ints_client.l 4 5
```
And you will get 
```
Requesting 4+5
4 + 5 = 9
```
And the server will print out 
```
Returning [4 + 5 = 9]
```

Wiki: ROS/Tutorials/WritingServiceClient(euslisp) (last edited 2019-09-20 01:50:38 by [Guilherme Affonso](/Guilherme%20Affonso "Guilherme Affonso @ 133.11.216.116[133.11.216.116]"))

Except where otherwise noted, the ROS wiki is licensed under the   

