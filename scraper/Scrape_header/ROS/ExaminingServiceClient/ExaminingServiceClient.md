

 var \_gaq = \_gaq || [];
 \_gaq.push(['\_setAccount', 'UA-17821189-2']);
 \_gaq.push(['\_setDomainName', 'wiki.ros.org']);
 \_gaq.push(['\_trackPageview']);

 (function() {
 var ga = document.createElement('script'); ga.type = 'text/javascript'; ga.async = true;
 ga.src = ('https:' == document.location.protocol ? 'https://ssl' : 'http://www') + '.google-analytics.com/ga.js';
 var s = document.getElementsByTagName('script')[0]; s.parentNode.insertBefore(ga, s);
 })();

ROS/Tutorials/ExaminingServiceClient - ROS Wiki

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
* [ExaminingServiceClient](/action/fullsearch/ROS/Tutorials/ExaminingServiceClient?action=fullsearch&context=180&value=linkto%3A%22ROS%2FTutorials%2FExaminingServiceClient%22 "Click to do a full-text search for this title")

#### ROS 2 Documentation

The ROS Wiki is for ROS 1. Are you using ROS 2 ([Humble](http://docs.ros.org/en/humble/ "http://docs.ros.org/en/humble/"), [Iron](http://docs.ros.org/en/iron/ "http://docs.ros.org/en/iron/"), or [Rolling](http://docs.ros.org/en/rolling/ "http://docs.ros.org/en/rolling/"))?   
[Check out the ROS 2 Project Documentation](http://docs.ros.org "http://docs.ros.org")  
Package specific documentation can be found on [index.ros.org](https://index.ros.org "https://index.ros.org")

# Wiki

* [Distributions](/Distributions "/Distributions")
* [ROS/Installation](/ROS/Installation "/ROS/Installation")
* [ROS/Tutorials](/ROS/Tutorials "/ROS/Tutorials")
* [RecentChanges](/RecentChanges "/RecentChanges")
* [ExaminingServiceClient](/ROS/Tutorials/ExaminingServiceClient "/ROS/Tutorials/ExaminingServiceClient")

# Page

* Immutable Page
* [Comments](# "#")
* [Info](/action/info/ROS/Tutorials/ExaminingServiceClient?action=info "/action/info/ROS/Tutorials/ExaminingServiceClient?action=info")
* [Attachments](/action/AttachFile/ROS/Tutorials/ExaminingServiceClient?action=AttachFile "/action/AttachFile/ROS/Tutorials/ExaminingServiceClient?action=AttachFile")
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

* [Login](/action/login/ROS/Tutorials/ExaminingServiceClient?action=login "/action/login/ROS/Tutorials/ExaminingServiceClient?action=login")

|  |
| --- |
| **Note:** This tutorial assumes that you have completed the previous tutorials: writing a simple service and client [(python)](/ROS/Tutorials/WritingServiceClient%28python%29 "/ROS/Tutorials/WritingServiceClient%28python%29") [(c++)](/ROS/Tutorials/WritingServiceClient%28c%2B%2B%29 "/ROS/Tutorials/WritingServiceClient%28c%2B%2B%29").  |

|  |
| --- |
| (!) Please ask about problems and questions regarding this tutorial on [answers.ros.org](http://answers.ros.org "http://answers.ros.org"). Don't forget to include in your question the link to this page, the versions of your OS & ROS, and also add appropriate tags. |

# Examining the Simple Service and Client

**Description:** This tutorial examines running the simple service and client.  

**Tutorial Level:** BEGINNER  

**Next Tutorial:** [Recording and playing back data](/ROS/Tutorials/Recording%20and%20playing%20back%20data "/ROS/Tutorials/Recording%20and%20playing%20back%20data")   

 Contents1. [Running the Service](#Running_the_Service "#Running_the_Service")
2. [Running the Client](#Running_the_Client "#Running_the_Client")
3. [Further examples on Service and Client nodes](#Further_examples_on_Service_and_Client_nodes "#Further_examples_on_Service_and_Client_nodes")

## Running the Service

Let's start by running the service: 
```
$ rosrun beginner_tutorials add_two_ints_server     (C++)
$ rosrun beginner_tutorials add_two_ints_server.py  (Python) 
```
You should see something similar to: 
```
Ready to add two ints.
```

## Running the Client

Now let's run the client with the necessary arguments: 
```
$ rosrun beginner_tutorials add_two_ints_client 1 3     (C++)
$ rosrun beginner_tutorials add_two_ints_client.py 1 3  (Python) 
```
You should see something similar to: 
```
Requesting 1+3
1 + 3 = 4
```
Now that you've successfully run your first server and client, let's learn how to [record and play back data](/ROS/Tutorials/Recording%20and%20playing%20back%20data "/ROS/Tutorials/Recording%20and%20playing%20back%20data"). 
## Further examples on Service and Client nodes

If you want to investigate further and get a hands-on example, you can get one [here](https://github.com/fairlight1337/ros_service_examples/ "https://github.com/fairlight1337/ros_service_examples/"). A simple Client and Service combination shows the use of custom message types. The Service node is written in C++ while the Client is available in C++, Python and LISP. 

Wiki: ROS/Tutorials/ExaminingServiceClient (last edited 2016-09-27 14:18:50 by [yoyekw](/yoyekw "yoyekw @ robot-NAT.elka.pw.edu.pl[194.29.160.190]"))

Except where otherwise noted, the ROS wiki is licensed under the   

