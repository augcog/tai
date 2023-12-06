

 var \_gaq = \_gaq || [];
 \_gaq.push(['\_setAccount', 'UA-17821189-2']);
 \_gaq.push(['\_setDomainName', 'wiki.ros.org']);
 \_gaq.push(['\_trackPageview']);

 (function() {
 var ga = document.createElement('script'); ga.type = 'text/javascript'; ga.async = true;
 ga.src = ('https:' == document.location.protocol ? 'https://ssl' : 'http://www') + '.google-analytics.com/ga.js';
 var s = document.getElementsByTagName('script')[0]; s.parentNode.insertBefore(ga, s);
 })();

ROS/Tutorials/DefiningCustomMessages - ROS Wiki

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
* [DefiningCustomMessages](/action/fullsearch/ROS/Tutorials/DefiningCustomMessages?action=fullsearch&context=180&value=linkto%3A%22ROS%2FTutorials%2FDefiningCustomMessages%22 "Click to do a full-text search for this title")

#### ROS 2 Documentation

The ROS Wiki is for ROS 1. Are you using ROS 2 ([Humble](http://docs.ros.org/en/humble/ "http://docs.ros.org/en/humble/"), [Iron](http://docs.ros.org/en/iron/ "http://docs.ros.org/en/iron/"), or [Rolling](http://docs.ros.org/en/rolling/ "http://docs.ros.org/en/rolling/"))?   
[Check out the ROS 2 Project Documentation](http://docs.ros.org "http://docs.ros.org")  
Package specific documentation can be found on [index.ros.org](https://index.ros.org "https://index.ros.org")

# Wiki

* [Distributions](/Distributions "/Distributions")
* [ROS/Installation](/ROS/Installation "/ROS/Installation")
* [ROS/Tutorials](/ROS/Tutorials "/ROS/Tutorials")
* [RecentChanges](/RecentChanges "/RecentChanges")
* [DefiningCustomMessages](/ROS/Tutorials/DefiningCustomMessages "/ROS/Tutorials/DefiningCustomMessages")

# Page

* Immutable Page
* [Comments](# "#")
* [Info](/action/info/ROS/Tutorials/DefiningCustomMessages?action=info "/action/info/ROS/Tutorials/DefiningCustomMessages?action=info")
* [Attachments](/action/AttachFile/ROS/Tutorials/DefiningCustomMessages?action=AttachFile "/action/AttachFile/ROS/Tutorials/DefiningCustomMessages?action=AttachFile")
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

* [Login](/action/login/ROS/Tutorials/DefiningCustomMessages?action=login "/action/login/ROS/Tutorials/DefiningCustomMessages?action=login")

|  |
| --- |
| **Note:** This tutorial assumes that you have completed the previous tutorials: [ROS tutorials](/ROS/Tutorials "/ROS/Tutorials").  |

|  |
| --- |
| (!) Please ask about problems and questions regarding this tutorial on [answers.ros.org](http://answers.ros.org "http://answers.ros.org"). Don't forget to include in your question the link to this page, the versions of your OS & ROS, and also add appropriate tags. |

# Defining Custom Messages

**Description:** This tutorial will show you how to define your own custom message data types using the ROS [Message Description Language](/ROS/Message_Description_Language "/ROS/Message_Description_Language").  

**Tutorial Level:**   

**Next Tutorial:** [Using a C++ class in Python](/ROS/Tutorials/Using%20a%20C%2B%2B%20class%20in%20Python "/ROS/Tutorials/Using%20a%20C%2B%2B%20class%20in%20Python")   

 Contents1. [Generating Messages](#Generating_Messages "#Generating_Messages")
2. [Including or Importing Messages](#Including_or_Importing_Messages "#Including_or_Importing_Messages")
	1. [C++](#C.2B-.2B- "#C.2B-.2B-")
	2. [Python](#Python "#Python")
3. [Dependencies](#Dependencies "#Dependencies")

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

## Generating Messages

Generating a message is easy. Simply place a .msg file inside the msg directory in a package. Please follow [previous tutorial about creating .msg files](/ROS/Tutorials/CreatingMsgAndSrv#Creating_a_msg "/ROS/Tutorials/CreatingMsgAndSrv#Creating_a_msg") (don't forget to choose build system type at the top of the page there). 
## Including or Importing Messages

### C++

Messages are put into a namespace that matches the name of the package. ie. 

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

document.write('<a href="#" onclick="return togglenumber(\'CA-8c8acf0cea72e86f6fa563c6cb301f649db09d79\', 1, 1);" \
 class="codenumbers">Toggle line numbers<\/a>');

```
 [1](#CA-8c8acf0cea72e86f6fa563c6cb301f649db09d79_1 "#CA-8c8acf0cea72e86f6fa563c6cb301f649db09d79_1") #include <std\_msgs/String.h>
 [2](#CA-8c8acf0cea72e86f6fa563c6cb301f649db09d79_2 "#CA-8c8acf0cea72e86f6fa563c6cb301f649db09d79_2") 
 [3](#CA-8c8acf0cea72e86f6fa563c6cb301f649db09d79_3 "#CA-8c8acf0cea72e86f6fa563c6cb301f649db09d79_3") std\_msgs::String msg;

```

### Python

document.write('<a href="#" onclick="return togglenumber(\'CA-3370392f843a00ef86c85a9b0065a438aeca9ac2\', 1, 1);" \
 class="codenumbers">Toggle line numbers<\/a>');

```
 [1](#CA-3370392f843a00ef86c85a9b0065a438aeca9ac2_1 "#CA-3370392f843a00ef86c85a9b0065a438aeca9ac2_1") from std\_msgs.msg import String
 [2](#CA-3370392f843a00ef86c85a9b0065a438aeca9ac2_2 "#CA-3370392f843a00ef86c85a9b0065a438aeca9ac2_2") 
 [3](#CA-3370392f843a00ef86c85a9b0065a438aeca9ac2_3 "#CA-3370392f843a00ef86c85a9b0065a438aeca9ac2_3") msg = String()

```

## Dependencies

If you are using the new custom message defined in a different package, remember to add: to manifest.xml: 
```
<depend package="name_of_package_containing_custom_msg"/>
```

to [package.xml](/catkin/package.xml "/catkin/package.xml"): 
```
<build_depend>name_of_package_containing_custom_msg</build_depend>
<exec_depend>name_of_package_containing_custom_msg</exec_depend>
```
and you will need to add this to your CMakeList.txt: 
```
add_dependencies(your_program ${catkin_EXPORTED_TARGETS})
```
If you are building C++ nodes which use your new messages, you will also need to declare a dependency between your node and your message, as described in the [catkin msg/srv build documentation](http://docs.ros.org/kinetic/api/catkin/html/howto/format2/cpp_msg_dependencies.html "http://docs.ros.org/kinetic/api/catkin/html/howto/format2/cpp_msg_dependencies.html") 

The [ROSNodeTutorialPython](/ROSNodeTutorialPython "/ROSNodeTutorialPython") tutorial shows an example of the previously described talker and listener tutorials using a custom message, with implementations in C++ and Python. 

Wiki: ROS/Tutorials/DefiningCustomMessages (last edited 2018-04-27 16:38:09 by [PaulBouchier](/PaulBouchier "PaulBouchier @ 144.121.11.130.lightower.net[144.121.11.130]"))

Except where otherwise noted, the ROS wiki is licensed under the   

