

 var \_gaq = \_gaq || [];
 \_gaq.push(['\_setAccount', 'UA-17821189-2']);
 \_gaq.push(['\_setDomainName', 'wiki.ros.org']);
 \_gaq.push(['\_trackPageview']);

 (function() {
 var ga = document.createElement('script'); ga.type = 'text/javascript'; ga.async = true;
 ga.src = ('https:' == document.location.protocol ? 'https://ssl' : 'http://www') + '.google-analytics.com/ga.js';
 var s = document.getElementsByTagName('script')[0]; s.parentNode.insertBefore(ga, s);
 })();

ROS/Tutorials/Using a C++ class in Python - ROS Wiki

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
* [Using a C++ class in Python](/ROS/Tutorials/Using%20a%20C%2B%2B%20class%20in%20Python "/ROS/Tutorials/Using%20a%20C%2B%2B%20class%20in%20Python")

#### ROS 2 Documentation

The ROS Wiki is for ROS 1. Are you using ROS 2 ([Humble](http://docs.ros.org/en/humble/ "http://docs.ros.org/en/humble/"), [Iron](http://docs.ros.org/en/iron/ "http://docs.ros.org/en/iron/"), or [Rolling](http://docs.ros.org/en/rolling/ "http://docs.ros.org/en/rolling/"))?   
[Check out the ROS 2 Project Documentation](http://docs.ros.org "http://docs.ros.org")  
Package specific documentation can be found on [index.ros.org](https://index.ros.org "https://index.ros.org")

# Wiki

* [Distributions](/Distributions "/Distributions")
* [ROS/Installation](/ROS/Installation "/ROS/Installation")
* [ROS/Tutorials](/ROS/Tutorials "/ROS/Tutorials")
* [RecentChanges](/RecentChanges "/RecentChanges")
* [Using a C++...s in Python](/ROS/Tutorials/Using%20a%20C%2B%2B%20class%20in%20Python "/ROS/Tutorials/Using%20a%20C%2B%2B%20class%20in%20Python")

# Page

* Immutable Page
* [Comments](# "#")
* [Info](/action/info/ROS/Tutorials/Using%20a%20C%2B%2B%20class%20in%20Python?action=info "/action/info/ROS/Tutorials/Using%20a%20C%2B%2B%20class%20in%20Python?action=info")
* [Attachments](/action/AttachFile/ROS/Tutorials/Using%20a%20C%2B%2B%20class%20in%20Python?action=AttachFile "/action/AttachFile/ROS/Tutorials/Using%20a%20C%2B%2B%20class%20in%20Python?action=AttachFile")
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

* [Login](/action/login/ROS/Tutorials/Using%20a%20C%2B%2B%20class%20in%20Python?action=login "/action/login/ROS/Tutorials/Using%20a%20C%2B%2B%20class%20in%20Python?action=login")

|  |
| --- |
| (!) Please ask about problems and questions regarding this tutorial on [answers.ros.org](http://answers.ros.org "http://answers.ros.org"). Don't forget to include in your question the link to this page, the versions of your OS & ROS, and also add appropriate tags. |

# Using a C++ class in Python

**Description:** This tutorial illustrates a way to use a C++ class with ROS messages in Python.  

**Keywords:** C++, Python, bindings  

**Tutorial Level:** ADVANCED   

**Next Tutorial:** [Packaging your ROS project as a snap](/ROS/Tutorials/Packaging%20your%20ROS%20project%20as%20a%20snap "/ROS/Tutorials/Packaging%20your%20ROS%20project%20as%20a%20snap")  

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
 Contents1. [Class without NodeHandle](#Class_without_NodeHandle "#Class_without_NodeHandle")
	1. [Creating the package and writing the C++ class](#Creating_the_package_and_writing_the_C.2B-.2B-_class "#Creating_the_package_and_writing_the_C.2B-.2B-_class")
	2. [Binding, C++ part](#Binding.2C_C.2B-.2B-_part "#Binding.2C_C.2B-.2B-_part")
	3. [Binding, Python part](#Binding.2C_Python_part "#Binding.2C_Python_part")
	4. [Glueing everything together](#Glueing_everything_together "#Glueing_everything_together")
	5. [Testing the binding](#Testing_the_binding "#Testing_the_binding")
2. [Class with NodeHandle](#Class_with_NodeHandle "#Class_with_NodeHandle")
3. [Class with container of ROS messages](#Class_with_container_of_ROS_messages "#Class_with_container_of_ROS_messages")
	1. [`std::vector<M>` as return type](#A.60std::vector.3CM.3E.60_as_return_type "#A.60std::vector.3CM.3E.60_as_return_type")
	2. [`std::vector<M>` as argument type](#A.60std::vector.3CM.3E.60_as_argument_type "#A.60std::vector.3CM.3E.60_as_argument_type")

 This tutorial illustrates a way to use a C++ class with ROS messages in Python. The Boost Python library is used. The difficulty is to translate Python objects of ROS messages written in pure Python into equivalent C++ instances. This translation will be done through serialization/deserialization. The source files can be found at [https://github.com/galou/python\_bindings\_tutorial](https://github.com/galou/python_bindings_tutorial "https://github.com/galou/python_bindings_tutorial"). Another solution is to use classical ROS services, the server written in C++ will be a wrapper around the C++ class and the client, C++ or Python, will call the service. The solution proposed here does not create a ROS node, provided the class to be wrapped does not make use of ros::NodeHandle. Another solution is to write a wrapper for all needed ROS messages and their dependencies. Some apparently deprecated package proposed some solutions for the automation of this task: [genpybindings](https://github.com/mkjaergaard/genpybindings.git "https://github.com/mkjaergaard/genpybindings.git") and [boost\_ python\_ros](https://github.com/bhaskara/boost_python_ros.git "https://github.com/bhaskara/boost_python_ros.git"). 
## Class without NodeHandle

Because roscpp is not initialized when calling rospy.init\_node. ros::NodeHandle instances cannot be used in the C++ class without generating an error. If the C++ does not make use of ros::NodeHandle, this is no issue though. 
### Creating the package and writing the C++ class

Create a package and create the C++ class for which we will want to make a Python binding. This class uses ROS messages as arguments and return type. 

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

document.write('<a href="#" onclick="return togglenumber(\'CA-22c83b30e90a8839880e376293ba5d1955d19547\', 1, 1);" \
 class="codenumbers">Toggle line numbers<\/a>');

```
 [1](#CA-22c83b30e90a8839880e376293ba5d1955d19547_1 "#CA-22c83b30e90a8839880e376293ba5d1955d19547_1") roscreate-pkg # ... to be completed
 [2](#CA-22c83b30e90a8839880e376293ba5d1955d19547_2 "#CA-22c83b30e90a8839880e376293ba5d1955d19547_2") cd python\_bindings\_tutorial/include/python\_bindings\_tutorial
 [3](#CA-22c83b30e90a8839880e376293ba5d1955d19547_3 "#CA-22c83b30e90a8839880e376293ba5d1955d19547_3") touch add\_two\_ints.h
 [4](#CA-22c83b30e90a8839880e376293ba5d1955d19547_4 "#CA-22c83b30e90a8839880e376293ba5d1955d19547_4") rosed python\_bindings\_tutorial add\_two\_ints.h

```

document.write('<a href="#" onclick="return togglenumber(\'CA-8aa525b03f510dc8c59c65b5ae73bbc6d8adfd8b\', 1, 1);" \
 class="codenumbers">Toggle line numbers<\/a>');

```
 [1](#CA-8aa525b03f510dc8c59c65b5ae73bbc6d8adfd8b_1 "#CA-8aa525b03f510dc8c59c65b5ae73bbc6d8adfd8b_1") catkin\_create\_pkg python\_bindings\_tutorial rospy roscpp std\_msgs
 [2](#CA-8aa525b03f510dc8c59c65b5ae73bbc6d8adfd8b_2 "#CA-8aa525b03f510dc8c59c65b5ae73bbc6d8adfd8b_2") cd python\_bindings\_tutorial/include/python\_bindings\_tutorial
 [3](#CA-8aa525b03f510dc8c59c65b5ae73bbc6d8adfd8b_3 "#CA-8aa525b03f510dc8c59c65b5ae73bbc6d8adfd8b_3") touch add\_two\_ints.h
 [4](#CA-8aa525b03f510dc8c59c65b5ae73bbc6d8adfd8b_4 "#CA-8aa525b03f510dc8c59c65b5ae73bbc6d8adfd8b_4") rosed python\_bindings\_tutorial add\_two\_ints.h

```

The content of include/python\_bindings\_tutorial/add\_two\_ints.h will be: 

document.write('<a href="#" onclick="return togglenumber(\'CA-6f94133aee876881b08e87046574b078b53285e0\', 1, 1);" \
 class="codenumbers">Toggle line numbers<\/a>');

```
 [1](#CA-6f94133aee876881b08e87046574b078b53285e0_1 "#CA-6f94133aee876881b08e87046574b078b53285e0_1") #ifndef PYTHON\_BINDINGS\_TUTORIAL\_ADD\_TWO\_INTS\_H
 [2](#CA-6f94133aee876881b08e87046574b078b53285e0_2 "#CA-6f94133aee876881b08e87046574b078b53285e0_2") #define PYTHON\_BINDINGS\_TUTORIAL\_ADD\_TWO\_INTS\_H
 [3](#CA-6f94133aee876881b08e87046574b078b53285e0_3 "#CA-6f94133aee876881b08e87046574b078b53285e0_3") 
 [4](#CA-6f94133aee876881b08e87046574b078b53285e0_4 "#CA-6f94133aee876881b08e87046574b078b53285e0_4") #include <std\_msgs/Int64.h>
 [5](#CA-6f94133aee876881b08e87046574b078b53285e0_5 "#CA-6f94133aee876881b08e87046574b078b53285e0_5") 
 [6](#CA-6f94133aee876881b08e87046574b078b53285e0_6 "#CA-6f94133aee876881b08e87046574b078b53285e0_6") namespace python\_bindings\_tutorial {
 [7](#CA-6f94133aee876881b08e87046574b078b53285e0_7 "#CA-6f94133aee876881b08e87046574b078b53285e0_7") 
 [8](#CA-6f94133aee876881b08e87046574b078b53285e0_8 "#CA-6f94133aee876881b08e87046574b078b53285e0_8") class AddTwoInts
 [9](#CA-6f94133aee876881b08e87046574b078b53285e0_9 "#CA-6f94133aee876881b08e87046574b078b53285e0_9") {
 [10](#CA-6f94133aee876881b08e87046574b078b53285e0_10 "#CA-6f94133aee876881b08e87046574b078b53285e0_10")  public:
 [11](#CA-6f94133aee876881b08e87046574b078b53285e0_11 "#CA-6f94133aee876881b08e87046574b078b53285e0_11")  std\_msgs::Int64 add(const std\_msgs::Int64& a, const std\_msgs::Int64& b);
 [12](#CA-6f94133aee876881b08e87046574b078b53285e0_12 "#CA-6f94133aee876881b08e87046574b078b53285e0_12") };
 [13](#CA-6f94133aee876881b08e87046574b078b53285e0_13 "#CA-6f94133aee876881b08e87046574b078b53285e0_13") 
 [14](#CA-6f94133aee876881b08e87046574b078b53285e0_14 "#CA-6f94133aee876881b08e87046574b078b53285e0_14") } // namespace python\_bindings\_tutorial
 [15](#CA-6f94133aee876881b08e87046574b078b53285e0_15 "#CA-6f94133aee876881b08e87046574b078b53285e0_15") 
 [16](#CA-6f94133aee876881b08e87046574b078b53285e0_16 "#CA-6f94133aee876881b08e87046574b078b53285e0_16") #endif // PYTHON\_BINDINGS\_TUTORIAL\_ADD\_TWO\_INTS\_H
 [17](#CA-6f94133aee876881b08e87046574b078b53285e0_17 "#CA-6f94133aee876881b08e87046574b078b53285e0_17") 

```
Write the class implementation into . 
```
roscd python_bindings_tutorial/src
touch add_two_ints.cpp
rosed python_bindings_tutorial add_two_ints.cpp
```
The content of src/add\_two\_ints.cpp will be: 

document.write('<a href="#" onclick="return togglenumber(\'CA-3ac1133f8f0ddbe3d9f9a51fc2771329f6cd18ff\', 1, 1);" \
 class="codenumbers">Toggle line numbers<\/a>');

```
 [1](#CA-3ac1133f8f0ddbe3d9f9a51fc2771329f6cd18ff_1 "#CA-3ac1133f8f0ddbe3d9f9a51fc2771329f6cd18ff_1") #include <python\_bindings\_tutorial/add\_two\_ints.h>
 [2](#CA-3ac1133f8f0ddbe3d9f9a51fc2771329f6cd18ff_2 "#CA-3ac1133f8f0ddbe3d9f9a51fc2771329f6cd18ff_2") 
 [3](#CA-3ac1133f8f0ddbe3d9f9a51fc2771329f6cd18ff_3 "#CA-3ac1133f8f0ddbe3d9f9a51fc2771329f6cd18ff_3") using namespace python\_bindings\_tutorial;
 [4](#CA-3ac1133f8f0ddbe3d9f9a51fc2771329f6cd18ff_4 "#CA-3ac1133f8f0ddbe3d9f9a51fc2771329f6cd18ff_4") 
 [5](#CA-3ac1133f8f0ddbe3d9f9a51fc2771329f6cd18ff_5 "#CA-3ac1133f8f0ddbe3d9f9a51fc2771329f6cd18ff_5") std\_msgs::Int64 AddTwoInts::add(const std\_msgs::Int64& a, const std\_msgs::Int64& b)
 [6](#CA-3ac1133f8f0ddbe3d9f9a51fc2771329f6cd18ff_6 "#CA-3ac1133f8f0ddbe3d9f9a51fc2771329f6cd18ff_6") {
 [7](#CA-3ac1133f8f0ddbe3d9f9a51fc2771329f6cd18ff_7 "#CA-3ac1133f8f0ddbe3d9f9a51fc2771329f6cd18ff_7")  std\_msgs::Int64 sum;
 [8](#CA-3ac1133f8f0ddbe3d9f9a51fc2771329f6cd18ff_8 "#CA-3ac1133f8f0ddbe3d9f9a51fc2771329f6cd18ff_8")  sum.data = a.data + b.data;
 [9](#CA-3ac1133f8f0ddbe3d9f9a51fc2771329f6cd18ff_9 "#CA-3ac1133f8f0ddbe3d9f9a51fc2771329f6cd18ff_9")  return sum;
 [10](#CA-3ac1133f8f0ddbe3d9f9a51fc2771329f6cd18ff_10 "#CA-3ac1133f8f0ddbe3d9f9a51fc2771329f6cd18ff_10") }

```

### Binding, C++ part

The binding occurs through two wrapper classes, one in C++ and one in Python. The C++ wrapper translates input from serialized content to C++ message instances and output from C++ message instances into serialized content. 

document.write('<a href="#" onclick="return togglenumber(\'CA-0e063beacad7ea41aeed8e75c49661d7a2f4fa1a\', 1, 1);" \
 class="codenumbers">Toggle line numbers<\/a>');

```
 [1](#CA-0e063beacad7ea41aeed8e75c49661d7a2f4fa1a_1 "#CA-0e063beacad7ea41aeed8e75c49661d7a2f4fa1a_1") roscd python\_bindings\_tutorial/src
 [2](#CA-0e063beacad7ea41aeed8e75c49661d7a2f4fa1a_2 "#CA-0e063beacad7ea41aeed8e75c49661d7a2f4fa1a_2") touch add\_two\_ints\_wrapper.cpp
 [3](#CA-0e063beacad7ea41aeed8e75c49661d7a2f4fa1a_3 "#CA-0e063beacad7ea41aeed8e75c49661d7a2f4fa1a_3") rosed python\_bindings\_tutorial add\_two\_ints\_wrapper.cpp

```
The content of src/add\_two\_ints\_wrapper.cpp will be: 

document.write('<a href="#" onclick="return togglenumber(\'CA-f80823c1a504b34d25b1cfddb17468cbc3d0fbff\', 1, 1);" \
 class="codenumbers">Toggle line numbers<\/a>');

```
 [1](#CA-f80823c1a504b34d25b1cfddb17468cbc3d0fbff_1 "#CA-f80823c1a504b34d25b1cfddb17468cbc3d0fbff_1") #include <boost/python.hpp>
 [2](#CA-f80823c1a504b34d25b1cfddb17468cbc3d0fbff_2 "#CA-f80823c1a504b34d25b1cfddb17468cbc3d0fbff_2") 
 [3](#CA-f80823c1a504b34d25b1cfddb17468cbc3d0fbff_3 "#CA-f80823c1a504b34d25b1cfddb17468cbc3d0fbff_3") #include <string>
 [4](#CA-f80823c1a504b34d25b1cfddb17468cbc3d0fbff_4 "#CA-f80823c1a504b34d25b1cfddb17468cbc3d0fbff_4") 
 [5](#CA-f80823c1a504b34d25b1cfddb17468cbc3d0fbff_5 "#CA-f80823c1a504b34d25b1cfddb17468cbc3d0fbff_5") #include <ros/serialization.h>
 [6](#CA-f80823c1a504b34d25b1cfddb17468cbc3d0fbff_6 "#CA-f80823c1a504b34d25b1cfddb17468cbc3d0fbff_6") #include <std\_msgs/Int64.h>
 [7](#CA-f80823c1a504b34d25b1cfddb17468cbc3d0fbff_7 "#CA-f80823c1a504b34d25b1cfddb17468cbc3d0fbff_7") 
 [8](#CA-f80823c1a504b34d25b1cfddb17468cbc3d0fbff_8 "#CA-f80823c1a504b34d25b1cfddb17468cbc3d0fbff_8") #include <python\_bindings\_tutorial/add\_two\_ints.h>
 [9](#CA-f80823c1a504b34d25b1cfddb17468cbc3d0fbff_9 "#CA-f80823c1a504b34d25b1cfddb17468cbc3d0fbff_9") 
 [10](#CA-f80823c1a504b34d25b1cfddb17468cbc3d0fbff_10 "#CA-f80823c1a504b34d25b1cfddb17468cbc3d0fbff_10") /\* Read a ROS message from a serialized string.
 [11](#CA-f80823c1a504b34d25b1cfddb17468cbc3d0fbff_11 "#CA-f80823c1a504b34d25b1cfddb17468cbc3d0fbff_11")  \*/
 [12](#CA-f80823c1a504b34d25b1cfddb17468cbc3d0fbff_12 "#CA-f80823c1a504b34d25b1cfddb17468cbc3d0fbff_12") template <typename M>
 [13](#CA-f80823c1a504b34d25b1cfddb17468cbc3d0fbff_13 "#CA-f80823c1a504b34d25b1cfddb17468cbc3d0fbff_13") M from\_python(const std::string str\_msg)
 [14](#CA-f80823c1a504b34d25b1cfddb17468cbc3d0fbff_14 "#CA-f80823c1a504b34d25b1cfddb17468cbc3d0fbff_14") {
 [15](#CA-f80823c1a504b34d25b1cfddb17468cbc3d0fbff_15 "#CA-f80823c1a504b34d25b1cfddb17468cbc3d0fbff_15")  size\_t serial\_size = str\_msg.size();
 [16](#CA-f80823c1a504b34d25b1cfddb17468cbc3d0fbff_16 "#CA-f80823c1a504b34d25b1cfddb17468cbc3d0fbff_16")  boost::shared\_array<uint8\_t> buffer(new uint8\_t[serial\_size]);
 [17](#CA-f80823c1a504b34d25b1cfddb17468cbc3d0fbff_17 "#CA-f80823c1a504b34d25b1cfddb17468cbc3d0fbff_17")  for (size\_t i = 0; i < serial\_size; ++i)
 [18](#CA-f80823c1a504b34d25b1cfddb17468cbc3d0fbff_18 "#CA-f80823c1a504b34d25b1cfddb17468cbc3d0fbff_18")  {
 [19](#CA-f80823c1a504b34d25b1cfddb17468cbc3d0fbff_19 "#CA-f80823c1a504b34d25b1cfddb17468cbc3d0fbff_19")  buffer[i] = str\_msg[i];
 [20](#CA-f80823c1a504b34d25b1cfddb17468cbc3d0fbff_20 "#CA-f80823c1a504b34d25b1cfddb17468cbc3d0fbff_20")  }
 [21](#CA-f80823c1a504b34d25b1cfddb17468cbc3d0fbff_21 "#CA-f80823c1a504b34d25b1cfddb17468cbc3d0fbff_21")  ros::serialization::IStream stream(buffer.get(), serial\_size);
 [22](#CA-f80823c1a504b34d25b1cfddb17468cbc3d0fbff_22 "#CA-f80823c1a504b34d25b1cfddb17468cbc3d0fbff_22")  M msg;
 [23](#CA-f80823c1a504b34d25b1cfddb17468cbc3d0fbff_23 "#CA-f80823c1a504b34d25b1cfddb17468cbc3d0fbff_23")  ros::serialization::Serializer<M>::read(stream, msg);
 [24](#CA-f80823c1a504b34d25b1cfddb17468cbc3d0fbff_24 "#CA-f80823c1a504b34d25b1cfddb17468cbc3d0fbff_24")  return msg;
 [25](#CA-f80823c1a504b34d25b1cfddb17468cbc3d0fbff_25 "#CA-f80823c1a504b34d25b1cfddb17468cbc3d0fbff_25") }
 [26](#CA-f80823c1a504b34d25b1cfddb17468cbc3d0fbff_26 "#CA-f80823c1a504b34d25b1cfddb17468cbc3d0fbff_26") 
 [27](#CA-f80823c1a504b34d25b1cfddb17468cbc3d0fbff_27 "#CA-f80823c1a504b34d25b1cfddb17468cbc3d0fbff_27") /\* Write a ROS message into a serialized string.
 [28](#CA-f80823c1a504b34d25b1cfddb17468cbc3d0fbff_28 "#CA-f80823c1a504b34d25b1cfddb17468cbc3d0fbff_28") \*/
 [29](#CA-f80823c1a504b34d25b1cfddb17468cbc3d0fbff_29 "#CA-f80823c1a504b34d25b1cfddb17468cbc3d0fbff_29") template <typename M>
 [30](#CA-f80823c1a504b34d25b1cfddb17468cbc3d0fbff_30 "#CA-f80823c1a504b34d25b1cfddb17468cbc3d0fbff_30") std::string to\_python(const M& msg)
 [31](#CA-f80823c1a504b34d25b1cfddb17468cbc3d0fbff_31 "#CA-f80823c1a504b34d25b1cfddb17468cbc3d0fbff_31") {
 [32](#CA-f80823c1a504b34d25b1cfddb17468cbc3d0fbff_32 "#CA-f80823c1a504b34d25b1cfddb17468cbc3d0fbff_32")  size\_t serial\_size = ros::serialization::serializationLength(msg);
 [33](#CA-f80823c1a504b34d25b1cfddb17468cbc3d0fbff_33 "#CA-f80823c1a504b34d25b1cfddb17468cbc3d0fbff_33")  boost::shared\_array<uint8\_t> buffer(new uint8\_t[serial\_size]);
 [34](#CA-f80823c1a504b34d25b1cfddb17468cbc3d0fbff_34 "#CA-f80823c1a504b34d25b1cfddb17468cbc3d0fbff_34")  ros::serialization::OStream stream(buffer.get(), serial\_size);
 [35](#CA-f80823c1a504b34d25b1cfddb17468cbc3d0fbff_35 "#CA-f80823c1a504b34d25b1cfddb17468cbc3d0fbff_35")  ros::serialization::serialize(stream, msg);
 [36](#CA-f80823c1a504b34d25b1cfddb17468cbc3d0fbff_36 "#CA-f80823c1a504b34d25b1cfddb17468cbc3d0fbff_36")  std::string str\_msg;
 [37](#CA-f80823c1a504b34d25b1cfddb17468cbc3d0fbff_37 "#CA-f80823c1a504b34d25b1cfddb17468cbc3d0fbff_37")  str\_msg.reserve(serial\_size);
 [38](#CA-f80823c1a504b34d25b1cfddb17468cbc3d0fbff_38 "#CA-f80823c1a504b34d25b1cfddb17468cbc3d0fbff_38")  for (size\_t i = 0; i < serial\_size; ++i)
 [39](#CA-f80823c1a504b34d25b1cfddb17468cbc3d0fbff_39 "#CA-f80823c1a504b34d25b1cfddb17468cbc3d0fbff_39")  {
 [40](#CA-f80823c1a504b34d25b1cfddb17468cbc3d0fbff_40 "#CA-f80823c1a504b34d25b1cfddb17468cbc3d0fbff_40")  str\_msg.push\_back(buffer[i]);
 [41](#CA-f80823c1a504b34d25b1cfddb17468cbc3d0fbff_41 "#CA-f80823c1a504b34d25b1cfddb17468cbc3d0fbff_41")  }
 [42](#CA-f80823c1a504b34d25b1cfddb17468cbc3d0fbff_42 "#CA-f80823c1a504b34d25b1cfddb17468cbc3d0fbff_42")  return str\_msg;
 [43](#CA-f80823c1a504b34d25b1cfddb17468cbc3d0fbff_43 "#CA-f80823c1a504b34d25b1cfddb17468cbc3d0fbff_43") }
 [44](#CA-f80823c1a504b34d25b1cfddb17468cbc3d0fbff_44 "#CA-f80823c1a504b34d25b1cfddb17468cbc3d0fbff_44") 
 [45](#CA-f80823c1a504b34d25b1cfddb17468cbc3d0fbff_45 "#CA-f80823c1a504b34d25b1cfddb17468cbc3d0fbff_45") class AddTwoIntsWrapper : public python\_bindings\_tutorial::AddTwoInts
 [46](#CA-f80823c1a504b34d25b1cfddb17468cbc3d0fbff_46 "#CA-f80823c1a504b34d25b1cfddb17468cbc3d0fbff_46") {
 [47](#CA-f80823c1a504b34d25b1cfddb17468cbc3d0fbff_47 "#CA-f80823c1a504b34d25b1cfddb17468cbc3d0fbff_47")  public:
 [48](#CA-f80823c1a504b34d25b1cfddb17468cbc3d0fbff_48 "#CA-f80823c1a504b34d25b1cfddb17468cbc3d0fbff_48")  AddTwoIntsWrapper() : AddTwoInts() {}
 [49](#CA-f80823c1a504b34d25b1cfddb17468cbc3d0fbff_49 "#CA-f80823c1a504b34d25b1cfddb17468cbc3d0fbff_49") 
 [50](#CA-f80823c1a504b34d25b1cfddb17468cbc3d0fbff_50 "#CA-f80823c1a504b34d25b1cfddb17468cbc3d0fbff_50")  std::string add(const std::string& str\_a, const std::string& str\_b)
 [51](#CA-f80823c1a504b34d25b1cfddb17468cbc3d0fbff_51 "#CA-f80823c1a504b34d25b1cfddb17468cbc3d0fbff_51")  {
 [52](#CA-f80823c1a504b34d25b1cfddb17468cbc3d0fbff_52 "#CA-f80823c1a504b34d25b1cfddb17468cbc3d0fbff_52")  std\_msgs::Int64 a = from\_python<std\_msgs::Int64>(str\_a);
 [53](#CA-f80823c1a504b34d25b1cfddb17468cbc3d0fbff_53 "#CA-f80823c1a504b34d25b1cfddb17468cbc3d0fbff_53")  std\_msgs::Int64 b = from\_python<std\_msgs::Int64>(str\_b);
 [54](#CA-f80823c1a504b34d25b1cfddb17468cbc3d0fbff_54 "#CA-f80823c1a504b34d25b1cfddb17468cbc3d0fbff_54")  std\_msgs::Int64 sum = AddTwoInts::add(a, b);
 [55](#CA-f80823c1a504b34d25b1cfddb17468cbc3d0fbff_55 "#CA-f80823c1a504b34d25b1cfddb17468cbc3d0fbff_55") 
 [56](#CA-f80823c1a504b34d25b1cfddb17468cbc3d0fbff_56 "#CA-f80823c1a504b34d25b1cfddb17468cbc3d0fbff_56")  return to\_python(sum);
 [57](#CA-f80823c1a504b34d25b1cfddb17468cbc3d0fbff_57 "#CA-f80823c1a504b34d25b1cfddb17468cbc3d0fbff_57")  }
 [58](#CA-f80823c1a504b34d25b1cfddb17468cbc3d0fbff_58 "#CA-f80823c1a504b34d25b1cfddb17468cbc3d0fbff_58") };
 [59](#CA-f80823c1a504b34d25b1cfddb17468cbc3d0fbff_59 "#CA-f80823c1a504b34d25b1cfddb17468cbc3d0fbff_59") 
 [60](#CA-f80823c1a504b34d25b1cfddb17468cbc3d0fbff_60 "#CA-f80823c1a504b34d25b1cfddb17468cbc3d0fbff_60") BOOST\_PYTHON\_MODULE(\_add\_two\_ints\_wrapper\_cpp)
 [61](#CA-f80823c1a504b34d25b1cfddb17468cbc3d0fbff_61 "#CA-f80823c1a504b34d25b1cfddb17468cbc3d0fbff_61") {
 [62](#CA-f80823c1a504b34d25b1cfddb17468cbc3d0fbff_62 "#CA-f80823c1a504b34d25b1cfddb17468cbc3d0fbff_62")  boost::python::class\_<AddTwoIntsWrapper>("AddTwoIntsWrapper", boost::python::init<>())
 [63](#CA-f80823c1a504b34d25b1cfddb17468cbc3d0fbff_63 "#CA-f80823c1a504b34d25b1cfddb17468cbc3d0fbff_63")  .def("add", &AddTwoIntsWrapper::add)
 [64](#CA-f80823c1a504b34d25b1cfddb17468cbc3d0fbff_64 "#CA-f80823c1a504b34d25b1cfddb17468cbc3d0fbff_64")  ;
 [65](#CA-f80823c1a504b34d25b1cfddb17468cbc3d0fbff_65 "#CA-f80823c1a504b34d25b1cfddb17468cbc3d0fbff_65") }

```
The line Error: No code\_block found creates a Python module in the form of a dynamic library. The name of the module will have to be the name of the exported library in CMakeLists.txt. 
### Binding, Python part

The Python wrapper translates input from Python message instances into serialized content and output from serialized content to Python message instances. The translation from Python serialized content (str) into C++ serialized content (std::string) is built in the Boost Python library. 
```
roscd python_bindings_tutorial/src
mkdir python_bindings_tutorial
roscd python_bindings_tutorial/src/python_bindings_tutorial
touch _add_two_ints_wrapper_py.py
rosed python_bindings_tutorial _add_two_ints_wrapper_py.py
```
The content of src/python\_bindings\_tutorial/\_add\_two\_ints\_wrapper\_py.py will be 

document.write('<a href="#" onclick="return togglenumber(\'CA-b40e1737bd854731702e99e0a9c37a7984f06e3f\', 1, 1);" \
 class="codenumbers">Toggle line numbers<\/a>');

```
 [1](#CA-b40e1737bd854731702e99e0a9c37a7984f06e3f_1 "#CA-b40e1737bd854731702e99e0a9c37a7984f06e3f_1") from StringIO import StringIO
 [2](#CA-b40e1737bd854731702e99e0a9c37a7984f06e3f_2 "#CA-b40e1737bd854731702e99e0a9c37a7984f06e3f_2") 
 [3](#CA-b40e1737bd854731702e99e0a9c37a7984f06e3f_3 "#CA-b40e1737bd854731702e99e0a9c37a7984f06e3f_3") import rospy
 [4](#CA-b40e1737bd854731702e99e0a9c37a7984f06e3f_4 "#CA-b40e1737bd854731702e99e0a9c37a7984f06e3f_4") from std\_msgs.msg import Int64
 [5](#CA-b40e1737bd854731702e99e0a9c37a7984f06e3f_5 "#CA-b40e1737bd854731702e99e0a9c37a7984f06e3f_5") 
 [6](#CA-b40e1737bd854731702e99e0a9c37a7984f06e3f_6 "#CA-b40e1737bd854731702e99e0a9c37a7984f06e3f_6") from python\_bindings\_tutorial.\_add\_two\_ints\_wrapper\_cpp import AddTwoIntsWrapper
 [7](#CA-b40e1737bd854731702e99e0a9c37a7984f06e3f_7 "#CA-b40e1737bd854731702e99e0a9c37a7984f06e3f_7") 
 [8](#CA-b40e1737bd854731702e99e0a9c37a7984f06e3f_8 "#CA-b40e1737bd854731702e99e0a9c37a7984f06e3f_8") 
 [9](#CA-b40e1737bd854731702e99e0a9c37a7984f06e3f_9 "#CA-b40e1737bd854731702e99e0a9c37a7984f06e3f_9") class AddTwoInts(object):
 [10](#CA-b40e1737bd854731702e99e0a9c37a7984f06e3f_10 "#CA-b40e1737bd854731702e99e0a9c37a7984f06e3f_10")  def \_\_init\_\_(self):
 [11](#CA-b40e1737bd854731702e99e0a9c37a7984f06e3f_11 "#CA-b40e1737bd854731702e99e0a9c37a7984f06e3f_11")  self.\_add\_two\_ints = AddTwoIntsWrapper()
 [12](#CA-b40e1737bd854731702e99e0a9c37a7984f06e3f_12 "#CA-b40e1737bd854731702e99e0a9c37a7984f06e3f_12") 
 [13](#CA-b40e1737bd854731702e99e0a9c37a7984f06e3f_13 "#CA-b40e1737bd854731702e99e0a9c37a7984f06e3f_13")  def \_to\_cpp(self, msg):
 [14](#CA-b40e1737bd854731702e99e0a9c37a7984f06e3f_14 "#CA-b40e1737bd854731702e99e0a9c37a7984f06e3f_14")  """Return a serialized string from a ROS message
 [15](#CA-b40e1737bd854731702e99e0a9c37a7984f06e3f_15 "#CA-b40e1737bd854731702e99e0a9c37a7984f06e3f_15") 
 [16](#CA-b40e1737bd854731702e99e0a9c37a7984f06e3f_16 "#CA-b40e1737bd854731702e99e0a9c37a7984f06e3f_16")  Parameters
 [17](#CA-b40e1737bd854731702e99e0a9c37a7984f06e3f_17 "#CA-b40e1737bd854731702e99e0a9c37a7984f06e3f_17")  ----------
 [18](#CA-b40e1737bd854731702e99e0a9c37a7984f06e3f_18 "#CA-b40e1737bd854731702e99e0a9c37a7984f06e3f_18")  - msg: a ROS message instance.
 [19](#CA-b40e1737bd854731702e99e0a9c37a7984f06e3f_19 "#CA-b40e1737bd854731702e99e0a9c37a7984f06e3f_19")  """
 [20](#CA-b40e1737bd854731702e99e0a9c37a7984f06e3f_20 "#CA-b40e1737bd854731702e99e0a9c37a7984f06e3f_20")  buf = StringIO()
 [21](#CA-b40e1737bd854731702e99e0a9c37a7984f06e3f_21 "#CA-b40e1737bd854731702e99e0a9c37a7984f06e3f_21")  msg.serialize(buf)
 [22](#CA-b40e1737bd854731702e99e0a9c37a7984f06e3f_22 "#CA-b40e1737bd854731702e99e0a9c37a7984f06e3f_22")  return buf.getvalue()
 [23](#CA-b40e1737bd854731702e99e0a9c37a7984f06e3f_23 "#CA-b40e1737bd854731702e99e0a9c37a7984f06e3f_23") 
 [24](#CA-b40e1737bd854731702e99e0a9c37a7984f06e3f_24 "#CA-b40e1737bd854731702e99e0a9c37a7984f06e3f_24")  def \_from\_cpp(self, str\_msg, cls):
 [25](#CA-b40e1737bd854731702e99e0a9c37a7984f06e3f_25 "#CA-b40e1737bd854731702e99e0a9c37a7984f06e3f_25")  """Return a ROS message from a serialized string
 [26](#CA-b40e1737bd854731702e99e0a9c37a7984f06e3f_26 "#CA-b40e1737bd854731702e99e0a9c37a7984f06e3f_26") 
 [27](#CA-b40e1737bd854731702e99e0a9c37a7984f06e3f_27 "#CA-b40e1737bd854731702e99e0a9c37a7984f06e3f_27")  Parameters
 [28](#CA-b40e1737bd854731702e99e0a9c37a7984f06e3f_28 "#CA-b40e1737bd854731702e99e0a9c37a7984f06e3f_28")  ----------
 [29](#CA-b40e1737bd854731702e99e0a9c37a7984f06e3f_29 "#CA-b40e1737bd854731702e99e0a9c37a7984f06e3f_29")  - str\_msg: str, serialized message
 [30](#CA-b40e1737bd854731702e99e0a9c37a7984f06e3f_30 "#CA-b40e1737bd854731702e99e0a9c37a7984f06e3f_30")  - cls: ROS message class, e.g. sensor\_msgs.msg.LaserScan.
 [31](#CA-b40e1737bd854731702e99e0a9c37a7984f06e3f_31 "#CA-b40e1737bd854731702e99e0a9c37a7984f06e3f_31")  """
 [32](#CA-b40e1737bd854731702e99e0a9c37a7984f06e3f_32 "#CA-b40e1737bd854731702e99e0a9c37a7984f06e3f_32")  msg = cls()
 [33](#CA-b40e1737bd854731702e99e0a9c37a7984f06e3f_33 "#CA-b40e1737bd854731702e99e0a9c37a7984f06e3f_33")  return msg.deserialize(str\_msg)
 [34](#CA-b40e1737bd854731702e99e0a9c37a7984f06e3f_34 "#CA-b40e1737bd854731702e99e0a9c37a7984f06e3f_34") 
 [35](#CA-b40e1737bd854731702e99e0a9c37a7984f06e3f_35 "#CA-b40e1737bd854731702e99e0a9c37a7984f06e3f_35")  def add(self, a, b):
 [36](#CA-b40e1737bd854731702e99e0a9c37a7984f06e3f_36 "#CA-b40e1737bd854731702e99e0a9c37a7984f06e3f_36")  """Add two std\_mgs/Int64 messages
 [37](#CA-b40e1737bd854731702e99e0a9c37a7984f06e3f_37 "#CA-b40e1737bd854731702e99e0a9c37a7984f06e3f_37") 
 [38](#CA-b40e1737bd854731702e99e0a9c37a7984f06e3f_38 "#CA-b40e1737bd854731702e99e0a9c37a7984f06e3f_38")  Return a std\_msgs/Int64 instance.
 [39](#CA-b40e1737bd854731702e99e0a9c37a7984f06e3f_39 "#CA-b40e1737bd854731702e99e0a9c37a7984f06e3f_39") 
 [40](#CA-b40e1737bd854731702e99e0a9c37a7984f06e3f_40 "#CA-b40e1737bd854731702e99e0a9c37a7984f06e3f_40")  Parameters
 [41](#CA-b40e1737bd854731702e99e0a9c37a7984f06e3f_41 "#CA-b40e1737bd854731702e99e0a9c37a7984f06e3f_41")  ----------
 [42](#CA-b40e1737bd854731702e99e0a9c37a7984f06e3f_42 "#CA-b40e1737bd854731702e99e0a9c37a7984f06e3f_42")  - a: a std\_msgs/Int64 instance.
 [43](#CA-b40e1737bd854731702e99e0a9c37a7984f06e3f_43 "#CA-b40e1737bd854731702e99e0a9c37a7984f06e3f_43")  - b: a std\_msgs/Int64 instance.
 [44](#CA-b40e1737bd854731702e99e0a9c37a7984f06e3f_44 "#CA-b40e1737bd854731702e99e0a9c37a7984f06e3f_44")  """
 [45](#CA-b40e1737bd854731702e99e0a9c37a7984f06e3f_45 "#CA-b40e1737bd854731702e99e0a9c37a7984f06e3f_45")  if not isinstance(a, Int64):
 [46](#CA-b40e1737bd854731702e99e0a9c37a7984f06e3f_46 "#CA-b40e1737bd854731702e99e0a9c37a7984f06e3f_46")  rospy.ROSException('Argument 1 is not a std\_msgs/Int64')
 [47](#CA-b40e1737bd854731702e99e0a9c37a7984f06e3f_47 "#CA-b40e1737bd854731702e99e0a9c37a7984f06e3f_47")  if not isinstance(b, Int64):
 [48](#CA-b40e1737bd854731702e99e0a9c37a7984f06e3f_48 "#CA-b40e1737bd854731702e99e0a9c37a7984f06e3f_48")  rospy.ROSException('Argument 2 is not a std\_msgs/Int64')
 [49](#CA-b40e1737bd854731702e99e0a9c37a7984f06e3f_49 "#CA-b40e1737bd854731702e99e0a9c37a7984f06e3f_49")  str\_a = self.\_to\_cpp(a)
 [50](#CA-b40e1737bd854731702e99e0a9c37a7984f06e3f_50 "#CA-b40e1737bd854731702e99e0a9c37a7984f06e3f_50")  str\_b = self.\_to\_cpp(b)
 [51](#CA-b40e1737bd854731702e99e0a9c37a7984f06e3f_51 "#CA-b40e1737bd854731702e99e0a9c37a7984f06e3f_51")  str\_sum = self.\_add\_two\_ints.add(str\_a, str\_b)
 [52](#CA-b40e1737bd854731702e99e0a9c37a7984f06e3f_52 "#CA-b40e1737bd854731702e99e0a9c37a7984f06e3f_52")  return self.\_from\_cpp(str\_sum, Int64)

```
In order to be able to import the class as python\_bindings\_tutorial.AddTwoInts, we import the symbols in \_\_init\_\_.py. First, we create the file: 

document.write('<a href="#" onclick="return togglenumber(\'CA-77e18c98840b944466d9264e88b96de228128f89\', 1, 1);" \
 class="codenumbers">Toggle line numbers<\/a>');

```
 [1](#CA-77e18c98840b944466d9264e88b96de228128f89_1 "#CA-77e18c98840b944466d9264e88b96de228128f89_1") roscd python\_bindings\_tutorial/src/python\_bindings\_tutorial
 [2](#CA-77e18c98840b944466d9264e88b96de228128f89_2 "#CA-77e18c98840b944466d9264e88b96de228128f89_2") touch \_\_init\_\_.py
 [3](#CA-77e18c98840b944466d9264e88b96de228128f89_3 "#CA-77e18c98840b944466d9264e88b96de228128f89_3") rosed python\_bindings\_tutorial \_\_init\_\_.py

```
The content of src/python\_bindings\_tutorial/\_\_init\_\_.py will be: 

document.write('<a href="#" onclick="return togglenumber(\'CA-98ea88fd4572e2d6b87c81ddd52b753920d9359f\', 1, 1);" \
 class="codenumbers">Toggle line numbers<\/a>');

```
 [1](#CA-98ea88fd4572e2d6b87c81ddd52b753920d9359f_1 "#CA-98ea88fd4572e2d6b87c81ddd52b753920d9359f_1") from python\_bindings\_tutorial.\_add\_two\_ints\_wrapper\_py import AddTwoInts

```

### Glueing everything together

Edit the CMakeLists.txt (rosed python\_bindings\_tutorial CmakeLists.txt) like this: 
```
Adapt CMakeLists.txt from the catkin version of this page.
```

```
cmake_minimum_required(VERSION 2.8.3)
project(python_bindings_tutorial)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roscpp_serialization
  std_msgs
)

## Both Boost.python and Python libs are required.
find_package(Boost REQUIRED COMPONENTS python)
find_package(PythonLibs 2.7 REQUIRED)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
catkin_python_setup()

###################################
## catkin specific configuration ##
###################################
catkin_package(
        INCLUDE_DIRS include
        LIBRARIES add_two_ints _add_two_ints_wrapper_cpp
        CATKIN_DEPENDS roscpp
        #  DEPENDS system_lib
)

###########
## Build ##
###########

# include Boost and Python.
include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIRS}
        ${PYTHON_INCLUDE_DIRS}
        )

## Declare a cpp library
add_library(add_two_ints src/add_two_ints.cpp)
add_library(_add_two_ints_wrapper_cpp src/add_two_ints_wrapper.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(add_two_ints ${catkin_LIBRARIES})
target_link_libraries(_add_two_ints_wrapper_cpp add_two_ints ${catkin_LIBRARIES} ${Boost_LIBRARIES})

# Don't prepend wrapper library name with lib and add to Python libs.
set_target_properties(_add_two_ints_wrapper_cpp PROPERTIES
        PREFIX ""
        LIBRARY_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION}
        )
```
The c++ wrapper library should be have the same name as the Python module. If the target name needs to be different for a reason, the library name can be specified with set\_target\_properties(\_add\_two\_ints\_wrapper\_cpp PROPERTIES OUTPUT\_NAME correct\_library\_name). The line 
```
catkin_python_setup()
```
 is used to export the Python module and is associated with the file setup.py 

document.write('<a href="#" onclick="return togglenumber(\'CA-742b543d8fd39fbf2b3f5d85e5ca6f0bcd091ec9\', 1, 1);" \
 class="codenumbers">Toggle line numbers<\/a>');

```
 [1](#CA-742b543d8fd39fbf2b3f5d85e5ca6f0bcd091ec9_1 "#CA-742b543d8fd39fbf2b3f5d85e5ca6f0bcd091ec9_1") roscd python\_bindings\_tutorial
 [2](#CA-742b543d8fd39fbf2b3f5d85e5ca6f0bcd091ec9_2 "#CA-742b543d8fd39fbf2b3f5d85e5ca6f0bcd091ec9_2") touch setup.py

```
The content of setup.py will be: 

document.write('<a href="#" onclick="return togglenumber(\'CA-db6c28650931aa126ee4e8b526d34c847a3bb63e\', 1, 1);" \
 class="codenumbers">Toggle line numbers<\/a>');

```
 [1](#CA-db6c28650931aa126ee4e8b526d34c847a3bb63e_1 "#CA-db6c28650931aa126ee4e8b526d34c847a3bb63e_1") # ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD
 [2](#CA-db6c28650931aa126ee4e8b526d34c847a3bb63e_2 "#CA-db6c28650931aa126ee4e8b526d34c847a3bb63e_2") 
 [3](#CA-db6c28650931aa126ee4e8b526d34c847a3bb63e_3 "#CA-db6c28650931aa126ee4e8b526d34c847a3bb63e_3") from distutils.core import setup
 [4](#CA-db6c28650931aa126ee4e8b526d34c847a3bb63e_4 "#CA-db6c28650931aa126ee4e8b526d34c847a3bb63e_4") from catkin\_pkg.python\_setup import generate\_distutils\_setup
 [5](#CA-db6c28650931aa126ee4e8b526d34c847a3bb63e_5 "#CA-db6c28650931aa126ee4e8b526d34c847a3bb63e_5") 
 [6](#CA-db6c28650931aa126ee4e8b526d34c847a3bb63e_6 "#CA-db6c28650931aa126ee4e8b526d34c847a3bb63e_6") # fetch values from package.xml
 [7](#CA-db6c28650931aa126ee4e8b526d34c847a3bb63e_7 "#CA-db6c28650931aa126ee4e8b526d34c847a3bb63e_7") setup\_args = generate\_distutils\_setup(
 [8](#CA-db6c28650931aa126ee4e8b526d34c847a3bb63e_8 "#CA-db6c28650931aa126ee4e8b526d34c847a3bb63e_8")  packages=['python\_bindings\_tutorial'],
 [9](#CA-db6c28650931aa126ee4e8b526d34c847a3bb63e_9 "#CA-db6c28650931aa126ee4e8b526d34c847a3bb63e_9")  package\_dir={'': 'src'})
 [10](#CA-db6c28650931aa126ee4e8b526d34c847a3bb63e_10 "#CA-db6c28650931aa126ee4e8b526d34c847a3bb63e_10") 
 [11](#CA-db6c28650931aa126ee4e8b526d34c847a3bb63e_11 "#CA-db6c28650931aa126ee4e8b526d34c847a3bb63e_11") setup(\*\*setup\_args)

```
We then build the package with rosmake. 

We then build the package with catkin\_make. 

### Testing the binding

You can now use the binding in Python scripts or in a Python shell 

document.write('<a href="#" onclick="return togglenumber(\'CA-b9c851381a415413b1ae10c58cc5f7b6f5f0711c\', 1, 1);" \
 class="codenumbers">Toggle line numbers<\/a>');

```
 [1](#CA-b9c851381a415413b1ae10c58cc5f7b6f5f0711c_1 "#CA-b9c851381a415413b1ae10c58cc5f7b6f5f0711c_1") from std\_msgs.msg import Int64
 [2](#CA-b9c851381a415413b1ae10c58cc5f7b6f5f0711c_2 "#CA-b9c851381a415413b1ae10c58cc5f7b6f5f0711c_2") from python\_bindings\_tutorial import AddTwoInts
 [3](#CA-b9c851381a415413b1ae10c58cc5f7b6f5f0711c_3 "#CA-b9c851381a415413b1ae10c58cc5f7b6f5f0711c_3") a = Int64(4)
 [4](#CA-b9c851381a415413b1ae10c58cc5f7b6f5f0711c_4 "#CA-b9c851381a415413b1ae10c58cc5f7b6f5f0711c_4") b = Int64(2)
 [5](#CA-b9c851381a415413b1ae10c58cc5f7b6f5f0711c_5 "#CA-b9c851381a415413b1ae10c58cc5f7b6f5f0711c_5") addtwoints = AddTwoInts()
 [6](#CA-b9c851381a415413b1ae10c58cc5f7b6f5f0711c_6 "#CA-b9c851381a415413b1ae10c58cc5f7b6f5f0711c_6") sum = addtwoints.add(a, b)
 [7](#CA-b9c851381a415413b1ae10c58cc5f7b6f5f0711c_7 "#CA-b9c851381a415413b1ae10c58cc5f7b6f5f0711c_7") sum

```

## Class with NodeHandle

As stated, a Python call to rospy.init\_node does not initialize roscpp. If roscpp is not initialized, instanciating ros::NodeHandle will lead to a fatal error. A solution for this is provided by the [moveit\_ros\_planning\_interface](http://moveit.ros.org "http://moveit.ros.org"). In any Python script that uses the wrapped class, two lines need to be added before instanciating AddTwoInts: 

document.write('<a href="#" onclick="return togglenumber(\'CA-3391c57627ee7e52435dc8487b575e5c19c37de8\', 1, 1);" \
 class="codenumbers">Toggle line numbers<\/a>');

```
 [1](#CA-3391c57627ee7e52435dc8487b575e5c19c37de8_1 "#CA-3391c57627ee7e52435dc8487b575e5c19c37de8_1") from moveit\_ros\_planning\_interface.\_moveit\_roscpp\_initializer import roscpp\_init
 [2](#CA-3391c57627ee7e52435dc8487b575e5c19c37de8_2 "#CA-3391c57627ee7e52435dc8487b575e5c19c37de8_2") roscpp\_init('node\_name', [])

```
This will create a ROS node. The advantage of this method over a classical ROS service server/client implementation is thus not as clear as in the case without the need of ros::NodeHandle. 
## Class with container of ROS messages

If the class uses containers of ROS messages, an extra conversion step must be added. This step is not specific to ROS but is part of the Boost Python library. 
### `std::vector<M>` as return type

In the file where the C++ wrapper class is defined, add these lines: 

document.write('<a href="#" onclick="return togglenumber(\'CA-a7046de9ba9e3cfad48d37c6823ac8e8fa5c5008\', 1, 1);" \
 class="codenumbers">Toggle line numbers<\/a>');

```
 [1](#CA-a7046de9ba9e3cfad48d37c6823ac8e8fa5c5008_1 "#CA-a7046de9ba9e3cfad48d37c6823ac8e8fa5c5008_1") // Extracted from https://gist.github.com/avli/b0bf77449b090b768663.
 [2](#CA-a7046de9ba9e3cfad48d37c6823ac8e8fa5c5008_2 "#CA-a7046de9ba9e3cfad48d37c6823ac8e8fa5c5008_2") template<class T>
 [3](#CA-a7046de9ba9e3cfad48d37c6823ac8e8fa5c5008_3 "#CA-a7046de9ba9e3cfad48d37c6823ac8e8fa5c5008_3") struct vector\_to\_python
 [4](#CA-a7046de9ba9e3cfad48d37c6823ac8e8fa5c5008_4 "#CA-a7046de9ba9e3cfad48d37c6823ac8e8fa5c5008_4") {
 [5](#CA-a7046de9ba9e3cfad48d37c6823ac8e8fa5c5008_5 "#CA-a7046de9ba9e3cfad48d37c6823ac8e8fa5c5008_5")  static PyObject\* convert(const std::vector<T>& vec)
 [6](#CA-a7046de9ba9e3cfad48d37c6823ac8e8fa5c5008_6 "#CA-a7046de9ba9e3cfad48d37c6823ac8e8fa5c5008_6")  {
 [7](#CA-a7046de9ba9e3cfad48d37c6823ac8e8fa5c5008_7 "#CA-a7046de9ba9e3cfad48d37c6823ac8e8fa5c5008_7")  boost::python::list\* l = new boost::python::list();
 [8](#CA-a7046de9ba9e3cfad48d37c6823ac8e8fa5c5008_8 "#CA-a7046de9ba9e3cfad48d37c6823ac8e8fa5c5008_8")  for(std::size\_t i = 0; i < vec.size(); i++)
 [9](#CA-a7046de9ba9e3cfad48d37c6823ac8e8fa5c5008_9 "#CA-a7046de9ba9e3cfad48d37c6823ac8e8fa5c5008_9")  (\*l).append(vec[i]);
 [10](#CA-a7046de9ba9e3cfad48d37c6823ac8e8fa5c5008_10 "#CA-a7046de9ba9e3cfad48d37c6823ac8e8fa5c5008_10") 
 [11](#CA-a7046de9ba9e3cfad48d37c6823ac8e8fa5c5008_11 "#CA-a7046de9ba9e3cfad48d37c6823ac8e8fa5c5008_11")  return l->ptr();
 [12](#CA-a7046de9ba9e3cfad48d37c6823ac8e8fa5c5008_12 "#CA-a7046de9ba9e3cfad48d37c6823ac8e8fa5c5008_12")  }
 [13](#CA-a7046de9ba9e3cfad48d37c6823ac8e8fa5c5008_13 "#CA-a7046de9ba9e3cfad48d37c6823ac8e8fa5c5008_13") };
 [14](#CA-a7046de9ba9e3cfad48d37c6823ac8e8fa5c5008_14 "#CA-a7046de9ba9e3cfad48d37c6823ac8e8fa5c5008_14") 
 [15](#CA-a7046de9ba9e3cfad48d37c6823ac8e8fa5c5008_15 "#CA-a7046de9ba9e3cfad48d37c6823ac8e8fa5c5008_15") class Wrapper : public WrappedClass
 [16](#CA-a7046de9ba9e3cfad48d37c6823ac8e8fa5c5008_16 "#CA-a7046de9ba9e3cfad48d37c6823ac8e8fa5c5008_16") {
 [17](#CA-a7046de9ba9e3cfad48d37c6823ac8e8fa5c5008_17 "#CA-a7046de9ba9e3cfad48d37c6823ac8e8fa5c5008_17") /\*
 [18](#CA-a7046de9ba9e3cfad48d37c6823ac8e8fa5c5008_18 "#CA-a7046de9ba9e3cfad48d37c6823ac8e8fa5c5008_18") ...
 [19](#CA-a7046de9ba9e3cfad48d37c6823ac8e8fa5c5008_19 "#CA-a7046de9ba9e3cfad48d37c6823ac8e8fa5c5008_19") \*/
 [20](#CA-a7046de9ba9e3cfad48d37c6823ac8e8fa5c5008_20 "#CA-a7046de9ba9e3cfad48d37c6823ac8e8fa5c5008_20")  std::vector<std::string> wrapper\_fun(const std::string str\_msg)
 [21](#CA-a7046de9ba9e3cfad48d37c6823ac8e8fa5c5008_21 "#CA-a7046de9ba9e3cfad48d37c6823ac8e8fa5c5008_21")  {
 [22](#CA-a7046de9ba9e3cfad48d37c6823ac8e8fa5c5008_22 "#CA-a7046de9ba9e3cfad48d37c6823ac8e8fa5c5008_22")  /\* ... \*/
 [23](#CA-a7046de9ba9e3cfad48d37c6823ac8e8fa5c5008_23 "#CA-a7046de9ba9e3cfad48d37c6823ac8e8fa5c5008_23")  }
 [24](#CA-a7046de9ba9e3cfad48d37c6823ac8e8fa5c5008_24 "#CA-a7046de9ba9e3cfad48d37c6823ac8e8fa5c5008_24") 
 [25](#CA-a7046de9ba9e3cfad48d37c6823ac8e8fa5c5008_25 "#CA-a7046de9ba9e3cfad48d37c6823ac8e8fa5c5008_25") };
 [26](#CA-a7046de9ba9e3cfad48d37c6823ac8e8fa5c5008_26 "#CA-a7046de9ba9e3cfad48d37c6823ac8e8fa5c5008_26") 
 [27](#CA-a7046de9ba9e3cfad48d37c6823ac8e8fa5c5008_27 "#CA-a7046de9ba9e3cfad48d37c6823ac8e8fa5c5008_27") BOOST\_PYTHON\_MODULE(module\_wrapper\_cpp)
 [28](#CA-a7046de9ba9e3cfad48d37c6823ac8e8fa5c5008_28 "#CA-a7046de9ba9e3cfad48d37c6823ac8e8fa5c5008_28") {
 [29](#CA-a7046de9ba9e3cfad48d37c6823ac8e8fa5c5008_29 "#CA-a7046de9ba9e3cfad48d37c6823ac8e8fa5c5008_29")  boost::python::class\_<Wrapper>("Wrapper", bp::init</\* ... \*/>())
 [30](#CA-a7046de9ba9e3cfad48d37c6823ac8e8fa5c5008_30 "#CA-a7046de9ba9e3cfad48d37c6823ac8e8fa5c5008_30")  .def("fun", &Wrapper::wrapper\_fun);
 [31](#CA-a7046de9ba9e3cfad48d37c6823ac8e8fa5c5008_31 "#CA-a7046de9ba9e3cfad48d37c6823ac8e8fa5c5008_31") 
 [32](#CA-a7046de9ba9e3cfad48d37c6823ac8e8fa5c5008_32 "#CA-a7046de9ba9e3cfad48d37c6823ac8e8fa5c5008_32")  boost::python::to\_python\_converter<std::vector<std::string, std::allocator<std::string> >, vector\_to\_python<std::string> >();
 [33](#CA-a7046de9ba9e3cfad48d37c6823ac8e8fa5c5008_33 "#CA-a7046de9ba9e3cfad48d37c6823ac8e8fa5c5008_33") }

```

### `std::vector<M>` as argument type

Cf. Boost.Python documentation. 

Wiki: ROS/Tutorials/Using a C++ class in Python (last edited 2017-06-16 21:36:43 by [kyrofa](/kyrofa "kyrofa @ 68-189-137-52.dhcp.wlwl.wa.charter.com[68.189.137.52]"))

Except where otherwise noted, the ROS wiki is licensed under the   

