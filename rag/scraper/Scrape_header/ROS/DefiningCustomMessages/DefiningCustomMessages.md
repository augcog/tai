

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

