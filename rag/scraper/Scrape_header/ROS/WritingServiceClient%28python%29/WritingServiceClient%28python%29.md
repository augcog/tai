

|  |
| --- |
| **Note:** This tutorial assumes that you have completed the previous tutorials: [examining the simple publisher and subscriber](/ROS/Tutorials/ExaminingPublisherSubscriber "/ROS/Tutorials/ExaminingPublisherSubscriber").  |

|  |
| --- |
| (!) Please ask about problems and questions regarding this tutorial on [answers.ros.org](http://answers.ros.org "http://answers.ros.org"). Don't forget to include in your question the link to this page, the versions of your OS & ROS, and also add appropriate tags. |

# Writing a Simple Service and Client (Python)

**Description:** This tutorial covers how to write a service and client node in python.  

**Tutorial Level:** BEGINNER  

**Next Tutorial:** [Examining the simple service and client](/ROS/Tutorials/ExaminingServiceClient "/ROS/Tutorials/ExaminingServiceClient")   

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
 Contents1. [Writing a Service Node](#rospy_tutorials.2FTutorials.2FWritingServiceClient.Writing_a_Service_Node "#rospy_tutorials.2FTutorials.2FWritingServiceClient.Writing_a_Service_Node")
	1. [The Code](#rospy_tutorials.2FTutorials.2FWritingServiceClient.The_Code "#rospy_tutorials.2FTutorials.2FWritingServiceClient.The_Code")
	2. [The Code Explained](#rospy_tutorials.2FTutorials.2FWritingServiceClient.The_Code_Explained "#rospy_tutorials.2FTutorials.2FWritingServiceClient.The_Code_Explained")
2. [Writing the Client Node](#rospy_tutorials.2FTutorials.2FWritingServiceClient.Writing_the_Client_Node "#rospy_tutorials.2FTutorials.2FWritingServiceClient.Writing_the_Client_Node")
	1. [The Code](#rospy_tutorials.2FTutorials.2FWritingServiceClient.The_Code-1 "#rospy_tutorials.2FTutorials.2FWritingServiceClient.The_Code-1")
	2. [The Code Explained](#rospy_tutorials.2FTutorials.2FWritingServiceClient.The_Code_Explained-1 "#rospy_tutorials.2FTutorials.2FWritingServiceClient.The_Code_Explained-1")
3. [Building your nodes](#rospy_tutorials.2FTutorials.2FWritingServiceClient.Building_your_nodes "#rospy_tutorials.2FTutorials.2FWritingServiceClient.Building_your_nodes")
4. [Video Tutorial](#Video_Tutorial "#Video_Tutorial")

## Writing a Service Node

Here we'll create the service ("add\_two\_ints\_server") node which will receive two ints and return the sum. Change directory into the beginner\_tutorials package, you created in the earlier tutorial, [creating a package](/ROS/Tutorials/CreatingPackage "/ROS/Tutorials/CreatingPackage"): 

Change directory into the beginner\_tutorials package, you created in the earlier tutorial, [creating a package](/ROS/Tutorials/CreatingPackage%3Fbuildsystem%3Dcatkin "/ROS/Tutorials/CreatingPackage%3Fbuildsystem%3Dcatkin"): 

```
$ roscd beginner_tutorials
```
Please make sure you have followed the directions in the previous tutorial for creating the service needed in this tutorial, [creating the AddTwoInts.srv](/ROS/Tutorials/CreatingMsgAndSrv#Creating_a_srv "/ROS/Tutorials/CreatingMsgAndSrv#Creating_a_srv") (be sure to choose the right version of build tool you're using at the top of wiki page in the link). 
### The Code

Create the **scripts/add\_two\_ints\_server.py** file within the beginner\_tutorials package and paste the following inside it: 

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

document.write('<a href="#" onclick="return togglenumber(\'rospy\_tutorials.2FTutorials.2FWritingServiceClient.CA-6e40b6a054990a096d1fbfef2d938f40b439dade\', 1, 1);" \
 class="codenumbers">Toggle line numbers<\/a>');

```
 [1](#rospy_tutorials.2FTutorials.2FWritingServiceClient.CA-6e40b6a054990a096d1fbfef2d938f40b439dade_1 "#rospy_tutorials.2FTutorials.2FWritingServiceClient.CA-6e40b6a054990a096d1fbfef2d938f40b439dade_1") #!/usr/bin/env python
 [2](#rospy_tutorials.2FTutorials.2FWritingServiceClient.CA-6e40b6a054990a096d1fbfef2d938f40b439dade_2 "#rospy_tutorials.2FTutorials.2FWritingServiceClient.CA-6e40b6a054990a096d1fbfef2d938f40b439dade_2") import roslib; roslib.load\_manifest('beginner\_tutorials')
 [3](#rospy_tutorials.2FTutorials.2FWritingServiceClient.CA-6e40b6a054990a096d1fbfef2d938f40b439dade_3 "#rospy_tutorials.2FTutorials.2FWritingServiceClient.CA-6e40b6a054990a096d1fbfef2d938f40b439dade_3") from beginner\_tutorials.srv import \*
 [4](#rospy_tutorials.2FTutorials.2FWritingServiceClient.CA-6e40b6a054990a096d1fbfef2d938f40b439dade_4 "#rospy_tutorials.2FTutorials.2FWritingServiceClient.CA-6e40b6a054990a096d1fbfef2d938f40b439dade_4") import rospy
 [5](#rospy_tutorials.2FTutorials.2FWritingServiceClient.CA-6e40b6a054990a096d1fbfef2d938f40b439dade_5 "#rospy_tutorials.2FTutorials.2FWritingServiceClient.CA-6e40b6a054990a096d1fbfef2d938f40b439dade_5") 
 [6](#rospy_tutorials.2FTutorials.2FWritingServiceClient.CA-6e40b6a054990a096d1fbfef2d938f40b439dade_6 "#rospy_tutorials.2FTutorials.2FWritingServiceClient.CA-6e40b6a054990a096d1fbfef2d938f40b439dade_6") def handle\_add\_two\_ints(req):
 [7](#rospy_tutorials.2FTutorials.2FWritingServiceClient.CA-6e40b6a054990a096d1fbfef2d938f40b439dade_7 "#rospy_tutorials.2FTutorials.2FWritingServiceClient.CA-6e40b6a054990a096d1fbfef2d938f40b439dade_7")  print "Returning [%s + %s = %s]"%(req.a, req.b, (req.a + req.b))
 [8](#rospy_tutorials.2FTutorials.2FWritingServiceClient.CA-6e40b6a054990a096d1fbfef2d938f40b439dade_8 "#rospy_tutorials.2FTutorials.2FWritingServiceClient.CA-6e40b6a054990a096d1fbfef2d938f40b439dade_8")  return AddTwoIntsResponse(req.a + req.b)
 [9](#rospy_tutorials.2FTutorials.2FWritingServiceClient.CA-6e40b6a054990a096d1fbfef2d938f40b439dade_9 "#rospy_tutorials.2FTutorials.2FWritingServiceClient.CA-6e40b6a054990a096d1fbfef2d938f40b439dade_9") 
 [10](#rospy_tutorials.2FTutorials.2FWritingServiceClient.CA-6e40b6a054990a096d1fbfef2d938f40b439dade_10 "#rospy_tutorials.2FTutorials.2FWritingServiceClient.CA-6e40b6a054990a096d1fbfef2d938f40b439dade_10") def add\_two\_ints\_server():
 [11](#rospy_tutorials.2FTutorials.2FWritingServiceClient.CA-6e40b6a054990a096d1fbfef2d938f40b439dade_11 "#rospy_tutorials.2FTutorials.2FWritingServiceClient.CA-6e40b6a054990a096d1fbfef2d938f40b439dade_11")  rospy.init\_node('add\_two\_ints\_server')
 [12](#rospy_tutorials.2FTutorials.2FWritingServiceClient.CA-6e40b6a054990a096d1fbfef2d938f40b439dade_12 "#rospy_tutorials.2FTutorials.2FWritingServiceClient.CA-6e40b6a054990a096d1fbfef2d938f40b439dade_12")  s = rospy.Service('add\_two\_ints', AddTwoInts, handle\_add\_two\_ints)
 [13](#rospy_tutorials.2FTutorials.2FWritingServiceClient.CA-6e40b6a054990a096d1fbfef2d938f40b439dade_13 "#rospy_tutorials.2FTutorials.2FWritingServiceClient.CA-6e40b6a054990a096d1fbfef2d938f40b439dade_13")  print "Ready to add two ints."
 [14](#rospy_tutorials.2FTutorials.2FWritingServiceClient.CA-6e40b6a054990a096d1fbfef2d938f40b439dade_14 "#rospy_tutorials.2FTutorials.2FWritingServiceClient.CA-6e40b6a054990a096d1fbfef2d938f40b439dade_14")  rospy.spin()
 [15](#rospy_tutorials.2FTutorials.2FWritingServiceClient.CA-6e40b6a054990a096d1fbfef2d938f40b439dade_15 "#rospy_tutorials.2FTutorials.2FWritingServiceClient.CA-6e40b6a054990a096d1fbfef2d938f40b439dade_15") 
 [16](#rospy_tutorials.2FTutorials.2FWritingServiceClient.CA-6e40b6a054990a096d1fbfef2d938f40b439dade_16 "#rospy_tutorials.2FTutorials.2FWritingServiceClient.CA-6e40b6a054990a096d1fbfef2d938f40b439dade_16") if \_\_name\_\_ == "\_\_main\_\_":
 [17](#rospy_tutorials.2FTutorials.2FWritingServiceClient.CA-6e40b6a054990a096d1fbfef2d938f40b439dade_17 "#rospy_tutorials.2FTutorials.2FWritingServiceClient.CA-6e40b6a054990a096d1fbfef2d938f40b439dade_17")  add\_two\_ints\_server()

```

document.write('<a href="#" onclick="return togglenumber(\'rospy\_tutorials.2FTutorials.2FWritingServiceClient.CA-1b44a8b12515c870975d3d35cd5cd98f6145a087\', 1, 1);" \
 class="codenumbers">Toggle line numbers<\/a>');

```
 [1](#rospy_tutorials.2FTutorials.2FWritingServiceClient.CA-1b44a8b12515c870975d3d35cd5cd98f6145a087_1 "#rospy_tutorials.2FTutorials.2FWritingServiceClient.CA-1b44a8b12515c870975d3d35cd5cd98f6145a087_1") #!/usr/bin/env python
 [2](#rospy_tutorials.2FTutorials.2FWritingServiceClient.CA-1b44a8b12515c870975d3d35cd5cd98f6145a087_2 "#rospy_tutorials.2FTutorials.2FWritingServiceClient.CA-1b44a8b12515c870975d3d35cd5cd98f6145a087_2") 
 [3](#rospy_tutorials.2FTutorials.2FWritingServiceClient.CA-1b44a8b12515c870975d3d35cd5cd98f6145a087_3 "#rospy_tutorials.2FTutorials.2FWritingServiceClient.CA-1b44a8b12515c870975d3d35cd5cd98f6145a087_3") from \_\_future\_\_ import print\_function
 [4](#rospy_tutorials.2FTutorials.2FWritingServiceClient.CA-1b44a8b12515c870975d3d35cd5cd98f6145a087_4 "#rospy_tutorials.2FTutorials.2FWritingServiceClient.CA-1b44a8b12515c870975d3d35cd5cd98f6145a087_4") 
 [5](#rospy_tutorials.2FTutorials.2FWritingServiceClient.CA-1b44a8b12515c870975d3d35cd5cd98f6145a087_5 "#rospy_tutorials.2FTutorials.2FWritingServiceClient.CA-1b44a8b12515c870975d3d35cd5cd98f6145a087_5") from beginner\_tutorials.srv import AddTwoInts,AddTwoIntsResponse
 [6](#rospy_tutorials.2FTutorials.2FWritingServiceClient.CA-1b44a8b12515c870975d3d35cd5cd98f6145a087_6 "#rospy_tutorials.2FTutorials.2FWritingServiceClient.CA-1b44a8b12515c870975d3d35cd5cd98f6145a087_6") import rospy
 [7](#rospy_tutorials.2FTutorials.2FWritingServiceClient.CA-1b44a8b12515c870975d3d35cd5cd98f6145a087_7 "#rospy_tutorials.2FTutorials.2FWritingServiceClient.CA-1b44a8b12515c870975d3d35cd5cd98f6145a087_7") 
 [8](#rospy_tutorials.2FTutorials.2FWritingServiceClient.CA-1b44a8b12515c870975d3d35cd5cd98f6145a087_8 "#rospy_tutorials.2FTutorials.2FWritingServiceClient.CA-1b44a8b12515c870975d3d35cd5cd98f6145a087_8") def handle\_add\_two\_ints(req):
 [9](#rospy_tutorials.2FTutorials.2FWritingServiceClient.CA-1b44a8b12515c870975d3d35cd5cd98f6145a087_9 "#rospy_tutorials.2FTutorials.2FWritingServiceClient.CA-1b44a8b12515c870975d3d35cd5cd98f6145a087_9")  print("Returning [%s + %s = %s]"%(req.a, req.b, (req.a + req.b)))
 [10](#rospy_tutorials.2FTutorials.2FWritingServiceClient.CA-1b44a8b12515c870975d3d35cd5cd98f6145a087_10 "#rospy_tutorials.2FTutorials.2FWritingServiceClient.CA-1b44a8b12515c870975d3d35cd5cd98f6145a087_10")  return AddTwoIntsResponse(req.a + req.b)
 [11](#rospy_tutorials.2FTutorials.2FWritingServiceClient.CA-1b44a8b12515c870975d3d35cd5cd98f6145a087_11 "#rospy_tutorials.2FTutorials.2FWritingServiceClient.CA-1b44a8b12515c870975d3d35cd5cd98f6145a087_11") 
 [12](#rospy_tutorials.2FTutorials.2FWritingServiceClient.CA-1b44a8b12515c870975d3d35cd5cd98f6145a087_12 "#rospy_tutorials.2FTutorials.2FWritingServiceClient.CA-1b44a8b12515c870975d3d35cd5cd98f6145a087_12") def add\_two\_ints\_server():
 [13](#rospy_tutorials.2FTutorials.2FWritingServiceClient.CA-1b44a8b12515c870975d3d35cd5cd98f6145a087_13 "#rospy_tutorials.2FTutorials.2FWritingServiceClient.CA-1b44a8b12515c870975d3d35cd5cd98f6145a087_13")  rospy.init\_node('add\_two\_ints\_server')
 [14](#rospy_tutorials.2FTutorials.2FWritingServiceClient.CA-1b44a8b12515c870975d3d35cd5cd98f6145a087_14 "#rospy_tutorials.2FTutorials.2FWritingServiceClient.CA-1b44a8b12515c870975d3d35cd5cd98f6145a087_14")  s = rospy.Service('add\_two\_ints', AddTwoInts, handle\_add\_two\_ints)
 [15](#rospy_tutorials.2FTutorials.2FWritingServiceClient.CA-1b44a8b12515c870975d3d35cd5cd98f6145a087_15 "#rospy_tutorials.2FTutorials.2FWritingServiceClient.CA-1b44a8b12515c870975d3d35cd5cd98f6145a087_15")  print("Ready to add two ints.")
 [16](#rospy_tutorials.2FTutorials.2FWritingServiceClient.CA-1b44a8b12515c870975d3d35cd5cd98f6145a087_16 "#rospy_tutorials.2FTutorials.2FWritingServiceClient.CA-1b44a8b12515c870975d3d35cd5cd98f6145a087_16")  rospy.spin()
 [17](#rospy_tutorials.2FTutorials.2FWritingServiceClient.CA-1b44a8b12515c870975d3d35cd5cd98f6145a087_17 "#rospy_tutorials.2FTutorials.2FWritingServiceClient.CA-1b44a8b12515c870975d3d35cd5cd98f6145a087_17") 
 [18](#rospy_tutorials.2FTutorials.2FWritingServiceClient.CA-1b44a8b12515c870975d3d35cd5cd98f6145a087_18 "#rospy_tutorials.2FTutorials.2FWritingServiceClient.CA-1b44a8b12515c870975d3d35cd5cd98f6145a087_18") if \_\_name\_\_ == "\_\_main\_\_":
 [19](#rospy_tutorials.2FTutorials.2FWritingServiceClient.CA-1b44a8b12515c870975d3d35cd5cd98f6145a087_19 "#rospy_tutorials.2FTutorials.2FWritingServiceClient.CA-1b44a8b12515c870975d3d35cd5cd98f6145a087_19")  add\_two\_ints\_server()

```

Don't forget to make the node executable: * 
```
chmod +x scripts/add_two_ints_server.py
```

Add the following to your CMakeLists.txt. This makes sure the python script gets installed properly, and uses the right python interpreter. 
```
catkin_install_python(PROGRAMS scripts/add_two_ints_server.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
```

### The Code Explained

Now, let's break the code down. There's very little to writing a service using [rospy](/rospy "/rospy"). We declare our node using init\_node() and then declare our service: 

document.write('<a href="#" onclick="return togglenumber(\'rospy\_tutorials.2FTutorials.2FWritingServiceClient.CA-4af5a2319367916304ba4d0dd6f697e4c2f5788c\', 12, 1);" \
 class="codenumbers">Toggle line numbers<\/a>');

```
 [12](#rospy_tutorials.2FTutorials.2FWritingServiceClient.CA-4af5a2319367916304ba4d0dd6f697e4c2f5788c_12 "#rospy_tutorials.2FTutorials.2FWritingServiceClient.CA-4af5a2319367916304ba4d0dd6f697e4c2f5788c_12")  s = rospy.Service('add\_two\_ints', AddTwoInts, handle\_add\_two\_ints)

```
 This declares a new service named add\_two\_ints with the AddTwoInts service type. All requests are passed to handle\_add\_two\_ints function. handle\_add\_two\_ints is called with instances of AddTwoIntsRequest and returns instances of AddTwoIntsResponse. Just like with the subscriber example, rospy.spin() keeps your code from exiting until the service is shutdown. 
## Writing the Client Node

### The Code

Create the **scripts/add\_two\_ints\_client.py** file within the beginner\_tutorials package and paste the following inside it: 

document.write('<a href="#" onclick="return togglenumber(\'rospy\_tutorials.2FTutorials.2FWritingServiceClient.CA-ac633897e45f717b1a6ad7685dbe0a93c834abbd\', 1, 1);" \
 class="codenumbers">Toggle line numbers<\/a>');

```
 [1](#rospy_tutorials.2FTutorials.2FWritingServiceClient.CA-ac633897e45f717b1a6ad7685dbe0a93c834abbd_1 "#rospy_tutorials.2FTutorials.2FWritingServiceClient.CA-ac633897e45f717b1a6ad7685dbe0a93c834abbd_1") #!/usr/bin/env python
 [2](#rospy_tutorials.2FTutorials.2FWritingServiceClient.CA-ac633897e45f717b1a6ad7685dbe0a93c834abbd_2 "#rospy_tutorials.2FTutorials.2FWritingServiceClient.CA-ac633897e45f717b1a6ad7685dbe0a93c834abbd_2") 
 [3](#rospy_tutorials.2FTutorials.2FWritingServiceClient.CA-ac633897e45f717b1a6ad7685dbe0a93c834abbd_3 "#rospy_tutorials.2FTutorials.2FWritingServiceClient.CA-ac633897e45f717b1a6ad7685dbe0a93c834abbd_3") from \_\_future\_\_ import print\_function
 [4](#rospy_tutorials.2FTutorials.2FWritingServiceClient.CA-ac633897e45f717b1a6ad7685dbe0a93c834abbd_4 "#rospy_tutorials.2FTutorials.2FWritingServiceClient.CA-ac633897e45f717b1a6ad7685dbe0a93c834abbd_4") 
 [5](#rospy_tutorials.2FTutorials.2FWritingServiceClient.CA-ac633897e45f717b1a6ad7685dbe0a93c834abbd_5 "#rospy_tutorials.2FTutorials.2FWritingServiceClient.CA-ac633897e45f717b1a6ad7685dbe0a93c834abbd_5") import sys
 [6](#rospy_tutorials.2FTutorials.2FWritingServiceClient.CA-ac633897e45f717b1a6ad7685dbe0a93c834abbd_6 "#rospy_tutorials.2FTutorials.2FWritingServiceClient.CA-ac633897e45f717b1a6ad7685dbe0a93c834abbd_6") import rospy
 [7](#rospy_tutorials.2FTutorials.2FWritingServiceClient.CA-ac633897e45f717b1a6ad7685dbe0a93c834abbd_7 "#rospy_tutorials.2FTutorials.2FWritingServiceClient.CA-ac633897e45f717b1a6ad7685dbe0a93c834abbd_7") from beginner\_tutorials.srv import \*
 [8](#rospy_tutorials.2FTutorials.2FWritingServiceClient.CA-ac633897e45f717b1a6ad7685dbe0a93c834abbd_8 "#rospy_tutorials.2FTutorials.2FWritingServiceClient.CA-ac633897e45f717b1a6ad7685dbe0a93c834abbd_8") 
 [9](#rospy_tutorials.2FTutorials.2FWritingServiceClient.CA-ac633897e45f717b1a6ad7685dbe0a93c834abbd_9 "#rospy_tutorials.2FTutorials.2FWritingServiceClient.CA-ac633897e45f717b1a6ad7685dbe0a93c834abbd_9") def add\_two\_ints\_client(x, y):
 [10](#rospy_tutorials.2FTutorials.2FWritingServiceClient.CA-ac633897e45f717b1a6ad7685dbe0a93c834abbd_10 "#rospy_tutorials.2FTutorials.2FWritingServiceClient.CA-ac633897e45f717b1a6ad7685dbe0a93c834abbd_10")  rospy.wait\_for\_service('add\_two\_ints')
 [11](#rospy_tutorials.2FTutorials.2FWritingServiceClient.CA-ac633897e45f717b1a6ad7685dbe0a93c834abbd_11 "#rospy_tutorials.2FTutorials.2FWritingServiceClient.CA-ac633897e45f717b1a6ad7685dbe0a93c834abbd_11")  try:
 [12](#rospy_tutorials.2FTutorials.2FWritingServiceClient.CA-ac633897e45f717b1a6ad7685dbe0a93c834abbd_12 "#rospy_tutorials.2FTutorials.2FWritingServiceClient.CA-ac633897e45f717b1a6ad7685dbe0a93c834abbd_12")  add\_two\_ints = rospy.ServiceProxy('add\_two\_ints', AddTwoInts)
 [13](#rospy_tutorials.2FTutorials.2FWritingServiceClient.CA-ac633897e45f717b1a6ad7685dbe0a93c834abbd_13 "#rospy_tutorials.2FTutorials.2FWritingServiceClient.CA-ac633897e45f717b1a6ad7685dbe0a93c834abbd_13")  resp1 = add\_two\_ints(x, y)
 [14](#rospy_tutorials.2FTutorials.2FWritingServiceClient.CA-ac633897e45f717b1a6ad7685dbe0a93c834abbd_14 "#rospy_tutorials.2FTutorials.2FWritingServiceClient.CA-ac633897e45f717b1a6ad7685dbe0a93c834abbd_14")  return resp1.sum
 [15](#rospy_tutorials.2FTutorials.2FWritingServiceClient.CA-ac633897e45f717b1a6ad7685dbe0a93c834abbd_15 "#rospy_tutorials.2FTutorials.2FWritingServiceClient.CA-ac633897e45f717b1a6ad7685dbe0a93c834abbd_15")  except rospy.ServiceException as e:
 [16](#rospy_tutorials.2FTutorials.2FWritingServiceClient.CA-ac633897e45f717b1a6ad7685dbe0a93c834abbd_16 "#rospy_tutorials.2FTutorials.2FWritingServiceClient.CA-ac633897e45f717b1a6ad7685dbe0a93c834abbd_16")  print("Service call failed: %s"%e)
 [17](#rospy_tutorials.2FTutorials.2FWritingServiceClient.CA-ac633897e45f717b1a6ad7685dbe0a93c834abbd_17 "#rospy_tutorials.2FTutorials.2FWritingServiceClient.CA-ac633897e45f717b1a6ad7685dbe0a93c834abbd_17") 
 [18](#rospy_tutorials.2FTutorials.2FWritingServiceClient.CA-ac633897e45f717b1a6ad7685dbe0a93c834abbd_18 "#rospy_tutorials.2FTutorials.2FWritingServiceClient.CA-ac633897e45f717b1a6ad7685dbe0a93c834abbd_18") def usage():
 [19](#rospy_tutorials.2FTutorials.2FWritingServiceClient.CA-ac633897e45f717b1a6ad7685dbe0a93c834abbd_19 "#rospy_tutorials.2FTutorials.2FWritingServiceClient.CA-ac633897e45f717b1a6ad7685dbe0a93c834abbd_19")  return "%s [x y]"%sys.argv[0]
 [20](#rospy_tutorials.2FTutorials.2FWritingServiceClient.CA-ac633897e45f717b1a6ad7685dbe0a93c834abbd_20 "#rospy_tutorials.2FTutorials.2FWritingServiceClient.CA-ac633897e45f717b1a6ad7685dbe0a93c834abbd_20") 
 [21](#rospy_tutorials.2FTutorials.2FWritingServiceClient.CA-ac633897e45f717b1a6ad7685dbe0a93c834abbd_21 "#rospy_tutorials.2FTutorials.2FWritingServiceClient.CA-ac633897e45f717b1a6ad7685dbe0a93c834abbd_21") if \_\_name\_\_ == "\_\_main\_\_":
 [22](#rospy_tutorials.2FTutorials.2FWritingServiceClient.CA-ac633897e45f717b1a6ad7685dbe0a93c834abbd_22 "#rospy_tutorials.2FTutorials.2FWritingServiceClient.CA-ac633897e45f717b1a6ad7685dbe0a93c834abbd_22")  if len(sys.argv) == 3:
 [23](#rospy_tutorials.2FTutorials.2FWritingServiceClient.CA-ac633897e45f717b1a6ad7685dbe0a93c834abbd_23 "#rospy_tutorials.2FTutorials.2FWritingServiceClient.CA-ac633897e45f717b1a6ad7685dbe0a93c834abbd_23")  x = int(sys.argv[1])
 [24](#rospy_tutorials.2FTutorials.2FWritingServiceClient.CA-ac633897e45f717b1a6ad7685dbe0a93c834abbd_24 "#rospy_tutorials.2FTutorials.2FWritingServiceClient.CA-ac633897e45f717b1a6ad7685dbe0a93c834abbd_24")  y = int(sys.argv[2])
 [25](#rospy_tutorials.2FTutorials.2FWritingServiceClient.CA-ac633897e45f717b1a6ad7685dbe0a93c834abbd_25 "#rospy_tutorials.2FTutorials.2FWritingServiceClient.CA-ac633897e45f717b1a6ad7685dbe0a93c834abbd_25")  else:
 [26](#rospy_tutorials.2FTutorials.2FWritingServiceClient.CA-ac633897e45f717b1a6ad7685dbe0a93c834abbd_26 "#rospy_tutorials.2FTutorials.2FWritingServiceClient.CA-ac633897e45f717b1a6ad7685dbe0a93c834abbd_26")  print(usage())
 [27](#rospy_tutorials.2FTutorials.2FWritingServiceClient.CA-ac633897e45f717b1a6ad7685dbe0a93c834abbd_27 "#rospy_tutorials.2FTutorials.2FWritingServiceClient.CA-ac633897e45f717b1a6ad7685dbe0a93c834abbd_27")  sys.exit(1)
 [28](#rospy_tutorials.2FTutorials.2FWritingServiceClient.CA-ac633897e45f717b1a6ad7685dbe0a93c834abbd_28 "#rospy_tutorials.2FTutorials.2FWritingServiceClient.CA-ac633897e45f717b1a6ad7685dbe0a93c834abbd_28")  print("Requesting %s+%s"%(x, y))
 [29](#rospy_tutorials.2FTutorials.2FWritingServiceClient.CA-ac633897e45f717b1a6ad7685dbe0a93c834abbd_29 "#rospy_tutorials.2FTutorials.2FWritingServiceClient.CA-ac633897e45f717b1a6ad7685dbe0a93c834abbd_29")  print("%s + %s = %s"%(x, y, add\_two\_ints\_client(x, y)))

```

Don't forget to make the node executable: 
```
$ chmod +x scripts/add_two_ints_client.py
```
Then, edit the catkin\_install\_python() call in your CMakeLists.txt so it looks like the following: 
```
catkin_install_python(PROGRAMS scripts/add_two_ints_server.py scripts/add_two_ints_client.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
```

### The Code Explained

Now, let's break the code down. The client code for calling services is also simple. For clients you don't have to call init\_node(). We first call: 

document.write('<a href="#" onclick="return togglenumber(\'rospy\_tutorials.2FTutorials.2FWritingServiceClient.CA-9230d01f45af4649ee54ce639f7c09dc5ba16698\', 10, 1);" \
 class="codenumbers">Toggle line numbers<\/a>');

```
 [10](#rospy_tutorials.2FTutorials.2FWritingServiceClient.CA-9230d01f45af4649ee54ce639f7c09dc5ba16698_10 "#rospy_tutorials.2FTutorials.2FWritingServiceClient.CA-9230d01f45af4649ee54ce639f7c09dc5ba16698_10")  rospy.wait\_for\_service('add\_two\_ints')

```
 This is a convenience method that blocks until the service named add\_two\_ints is available. Next we create a handle for calling the service: 

document.write('<a href="#" onclick="return togglenumber(\'rospy\_tutorials.2FTutorials.2FWritingServiceClient.CA-b0b6cdf636d5aadc585acc35117f3e34dd9c4276\', 12, 1);" \
 class="codenumbers">Toggle line numbers<\/a>');

```
 [12](#rospy_tutorials.2FTutorials.2FWritingServiceClient.CA-b0b6cdf636d5aadc585acc35117f3e34dd9c4276_12 "#rospy_tutorials.2FTutorials.2FWritingServiceClient.CA-b0b6cdf636d5aadc585acc35117f3e34dd9c4276_12")  add\_two\_ints = rospy.ServiceProxy('add\_two\_ints', AddTwoInts)

```
 We can use this handle just like a normal function and call it: 

document.write('<a href="#" onclick="return togglenumber(\'rospy\_tutorials.2FTutorials.2FWritingServiceClient.CA-b52e2d1fa20a2403ab030cc359489fe0daeb1cfb\', 13, 1);" \
 class="codenumbers">Toggle line numbers<\/a>');

```
 [13](#rospy_tutorials.2FTutorials.2FWritingServiceClient.CA-b52e2d1fa20a2403ab030cc359489fe0daeb1cfb_13 "#rospy_tutorials.2FTutorials.2FWritingServiceClient.CA-b52e2d1fa20a2403ab030cc359489fe0daeb1cfb_13")  resp1 = add\_two\_ints(x, y)
 [14](#rospy_tutorials.2FTutorials.2FWritingServiceClient.CA-b52e2d1fa20a2403ab030cc359489fe0daeb1cfb_14 "#rospy_tutorials.2FTutorials.2FWritingServiceClient.CA-b52e2d1fa20a2403ab030cc359489fe0daeb1cfb_14")  return resp1.sum

```
 Because we've declared the type of the service to be AddTwoInts, it does the work of generating the AddTwoIntsRequest object for you (you're free to pass in your own instead). The return value is an AddTwoIntsResponse object. If the call fails, a rospy.ServiceException may be thrown, so you should setup the appropriate try/except block. 
## Building your nodes

We use CMake as our build system and, yes, you have to use it even for Python nodes. This is to make sure that the [autogenerated Python code for messages and services](http://www.ros.org/wiki/ROS/Tutorials/CreatingMsgAndSrv#Creating_a_srv "http://www.ros.org/wiki/ROS/Tutorials/CreatingMsgAndSrv#Creating_a_srv") is created. We also use a Makefile for a bit of convenience. roscreate-pkg automatically created a Makefile, so you don't have to edit it. Now run make: 
```
$ make
```

Go to your catkin workspace and run catkin\_make. 
```
# In your catkin workspace
$ cd ~/catkin_ws
$ catkin_make
```

 Now that you have written a simple service and client, let's [examine the simple service and client](/ROS/Tutorials/ExaminingServiceClient "/ROS/Tutorials/ExaminingServiceClient"). 
## Video Tutorial

The following video presents a small tutorial on ROS services  

