

|  |
| --- |
| **Note:** This tutorial assumes that you have completed the previous tutorials: [Using rqt\_console and roslaunch](/ROS/Tutorials/UsingRqtconsoleRoslaunch "/ROS/Tutorials/UsingRqtconsoleRoslaunch").  |

|  |
| --- |
| (!) Please ask about problems and questions regarding this tutorial on [answers.ros.org](http://answers.ros.org "http://answers.ros.org"). Don't forget to include in your question the link to this page, the versions of your OS & ROS, and also add appropriate tags. |

# Using rosed to edit files in ROS

**Description:** This tutorial shows how to use [rosed](/rosbash "/rosbash") to make editing easier.  

**Tutorial Level:** BEGINNER  

**Next Tutorial:** [Creating a Msg and Srv](/ROS/Tutorials/CreatingMsgAndSrv "/ROS/Tutorials/CreatingMsgAndSrv")   

 Contents1. [Using rosed](#Using_rosed "#Using_rosed")
2. [Using rosed with tab completion](#Using_rosed_with_tab_completion "#Using_rosed_with_tab_completion")
3. [Editor](#Editor "#Editor")

## Using rosed

rosed is part of the [rosbash](/rosbash "/rosbash") suite. It allows you to directly edit a file within a package by using the package name rather than having to type the entire path to the package. Usage: 
```
$ rosed [package_name] [filename]
```
Example: 
```
$ rosed roscpp Logger.msg
```
This example demonstrates how you would edit the Logger.msg file within the roscpp package. If this example doesn't work it's probably because you don't have the vim editor installed. Please refer to [Editor](/ROS/Tutorials/UsingRosEd#Editor "/ROS/Tutorials/UsingRosEd#Editor") section. If you don't know how to get out of vim, [click here](http://kb.iu.edu/data/afcz.html "http://kb.iu.edu/data/afcz.html"). If the filename is not uniquely defined within the package, a menu will prompt you to choose which of the possible files you want to edit. 
## Using rosed with tab completion

This way you can easily see and optionally edit all files from a package without knowing its exact name. Usage: 
```
$ rosed [package_name] <tab><tab>
```
Example: 
```
$ rosed roscpp <tab><tab>
```
* 
```
Empty.srv                   package.xml
GetLoggers.srv              roscpp-msg-extras.cmake
Logger.msg                  roscpp-msg-paths.cmake
SetLoggerLevel.srv          roscpp.cmake
genmsg_cpp.py               roscppConfig-version.cmake
gensrv_cpp.py               roscppConfig.cmake
msg_gen.py                  
```

## Editor

The default editor for rosed is vim. The more beginner-friendly editor nano is included with the default Ubuntu install. You can use it by editing your ~/.bashrc file to include: 
```
export EDITOR='nano -w'
```
To set the default editor to emacs you can edit your ~/.bashrc file to include: 
```
export EDITOR='emacs -nw'
```
***NOTE:*** *changes in .bashrc will only take effect for new terminals. Terminals that are already open will not see the new environmental variable.* Open a new terminal and see if EDITOR is defined: 
```
$ echo $EDITOR
```
* 
```
nano -w
```
or 
```
emacs -nw
```

Now that you have successfully configured and used rosed, let's [create a Msg and Srv](/ROS/Tutorials/CreatingMsgAndSrv "/ROS/Tutorials/CreatingMsgAndSrv"). 

