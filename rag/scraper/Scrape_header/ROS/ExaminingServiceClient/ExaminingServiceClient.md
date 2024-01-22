

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

