# Week 1: Welcome to EECS 106B/206B!
**User:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4133197
 Hi everyone! We’re so excited to have you in EECS C106B/206B! This class dives into open research problems, building off the foundation established by 106A/206A. We cover much more cutting-edge topics and aim to build skills in analysis, writing, and experimentation.

Just like 106A/206A, we’ll be sending out weekly announcements with important information. Please read these to avoid missing any important class information!

The class is split up into a few different parts. We will have weekly lectures, discussion sections, and journal clubs along with biweekly homework and projects. As mentioned in lecture today, instead of 3-hour labs, we have 1-hour journal clubs meant to introduce projects (which are completed in your own time) and conduct paper presentations. There will also be a research-based final project. All parts of the class can be done asynchronously except the journal clubs, which have mandatory attendance. This is a link to today's logistics presentation in class.

General Course Info:

Course Website Link: https://ucb-ee106.github.io/106b-sp24site/

Save this link! Lecture topics are subject to change. We want to cover as much information on the horizon of robotics as possible, so some classes may be shifted around.

The Gradescope entry code is WBZ7DE. Everyone should be added, but if not you can add yourself to the course with that code. 

Ed is our home for questions and communications.

If you are a DSP student, please ensure that your DSP letter has been sent to the class so that we can provide you with any accommodations you need.

If you are a waitlist student, please know that we're trying our best to get definite answers regarding enrollment, but we don't know any additional definite information (ie possibility of expansion, etc.) about enrollment than what's already been mentioned. 

If you haven't taken 106A, an assessment will be released tomorrow on the course website and Gradescope. Completion of this assessment is required if you haven't taken 106A. This is a great benchmark for whether you have the foundational background to succeed in this class. 

Discussion Sections

Discussion sections will be on Thursday from 4-5pm (Cory 299) and Friday from 11am-12pm (Cory 521), led by either Tarun or Nima.

They will have a review of important topics and go over practice problems related to the material to supplement lecture content.

One of the sections will be recorded and posted to the website for asynchronous viewing.

Journal Club

During journal club, we will have two paper presentations per day, done in groups of two (the signup form will be posted next week). Also, any new projects will be introduced.

Journal club must be attended synchronously, and it begins next week. Tomorrow at 8 pm, we will send out a form to write preferences for a particular section.


Projects/Labs

Projects will be done in groups of 3 people, and they happen during your own time. Through this system, we hope to build skills that allow you to perform more effective research. The submission for each project involves a conference-style report.

We have a reservation system for the robots like the final project last semester (see the course website for the links). A robot can be reserved for 2 hours at a time, and a new reservation can be made after the current one is finished.

Feel free to come to lab OH for help on projects.

Project 0 will be released tomorrow. It's optional (and doesn't have a due date) but it's highly recommended as ROS refresher.

Before using the lab robots, you must complete the Robot Usage Quiz on Gradescope (same as last semester). 

Homework

Homework is used to build a better understanding of the material through practice problems. You're generally going to have 2 weeks to do each homework, except for Homework 1 that's due next Wednesday. It's already released on the website, and a thread will be created here on Ed, as well. 

Midterm

We will have one midterm (which will happen pretty close to the end of the semester).

We don’t want this to be too stressful and will design it accordingly. More details will be released closer to the exam date.

Final Project

Everyone will have to do a research-based final project. This will be in groups of 4.

Excited for the semester!!:)




### Comment
**User:** Trinity Chung
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4133197?comment=9514986
Can we have the lab/journal time slots in advance so we can figure out our schedules?
**User:** Matthew Sulistijowadi
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4133197?comment=9515660
I think it already up on weekly schedule assuming they up to date 
**User:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4133197?comment=9517984
Yes - the potential slots are already on the calendar! 

### Comment
**User:** Trinity Chung
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4133197?comment=9528553
where is the journal signup >:0
**User:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4133197?comment=9528600
u got me a min 2 early :')


### Comment
**User:** Daniel Tan
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4133197?comment=9535247
Do we have discussions this week? If so are the discussion locations out yet?

Also btw where will lecture recordings be?
**User:** Tarun Amarnath
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4133197?comment=9541269
We're trying to get the lecture recordings, but it seems like there are some issues of approval. Working on it!
### Comment
**User:** Daniel Tan
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4133197?comment=9573334
How early can we start to reserve lab time and go do projects? I tried the reservation links but they're both 404.
**User:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4133197?comment=9588006
Hi! You should be able to reserve time on these calendars found on the policies page (they're working on my end at the moment):

Arm Robot Calendars Reserve Here

Turtlebot Calendars Reserve Here
# Homework 1
**User:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4133203
 Feel free to post any questions for Homework 1 in the thread!
### Comment
**User:** Michael Chigaev
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4133203?comment=9520064
To clarify, here: |ẋ| refers to the norm of the velocity vector?


**User:** Tarun Amarnath
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4133203?comment=9526347
Yup, exactly
### Comment
**User:** Michael Chigaev
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4133203?comment=9528532
For 1.1, I am able to derive the equation of motion using Newtonian mechanics, but I'm confused as to how to do it via Lagrangian mechanics. With L = T - V,  I derive an expression for the kinetic and potential energies and then use the Euler-Lagrange equation, but this always results in an expression which only includes the force due to gravity. 

How should one go about ensuring the other quantities end up being included?
**User:** Nima Rahmanian
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4133203?comment=9538017
What choice of generalized coordinates did you make? This plays a big role in solving the question.


**User:** Zekai Wang
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4133203?comment=9571677
Same question. Can any one point me to some resources about how to deal with outside forces.
**User:** Max Wilcoxson
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4133203?comment=9583605
I would strongly recommend reading through discussion materials from 106A if you haven't yet, in particular discussion 8 dynamics. The solution for that worksheet as well as a walkthrough video are available, and they cover similar problems. They can be found here: https://ucb-ee106.github.io/eecs106a-fa23site/.



### Comment
**User:** Jackson Hilton
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4133203?comment=9567175
For question 4.2, is the n in the inequality: 

the same as the n in the size of the matrix: 

or is it supposed to be an arbitrary timestep?
**User:** Nima Rahmanian
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4133203?comment=9579157
It's meant to be an arbitrary timestep. Good catch.
### Comment
**User:** Zekai Wang
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4133203?comment=9571698
Are we required to use LaTeX for HW?
**User:** Nima Rahmanian
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4133203?comment=9579175
No, but please make sure your handwriting is clear. If your handwriting is not clear, you might not receive full credit for your work.


### Comment
**User:** Trinity Chung
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4133203?comment=9588042
gradescope assignment?
**User:** Nima Rahmanian
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4133203?comment=9593546
It should be up!
### Comment
**User:** Joyee Chen
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4133203?comment=9639798
For 2.2 and 2.3, is it possible to force the limit to be 0 regardless of any conditions on lambda, breaking the desired conclusion, simply by setting x(0) = 0?
**User:** Tarun Amarnath
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4133203?comment=9644990
The question asks you to prove the answer for any initial condition 
### Comment
**User:** Derek Guo
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4133203?comment=9662342
For 4.1, can we assume any other conditions on u? If u is unrestricted it seems like the limit might not exist as delta t approaches zero, i.e. if u is Thomae's function.

Additionally, does the condition of Lipschitz continuity bound the change in f based on the norm of the change in [x, u], based on the change in x and u separately, or just one of them?
**User:** Tarun Amarnath
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4133203?comment=9671203
Good question - I believe for multivariable functions, the Lipschitz constant is the maximum of that for $x$ and $u$. This should answer your first question as well - $u$ must be continuous as well.


# Project/Homework Teammatch
**User:** Jiachen Lian
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4133235
 Hey, is it possible to open a thread for teammatch for lab/projects/homeworks?
### Answer 
**Name:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4133235?answer=9517963
Yes! Will open a thread tonight for this! 
# Waitlist Priority
**User:** Jacob Gottesman
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4134748
 I asked this in class, but just wanted to double check.

Do EECS/BioE students have waitlist priority over other non-EECS/BioE students on waitlist?

I'm currently #4 on the waitlist as an ME student and was curious if EECS student behind me on the waitlist would get in before me or not.
IK that's how it works for some of my ME classes. 

Thanks!
### Answer 
**Name:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4134748?answer=9528851
Hi Jacob, unfortunately, staff aren't super familiar with (and are not in control of) the waitlist priority for this semester. It would be best if you email the advisors regarding this, so sorry! 


**User:** Jacob Gottesman
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4134748?comment=9528896
My assumption is that ME is lower priority as I'm still #4 but the waitlist has gone down by 3 (assuming students didn't leave the waitlist)


# Waitlist
**User:** Harshika Jalan
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4136294
 Do graduating seniors on the waitlist get priority?
### Answer 
**Name:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4136294?answer=9528863
Hello! Unfortunately, staff aren't super familiar with (and are not in control of) the waitlist priority for this semester. It would be best if you email the advisors regarding this! Sorry about that!


# Teammate Matching Thread!
**User:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4137882
 Looking for teammates to work on labs/projects or the final project with? Or a study group to do homework with? Feel free to introduce yourself in the thread below! 

Remember that labs/projects should be done in a group of 3, and the final project should be done in a group of 4. You can mark your introduction as resolved if you no longer need more people to work with after you post.


### Comment
**User:** Alejandro Marquez
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4137882?comment=9529003
Hi everyone! My name is Alejandro and I am a 4th year EECS major. I don't have a group yet for labs or projects so I am open to start one with anyone :) I try to get started early on big assignments and I am aiming for an A in this course so I am looking for people with similar goals. You can email me if interested: alejandromarquez@berkeley.edu
### Comment
**User:** Jiachen Lian
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4137882?comment=9530558
Hey. Thanks instructors for organizing this. I am 3rd year EECS PhD working on speech and active interaction. I also took 106/206A last fall semester. I plan to take this course almost 100%. My only concern is that I might not have too much time solving all homeworks due to my other research load. But I will try my best. In the meantime, I am looking for lab/project mates. Welcome to pin me at jiachenlian@berkeley.edu
### Comment
**User:** Joyee Chen
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4137882?comment=9544420
Hi there! I'm Joyee, a 4th year EECS undergrad who really wants to do a project related to AI/RL safety! Email me back at joyeechen@berkeley.edu or text at 714-773-2329
### Comment
**User:** Zekai Wang
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4137882?comment=9555552
Hi! I’m Zekai Wang, a second year CS major. Although I haven’t taken 106A, I have some experience with ML/RL, have good coding skills, and am willing to put into a lot of time for the projects. Currently I have no teammates, and I’m open to any kind of groups! Email me at zekai.wang@berkeley.edu (plz), thanks!
### Comment
**User:** Matthew Thomas
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4137882?comment=9556764
Hi! I'm Matthew Thomas, a 4th year ME major. I took 106A, and I think I have decent programming skills (I somehow survived CS 162). I'm open to doing a project on any subject, so let me know: calmechestudent@berkeley.edu 
### Comment
**User:** Katherine How
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4137882?comment=9560778
If we have a group of four for the final project can we keep the same group for labs or do we need to split up and find 2 other students to make 2 groups of 3?
**User:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4137882?comment=9561442
You need to have a group of 3 for your labs/projects and a group of 4 for your final project. How you split that up is completely up to you!
**User:** Encheng Liu
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4137882?comment=9619126
Still want to clarify that can we keep the three persons for our final project as our lab?


**User:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4137882?comment=9650226
Not sure I fully understand that question but you need a group of 3 for labs/projects and a group of 4 for your final project. Who they are is up to you!


### Comment
**User:** Pranit Panda
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4137882?comment=9611031
Hi! My name is Pranit and I'm a 4th year EECS major. I took 106a and my final project was a robot that could play tic tac toe. I specifically did the computer vision part where I extracted the board from the whiteboard and added checks for robustness (for example making sure to not extract a board state when the whiteboard is obstructed by an arm). I'm open to any project and am taking one other technical course this semester. If you are interested in working with me please email me at pranit@berkeley.edu or text me at 5103862741
### Comment
**User:** Bill Zheng
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4137882?comment=9616540
Hi! My name is Bill Zheng, and I'm a 3rd year EECS major. I took 106A last semester and I'm currently doing research in robot learning and reinforcement learning. I'm currently thinking about a project in that direction, but I'm also open to any other ideas. Please let me know if you're interested in working with me at bill.cy.zheng@berkeley.edu or text me at (626)341-8333
### Comment
**User:** Edward Chang
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4137882?comment=9620559
Hey yall! I'm Edward and I'm a 4th year EECS Major looking for a partner for the labs/projects. Although I do not have a final project idea yet, I am open to discuss once we have formed our group. You can email me at edchang23@berkeley.edu
### Comment
**User:** Katherine How
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4137882?comment=9622102
Hi! My friend and I are looking for one last partner for a lab group. We’re hoping to start on labs early so if anyone still needs a group and would like to get started on project 1, feel free to email me at katherineh510@berkeley.edu! 
### Comment
**User:** Joshua Tsai
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4137882?comment=9625847
Hey everyone, my partner and I are looking for one more to join us for our lab/project group. Email me at jtsai24@berkeley.edu if you are interested!
### Comment
**User:** Michael Chigaev
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4137882?comment=9642238
Hi everyone,

I am currently looking for a lab/project group! I prefer to start labs earlier rather than later, and I am aiming for an A in this course!

For the final project, I'd love to explore using Computer Vision in some fashion.

Send me an email at mchigaev@berkeley.edu if you/your group is interested!
### Comment
**User:** Johnny Lau
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4137882?comment=9730982
Hi everyone, I'm currently still looking for a lab/project group. I am open to any kind of group, and I can join any group of 3 as your fourth.  Feel free to email me at kitwail302@berkeley.edu, thanks!


### Comment
**User:** Leo Huang
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4137882?comment=9740676
Hey everyone, I'm still looking for a lab group. Send me an email at klhftco@berkeley.edu if you are interested!
### Comment
**User:** Arun Yadavalli
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4137882?comment=9756270
Hi everyone! My name is Arun and I am a 2nd year studying EECS + ME major. I am still looking for a group to join and am willing to join anyone's team. You can email me if interested: arunway@berkeley.edu 408-469-3571


### Comment
**User:** Antony Zhao
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4137882?comment=9788951
Hey everyone, is there anyone who hasn't done/is still working on Project 1 that needs a partner or member for their group? Email me at ayzhao7761@berkeley.edu if you are interested.
### Comment
**User:** Jiachen Lian
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4137882?comment=9807110
Hey. We still need one more member in our group. If you have taken 106a or have experience on ROS, please email jiachenlian@berkeley.edu.
### Comment
**User:** Matthew Thomas
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4137882?comment=9994368
Is anyone still looking for a team for Project 2? One of my team members dropped the class. Fill free to email me: calmechestudent@berkeley.edu.
### Comment
**User:** Johnny Lau
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4137882?comment=10027321
Hi everyone, my group is currently looking for one more member for Project 2. Feel free to email kitwail302@berkeley.edu. 


### Comment
**User:** Daniel Tan
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4137882?comment=10307458
Hey if anyone is still looking for an honest working teammate, text me at 3417668498.


### Comment
**User:** Harshika Jalan
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4137882?comment=10936033
Hey we are looking for a 4th teammate for the final project. Email me at harshikajalan@berkeley.edu
### Comment
**User:** Jiachen Lian
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4137882?comment=11097457
Hey we are looking for 2 more guys for final project. We can discuss when you email jiachenlian@berkeley.edu


# What days of the week are HWs and projects due?
**User:** Joyee Chen
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4138041
 I checked the Ed mega post but it didn’t mention a day; I checked the course website but it seemed its dates weren’t updated from 2023.
### Answer 
**Name:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4138041?answer=9528442
Homeworks are due on Wednesdays and projects are due on Fridays. The tentative dates are outlined in the policies tab of the website! 
# DSP questions
**User:** Joyee Chen
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4138079
 What extensions can we get if we have DSP assignment extension? What about DSP exam time 150%?
### Answer 
**Name:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4138079?answer=9528916
Hello! We will follow up with everyone who needs DSP extensions over email regarding any personal accommodation as soon as we can!
# Journal club signups & more!!
**User:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4138521
 Hi everyone! Hope you had a great Wednesday:)

Journal Club Signups

This is the link to the form to sign up: https://forms.gle/8L1DNnJEwfTMQEhu6. Please use this form to indicate your preferences for journal club. Section assignments will be released next week. 

Please fill out this form even if you are pending enrollment (waitlist/concurrent enrollment). People who fill out the form sooner will more likely get their first choice slot. This form is due by 11:59 pm on Sunday, January 21st. If you do not fill out this form before the deadline, you will be randomly assigned to a journal club section. 

Use this thread to post any questions about this!

Other Important Stuff

Optional Project 0 is released! Please note: you may hit some start-of-semester bugs, and if you do, please either post in the Project 0 thread or come in during office hours.

106A Assessment is released! If you have not taken 106A, please complete this assessment. A place to submit will be created on Gradescope. 

Discussion section will be held in Cory 299 on Thursdays at 4pm and Cory 521 on Fridays at 11am.

If you emailed me about something, I will get back to you as soon as possible! In the future, make a private post on Ed so all staff can access it (and you'll get a response sooner). But if you already emailed me don’t make a duplicate post, please! 
### Comment
**User:** Cecilia Pham
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4138521?comment=9530083
Hi, 

When is the due date for the 106A Assessment? Thank you!
**User:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4138521?comment=9547014
Hello! Sorry for not mentioning this above, but it's due by the Early Add/Drop Deadline (January 26th) -- this will be reflected in Gradescope shortly!
### Comment
**User:** Joyee Chen
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4138521?comment=9538205
What day of the week are discussions generally released?
**User:** Tarun Amarnath
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4138521?comment=9543642
They will certainly be posted by section. We will try to get them out earlier, but no promises on that front.
### Comment
**User:** Alejandro Marquez
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4138521?comment=9606674
When will journal club section assignments be released?


**User:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4138521?comment=9607175
Just released here!
### Comment
**User:** Charles Paxson
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4138521?comment=9611734
I joined the waitlist on Friday and was added to Ed after the journal club signup sheet was closed, what happens if I can't make it to the section I was assigned?


**User:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4138521?comment=9613056
As I mentioned in the Week 2 announcements, please make a private post about it with all the sections that you can make, and we will try our best to switch you to one of those.
**User:** Charles Paxson
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4138521?comment=9616106
I see, thank you!

# Project 0 Thread
**User:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4138532
 Please use this thread for any questions regarding Project 0! This is an optional project. 

It can be found on the course website: https://ucb-ee106.github.io/106b-sp24site/assets/proj/proj0.pdf. 
### Comment
**User:** Nithila Poongovan
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4138532?comment=9538032
Are there any class resources that I can use to set up ROS on my personal laptop?
**User:** Tarun S Ganamur
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4138532?comment=9544356
This walks you through installing ROS noetic, but you need to have Ubuntu 20.04 or higher as your OS (doesn’t work on widows or macOS). 
 
http://wiki.ros.org/noetic/Installation/Ubuntu
**User:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4138532?comment=9547032
Also, this is a guide from last year that may be helpful: https://docs.google.com/document/d/1Ad9PDpqAI6W5701Lm51QJgKpjSAzO90iAQAEw_TGnII/edit?usp=sharing
**User:** Tarun Amarnath
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4138532?comment=9552121
^^ Use Han's doc! But one thing to remember is that you won't be able to use the Turtlebots/Sawyers with your own laptop. 
**User:** Nithila Poongovan
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4138532?comment=9554349
amazing - thank you so much!

### Comment
**User:** Max Wilcoxson
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4138532?comment=9565591
Where do we schedule time slots to work with the robots? I can't find it on the website. Thanks!
**User:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4138532?comment=9588013
Hi! You should be able to reserve time on these calendars found on the policies page:

Arm Robot Calendars Reserve Here

Turtlebot Calendars Reserve Here

Let me know if you're having any trouble with the links!
# Waitlist Teammatching
**User:** Matthew Thomas
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4138842
 Should people on the waitlist try to team up with other waitlisted people? Like if one enrolled student is paired with another student who doesn't get into the class, would that pairing be detrimental to the enrolled student? (Although, if some waitlisted students get in and others don't, you could wind up with the same problem I guess.)
### Answer 
**Name:** Tarun Amarnath
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4138842?answer=9552176
I think it depends on how long you're planning to stay on the waitlist, but I do think it's probably in enrolled students' favors to partner with another enrolled student. The waitlist will move somewhat, so in the end, it's up to you on how to balance that. 
# Question on Journal Club
**User:** Encheng Liu
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4139432
 Hi, I am still confused on the journal club and the formal Lab.  Is the journal club a part of "1 hour of organized paper-reading time in lab sections." mentioned in the website policy, Or if that week has a formal Lab, and there will not be any journal club?

Are they mandatory and held weekly? Because I have a class end at 10:15 once every four weeks on Thursday, I want to know if I could be a little bit late for about 15 min once every four weeks at the journal club when I signing up for it. 

Thanks!


### Answer 
**Name:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4139432?answer=9546891
Hi! Sorry about the confusion! The journal club is the 1 hour of organized paper-reading time. There's no other synchronous/in-person component of lab. Working on projects is done on your own time.

They are mandatory and they are held weekly.

As far as your conflict, I'd encourage you to choose a different section than the 10 am one, since coming late will not be great (defeats the entire purpose since it's only an hour-long meeting). Plus if you factor in the time it takes for you to walk from your class, you'd be missing half the meeting. On the other hand, since it's only every four weeks, it's probably okay -- you'd have to get it excused by whoever your Lab TA is, and make sure that you're not presenting on any of those days. But it's better for everyone if you prioritize a section you can always make it to, barring any extenuating circumstances. 


**User:** Encheng Liu
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4139432?comment=9547055
Thank you for your response!

So the journal club is once four week right?
**User:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4139432?comment=9547246
The journal club is every week!
**User:** Encheng Liu
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4139432?comment=9547263
gotcha, Thank you!




# When is 106A assessment due?
**User:** Cecilia Pham
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4141989
 Hi, I couldn't find information on the previous threads about when the assessment for students who haven't taken 106A will be due and don't see it on the pdf of the assignment. Thank you!


### Answer 
**Name:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4141989?answer=9546936
Hello! Great question - the latest we'd like you to complete it by is the Early Add/Drop Deadline (January 26th). I'll make sure this is on Gradescope as soon as I can as well!


# Do we have journal club today?
**User:** Encheng Liu
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4142430
 Hi, I want to know do we have journal club today?

I saw the logistics slides here, but there are no people showing up.


### Answer 
**Name:** Sandip Maharaj
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4142430?answer=9540423
I think journals start next week as it is in the calendar for next week and not in this week's calendar.
### Answer 
**Name:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4142430?answer=9546985
Sorry yes - journal club starts next week! That is last year's version of the slides. Where did you find that?
**User:** Encheng Liu
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4142430?comment=9547008
Maybe on ed or our course website, I found it from the internet during our first class


**User:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4142430?comment=9547047
Thanks! Trying to hunt down where on the website it could've been in case I linked an old version somewhere lol



# Recorded Lecture
**User:** Sunny ME
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4142517
 Where are the recorded lectures posted.
### Answer 
**Name:** Sandip Maharaj
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4142517?answer=9540449
It will be on bCourses. Seems it's not up yet. 

I am in isolation due to covid so it would be nice if lectures were posted more promptly so we don't fall behind.
### Answer 
**Name:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4142517?answer=9547000
Usually, it's posted to bCourses -- we're waiting to see if Course Capture will work for the past 2 lectures, but it may not be due to some technical issues. If not, we will do our best to get the slides up. However, it should be posted promptly starting the next lecture at the latest.


# Discussion Room Change
**User:** Tarun Amarnath
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4143297
 Hi everyone! It's me again. 

If any of you were curious, I ended up seeing a sea turtle! Two, in fact!

Our discussion rooms have been switched around. Sorry about the late notice:

Thursday 4pm: Cory 299

Friday 11am: Cory 521

See you soon!


### Comment
**User:** Anish Dhanashekar
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4143297?comment=9680883
Where did you sea the two see turtles?
# Waitlist position
**User:** Cedric Murphy
**Role:** 
**URL:** https://edstem.org/us/courses/54072/discussion/4147834
 Hello, I am currently 16th on the waitlist and I would like to know what are my chances of getting in?  I am a little antsy since I got the automated waitlist email: 
### Answer 
**Name:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4147834?answer=9561429
Hi Cedric, we don't know at this point. Staff don't have control over the waitlist, so it would be best to contact any advisors on this. So so sorry!
**User:** Cedric Murphy
**Role:** 
**URL:** https://edstem.org/us/courses/54072/discussion/4147834?comment=9562761
Actually you guys opened up a bunch of spots yesterday, so I'm in lol
# Discord server for this class?
**User:** Edward Chang
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4148818
 Is there an official or unofficial discord server for this class?

### Answer 
**Name:** Tarun Amarnath
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4148818?answer=9644974
Here you go: https://discord.gg/vcBQ6t8yUX (we won't check it regularly or promise answers on here)
# Accessibility of class materials
**User:** Sandip Maharaj
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4162184
 Why is it taking so long to have access to lectures and discussions? 
I was under the assumption that the class would be accessible to students who can not make it to class due to health reasons out of consideration for others. 
Other classes are able to post the lectures within hours after the lecture so the resources are there for this to be done. 
Should we not be considerate towards others and come to class even if we are sick because it appears that I am doing this at personal cost by falling behind already. 
### Answer 
**Name:** Tarun Amarnath
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4162184?answer=9592021
I'm very sorry about that.

The first lecture unfortunately won't be available as a recording because of university issues. You should be ok though - it was background and logistics. You'll be ok if you watch the second lecture onwards.

Sorry about discussion - slipped my mind with some other things I was doing. Will release in the next couple hours.


**User:** Sandip Maharaj
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4162184?comment=9666995
Thank you. I appreciate your efforts 🫶
# Discussion 1 Solutions
**User:** Edward Chang
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4163661
 Just wondering when the solutions for Discussion 1 will be released.
### Answer 
**Name:** Tarun Amarnath
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4163661?answer=9607465
Sorry, they are released!
# Robot Usage Quiz Autograder
**User:** Eddie Shi
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4165862
 I have filled out the robot usage quiz on gradescope and submitted it, however, it doesn't tell me if I got the questions correct or not. To my knowledge, I cannot use the robots unless I know I got all the questions on the robot usage quiz correct. 

Since it is the same quiz as 106a/206a, can the quiz be switched to autograder so we know if we passed the quiz or not? Thanks


### Answer 
**Name:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4165862?answer=9607169
Hi - you're so right! I just fixed it I think! Please let me know if you're still unable to see your score.


# Discussion Recordings
**User:** Cecilia Pham
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4168586
 Hi, I tried to find this information on the syllabus but will discussion recordings be released if we miss it? Additionally, where can we find discussion materials (slides, solutions)? Thank you!
### Answer 
**Name:** Tarun Amarnath
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4168586?answer=9607467
Discussion materials will be on the website! (Just published, sorry for the delay)
**User:** Japinder Narula
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4168586?comment=9724938
Are the discussion 2 materials going to be published as well?
**User:** Tarun Amarnath
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4168586?comment=9756988
just saw, but it's already up!

**User:** Thomas Kragerud
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4168586?comment=9975209
Hi would it be possible to publish the discussions 3 recording 
**User:** Tarun Amarnath
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4168586?comment=9988658
They should already be up?

**User:** Cecilia Pham
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4168586?comment=10101643
Hi would it be possible to publish the discussion 5 recording materials? Thank you!


# Week 2 Announcements!
**User:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4169089
 Hope you had a great first Monday of the semester everyone! Welcome back to another installment of 106B announcements:) My update is this weekend, I went on a retreat near Yosemite, and it was fun!

Here’s what’s happening this week in 106B:

Tuesday - Project 1 will be released in the evening. Please use the robot calendars to book time on the Sawyers. Here's the Sawyer Robot Calendar, also available on the policies tab of the website. You will have 1 week to do this project, but it'll be a lot shorter than the upcoming longer projects.

Wednesday - Homework 1 is due! If you require DSP assignment extensions, we will be contacting you ASAP.

Thursday - Homework 2 will be released! You will have 2 weeks to do this homework.

Thursday & Friday - We’ll have a discussion on stability on Thursday/Friday by Tarun!

Wednesday to Friday - We have journal club starting this week. It’ll be an intro to how journal club works and an opportunity to ask any questions about Project 1. 

These are the journal club assignments. Attending journal club is a key component of this class. If your assigned journal club slot absolutely doesn’t work for you, please make a private post on Ed about it ASAP including your email address & times you are available. Thanks!

Paper presentation sign-ups will be released shortly. Each person will be required to sign up for 2 papers. We’ll go over this requirement in journal club, so please don’t worry about this if it doesn’t make sense yet!

Monday to Friday - Office Hours start from this week! Please come to our office hours to utilize all the available resources in this class. It’s a great way for you to get deeper on the content, or ask any project questions. (If you missed homework/project party today, you should stop by next week!)

Friday - 106A assessment due for those who are taking the course without having taken 106A before.

That’s it, everybody! See you at lecture this week, where we’ll be talking about controls (quadrotors, linear control, Lyapunov, and more).

P.S.  This class can be a lot with things to keep up with, so don't hesitate to ask any questions and get help! Also, there may be a couple few things that I need to update on the website still (dates and links), I'm working on getting to those super soon!
### Comment
**User:** Michael Chigaev
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4169089?comment=9609358
Where is journal club held?
**User:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4169089?comment=9610528
Cory 105 (the lab room)!
### Comment
**User:** Tianqi Zeng
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4169089?comment=9634452
Just want to make sure that can each one on the team reserve 2 hours.
**User:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4169089?comment=9636790
No, it should be 2 hours per team.


### Comment
**User:** Leo Huang
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4169089?comment=9674643
Did homework 2 get released? I wasn't able to find it on the website.
**User:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4169089?comment=9677884
It'll be released shortly edit: it was released 3 hours ago lol
# journal time change
**User:** Yifeng Zhang
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4169632
 I have BUS ADM X489.8 in the Wednesday 10am-12pm and STAT241B in the Wednesday 2pm-3pm  , so can my journal time be changed to Wednesday 1pm-2pm? My email is yifengzhang@berkeley.edu. Thank you for your help.
### Answer 
**Name:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4169632?answer=9610563
Sure, Yifeng, for now. Since I haven't been informed about concurrent enrollment students, I'll let you know if once you get into the course, you would potentially need to go to the Friday 5-6pm section or Thursday 10-11am section (depends on how full the Wednesday 1-2 pm section is once you get in and/or if anyone else needs to be switched out).
**User:** Yifeng Zhang
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4169632?comment=9610626
If it means this week i should attend Friday 5-6pm or Thursday 10-11am journal club

**User:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4169632?comment=9612888
This week, go ahead and attend Wednesday 1-2 pm! I'll let you know if it needs to change once I find out if you've been enrolled in the course.


**User:** Yifeng Zhang
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4169632?comment=9612918
Ok.Thank you so much！


# Journal club signup
**User:** Lican Hou
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4170786
 Hi, I am concurrent enrollment student. My application is still pending, and I have not signed the journal club yet. Could you add me on the Journal Club Assignments on Wednesday, 11 am to 12 pm?  

My SID: 3039831317

Email: ryan_hou@berkeley.edu
### Answer 
**Name:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4170786?answer=9612920
Sure, Ryan, for now, I've added you to that section. Since I haven't been informed about concurrent enrollment students, I'll let you know if once you get into the course, you would potentially need to go to a different section.
# Journal Club not assigned
**User:** Alex Beaudin
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4171081
 Hi, I enrolled late, and wasn't assigned a journal section. 

My email is a.b@berkeley.edu. 

I absolutely can't make it to the Wednesday 1-3pm sessions.

Thanks, 

Alex Beaudin
### Answer 
**Name:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4171081?answer=9613022
Thanks, Alex! Do you have a preferred section out of the remaining ones? Put you down for my 10-11 am section but I could put you in the Friday 5-6 one or the Wednesday 11-12 if either of those timings work better for you.
**User:** Alex Beaudin
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4171081?comment=9613200
That's perfect, thank you. And thank you for responding to my email!
# Journal Club Assignment
**User:** Charles Paxson
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4172793
 Hi!

I joined the waitlist and got added to Ed after the journal club signup sheet was closed, and was unfortunately assigned to a section I can't attend.  The only section I can make it to is the Wednesday 2-3 one.

My email is chpaxson@berkeley.edu

Thank you, and sorry for the bother!

Charles Paxson
### Answer 
**Name:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4172793?answer=9617056
Just switched you, Charles!
**User:** Charles Paxson
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4172793?comment=9622059
Thank you!
# Lecture slides
**User:** Tianqi Zeng
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4173109
 Hi, will the lecture slides be posted? 
### Answer 
**Name:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4173109?answer=9617070
Yes. I will try to post them at the latest a couple of days after the lecture. The slides from week 1 will be up shortly.
# Request to be added Journal Club
**User:** Jiahui Zhang
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4174283
 Hi, I want to be added into the journal club, but the sign up form was closed. So my email address is jhzhang0341@berkeley.edu . And this calendar shows my available time. Thank you!
### Answer 
**Name:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4174283?answer=9633732
Hi Jiahui, sorry I thought I responded to your message, but I wanted to let you know that you've been added to the Friday 5-6 pm journal club section.


**User:** Jiahui Zhang
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4174283?comment=9636251
Oh！Thank you！
# Missing first Journal Club
**User:** Ethan Pang
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4174871
 Hello, I am in journal club with Michael Wed 1-2 and wanted to give a heads up that I will not be able to make it to the first session due to a job interview being scheduled at that time. I would still like to stay in this journal club time, as I'll be able to schedule around it in the future now that it has been assigned. I'll make sure to catch up with my lab partner on the details that I miss. Thank you!

Also, I was looking for Michael's email and I think the wrong one is up on the website.


### Answer 
**Name:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4174871?answer=9626949
Hi Ethan! Sorry for the typo, thanks for catching it! You can email Michael at psenka@eecs.berkeley.edu for this. It's up to his discretion for excused absences.


# Journal Club Assignment Conflict
**User:** Moaaz Akbar
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4175980
 I somehow got assigned a journal club time slot (Wed 2-3pm) which directly conflicts with another required attendance class i have. Would it be possible to switch into the Wednesday 11am-12pm section? Thanks!
### Answer 
**Name:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4175980?answer=9626414
Just switched you in, Moaaz!


**User:** Moaaz Akbar
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4175980?comment=9626550
Awesome, thanks Kirthi!
# 80% Rule for HW
**User:** Sunny ME
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4176670
 Does the 80% rule apply for HW? Can we drop any of the HWs?
### Answer 
**Name:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4176670?answer=9626425
There are no homework drops in the class, however there is extra credit offered for filling out the surveys. As far as the 80% rule, as of now, do the homework as it does not apply. Staff will have to discuss whether we end up offering this policy or not.


**User:** Charles Paxson
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4176670?comment=9877912
Was this decided on?
**User:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4176670?comment=9883286
Not yet. Will be soon though. We will announce once we decide on it!



# Project 1!
**User:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4177195
 Hi everyone!! Sorry about the late release of Project 1, I have been fighting a terrible cough all day and just dragged myself out of bed for the past couple hours. 

This is a link to the lab doc: https://ucb-ee106.github.io/106b-sp24site/assets/proj/proj1.pdf (parts of the website are still being updated, please be patient over the next two days since I'm trying my best while ill).

As noted on the doc, this project is due next week, on Friday 2/2. This is supposed to be a short lab to introduce you to working with the Sawyers a little bit beyond Lab 7 from 106A.

To do this lab, you'll have to book time on the Sawyer robots. Before you book Amir, please note that the setup is slightly worse than the other robots. So please try to book other robots if they're available.

Please remember the ground rules of the lab (no eating/drinking, use cntrl+c & pkill -u $(whoami) when logging out, put hand on e-stop, etc etc) and the booking rules for the robots -- only 2 hours of bookings on the calendar at a time. If we notice that you're booking more than this or not following any of the rules listed in the Robot Usage Guide, you will immediately fail this project. The same goes for if you don't finish the robot usage quiz on Gradescope before you start the project. 

A gradescope assignment will be released shortly for submission. 

Journal club starts from Wednesday (technically, today, since it's so late now). Please show up as attendance will be taken and you'll have the opportunity to ask any questions about the project and/or other parts of the class. If you have to miss journal club for any reason, please email your GSI directly, they can grant you an excused absence. 
### Comment
**User:** Max Wilcoxson
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4177195?comment=9640396
Small heads up, the project repo appears to actually be in https://github.com/ucb-ee106/106b-sp24-labs-starter (hyperlinked starter repository in section 3.2), not the repo listed in the command that follows this hyper link (git clone https://github.com/ucb-ee106/106b-sp23-project-starter). 


**User:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4177195?comment=9644330
Oof I'm sorry I thought I fixed all the lab doc links. Will fix & upload soon!
### Comment
**User:** Daniel Tan
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4177195?comment=9651980
Lab slides says access denied.
**User:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4177195?comment=9654038
Everyone should be able to view now


### Comment
**User:** Max Wilcoxson
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4177195?comment=9661477
With respect to trajectory following, should the robot follow the tag live as a user moves the tag (with as minimal lag as possible), or should the robot instead smoothly move from current space to the desired location with respect to the tag, and only check again for the tag location after reaching the initial destination?
**User:** Karim El-Refai
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4177195?comment=9677885
It should be real time so it should be constantly following the ar tag
### Comment
**User:** Trinity Chung
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4177195?comment=9676919
in proj1_pkg/src there is controllers/controllers.py (correct) and controllers.py (incorrect?)
**User:** Kaitlyn Lee
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4177195?comment=9699107
Also have this same question! What is the purpose of the two files and which one do we edit?
**User:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4177195?comment=9704163
The controllers.py in proj1_pkg/src was carryover from the old version of the project where we had a portion in simulation. Use the one in controllers/controllers.py, that will be the one for the sawyer. Sorry about that confusion!

### Comment
**User:** Trinity Chung
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4177195?comment=9677291
in tf, right_hand_gripper does not exist, only right_hand, right_gripper_tip, right_gripper_base
**User:** Karim El-Refai
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4177195?comment=9677876
Right_gripper_tip should be okay. You can visualize the urdf of the sawyer using the commands in lab 5 of 106a in section "2.1 Specify a robot with a URDF". The name of the gripper should be specified in the generated pdf.


### Comment
**User:** Matthew Sulistijowadi
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4177195?comment=9684517
outta curiosity why do we have a proj1a and proj1b vs keeping them in the same folder? 
**User:** Max Wilcoxson
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4177195?comment=9684950
I believe you may be looking at the old repo, https://github.com/ucb-ee106/106b-sp23-project-starter, not https://github.com/ucb-ee106/106b-sp24-labs-starter. (See my comment below). Hope this helps!


**User:** Matthew Sulistijowadi
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4177195?comment=9684971
thank you! we happened to already figure it out tho luckily so we aren't stuk on the side quest anymore



### Comment
**User:** Kalie Ching
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4177195?comment=9685106
We are running into this error when running main.py, even though matplotlib seems to be installed properly. Any ideas for how to fix this?

ImportError: cannot import name '_api' from partially initialized module 'matplotlib' (most likely due to a circular import)


**User:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4177195?comment=9686147
Could be a software issue across the board (based on what I've seen on stackoverflow lol). I'll have ISG check it out if other groups are also experiencing this problem? Hard to try to fix it wo being there tho
**User:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4177195?comment=9704047
Which robot was the issue on?
**User:** Harshika Jalan
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4177195?comment=9721439
amir
**User:** Kalie Ching
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4177195?comment=9721754
We think it might be a good idea to update / reinstall matplotlib on Amir's computer because there seems to be a circular dependency with matplotlib itself. We run into an OSError when we try it ourselves so we might not have permissions.
**User:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4177195?comment=9779163
Unfortunately, ISG wasn't able to repro this issue:( let me know if you guys were able to come up with a solve for it
**User:** Kalie Ching
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4177195?comment=9779290
We changed the directory name of matplotlib to matplotlib2.




### Comment
**User:** James Ni
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4177195?comment=9685277
We're running into this error when we're trying to launch the camera:


ERROR: cannot launch node of type [ar_track_alvar/individualMarkersNoKinect]: Cannot locate node of type [individualMarkersNoKinect] in package [ar_track_alvar]. Make sure file exists in package path and permission is set to executable (chmod +x)

When we manually inspect the ar_track_alvar package, we notice that there is an IndividualMarkersNoKinect.cpp (with capital I). Could this be a typo that's causing the launch to fail?
**User:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4177195?comment=9686060
Make sure you run chmod +x *.py in any folder that has python files that you're trying to run. And make sure you've run catkin_make in the root, source devel/setup.bash all terminal tabs
**User:** James Ni
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4177195?comment=9699877
Iric we verified that all .py files in the package have execute permissions, and we've setup the ROS workspace and sourced devel/setup.bash in all tabs. We will double check this again next time, but we think this may not be the issue.


**User:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4177195?comment=9704040
Which robot?


**User:** Kalie Ching
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4177195?comment=9720280
Amir
**User:** James Ni
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4177195?comment=9720831
We figured it out, it was because we installed the ar_track_alvar package prior to catkin_make, and did not rebuild from scratch.



### Comment
**User:** Edward Lee
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4177195?comment=9702147
For the PD joint torque controller, the Coriolis matrix returned by sawyer_pykdl is of shape (7, 1) instead of a (7, 7) matrix -- is this correct? If so, how are we supposed to be using this?
**User:** Max Wilcoxson
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4177195?comment=9750819
I also experienced this, not sure why this is the case
**User:** Max Wilcoxson
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4177195?comment=9751131
It is my understanding that this method actually returns $C(\theta, \dot{\theta})\dot{\theta}$, not $C(\theta, \dot{\theta})$, hence the difference in dimensions. Is this correct?
**User:** Edward Lee
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4177195?comment=9751646
I talked to Karim, and yup this is what he said!
**User:** Karim El-Refai
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4177195?comment=9757118
Yes that is correct


### Comment
**User:** Edward Lee
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4177195?comment=9702869
For the workspace velocity controller, is there a function to take logarithm of the SO(3) matrix, or do we have to code the algorithm ourselves (using the formula from the textbook as a reference)?
**User:** Max Wilcoxson
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4177195?comment=9750832
I believe you are allowed to assume that there is no rotational component of the transformation (the gripper will always stay the same orientation), which significantly simplifies the math if you analyze it
**User:** Karim El-Refai
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4177195?comment=9779020
We do not provide this and you would have to implement it on your own. Like Max said though you can assume there's no rotation.
### Comment
**User:** Harshika Jalan
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4177195?comment=9735183
Are we allowed to restrict the velocity to a certain range for safety reasons? We experienced a velocity spike which caused really unstable behaviour by the robot.
**User:** James Ni
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4177195?comment=9735802
To clarify, the velocity spike is in the desired velocity, which is unexpected because the position graphs are smooth and look like what we expect. Attached is a picture of our workspace error.
**User:** Karim El-Refai
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4177195?comment=9779013
Yes this is totally valid
### Comment
**User:** Angel Aldaco
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4177195?comment=9746781
why do we have 2 trajectories.py?  Am I missing the point of one of them cause one of them seems redundant?


**User:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4177195?comment=9779159
Yeah one of them is a copy error from the old version of the lab. Feel free to pick one and use it. 
### Comment
**User:** Max Wilcoxson
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4177195?comment=9751596
One thing I found that I hope might be useful for others: 
It seems that some of the library functions like self._kin.inertia return np.matrix objects instead of np.array objects. These don't seem to follow certain conventions you might expect np.array objects to follow with respect to matrix multiplication dimensions and data type of elements of these arrays. 
### Comment
**User:** Max Wilcoxson
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4177195?comment=9752344
To clarify the specification, do we need to implement all three controllers and make a video demo of one, or just implement one controller? 
**User:** Karim El-Refai
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4177195?comment=9757161
Just one controller is sufficient! 
### Comment
**User:** Max Wilcoxson
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4177195?comment=9752577
Also the project spec mentions that "We have found that gravity vector returned by KDL is quite wrong. So, we have implemented our own package to compute the gravity vector. This is the baxter pykdl package (which contains support for Baxter and Sawyer), and you should use this to access the robot’s inertia, gravity, and coriolis terms." 

Does the existing import phrase from sawyer_pykdl import sawyer_kinematics in follow_ar.py take the correct sawyer_pykdl in the baxter pykdl package, or do we need to import a different kinematics package? Thanks!
**User:** Karim El-Refai
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4177195?comment=9757212
This import should be correct
### Comment
**User:** Bill Zheng
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4177195?comment=9777259
What is the latency of the movement we are supposed to expect from the robot?
**User:** Karim El-Refai
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4177195?comment=9779010
The rate at which kirthi's video in the journal slides is very good. Something slightly more delayed than this would also be good. If you are dealing with a large amount of lag reconsider how often you're replanning if at all and if you're doing unnecessary or excessive computation
### Comment
**User:** Max Wilcoxson
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4177195?comment=9790158
With respect to "experimental design" and "control groups" referenced in the lab report guidelines, how might we apply this notion to the lab? It feels like the objective of the lab is to create a controller, what is expected with respect to experimental design? Thanks for the clarification!
**User:** Karim El-Refai
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4177195?comment=9795822
If your controller doesn't work first try the idea is that you're a bit methodical with how you go about diagnosing the issue and working to fix it. Experiments in this project can be as simple as just moving the AR tag around and seeing how it's performing
### Comment
**User:** Max Wilcoxson
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4177195?comment=9790514
With respect to the theory section of the report, to what level do we need to "derive" the equations? Like for joint space PD controller, the equation is well known/shown in lecture. Do we need to derive this from scratch?
**User:** Karim El-Refai
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4177195?comment=9795839
You dont need to derive from scratch but should explain why the equation is the way it is and how it impacts the system
**User:** Kalie Ching
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4177195?comment=9803980
Does this apply to the other controllers as well? Would it work start with a high level description of the controller, then state the controller equations and explain each part of the equations for how it contributes to the overall system?


**User:** Karim El-Refai
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4177195?comment=9805461
Yeah this is sufficient though some mathematical intuition should also be included when relevant


### Comment
**User:** Charles Paxson
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4177195?comment=9790633
Should we make the GitHub repo public, or just share it with a particular set of accounts, to satisfy this?


**User:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4177195?comment=9803808
Please make your repos private! In the future, we'll have github classroom to handle this. 

Share it with eecsbioeme-106@berkeley.edu. Thank you!
### Comment
**User:** Max Wilcoxson
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4177195?comment=9793061
For the workspace velocity controller, can we also add a derivative feedback term, or should we do a purely positional controller as modeled in the implementation description?
**User:** Karim El-Refai
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4177195?comment=9795314
Feel free to make any modifications to your controller to get it to work better! 
### Comment
**User:** Alejandro Marquez
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4177195?comment=9808259
For the Jointspace velocity controller do we need to set both the joint velocities and joint torques? I'm confused on why step_control takes in a target acceleration


**User:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4177195?comment=9811872
For the jointspace velocity controller, you can just set joint velocities. In step control, you can set target acceleration to zero 


### Comment
**User:** Jiachen Lian
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4177195?comment=9808352
What is "Difficulties section" in writeup?
**User:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4177195?comment=9811837
Any difficulties you encountered in the project! Can be with software, hardware, any other challenges, etc. 
**User:** Anish Dhanashekar
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4177195?comment=9814060
Since a difficulties section was not outlined in the deliverables within the project spec, I was not aware it existed until just now, when submitting my team's report to Gradescope. However, we elaborate on difficulties encountered when going over our tuning process in the experimental results section - would it be sufficient to match those pages for the difficulties question on Gradescope? Thanks!
**User:** Karim El-Refai
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4177195?comment=9814611
Sure
**User:** Anish Dhanashekar
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4177195?comment=9814691
Thanks!





### Comment
**User:** Daniel Tan
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4177195?comment=9818609
Robot issue: Ada is giving back faulty joint position and velocity. The joint velocity it's reporting is always 0.
**User:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4177195?comment=9820735
You can try restarting the robot. Other than that, ESG will check it out on Monday/Tuesday.
# Journal Club/Lab
**User:** Michael Chigaev
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4179555
 Hi!

I badly sprained my ankle yesterday and I am not able to physically come in for my assigned journal club slot with Karim from 11am-12 today.

Would it be possible to get an overview of what the first journal club session is? Or would it be possible for me to come in for the sections on Thursday or Friday this week?



Thank you so much!

Michael


### Answer 
**Name:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4179555?answer=9633654
Hi Michael, please email Karim directly about this! For this first week, you're more than welcome to come in to the Thursday or Friday sections.
**User:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4179555?comment=9635903
Michael, I'm looking for one person to switch into a different section from Karim's 11-12 section. Is there another section that you could make it to permanently? No worries if not, just thought I would ask.


# Journal Club
**User:** Harshika Jalan
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4179683
 Hi I'm enrolled in the Friday 5pm journal club but I'm not sure if I can make it to that one. Would it be possible for me to switch to the Wednesday 11am-12pm or Thursday 11am-12pm one? 
### Answer 
**Name:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4179683?answer=9633714
Hi Harshika, unfortunately at the moment we may not be able to switch you. This week, feel free to attend either of the ones you can, and I'll get back to you. Maybe if you make a public post asking if someone would be down to switch with you, I could work that out? 
**User:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4179683?comment=9635829
Wait sorry Harshika, just to clarify would you be good with Thursday 10-11 am? If so I'd be able to switch you


**User:** Harshika Jalan
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4179683?comment=9745502
yes that works! can you please switch me?



# DSP homework extension
**User:** Sandip Maharaj
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4182979
 I just got my assignment deadline extension accommodation approved. It will be available in the DSP portal tomorrow. I will update the letter and send it again tomorrow with the new accommodation. Just curious if I can use it on homework 1 that is due tonight or do I use my slip days for it?
### Answer 
**Name:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4182979?answer=9644451
Hi Sandip, sure you can use it on assignment 1 (I'll add the extension tonight, but will expect to verify in the letter tomorrow). Here are the guidelines regarding the assignment deadline extension accommodation: 

Due to your DSP accommodations, you will be given a 2-day extension for each homework and project assignment. As a result, you will not need to rely on any slip days for that. Unfortunately, you won't be able to apply additional slip days on top of this 2-day extension since we aim to release solutions 2 days after the homework is due. If you need a longer project extension, we may be able to accommodate it if any circumstances arise, which will need to be communicated to us promptly. However, we encourage you to submit it sooner as otherwise, it can be easy to fall behind and project time will cut into the final project.

So, your homework deadlines will be on Fridays instead of Wednesdays. For example, tonight's homework 1 will be due on Friday (1/26). Your project deadlines will be on Sundays instead of Fridays, so Project 1 will be due on Sunday (2/4). I hope this clarifies how the assignment extensions will work for your DSP accommodation! Let me know if you have any questions. 
**User:** Sandip Maharaj
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4182979?comment=9667051
Thanks. I understand. Not sure about the projects though since I will be working with 2 other students. It’s ok if I submit for the team and they won’t get penalized for it. Essentially I am asking if my extension will apply to my team for the projects. 
Also the DSP system is still processing the update I’ll use the slip days on this one. If it does not get processed soon then I will try to generate a pdf and email it.
**User:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4182979?comment=9678206
Hey Sandip, no worries I already added the extension to Gradescope so don't worry about it. And yes, your project partners won't be penalized for submitting 2 days late. You won't even have to submit for the team, the extension will just be applied to your group. Hope that clarifies it! 


**User:** Sandip Maharaj
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4182979?comment=9723192
Yes that clarifies all my questions. 
**User:** Sandip Maharaj
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4182979?comment=10032021
The solutions are not being released 2 days after the homework due date and normally in other classes they give 3 days extension and I think I need that extra day because I have been able to compete the homework’s so far with the time given. Especially because I have to do a lot of self learning since I do not find the lectures useful for my learning style. 
**User:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4182979?comment=10405357
Hi Sandip, for whatever reason I missed this (in the future please make a new private post). You can have an extra day for the homework going forward. Sorry about that. 





# 106A Assessment
**User:** Akhil Vemuri
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4184916
 It seems like I can't access the 106A assessment on the website. Could someone link it please?
### Answer 
**Name:** Akhil Vemuri
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4184916?answer=9645801
Nevermind, the link is wrong on the website. Should be https://ucb-ee106.github.io/106b-sp24site/assets/misc/106a_assignment.pdf.
### Answer 
**Name:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4184916?answer=9650189
Fixed this now! Thanks for the catch! 
# Journal club assignment
**User:** Lican Hou
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4185234
 Sorry, I forgot that there is a class scheduled during the same time as the journal club on Wednesdays from 1 to 2 pm. Can I change the time of the journal club to be on Thursdays from 10 to 11 am? My email is ryan_hou@berkeley.edu, SID is 3039831317. Thank you!


### Answer 
**Name:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4185234?answer=9650119
Sure, done. 
# Prof OH
**User:** Jiachen Lian
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4187532
 Hey. Will Prof Shankar hold his OH in the office? 
### Answer 
**Name:** Jiachen Lian
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4187532?answer=9652405
Hey. Solved! 
### Answer 
**Name:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4187532?answer=9654024
Yep! For everyone's reference, Prof Sastry's OH is held in Cory 333B!
# CSM 16B JM/AM/CM Application Reopening
**User:** Tarun Amarnath
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4188818
 Posting on behalf of CSM:

Computer Science Mentors (CSM) is a student-run organization that provides guidance and resources in a smaller classroom environment through group tutoring sessions for CS 61A, CS 61B, CS 61C, CS 70, CS 88, EECS 16A, and EECS 16B.

We are reopening the application for EECS 16B, for Junior Mentor (JM), Associate Mentor (AM) and Content Mentors (CM).

This application is available at https://tinyurl.com/sp24-csm-app. The extended deadline is Sunday, January 28 at 11:59pm PT. No late applications will be accepted!

In CSM, we are committed to diversity and inclusion and value applicants of all backgrounds. We consider applications holistically and value passion for mentoring and willingness to learn over your grades and struggles during your time as a student - we do not consider grades in our application process. Overall, we encourage all who are interested to apply and to give the application your best effort!

Want to hear from real mentors what CSM is like? See our info session recording and slides.

Visit https://csmentors.berkeley.edu for more information or reach out to us at mentors@berkeley.edu!
# Midterm
**User:** Cecilia Pham
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4191824
 Hi! I was reading on the website that there will be a midterm synchronously April 11th though I recall on the first day of lectures that the midterm would be take home. Did this get changed? Thank you!
### Answer 
**Name:** Enyang Zou
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4191824?answer=9664569
I am also curious about this issue.

I took this photo in class,

While on the Policies - EECS C106B (ucb-ee106.github.io), it says There will be one midterm to ensure students are caught up with course material. It will be an in-person, synchronous exam on April 11th. Logistics will be released closer to the test date.

Personally, I hope it would be a take home exam.
**User:** Encheng Liu
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4191824?comment=9677263
Same
**User:** Edward Chang
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4191824?comment=9868154
fingers crossed for take home 😭 
**User:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4191824?comment=9883293
Sorry:( we announced the in-person midterm a couple announcements ago



### Answer 
**Name:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4191824?answer=9677963
Hi - yes, it was a take-home exam last year, but staff are discussing the possibility of making this an in-person exam versus a take-home exam due to alleged cases of academic dishonesty that occurred last year. If we do have an in-person/synchronous exam, it will be on April 11th. We will make a final decision on Monday, which will be released in the weekly announcements!
# Camera on Alice
**User:** Sandip Maharaj
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4197363
 Camera on Alice does not work. 
### Answer 
**Name:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4197363?answer=9677975
When you say it doesn't work, what have you tried so far? Sometimes it seems like it's not working but it actually is. But it could also be not working!
**User:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4197363?comment=9677994
Some common things that can be missed:

Running roslaunch step for the usb_cam

The AR tag not being in view (you can check on rviz)

Also a common fix could be restarting the computer lol, or unplugging the camera and replugging it/plugging it into a different port. 
### Answer 
**Name:** Katherine How
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4197363?answer=9681090
I tried using it yesterday and debugging with similar methods in 106A by using an online webcam tester to see if there was any camera feed available but the website was unable to detect a camera despite it being plugged in
**User:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4197363?comment=9684246
Did you try using different USB ports? Sometimes some of the ports don't like the cameras every once in a while.
**User:** Sandip Maharaj
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4197363?comment=9723163
Yes tried that. Tried plugging and unplugging. Tried different ports. AR tag was in view and we did run the roslaunch for the usb cam too. 
**User:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4197363?comment=9723185
Will get back to you, will send someone in to check it out today
**User:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4197363?comment=9723235
Actually, wait this happened to me before and I restarted the computer bc of some issues w the driver. Can you try that?
**User:** Cecilia Pham
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4197363?comment=9773974
hi is there any updates on the camera on Alice? It is also not working for us and we tried unplugging and replugging the camera into multiple ports.
**User:** Cecilia Pham
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4197363?comment=9774808
Fixed it, we restarted the computer!




### Answer 
**Name:** Jackson Hilton
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4197363?answer=9726173
Resolved via full power cycle (unplug & replug)
# Homework 2 Released!
**User:** Nima Rahmanian
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4197725
 Fresh out of the oven, ready for you to enjoy.
### Comment
**User:** Anish Dhanashekar
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4197725?comment=9675398
Thanks, Nima!
### Comment
**User:** Jiachen Lian
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4197725?comment=9675647
what is ddl
**User:** Nima Rahmanian
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4197725?comment=9675684
I'm not sure I know what you mean.
**User:** Jiachen Lian
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4197725?comment=9675694
sorry I mean when does it due , I mean deadline, sorry


**User:** Jiachen Lian
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4197725?comment=9675703
Well I just saw that it is in two weeks


**User:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4197725?comment=9678002
Yep deadline is 2/7!



### Comment
**User:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4197725?comment=9678007
Scrumptious 🥐 
### Comment
**User:** Jiachen Lian
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4197725?comment=9681219
For problem1.4, I feel like it is so straightforward, I have no idea what we should do, I don't even think the proof is required. Please correct me if I am wrong. 
**User:** Nima Rahmanian
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4197725?comment=9694516
It might be more straight forward than other problems, but make sure to write out your thoughts.
### Comment
**User:** Jiachen Lian
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4197725?comment=9681877
BTW, does 1.25's lecture cover all stuff 
### Comment
**User:** Max Wilcoxson
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4197725?comment=9685340
Are we allowed to use tools like Sympy and Matlab, if we show our code and explain how/when we used these tools (as allowed in homework 1)?
**User:** Nima Rahmanian
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4197725?comment=9694514
Sure.
### Comment
**User:** Max Wilcoxson
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4197725?comment=9687773
This is question 3.6. Is the shift from using the notation V(x) in the previous problem 3.5 equation 31 to V(t) in this section intentional? Does this mean we need to incorporate x(t) into our description of V?
**User:** Nima Rahmanian
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4197725?comment=9694527
Yes. Notice how the initial state is fixed in this problem. This means that we can simplify V(x, t) as V(t). The idea is all that really matters. I wouldn’t get too caught up in the notational nuances.
**User:** Max Wilcoxson
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4197725?comment=9697433
I thought that V was dependent on x and t, and that x also varied with time. Isn't this why when we calculate the derivative of V(x, t) with respect to the trajectory we need to sum the partial of V with respect to t and the partial of V with respect to x times the partial of x with respect to t (by chain rule)? (ie. x also depends on t, so fixed initial state doesn't allow us to treat x as a constant?) Thanks for the clarification!
**User:** Nima Rahmanian
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4197725?comment=9708821
These are great questions! Please keep asking away!



Everything you have said is correct. x is not a constant. However, in this question, since we have been given an initial condition, the value of x is implied by the value of t. For this reason, V only really needs to take the parameter t.



By writing V only as a function of t, it becomes more clear how to solve this problem. That is, it will help you find a function which upper bounds V(t). 


### Comment
**User:** Charles Paxson
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4197725?comment=9700256
When will this homework be due?
**User:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4197725?comment=9704055
February 7th!
### Comment
**User:** Japinder Narula
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4197725?comment=9756322
Are we supposed to tune gamma based on the animation? I put in a value to start with, got the animation, and noticed that the drone hovers/circles around the target pos a bit before aligning properly. Can we perhaps get a ball park range of what value to test (i.e what order of 10?)
### Comment
**User:** Kev Wang
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4197725?comment=9835613
Problem 3.2 asks for an explanation of the cost function $u^TQu$ of the CLF-QP controller. However the problem doesn't really talk about what this $Q$ is, other than the fact that it is PSD, so I'm not really sure what it means to optimize for this function, which depends on $Q$. Help would be appreciated!
**User:** Tarun Amarnath
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4197725?comment=9858711
It’s just asking for a couple sentences of intuition. Why is it important to have a term in the cost function with the input and a PSD matrix? What exactly would you be minimizing with this cost, and why?
### Comment
**User:** Alex Beaudin
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4197725?comment=9863587
Where are we supposed to submit the homework? I don't see a gradescope submission
**User:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4197725?comment=9864137
So sorry I just created it!!
**User:** Alex Beaudin
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4197725?comment=9864733
It only has three questions to select out of the five. :(
**User:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4197725?comment=9869369
Sorry about that, fixed!


### Comment
**User:** Trinity Chung
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4197725?comment=9903610
where do we download the files for the coding portion of the hw (non-jupyter notebook)?
**User:** Nima Rahmanian
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4197725?comment=9913004
Which question in particular?
**User:** Trinity Chung
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4197725?comment=9913029
theres only one problem with coding, question 4. its ok i already did it in the notebook. next time please provide the non notebook version :(
**User:** Nima Rahmanian
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4197725?comment=9913042
Sure thing.
**User:** Noah Adhikari
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4197725?comment=9930266
You can download the colab notebook locally if you prefer to do it in something like vscode, but not sure how to get the individual files


### Comment
**User:** William Molnar-Brock
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4197725?comment=9907869
In problem 3, it seems that there is an inconsistency in the dimension of u. 



I'm wondering if this is a typo (the decision variable should actually be within R^m) or if the u in (23) is different from the control input in (22)


**User:** Nima Rahmanian
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4197725?comment=9913049
Yes, that is a typo. The most general formulation should have u in Rm. That is, u need not be in the same vector space as x.
### Comment
**User:** Edward Chang
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4197725?comment=9908071
How do we submit the coding portion?

**User:** Trinity Chung
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4197725?comment=9908122
it says to attach the plots
### Comment
**User:** Matthew Sulistijowadi
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4197725?comment=9944287
for 5.6, after applying the hint, I'm not sure how to relate K to P since they have different dimensions. Or also, not sure how the form z_dot = Pz is helpful to solving the problem 
**User:** Daniel Tan
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4197725?comment=9947474
Try using results from previous parts.
# Journal Club
**User:** Morten Svendgard
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4198156
 Hi, I am a concurrent enrollment student that have not been assigned to any Journal Club. I was not added to ed at the time when the signup were done. If I understand it correctly, the Journal clubs are at the lab-times. Thursdays are the time that would work the best for me. Fridays are not ideal. Especially the last section on fridays would not work.



I am sorry that I am a bit late. As a concurrent enrollment student, putting together my schedule have been surprisingly difficult. Hope this won't impact my results / grade. I am motivated to do a very high effort from this point on.



Thank you!
### Answer 
**Name:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4198156?answer=9678099
Hi Morten, 

I'll go ahead and add you to my Thursday discussion (10-11 am). Don't worry about having missed the first one, it was pretty chill so you can just look over the slides that are linked on the website. Let me know if you have any questions!


**User:** Morten Svendgard
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4198156?comment=9680896
Hi Kirthi,

Thank you very much. Sounds great!
# Project 1 Extension
**User:** Katherine How
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4199966
 Hi! I’m so sorry but I just tested positive for Covid and I’m the only member of our lab group who took 106A and has experience with using and debugging the Sawyer robots. Would it be possible for our group to get an extension on the project if needed? so far we’ve completed the setup for the lab but I’m worried my group would have trouble debugging since I can’t be there in person. 
### Answer 
**Name:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4199966?answer=9684314
Hi Katherine. Sorry to hear that - I hope you feel better soon. I'll go ahead and give your group a 2-day extension. In the future though, you should use your slip days. You'll be able to apply 2 additional slip days on top of this extension, as well, but I'd discourage you from doing that because 1) you don't want to fall further behind for project 2, and 2) the other projects are much harder.

You can always do something like set up a Zoom and VS Code collaborative coding (that's what I did with my group when I got injured last year and couldn't walk to the lab). The only reason I'm suggesting doing this is if you start falling behind from the first project, it'll cause you to keep falling behind and ultimately cut into final project time.
# Lecture slides
**User:** Tianqi Zeng
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4204633
 Hi, when will the lecture slides and discussion for week 2 be posted? Thank you!
### Answer 
**Name:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4204633?answer=9704213
I'll post the lecture slides once the Prof sends them to me (should be soon, hopefully)! Tarun will post discussion stuff soon as well! 


# Homework 1 Solutions
**User:** Nima Rahmanian
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4212455
 Hi everyone. I have attached solutions for hw1. I would like to apologize for a mistake I made in writing question 4.2 (which also affected question 4.3), and any inconveniences associated with this. The question should have written "You may assume A is symmetric." If A is symmetric, then the magnitude of the eigenvalues equal the singular values. This line of reasoning is used in the proof. Due to this error, we will be giving everyone full credit for question 4.3.
### Comment
**User:** Sandip Maharaj
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4212455?comment=10289346
When will solution for homework 2 be posted?
# Week 3 Announcements!
**User:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4215237
 Woohoo another week! I spent my weekend isolating but I enjoyed the new Kali Uchis album, and catching up on some classics on Netflix:) Here's everything to know for the week~

Project

Project 1 is released & due this Friday (2/2)! Please fill out the project team form before you submit Project 1.

Announcing our project slip day policy 🎊 - You will have 5 slip days (independent from homework) to be used for projects, only 2 max can be applied per project. Read the policies section of the website for more.

Journal Club

Use this spreadsheet to sign up for the two papers you present at journal club. You will be working with one other person to present the paper (generally a 10-15 min presentation, with 5-10 mins of Q&A/discussion). Please choose the dates for which you are certain you'll have no conflicts, and be sure to reach out to your pairs early! 

Please don't overwrite anyone's name. It doesn't matter whether you're email 1 or email 2.  All of these papers are cool & interesting! 

You should sign up for one paper presentation on or before the 3/13-3/15 set of journal clubs and one on or after (it's shown in the spreadsheet).

There is also an option to present this week, which if you choose, will be a great learning opportunity for everyone involved plus we'll be very easy with our grading of the presentation.

If you have a conflict with your journal club, please make a private post on Ed. Even if I said we couldn't switch you before, we have some wiggle room now so please let me know ASAP (before this next round of journal clubs) so I can move you.

If you miss a journal club, you'll have an option to make it up by 1) signing up for an additional paper presentation (if there are any remaining slots available), or 2) sending your Journal Club TA a paragraph summarizing the paper(s) that you missed discussing. 

Other

This is the discord link created by Tarun for the class, but don't expect to see us on there often (or me at all lol, I forgot my discord password), it's mainly for all of you to be able to discuss.

Last year's TA, Max, created these super helpful notes for the course, highly recommend referencing them! They're the course notes linked on the website.

Homework 2 is released and the deadline is 2/7. Also, homework 1 solutions have been released and so have lecture slides from last week.

Update about our midterm 🚀 - we've decided to change the midterm to in-person on April 11th. If you have a conflict with this date, please make a private post as soon as possible and we will try to make some accommodations. More logistics will be released closer to the midterm, but if you have any questions regarding the switch from take-home to in-person, please ask Prof. Sastry in OH! 

Action Items

Fill out the project team form before you submit Project 1

Sign up for your journal club papers ASAP!

Be sure to check the calendar for office hours - they may shift around from week to week in case staff's schedules change (or are sick, like me, and need to cancel OH)
### Comment
**User:** Matthew Sulistijowadi
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4215237?comment=9720867
using berkeley email but journal club signup still says need perms


**User:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4215237?comment=9720877
Just fixed, thanks for the catch!
**User:** Matthew Sulistijowadi
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4215237?comment=9721235
thanks! 



### Comment
**User:** Tarun Amarnath
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4215237?comment=9720888
I have never heard of Kali Uchi
**User:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4215237?comment=9725969
Don't worry once I'm out of isolation I'll play a few of her songs for you
### Comment
**User:** Jiachen Lian
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4215237?comment=9722402
Hey just to confirm we only sign up for 2 papers this semester right? Also I choose one paper on 3/15 and one paper after. Is that also ok?


**User:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4215237?comment=9722968
Yes - you sign up for 2 papers for the semester. 1 on or before 3/15 and one on or after 3/15. 
### Comment
**User:** Kev Wang
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4215237?comment=9722729
The first paper is for 1/24-1/26, but the journal clubs for that week have already met. When will presenters for that paper present?
**User:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4215237?comment=9722989
The first paper is 1/31-2/2 (this week). Please check the dates carefully on the left side! 
**User:** Kev Wang
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4215237?comment=9724088
Oh I see. Sorry I was confused about the formatting.
**User:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4215237?comment=9725992
No worries it can be a little confusing! Got me panicked there for a second bc I did all the dates super late last night




### Comment
**User:** Zekai Wang
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4215237?comment=9735225
Are we going to present the same paper twice? 
**User:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4215237?comment=9779166
It's fixed now! That date still has a perception paper, just a different one. Thanks for the catch!
### Comment
**User:** Anish Dhanashekar
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4215237?comment=9754064
Is there an explicit format/guidelines for the journal club presentation?
**User:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4215237?comment=9758352
Spend the time summarizing the key points paper (make sure you get to the relevant specifics). Emphasize how aspects of the paper connect to class concepts.

Most importantly: include a couple of discussion questions at the end. The way my partner and I made the presentation for journal club last year was by starting with each of the headings and allocating a couple of slides for each.

This was one of my presentations from last year for reference. 


**User:** Anish Dhanashekar
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4215237?comment=9758455
Thanks! Presenting tomorrow (technically today lol) and am wrapping up the presentation now - thanks for the advice to add discussion questions at the end!

# Homework Solutions Megathread
**User:** Tarun Amarnath
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4215257
 Homework 1: #49

Homework 2: #119 

Homework 3: #124 

Homework 4: #206


### Comment
**User:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4215257?comment=11010587
Homework 3 Grade Distribution 
### Comment
**User:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4215257?comment=11030853

### Comment
**User:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4215257?comment=11043130

### Comment
**User:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4215257?comment=11043139

### Comment
**User:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4215257?comment=11109592

**User:** Max Wilcoxson
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4215257?comment=11139489
The distribution seems cut off -- would it be possible to see the statistics? Thanks!
### Comment
**User:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4215257?comment=11134200

### Comment
**User:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4215257?comment=11147652

# Cool talk today!
**User:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4215431
 Please join us today Monday, January 29, at 4:00 PM for a DREAMS seminar by Dr. Raphael Zufferey who is visiting us from EPFL, Switzerland. Details below.

Title: Flying Robots: Exploring Hybrid Locomotion and Physical Interaction

Speaker: Dr. Raphael Zufferey

Location: 540AB Cory Hall. A link to the webcast will be provided before the seminar.

Time: 4-5 PM PT, Monday, January 29, 2024

Abstract:

Autonomous flying robots have become widespread in recent years, yet their capability to interact with the environment remains limited. Moving in multiple fluids is one of the great challenges of mobile robotics, and carries great potential for application in biological and environmental studies. In particular, hybrid locomotion provides the means to cross large distances and obstacles or even change from one body of water to another thanks to flight. At the same time, they are capable of operating underwater, collecting samples, video and aquatic metrics. However, the challenges of operating in both air and water are complex. In this talk, we will introduce these challenges and cover several research solutions which aim to address these in different modalities, depending on locomotion and objectives. Bio-inspiration plays a crucial role in these solutions, and the topic of flapping flight in the context of physical interaction will also be presented.

Bio:

Dr. Raphael Zufferey received his PhD in Aeronautics from Imperial College London, UK in 2020. He is currently a Marie Curie postdoctoral fellow at EPFL, Switzerland. His main research interests are in novel, integrated robotic systems that aim to address difficult locomotion and interaction tasks, covering questions relating to fluid dynamics, mechanical design, embedded electronics and control. In particular, hybridizing flight in small-scale robots has been one of his core topics of investigation. Raphael was also a researcher at Harvard and University of Seville. He was awarded the best PhD in Robotics UK award for his accomplishments.
# Lyupanov supplemental
**User:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4215634
 Would highly recommend checking this out!
### Comment
**User:** Jiachen Lian
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4215634?comment=9722292
Thanks. Although I remember Shankar said you don't need to grasp the proof part lol.
# The number of member of project1
**User:** Yifeng Zhang
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4216851
 May I ask whether the number of people in project 1 must be three or two
### Answer 
**Name:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4216851?answer=9725932
3 people
### Answer 
**Name:** Yifeng Zhang
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4216851?answer=9726064
Thank you！
# Teammate Matching
**User:** Johnny Lau
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4218135
 Hi, I have reached out to six different groups/people with the unresolved mark in the teammate matching thread several days ago, however, I don't think I have gotten any reply at all, and I know project 1 is due this Friday. What should I do in this situation as I assume "everyone" already has a group at this point?


### Answer 
**Name:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4218135?answer=9730423
If you can temporarily find a group of 3 to join as their fourth, please go ahead and do that, so that you don't fall behind in the course. It could also be a good idea to make a public post asking if anyone is still looking for a group at this point. 

Even if you can find one other person to work with, I think that would be a good start.


# Running late to OH
**User:** Karim El-Refai
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4218552
 Going to be slightly late to my OH today sorry for the delay. I’ll be staying longer after proj/hw party to make up for it.
# Friday discussion is still happening this week
**User:** Nima Rahmanian
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4218833
 Please discard my previous message. I will be running discussion on both Thursday and Friday this week. Looking forward to meeting you all.


# Proj 1 ImageTK import error
**User:** Sergio Peterson
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4224675
 Hi, 

We are having trouble running Project 1, It is giving us an error stating how it cannot import ImageTK from PIL when running main.py. We tried moving computers, pip installing PIL, we are stuck and don't know how to continue. We talked to Michael during OH and he suggest moving computers but it seems that we still keep running into this problem. Thank you. 
### Answer 
**Name:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4224675?answer=9779121
It's kind of hard to debug this without some pictures of what's going on. One fix is to restart the computer because it might be a driver issue. But let me know if you keep hitting this issue
# Project 1 PDJointTorqueController
**User:** Trinity Chung
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4229565
 I believe I am correctly calculating the torque based on this comment about the output coriolis matrix and the given formula in the textbook/docstring, but when running my torque controller on line trajectory (line trajectory has been tested with another controller) the robot moves down and turn?s the wrist (such that it faces toward the computer). Any hints on what might be going on
### Answer 
**Name:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4229565?answer=9779116
Hey Trinity! I know I'm late to this so I could take a closer look at your code but it will probably be way easier if you use the workspace controller tbh. Have you been able to make progress on that one?


**User:** Trinity Chung
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4229565?comment=9779174
I got ar tracking complete with PDJointVelocity controller. It would still be great if I could figure out what I'm doing wrong here.
**User:** Trinity Chung
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4229565?comment=9779185
tbh I'm a little disappointed in the hand-waviness of the support for projects. I hope next project is more thoroughly debugged
**User:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4229565?comment=9803472
Hey - yeah, I understand where you’re coming from. For what it’s worth, 106A and 106B are fundamentally different courses. It would be impossible for us to provide the same level of support as 106A, which is an introductory course where labs are more fill in the blank. 106B is a lot more open ended, and it’s definitely going to be more of a challenge. I’m sorry that you feel like we’re hand waving but there’s only so much support we can give before we hand out solutions line by line like in 106A. We also have a much smaller staff on 106B, so it’s harder for us to provide immediate support, so I’m really sorry we weren’t able to respond to you sooner. 

**User:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4229565?comment=9803604
Has your code changed at all from what’s above? Here’s what I noticed (also this is for the JointTorqueController, not JointVelocity):

T = M @ ddq + C + G - Kp @ e - Kv @ e (you might have some dimension errors so make sure they're aligned) 
**User:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4229565?comment=9803609
I think you're currently multiplying M with everything in the parenthesis, but you should only have to multiply it with ddq if that makes sense


# Journal Club Time Change
**User:** Vishnu Murali
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4233331
 Hi Course Staff,

I am currently in the Thursday 10am Journal Club with Kirthi. I had a bit of churn in my schedule and now have another class at the same time. Is there any chance I can move to the Friday slot?

Thank you,

Vishnu


### Answer 
**Name:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4233331?answer=9779107
Gotcha - you're switched! 
# Project 1 Import Error
**User:** Matthew Sulistijowadi
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4236270
 We encountered this error on both Monday and Wednesday (today), we asked Michael a couple times and he tried to help tho the software remained stubborn and he mentioned he'd bring it up with the rest of staff. Does anyone (TA or students) have any ideas on how to resolve it? 
### Answer 
**Name:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4236270?answer=9779150
What if you just.... deleted that line lol. Not sure what lines are dep on it & very very strange cuz I didn't hit this error while devving


# Alan Grippers Broken
**User:** Eddie Shi
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4236672
 My project group broke both gripper arms on Alan. It happened suddenly and the gripper arms snapped when hitting the table in front of the robot where the AR tag sits when we were testing our project 1 code.
### Comment
**User:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4236672?comment=9779130
Aw man:( Be careful and don't hesitate to use the E-Stop in the future. This lab doesn't really need grippers so we should be okay, hopefully we can get them replaced over the next few days.
### Comment
**User:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4236672?comment=9779153
Also wanted to say thank you for owning up lol!! allows us to get maintenance done sooner
# How does auditing work?
**User:** Pranit Panda
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4240030
 Wanted to ask a question about how auditing for this class works. I'm guessing we will still be able to access content on bcourses and the course website. Was wondering if we could also use the robots when they are not being used by enrolled students.
### Answer 
**Name:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4240030?answer=9803650
Hi! You can still use the robots when they're not being used by enrolled students (ie when there are no reservations)! However, please don't make any reservations as enrolled students need priority to access the robots.
# Project 1 (Haven't found any partners)
**User:** Antony Zhao
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4244220
 Is there any way I can work on this initial project alone? I've been asking around and haven't found anyone to work on it with (one person didn't plan on working on it because they were busy, and the other was dropping the course). I can keep looking in the meantime, though.
### Comment
**User:** Antony Zhao
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4244220?comment=9789051
Or can I have a friend join me from outside the course while I work on it? I know the usage guide says not to do it alone, though this would mean that the submission would still be a solo member.
### Answer 
**Name:** Antony Zhao
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4244220?answer=9797984
Just found a group nevermind
**User:** Antony Zhao
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4244220?comment=9803182
Double never mind they apparently got denied from taking the course?
**User:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4244220?comment=9803666
Really? Do you know why?
**User:** Antony Zhao
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4244220?comment=9803826
I think they’re here through a study abroad program? So maybe that’s why
**User:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4244220?comment=9803983
Oh huh I see



### Answer 
**Name:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4244220?answer=9803681
In any case, you can work on it alone this time. Or join a group as a group of 4 if can! Or if you can find just one other person at least that would be good.
**User:** Antony Zhao
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4244220?comment=9803847
Is there any chance I can get an extended deadline? I can provide screenshots of the emails if needed. I’m fine with using slip days otherwise but just checking.
**User:** Antony Zhao
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4244220?comment=9803881
Or I can forward the conversations I just remembered that would probably be better (if needed).
**User:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4244220?comment=9803991
You don't need to send me the emails lol. I can give you a 2-day extension.
**User:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4244220?comment=9804000
You can apply slip days on top of the extension if needed
**User:** Antony Zhao
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4244220?comment=9805168
Okay, thank you so much. Also hope you're feeling better.




# Question on Discussion 2
**User:** Cecilia Pham
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4248522
 Hi, I had a question about the solution to part 4 problem 2 of discussion 2. In the definition of an energy like function v(X) that is locally positive definite is 1) that v(0,t) = 0 for all t and 2) v(x,t) is greater than or equal to a different function alpha based on the magnitude of x that is strictly increasing and continuous. In the solution I don't see where condition 2 is being met or why we are proving that v >= 0 since 0 is not a function that is strictly increasing and based on the magnitude of x. On an earlier example, we proved LPD by proving that v(x,t) was greater than a function e.v min * magnitude of x^2 but I don't see where this applies to this solution. Would someone be able to explain why we are using 0 as our alpha function here? Thank you!
### Answer 
**Name:** James Ni
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4248522?answer=9803620
I also thought that simply proving $V >= 0$ in a neighborhood was insufficient. But the claim is still true since you can use Taylor's theorem to get $$V(\theta, \dot{\theta})) = \frac{1}{2} \dot{\theta}^2 + a(\frac{1}{2} \theta^2 + O(\theta^4)) \geq \min(\frac{1}{2}, a) (\theta^2 + \dot{\theta}^2) + O(\theta)^4.$$ This implies the existence of some $\alpha$ locally.
### Answer 
**Name:** Tarun Amarnath
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4248522?answer=9845536
Ah yeah so I think I explained this but didn't write it down in the boardwork. The quadratic term will be larger than the linear term and therefore can be used as the alpha function. 
# ABout Lab Team
**User:** Jiachen Lian
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4251356
 Hey. We already formed a team of three. However, we feel like one member of our team has zero experience for this course. For example, he does not know python, github, ros, and everything. Every time he just sit down and does not even speak during the entire lab session.  Can we only have a team of 2 or can we replace him with somebody else?
### Answer 
**Name:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4251356?answer=9805695
Hi Jiachen, sure. For this project, do you want to add an extra person to your group? And then for the next lab you could tell them to find another group I guess. I’d suggest reaching out to ayzhao7761@berkeley.edu who messaged us about still not finding a group yet


**User:** Jiachen Lian
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4251356?comment=9805772
Thanks. I will pin my another active teammate for this and will reply to you soon
**User:** Jiachen Lian
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4251356?comment=9806769
I just emailed him to ask if he has taken 106a or experience on ROS. If yes, we can definitely take him in.
**User:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4251356?comment=9811815
Okay! Feel free to coordinate with him directly and make any adjustments to your team as needed.

# Submitting Project 1 Code
**User:** Karim El-Refai
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4251646
 Please share your github repo with eecsbioeme-106@berkeley.edu 

Do NOT make your repo public. If your codebase is not already on a private github repo please move it over as this is the only format where we'll be accepting code.

In the future we'll be moving to github classroom to streamline the process. 


# Journal Club Presentation Information
**User:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4251792
 Hi everyone! By this point, everyone should’ve signed up to present two papers in journal club (if not, please sign up here). Here’s some additional information to help you with your journal club presentations:

Spend the time summarizing the key points paper (make sure you get to the relevant specifics). One way to do this is by starting with each of the headings and allocating a couple of slides for each. The presentation should last 15-20 minutes and leave 5-10 minutes for discussion.

Emphasize how aspects of the paper connect to class concepts, as well as the results of the paper.

Most importantly: include a couple of discussion questions at the end.

Submission: Please submit the presentation to Gradescope via a share link and add your partner to the submission.

This was one of my presentations from last year for reference.

Finally, please be sure to fill out the attendance form each time! If you have to miss a journal club, please let your TA know via email. There are two methods to make up journal club attendance:

Sign up for an additional paper presentation (if there are any remaining slots available in your section)

Send your Journal Club TA a paragraph each as a summary of the paper(s) that you missed discussing
# Possible extension for project 1
**User:** Zekai Wang
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4252166
 I believe I am experiencing medical conditions that makes it hard for me to come to lab in person or work on the projects. I understand there are slip days and it’s a group project, but I wonder if it is possible to submit the report more than two days after the due. If so, what kind of documents should I present and how can I proceed. Thank you very much!
### Answer 
**Name:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4252166?answer=9811804
Hi Zekai, I just responded to your email, let's continue the conversation there!
# Slips day question
**User:** Thien An Phan
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4254219
 Out group is planning on using slip days for the project and was wondering if we have to do anything or does it just automatically applies? 
Thanks


### Answer 
**Name:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4254219?answer=9811819
It'll automatically apply:)
# Possible extension of Lab1
**User:** Jiachen Lian
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4254505
 Dear Instructors:

Is it possible to slightly extend Lab 1? We feel that it is taking more time than we initially anticipated, and one week doesn't seem to be enough for us. Additionally, our team is still limited to only 2 members, and it appears that other teams are also facing challenges with the project. We are uncertain if a slight extension might be considered.

Thanks!
### Answer 
**Name:** Thien An Phan
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4254505?answer=9812100
I agree, a lot of our team members are still figuring out their schedules out so there were a lot of schedule clash  in addition to this first project being a lot harder than we expected as well.


### Answer 
**Name:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4254505?answer=9848430
Hey everyone, sorry we were not able to extend the project. If you have extenuating circumstances as an individual or as a group, please let us know via a private post on Ed and we can extend the project on a case-by-case basis. Most groups were able to use slip days -- the future projects should be a little less tighter with the deadline and we will increase the allowed booking time to 3 hours rather than 2 hours.
# Switching Teams for Projects
**User:** Kai Xu
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4256358
 Hi! Are we allowed to switch teams across projects? I would like to get to know and collaborate with different people across projects. Thank you so much!
### Answer 
**Name:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4256358?answer=9848435
Sure:)
# Problems on Programming in HW2 Problem4.3 - 4
**User:** Enyang Zou
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4260411
 I use @ for the multiplication in calculating Vdot, but this error keeps coming.

ValueError: matmul: Input operand 0 does not have enough dimensions (has 0, gufunc core with signature (n?,k),(k,m?)->(n?,m?) requires 1)

And I traced back, finding this is because that the evaluation of Vdot is called by a function called (eval_force_vec) under controller. 

The function eval_LyapunovDerivs takes in argument (self,f,t), where f is input of the system.

and in eval_force_vec it is like this:

#set up optimization problem

 opti = ca.Opti()

 #set up optimization variables

 u = opti.variable(3, 1) #the input here is a force vector only! Will be 3D.

 #get the CLF value and its derivative

 V = self.lyapunov.evalLyapunov(t)

 VDot = self.lyapunov.evalLyapunovDerivs(u, t) #pass the input into the function



So, I found that it is passing a casadi variable u to evalLyapunovDerivs. And this variable is to be optimized to find the value, so it cannot be calculated by numpy expressions.

I wonder how should I deal with this problem? 


### Answer 
**Name:** Encheng Liu
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4260411?answer=9894332
Shape of f: (3, 1)
Shape of m: ()
Shape of aD: (3, 1)
Shape of eV: (3, 1)
Shape of epsilon: ()
Shape of eX: (3, 1)
Shape of alpha: ()
Shape of (f/m - aD).T: (1, 3)
Shape of (m * eV + epsilon * eX): (3, 1)
Shape of alpha * eV.T @ eX: (1, 1)
Shape of epsilon * eV.T @ eV: (1, 1)
((opti16_x_1/0.92)-[-0, 0.548311, 0.548311])

I print out all necessary shape for this expression. You must use @ to calculate  Vector vs Matrix or Matrix vs Matrix, It is equivalent to dot(), and you must also use * to do any multiply on a scalar with a matrix


**User:** Enyang Zou
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4260411?comment=9894386
Oh, thx, seems the problem is solved
# A potential petition for project1/lab1
**User:** Jiachen Lian
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4260578
 Hey instructors. Our team was officially formed yesterday, which was the deadline for project1. The reason for this delay is that we were initially part of different teams, but most of members dropped the class as late as yesterday. Consequently, we have just come together and are essentially restarting the project. We have finished it now and plan to make our submission today.

In light of this unique situation, we would like to inquire whether we can be granted an exception to not use a slip day for this project.
### Answer 
**Name:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4260578?answer=9848451
Hi Jiachen, sure, I know you had mentioned your team issues in a prior post so I will grant your team an extension until tonight. You may use slip days on top of this if necessary.


# Question regarding project 1 slip day
**User:** Thien An Phan
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4262071
 Hi, 

Me and my group just submitted the project paper at 12:00 and was wondering if that counts towards us using 2 slip days? 

Thank you.
### Answer 
**Name:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4262071?answer=9848475
Hey! Gradescope will show it's 2 days late but we'll manually correct it when we calculate slip days -- so feel free to consider it as only 1 slip day used:)
# URGENT: Wrong Video link in Proj 1 Report
**User:** Kai Xu
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4266438
 Hi! We submitted our Project 1 report, but we realized today that there is an error in the Youtube link we provided. I was wondering if there is a way for us to specify the right youtube link without expending the slip days. Thank you so much for your help and time! Have a great day!


### Answer 
**Name:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4266438?answer=9848489
Hi Kai! Yeah, feel free to drop the correct link here, don't worry about using additional slip days:)
**User:** Kai Xu
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4266438?comment=9872349
Thank you so much Kirthi! I think one of my teammates emailed you yesterday but here's the link again just in case: https://www.youtube.com/watch?v=oS_vK1OPlIA&ab_channel=KaiXu Thank you so much! 
**User:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4266438?comment=9872352
Gotcha! Thank you

# Cancelling My OH 5-6 Today
**User:** Karim El-Refai
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4276255
 Have a pretty bad headache right now so I'll be canceling my OH today. HW party will still be running 6-8 so make sure to come by then!
# Alan Camera not working
**User:** Kai Xu
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4276945
 Hi, in case no one posted about it. Alan's usb camera hasn't been working since Saturday. The error message was something relating to not being able to find /dev/video0. I just remembered it and thought I should post about it. Thanks a lot!
### Answer 
**Name:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4276945?answer=9869158
When this happens, you have to restart the computer. I will tell ISG about this too.
# Week 4 Announcements!
**User:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4278569
 Helloooo everyone!! Another week of 106b 😅 last week was a tough one so hopefully we can go strong on this next one 💪

Projects

Congrats on completing project 1!!! 🥂 It was challenging so great job on making it through!

Project 2 will be released tomorrow. Please be sure to keep an eye out for it and get started early!! As you all just experienced, projects in this class take a lot more time and effort than usually anticipated.

Also - we are updating the robot booking policy to allow people to book robots for 3 hours at a time instead of 2 effective at midnight tonight. The trek to Cory is no joke ⛰️

Project team form, if you want to switch teams for project 2 (or any future project): https://forms.gle/8WWhMEJDXxrb2PZ68

Journal Club Information

This thread has important info

Sign up for your papers BY TONIGHT! I will be emailing anyone who hasn’t. 🫵

Other

Please put [106B] or [206B] in the email subject if you ever email us about anything. Otherwise, we could miss it😟

Homework 2 is due on Wednesday, and Homework 3 will be released on Thursday. Keep an eye out for it

If you’re a concurrent enrollment student who has gotten into this class, please let me know as soon as possible if you aren’t able to make your assigned journal club section or if you aren't assigned to a section. If you haven’t gotten into the class yet, it is unlikely at this point.

We will be giving out extra credit at random for attending lecture (and trust me, it'll be worth it in the end). I’d be there tomorrow (and going forward) if I were you🤌

Please come to office hours to get the support you need for this course!

Anyways, I read this article in WSJ about moody robots in manufacturing and was entertained.
### Comment
**User:** Anish Dhanashekar
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4278569?comment=9870562
I wish I could attend lectures in person, but I have another class at the exact same time with mandatory attendance :,(

Will there be any other ways to earn extra credit for those with time conflicts?
**User:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4278569?comment=9872136
Unfortunately, not lecture attendance extra credit specifically:( But of course there will always be credit to be earned from EPA!


### Comment
**User:** Angel Aldaco
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4278569?comment=9871764
Major Dub for the 3 hours instead of 2 hours! Thank you so much staff I and others are forever grateful
### Comment
**User:** Matthew Sulistijowadi
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4278569?comment=9949454
since 19th is holiday prsident day?) would OH's and HW party be rescheduled? (a lotta them that day XD) 
**User:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4278569?comment=9951355
Yes - we'll reschedule them! Thanks for the catch:)
### Comment
**User:** Leo Huang
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4278569?comment=9949757
Would it be possible for the journal club slides to be posted? (especially for Project 2 Intro, they were really helpful!)
**User:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4278569?comment=9951353
Journal club slides will be posted shortly!
# Project 2 Released!
**User:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4288480
 Hello everyone!!

Hope you had a great Tuesday:) Project 2 is out & due February 23rd!

Assignment: https://ucb-ee106.github.io/106b-sp24site/assets/proj/proj2.pdf 

Link to repo w starter code: https://github.com/ucb-ee106/106b-sp24-labs-starter 

You need to pull the package out into its own folder, and then pull in a couple other repos. 

Project 2 is notoriously hard so get help early & please help each other out too! The Gradescope assignment to submit will be released soon.

For those who are Cory-averse, this project is fully in simulation, so you could set it up on your personal computer, and this should take some of the booking load off since you'll be able to use both sides of the computers (Sawyer & Turtlebot). Refer to the website for the links to book the workstations. 

If you ever need to switch up your teams, please do so with the team signup form: https://forms.gle/jqJBsYQYCsc8Cim88 
### Comment
**User:** Anish Dhanashekar
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4288480?comment=9953148
Can we ssh into the lab machines? If so, how?
**User:** James Ni
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4288480?comment=9957669
ssh ee106b-###@c105-##.eecs.berkeley.edu

+ GlobalProtect
**User:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4288480?comment=9979256
ISG/ESG does not want people to ssh into the lab machines. Please avoid doing this!

### Comment
**User:** Max Wilcoxson
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4288480?comment=10011378
For those with a MacBook looking to setup ROS on your laptop, I would recommend doing a 14 day free trial of Parallels Desktop.

Once you install Parallels desktop, then download and install Ubuntu 20.04 desktop x86 (if you have an Intel Mac) or Ubuntu 20.04 Server Arm (select ARMv8 build) (if you have an Apple Silicon Mac). 

Note that the Arm build is Ubuntu server, so after installing OS on VM you will need to install a desktop environment. 

Finally, follow the official installation instructions for ROS Noetic. 

You will also need to install packages listed in the project document. I also encountered a bug where /usr/bin/python does not exist, since Ubuntu now saves Python to /usr/bin/python3. You can install python-is-python3 to fix this. 

Hope this is helpful to someone!
**User:** Karim El-Refai
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4288480?comment=10074576
Additionally, if you want a permanent free alternative I've found UTM to be pretty decent though I haven't tried running STDR on it.


**User:** Trinity Chung
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4288480?comment=10090545
A better alternative would be to run it in a Docker container, it's faster and using x11 you can get the gui windows to appear natively :)
**User:** Kev Wang
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4288480?comment=10132584
How do you get GUI with Docker containers?


### Comment
**User:** Karim El-Refai
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4288480?comment=10078938
For anyone having questions about steering with sinusoids I highly recommend this doc written by a former 106B TA on what's going on. Note that it covers in detail a lot of dealing with singularities which is not required for this project


**User:** Kev Wang
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4288480?comment=10132566
What details on singularities is not required?
**User:** Karim El-Refai
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4288480?comment=10133331
Sorry my wording was imprecise. Everything it covers is necessary/helpful if you do want to deal with singularities. But dealing with singularities is not required for this project as many students in the past have had issues dealing with it. 
**User:** Kev Wang
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4288480?comment=10145046
Ah good to know. Thanks for clarifying!


### Comment
**User:** Anish Dhanashekar
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4288480?comment=10229755
How does one screen record on the lab computers?
**User:** Max Wilcoxson
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4288480?comment=10239825
This might be a helpful reference: https://help.ubuntu.com/stable/ubuntu-help/screen-shot-record.html. They mention Shift+Ctrl+Alt+R as a keyboard shortcut to start screen recording. Hope this helps!
### Comment
**User:** Edward Lee
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4288480?comment=10269416
Wanted to check -- for the point turn, we have to pass $\theta = 90^\circ$, so we will always hit the singularity for the sinusoidal planner?


**User:** Karim El-Refai
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4288480?comment=10277053
Yes
### Comment
**User:** Thien An Phan
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4288480?comment=10277602
Are we still sharing our private repo with the eecs-biome email? Or is there a Git Classroom up yet? Thanks
**User:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4288480?comment=10287153
No github classroom for this project unfortunately, share the priv repo with the email as it says in the project instructions! 


### Comment
**User:** Noah Adhikari
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4288480?comment=10279310
Hello,

I accidentally implemented collision avoidance for the sinusoidal planner. Would this warrant some extra credit? I spent many an hour trying to figure it out oops.

Thank you 106B staff!


**User:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4288480?comment=10287138
Hey, super cool that you implemented that. However, it wouldn't be equitable for everyone who didn't know about this opportunity if you got extra credit for doing this, since we made it pretty clear that no sinusoidal stuff was necessary. But def explain what you did in your report! I'm sorry it took so much time!


### Comment
**User:** Colin K.
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4288480?comment=10284480
What are the input limits for our sinusoid planner? In particular, the bounds for a1/a2 to our sinusoidal planner.

Right now, if we exceed the "recommended bounds", our sinusoidal planner works. Is this okay?


**User:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4288480?comment=10287143
Yeah that's fine just explain what's going on/why you think that is in your report
### Comment
**User:** Kalie Ching
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4288480?comment=10286494
would it be acceptable if we didn't screen record and took videos of our screen instead?
**User:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4288480?comment=10287108
Yeah that's fine
# Journal Club 11-12am Wednesday Cancelled
**User:** Karim El-Refai
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4288630
 Not feeling well currently so I will have to cancel my journal club for today. Highly encourage you all to visit other sections though as they'll give you a rundown of what project 2 is like and cover the paper for the week!
# trans not defined in scope error (project 1)
**User:** Rani Jyoti
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4309731
     tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    try:
        trans = tfBuffer.lookup_transform('base', 'right_hand_gripper', rospy.Time(0), rospy.Duration(10.0))
    except Exception as e:
        print(e)

    current_position = np.array([getattr(trans.transform.translation, dim) for dim in ('x', 'y', 'z')])
    print("Current Position:", current_position)


This is from the starter code, but getting this error: 

NameError: free variable 'trans' referenced before assignment in enclosing scope

Anyone else run into / resolve the issue, tips appreciated? 
### Answer 
**Name:** Daniel Tan
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4309731?answer=9946386
Might be the camera not seeing the assigned AR tag.
# Journal Club Excuse Request
**User:** Justin Im
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4309780
 Hello!

I have a mandatory event for another class that overlaps this week's journal club - attached is a confirmation from the pertinent class's professor. Is it possible to have it excused?

Thank you!
### Answer 
**Name:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4309780?answer=9951343
Sure!
# 5.6
**User:** Matthew Sulistijowadi
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4310726
 (Private as probs too much progress to post publicly?) 

After doing a bunch of stuff I got a P matrix that contains K, tho not sure what to do from here, admittedly don’t wanna do the work of finding E.Val’s for a 4x4 matrix if not necessary 😅 got a tip about splitting the 4x4 into 4 2x2 matrices tho not sure how relevant that is. Or if I’m done, not sure how I showed v = -Kz was valid (unless it valid due to being able to insert it into P?
### Answer 
**Name:** Tarun Amarnath
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4310726?answer=10005010
You should make sure your final matrix describing your differential equation is fully controlled by K. To do this, you can check to see if the characteristic polynomial is controlled by K.
# Extension for HW2
**User:** Angel Aldaco
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4316389
 Hi! 



  I was working on finishing HW2 tonight in MLK but ran and left my stuff except my laptop (including my phone, keys, etc.) at MLK when I saw the shooter a few meters nearby. I couldn't get my stuff back until now cause of proper procedure for the police, and MLK being closed. I just got my stuff back right now after a long night of not being able to go home due not having any keys and was wondering if I could get an extension on the homework as otherwise I would have no submission for the homework. I can provide screenshots collaborating this from my work slack including some poorly aged messages



Best, 

Angel
### Answer 
**Name:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4316389?answer=9979248
Resolved over email!


# HW 3
**User:** Harshika Jalan
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4331899
 Hi, is homework 3 not out yet? The website says it's due next week. Also will we have homework party today?
### Answer 
**Name:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4331899?answer=10003437
HW 3 is not out yet but will be released shortly. There was HW party today, the calendar is always up to date!


# has hw3 been released yet?
**User:** Matthew Sulistijowadi
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4331922
 has hw3 been released yet? website says it would've been released on 2/8 but atm don't see it. Just concerned if we would have the same due date of 2/21. Thank you! 


### Answer 
**Name:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4331922?answer=10003442
Homework 3 will be released shortly!
# Robot Usage Quiz Not Done
**User:** Erick Brito
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4333681
 Hello, I thought that if we had already taken 106A the Robot usage quiz was also optional like the assessment. In a lab worksession a partner reminded that this was not the case, and I am wondering if I will be docked points or if I could make this assignment up somehow? I did do the quiz for 106a so I do not believe I was unsafe with the robots at all. Thank you


### Answer 
**Name:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4333681?answer=10003469
Hi Erick! You won't be docked any points but please still complete the quiz. ISG/ESG really want everyone to be refreshed on it. I just extended the assignment a bit so you can submit late.
**User:** Erick Brito
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4333681?comment=10008487
Thank you!
# Week 5 Announcements!
**User:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4335138
 Hey hey everyone! Another week of 106B:)

Project 2 is released! Come to OH/Project Party with any questions.

Homework 3 will be released shortly! Unfortunately, there will be no "80% rule" for the homeworks. It's the main way to understand what's going on and the best way to do well in the homework is to come to office hours. Please come out to the discussions + OH + HW Party! They've been a lil... empty? 🤔 

President's Day is the 19th (next Monday) so all OH/Parties will be cancelled that day. Instead, we'll spread out those office hours through the rest of the week so make sure you come out (they will be updated for next week soon). As a result of the late release and moved OH, we will extend the homework 3 deadline to 2/26. Should be plenty of time!

Extra Credit

If there's a slot marked as red in the journal club sign-up sheet, if you sign up to present the paper that week, you'll get extra credit (or credit back for missing a journal club section)! Sign up so it's not just us yapping.

Lecture attendance extra credit is soo worth it I promise🔥 -- make sure you come out this week as we go over nonholonomic control & more! 

EPA - in general we really notice the people who are helping other people out in the course:) A great way to test your understanding of what's going on is by explaining something to someone else, and a great way to make a friend is struggling through a proof together! We've all been there!

The website will be updated soon! My bed called, and it said I should call it a night:)

P.S. Here's the least heinous Valentine's Day pun I could acquire! The funniest pun in the thread gets some extra EPA:)
### Comment
**User:** Bill Zheng
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4335138?comment=10003955
Can we sign up for red slots even if we're not in the same journal club section?
**User:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4335138?comment=10003963
Yeah go ahead! But you just have to be able to make it to that time to present.
### Comment
**User:** Daniel Tan
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4335138?comment=10005248
Can we have both presentations after the line? With one red and one normal?
**User:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4335138?comment=10008533
Not sure if I exactly understand the question, but the extra credit presentations you sign up for can be whenever (before or after the line)
**User:** Daniel Tan
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4335138?comment=10014084
So we have already filled in one slot before the line and one slot after. However, there is an unexpected situation that is likely to cause us to miss our first presentation slot. There is no other slot avaliable before the line. However there is a red slot after the line  in our session, so I was wondering if we can have both our presentations after the line, but with one of them being red as a make up for missing the one before the line?


**User:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4335138?comment=10020659
Yes, that works!
**User:** Daniel Tan
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4335138?comment=10022496
Thanks! By the way, how do we submit our presentations since the gradescope for first one closes way before our earlier presentation time?
**User:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4335138?comment=10024645
If you're in this situation, you can email it to me directly.
**User:** Daniel Tan
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4335138?comment=10024893
👌 





### Comment
**User:** Daniel Tan
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4335138?comment=10028213
Ah, looks like I don't have access to the week 4 journal club slides on the website. (logged in using Berkeley email)
**User:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4335138?comment=10099745
It should be fixed now!
# [Homework 3] Problem 1: Convex Model Predictive Control
**User:** Tarun Amarnath
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4335621
 Ask questions about problem 1 below!
### Comment
**User:** Eddie Shi
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4335621?comment=10011436
Q1.2: We state that the problem we are solving is a linear system. Are we able to assume the P Q R matrices are also diagonal matrices (in addition to positive semi-definite) because of this?

I’m getting non-linear terms with only the positive semi-definite assumption.
**User:** Nima Rahmanian
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4335621?comment=10023037
This is a cost written in quadratic form, so you should expect there to be quadratic terms. You do not need to assume the matrices are also diagonal. It might help to think about the properties of positive semi-definite matrices and how they can help you solve this problem.
### Comment
**User:** Max Wilcoxson
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4335621?comment=10014043
Shouldn't the interval be exclusive on one side (ie. [t, t + \Delta t))? Or do we just ignore the points of discontinuity? Because equality on both endpoints seems to imply the signal never changes?
**User:** Nima Rahmanian
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4335621?comment=10022969
Yes, I think a more rigorous statement would be exclusive on one side.
### Comment
**User:** Max Wilcoxson
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4335621?comment=10018416
Shouldn't the $U$ matrix include $u_0$ to $u_{N-1}$, not just $u_1$ to $u_{N-1}$? Since we have $N$ inputs starting at index 0?
**User:** Nima Rahmanian
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4335621?comment=10022942
Yes, I think that is a typo.
# [Homework 3] Problem 2: Planning with Nonlinear MPC
**User:** Tarun Amarnath
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4335622
 Ask questions below!
### Comment
**User:** Trinity Chung
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4335622?comment=10090625
small request: please include requirements.txt when releasing code it is good practice thank you 
**User:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4335622?comment=10265532
Hi Trinity - we'll try our best to do this in the future!
### Comment
**User:** Joshua Tsai
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4335622?comment=10156845
Did anyone else come across this error: "Wrong number or type of arguments for overloaded function 'new_DM'." when calling xGuess = ca.DM(xGuess) after using np.linspace to create xGuess?


**User:** Nima Rahmanian
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4335622?comment=10261476
Is this still an issue? I think I might have helped you out in OH. If not, I would suggest you print out the datatypes of all of your objects. Make sure the dimensions are lined up appropriately.
**User:** Alec Li
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4335622?comment=10263634
I solved this by using ca.linspace rather than np.linspace; it required a little more reshaping though.
### Comment
**User:** Noah David
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4335622?comment=10229484
For 2.5 it says we make the eigen value of Q that is multiplied by theta error equal to 0, because in this question we don't care about the orientation. It doesn't mention this in the context of the P matrix, but I just want to confirm that this is also the case for the P matrix?


**User:** Nima Rahmanian
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4335622?comment=10261442
It's not specified in the problem, but I agree with you that it would make sense to also make sense for the P matrix (defines the norm on the final error vector). This is because in the problem we generally don't care about the orientation of the robot as much as its position.
### Comment
**User:** Jackson Hilton
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4335622?comment=10258483
What are we supposed to submit for 2.4? There doesn't seem to be a deliverable in the question
**User:** Tarun Amarnath
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4335622?comment=10261355
There's no deliverable for 2.1-2.4
**User:** Jackson Hilton
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4335622?comment=10261382
Ok thank you. Should we just submit a blank image on Gradescope then? It's a required question
**User:** Tarun Amarnath
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4335622?comment=10261602
ah I will remove it as required rn




# [Homework 3] Problem 3: Kinematic Constraints
**User:** Tarun Amarnath
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4335624
 Ask questions below!
### Comment
**User:** Joshua Tsai
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4335624?comment=10095629
What is meant by G.C. positions in problem 3.2? Does it stand for generalized coordinates positions?
**User:** Tarun Amarnath
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4335624?comment=10111210
Yup generalized coordinate!
### Comment
**User:** Tianqi Zeng
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4335624?comment=10106336
could anyone give a hint of how to solve problem 1? I am really confused
**User:** Tarun Amarnath
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4335624?comment=10111270
If I tell you that $x+y=0$, can you write out the value of $y$ using the other coordinates in the system, in this case $x$?

Use this same logic to write one of the generalized coordinates as a function of the other GCs and $h$. Then, differentiate.
### Comment
**User:** Joshua Tsai
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4335624?comment=10140729
I’m confused at the holonomic vs. nonholonomic intuitions. Shouldn’t the TurtleBot’s positions still be affected by the constraints since it can no longer go into certain areas? The only difference I’m thinking of is that it is not entirely constrained to a path in G.C. (much like a function h(q)), whereas the roller coaster has such an equality constrained upon it. 
**User:** Nima Rahmanian
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4335624?comment=10152574
Imagine you are in an empty room with your TurtleBot. Ask yourself this question without thinking about anything taught in 106B: could the TurtleBot's center of mass reach any location on the floor (or more formally, a two dimensional plane)? Moreover, could the TurtleBot reach any location on the floor in any heading?



Since the answer to these questions is yes, we know that the TurtleBot's kinematic constraints are not integrable. If they were, you would answer no to at least one of those questions. This is why the TurtleBot is a non-holonomic system.
### Comment
**User:** Noah David
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4335624?comment=10211278
I am really confused by number 1. I don't understand what the pfaffiant constraint has to do with the Lagrangian dynamics. And since all we know is that w(q)*q_dot = 0, and h(q) = 0, and we nothing about the functions w(q), or h(q), themselves, how are we supposed to isolate q_n double dot?


**User:** Nima Rahmanian
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4335624?comment=10261645
This question is more about taking the algebraic equation h(q) = 0 and arriving at an expression for q_n ddot. Since h(q) = 0 is an algebraic constraint, we can find another function, say h_2(q1, ..., qn-1) such that h_2(q1, ..., qn-1) = qn. This process is essentially isolating one of the variables in your algebraic constraint. At this point, you can use h_2 to arrive at an expression for q_n ddot.



The system's Lagrangian dynamics provides us with a set of differential equations such that we can solve for q_1 to q_n-1 (this is given by the problem). This problem is stating that given the values these variables must take on, we can completely determine q_n ddot. At a high level, this means that q_n ddot "isn't special" -- it's determined by the rest of its buddies (q1, ..., qn-1). That is, we lose a degree of freedom. 
**User:** Noah Adhikari
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4335624?comment=10289819
I am struggling to understand this. How do you know that $h(q)$ is separable in $q_n$? For example, something like $h(q) = q_1 + q_1 q_2 + q_2 \sin(q_1 + q_2) = 0$. 

Then I don't think you can decouple $q_n$ from $q_0, \dots, q_{n-1}$, so there is no way to uniquely define $q_n$ in terms of $q_0, \dots, q_{n-1}$ and their time derivatives.



# [Homework 3] Problem 4: Nonholonomic Controllability
**User:** Tarun Amarnath
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4335625
 Ask questions below!
# Homework 3 (Path Planning) Released!
**User:** Tarun Amarnath
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4335632
 Hello everyone! 

Homework 3 on path planning is now released on the website! It will be due on 2/26 (modified from the original 2/21 deadline). 

Feel free to ask questions in the following threads:

Problem 1: #92 

Problem 2: #93 

Problem 3: #94 

Problem 4: #95 

You can find the PDF here and the starter code here. Good luck modeling predictive control!
### Comment
**User:** Max Wilcoxson
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4335632?comment=10021797
Is Sympy allowed for doing algebra and derivatives if we cite our code? As in HW1/2?


**User:** Nima Rahmanian
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4335632?comment=10022873
Sure, go for it.
# Nima’s Oh
**User:** Sandip Maharaj
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4346722
 Are we still having Nima’s OH today?
# Project 2 Extension
**User:** Cecilia Pham
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4359678
 Hi! In lecture on Tuesday, I recall something mentioned about a possible extension for Project 2. If so, would we be able to get an update on this? Thank you so much -- an extension would be really helpful especially with the longer weekend this week and homework! 


### Answer 
**Name:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4359678?answer=10099730
Hi, if we decide to extend the project we'll announce it on Tuesday alongside weekly announcements. Please continue to work on the project to mostly have it completed by the currently scheduled deadline. 
### Answer 
**Name:** Daniel Tan
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4359678?answer=10145867
Is there any follow up on this?
# Will Discussion 5 Recording be posted?
**User:** Noah David
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4386358
 I wasn't able to attend week 5's discussion in person, so I was wondering if the recording will be uploaded to the website?


### Answer 
**Name:** Nima Rahmanian
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4386358?answer=10152454
It was not recorded this week. I would recommend watching the recorded section from last year. I think I teach better sections when I work directly on the board, so I might continue doing this.
**User:** Max Wilcoxson
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4386358?comment=10190126
Would it be possible for the worksheet to be posted to the website? Thanks!
# Lecture hall safety issue
**User:** Sandip Maharaj
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4395862
 Can people with scooter not block the doorway with their scooters in the lecture hall. 
# Details for Lecture on Thursday (2/22)
**User:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4398625
 Lecture on Thursday will be in Banatao Auditorium (Sutardja Dai Hall) from 2 to 3 pm, followed by a talk by NYU Prof. Giuseppe Loianno from 3 to 4 pm! This special lecture will not be recorded.

Abstract for the talk: Autonomous mobile robots will play a critical role in addressing diverse time-sensitive and hazardous tasks across various application scenarios. These include logistics, reconnaissance, search and rescue missions, monitoring, transportation, space missions, and construction. Future smart cities will need to reconsider their current transportation and delivery systems. Small-scale aerial and ground robots will navigate in dense urban environments to deliver in a timely manner various items. To handle these challenges effectively, robots must demonstrate the ability to execute agile maneuvers and collaborate efficiently within unknown, uncertain, and confined spaces. In this talk, I will discuss several recent research results on developing Super Autonomous or USARC: Unmanned, Small, Agile, Resilient, and Collaborative robots. Achieving this goal requires reimagining the existing perception-action paradigm governing robot autonomy. This involves shifting the existing paradigm from a sequential to a concurrent approach by combining in a principled manner physics-based and data-driven techniques across modeling, perception, learning, and control. This will result in an architecture which proactively and continuously improves its navigation performances and that naturally scales to multi-robot settings by leveraging data from multiple agents. Consequently, this will boost agility, resilience, and improve both individual and collaborative decentralized decision-making processes for small-scale robots.

Bio: Giuseppe Loianno is currently an Assistant Professor at the New York University, New York, NY, USA, and the Director of the Agile Robotics and Perception Lab (https://wp.nyu.edu/arpl/) working on autonomous robots. Prior to joining NYU, he was a Research Scientist and Team Leader with GRASP Lab, University of Pennsylvania, Philadelphia, PA, USA. He has authored or coauthored more than 70 conference papers, journal papers, and book chapters. His research interests include perception, learning, and control for autonomous robots. Dr. Loianno is a recipient of the NSF CAREER Award, the DARPA Young Faculty Award, the IROS Toshio Fukuda Young Professional Award in 2022, Conference Editorial Board Best Associate Editor Award at ICRA 2022, and Best Reviewer Award at ICRA 2016. He is senior editor for IROS 2023-2025 and IEEE Robotics and Automation Letters as well as an associate editor for multiple other robotics journals including IEEE Transactions on Robotics, International Journal of Robotics Research, Field Robotics, and Frontiers in Robotics and AI. He is chair of the IEEE RAS Robotics Foundations cluster and the IEEE RAS TC on Aerial Robotics and Unmanned Aerial Systems. He was the General Chair of the IEEE International Symposium on Safety, Security and Rescue Robotics (SSRR) in 2021 as well as Program Chair in 2019, 2020, and 2022. His work has been featured in a large number of renowned international news and magazines such as IEEE Spectrum, Popular Science, Scientific American, TechCrunch, MIT Technological Review, and Quartz. 
### Comment
**User:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4398625?comment=10211963
Recording of 2-3 pm of the lecture: https://youtu.be/1ZjUC1QOTWI (you must sign in to your Berkeley email to view it)
# Week 6 Announcements!
**User:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4398747
 Hey everyone! It's so wild that we're already on week 6, but I hope you're doing well! Here are all the things top of mind for this class:

Thursday's Lecture will be in Banatao Auditorium at Sutardja Dai Hall from 2 to 3 pm, followed by a talk by NYU Prof. Giuseppe Loianno. Please show up for this very exciting opportunity! #101

Homework 3 is due Monday, 2/26 (also extended from the original due date). 

Project 2 has been extended to 2/27 (Tuesday) due to President's Day! The next projects will not be shifted due to this, so please don't put the project or homework off. We have a lot of office hours to help you out this week and on Monday next week. Please show up with questions, we're here to help!

Just wanted to highlight that Tarun's OH from 1-2 pm on Thursdays is permanent, so please show up to get help! 

Here's the schedule for this week, important since a lot has been shifted around due to President's Day. The calendar on the website will always be the most up-to-date, but here's a screenshot for your convenience:

Finally, Homework 1 scores have been released. Overall, a really solid job on this homework:) 



Anyway, I hope everyone enjoyed their long weekend -- I bought a baguette from one of my favorite bakeries in Berkeley, La Farine! 
### Comment
**User:** Charles Paxson
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4398747?comment=10153469
Is the midterm being switched back to take-home?  I think the professor had mentioned that at the end today...  Maybe I'm wrong?
**User:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4398747?comment=10157345
He mentioned that we're considering it. I'll confirm in the announcements next week (after staff meets on Monday)!


### Comment
**User:** Derek Guo
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4398747?comment=10154644
I had two questions about HW1 grades:

In 2.2, why was it necessary that we explicitly accounted for imaginary eigenvalues? I simply stated that 

$$x_0e^{\lambda t}$$

converges to 0 for all x_0 if and only if lambda has a negative real part. This applies to complex eigenvalues as well, so there was no need to specifically handle imaginary eigenvalues.

In 4.1, while I understand how the solution brings us towards what we are trying to prove, how would compounding errors (i.e. error due to the fact that

$$\hat{x}(k)\ne x(k\Delta t)$$

) be handled? I assume that we would use smoothness to prove that these compounding errors go to zero as well. I find it plausible that it's true, I just think the proof given in the solutions is somewhat incomplete. The solution seems to just handwave this problem away by assuming that 

$$\hat{x}(k)=x(k\Delta t)$$

exactly, but this seems to be begging the question as we are trying to prove that the Euler estimate, xhat, is equal to the true value of x. Lastly, as brought up in a clarification during the original homework, I think this statement is only true when u(t) is also smooth, which also makes the solution feel handwavy as it appears to work regardless of the choice of u(t).
**User:** Michael Psenka
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4398747?comment=10157444
Please use the "request regrade" feature of Gradescope in the future for such questions for 2.2, since these always vary based on the submitted solution. I've left a comment on your assignment, and points were re-awarded.



For 4.1, the main point of the question is to get some intuitive understanding for why the Euler approximation is, in some sense, a "valid approximation". Any fully rigorous proof will have to be rigorous about what we mean by "convergence" (finite horizons, C^0 norm, C^1 norm, etc.), which can be done in a lot of ways, but all of which are outside of the mathematical skillset required for this course, so a wide variety of solutions that communicated a general understanding of why Euler works were accepted here.



For anyone interested, the following is a cute proof that also touches on the idea of convergence rates of approximation methods, and doesn't require a ton of background: na.numerical analysis - Conditions for convergence of Euler's method - MathOverflow 
### Comment
**User:** Alejandro Marquez
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4398747?comment=10255944
When will the slides and recordings for this week's lectures be posted? I see the topic is on steering with sinusoids which is what my group is kind of stuck on for the project


**User:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4398747?comment=10261281
Sorry, I got the slides a bit late - should be posted tonight at the latest! 


# Debugging Sinusoid Planner
**User:** Kalie Ching
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4400343
 My group has been working on the sinusoid planner but we're seeing a very interesting behavior. This is the planned path, but when it is executed, the actual trajectory it takes is the one drawn in red. Is there any part of our code for the planner that looks off? We're thinking it might be an issue with the steer_y, and have already caught a few issues with it. Is there some factor of 2 we're missing?

https://github.com/jaynye/106b-sp24-projs/blob/main/project2/src/proj2_pkg/src/proj2/planners/sinusoid_planner.py#L243




### Answer 
**Name:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4400343?answer=10211820
Hi Kalie! I know we spoke about this in OH briefly but were you able to resolve this (esp with Karim's new announcement)?
**User:** Kalie Ching
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4400343?comment=10260063
Unfortunately we are still having issues with the point turn. Do you have any advice on how to adjust things if this is our result for the plan? Thank you!
# RRT Planner Motion Primitives Help
**User:** Michael Campos
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4400682
 From my general understanding of motion primitives for RRT planners, motion primitives are small motions that span the feasible work space of the current robot configuration. And, we want to move along the motion primitive path for dt amount of time. For example, a set of motion primitives with velocities would be: 

This would mean that would need to use different inputs bounded by some range.

primitive_u1 = np.linspace(self.input_low_lims[0], self.input_high_lims[0], 5).tolist()
primitive_a2 = np.linspace(self.input_low_lims[1], self.input_high_lims[1], 3).tolist()
primitive_angles = np.linspace(0, np.pi, 5).tolist()
primitives = list(itertools.product(primitive_u1, primitive_a2, primitive_angles))


Where u1 would be the velocity input applied to x_dot, y_dot, and theta_dot and a2 would be the velocity applied to sinusoid steering model for phi_dot following the example from 1.5.2 The Local Planner from the lab doc. Ultimately, in the local plan function of configuration space class, I would traverse along the primitive path for dt amount of time from c1, and choose c_new that is closest to c2.

...
x, y, theta, phi = c1
v1 = np.array([np.cos(theta), np.sin(theta), np.tan(theta)/self.robot_length, 0])
v2 = np.array([0, 0, 0, 1])

best_plan, smallest_distance = None, np.Infinity
for primitive in primitives:
    u1, a2, angle = primitive
    phi_dot = a2 * np.cos(angle)
    new_v1 = v1 * u1 * dt
    new_v2 = v2 * phi_dot * dt
    new_c = c1 + new_v1 + new_v2
    distance = self.distance(new_c, c2)
    if distance < smallest_distance:
        smallest_distance = distance
        times = np.array([dt])
        positions = np.array([new_c])
        velocities = np.array([u1, a2])
        best_plan = Plan(times, positions, velocities, dt=dt)
return best_plan


If my intuition is correct, my question is what should be the size for times, positions, and velocities objects? As of right now, since we moved for such little amount of time, I assumed they should be of size one. I am also having trouble understanding what is ω in φ ̇ = a2 cos(ωt).  I ended up thinking that ω is just one of the inputs we needed to add, ωt ended up being theta, however φ ̇ is independent of time. Also for calculated distance between 2 configurations, I first compared the angles using diff = min (|θ1 − θ2|, 2π − |θ1 − θ2|) which is equation 5.7 from the Sampling-Based Motion Planning chapter. Then to calculate the distance, I used 

$$d\ =\ \sqrt{\left(x_1-x_2\right)^2+\left(y_1-y_2\right)^2+diff^2}$$

but I'm not sure if my intuition is correct for calculating distance involving angles.
### Answer 
**Name:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4400682?answer=10211893
Hi Michael! So in terms of the sizes, it's up to you (think about how long you want each "chunk" of planning to be). You can try out many different things and go with what works most optimally -- I'm happy to share what my group did when I implemented it if it's helpful! But just so you know it was not size one. In terms of the rest, generally, your intuition seems correct -- although it's a bit tricky to debug without seeing the behavior. I'm also happy to share what my group did in terms of primitives if that's helpful! 
**User:** Michael Campos
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4400682?comment=10230606
We've been stuck on this for a while, and we're not getting anywhere. The following is our latest code for the local planner function. 

def local_plan(self, c1, c2, dt=0.01):
        # Perform DFS until we reach the time limit
        def get_plans(current_pos, t, current_plan = None):
            nonlocal plans
            if t * dt >= self.TOTAL_TIME:
                plans += [current_plan]
                return

            # Dynamics
            theta = current_pos[0]
            v1 = np.array([np.cos(theta), np.sin(theta), np.tan(theta)/self.robot_length, 0])
            v2 = np.array([0, 0, 0, 1]) 

            for primitive in self.primitives:
                u1, u2 = primitive
                velocity = u1 * v1 + u2 * v2
                new_position = current_pos + velocity * dt
                root_plan = copy.deepcopy(current_plan) # Make copy of root plan to make new paths for each primitive

                # If the new position and primitive violates the contraints, do not build on top of this plan
                if self.check_constraints(new_position, primitive):
                    plans += [current_plan]
                    continue
                
                # Parameters for plan object
                times = np.array([0, dt]) # At time 0
                positions = np.array([current_pos, new_position]) # We're at current position
                inputs = np.array([u1, u2]) # We apply these primitives to obtain new position
                
                # Construct the plan with those paramters
                new_plan = Plan(times, positions, inputs, dt=dt)
                # Chain the root path with the new plan if necessary
                if current_plan != None:
                    new_plan = root_plan.chain_paths(new_plan)
                get_plans(new_position, t+1, new_plan)
        
        plans = []
        get_plans(c1, 0)
        best_plan, smallest_distance = None, np.Infinity
        for plan in plans:
            latest_position = plan.end_position()
            distance = self.distance(latest_position, c2)
            if distance < smallest_distance:
                best_plan = plan
                smallest_distance = distance
        return best_plan


# Will Be Slightly Late to Journal Club
**User:** Karim El-Refai
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4403075
 Running late. For the people presenting today feel free to start at Berkeley time
# Water Bottle Lost and Found at Cory 105
**User:** Zekai Wang
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4409580
 In front of the Turtlebot Kiwi
# Lecture today is NOT in Mulford, it's at Banatao Auditorium in Sutardja Dai Hall!
**User:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4412402
 Hi everyone! Just a reminder that the lecture today at 2 pm will be at Banatao Auditorium in SDH, not in our usual Mulford room. From 3-4 pm, we'll be joined by a special guest lecturer (Prof. Giuseppe Loiano from NYU: #101). Hope to see you there!


# If you're at Mulford right now, you're in the wrong place
**User:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4414029
 Make sure you come out to Banatao Auditorium for lecture and our special guest!!


# Discussion 5 Worksheet
**User:** Edward Chang
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4414951
 Will the Discussion 5 worksheet be published on the website?
### Answer 
**Name:** Tarun Amarnath
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4414951?answer=10205414
I think Nima used last year's worksheet directly. I'll get it posted on the website as well later today. 

Last year's worksheet

Solutions

Boardwork

Video
# is there still office hours today?
**User:** Noah David
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4420468
 Is Daniel still having OH today at 10 in Cory 105?
### Comment
**User:** Daniel Bostwick
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4420468?comment=10207284
Sorry Noah, thank you for waiting.
### Answer 
**Name:** Tarun Amarnath
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4420468?answer=10205384
Sorry, he got caught up in research meetings with Professor Sastry. He'll walk over right now. 
# [Project 2] Not Necessary for Sinuisoid and RRT Vids to Work
**User:** Karim El-Refai
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4421743
 We've noticed a lot of students having issues with executing the paths generated by the sinusoid planner and the RRT planner on the STDR simulator. While we do want you to submit these videos and comment what you think is going on. We don't expect your videos for these two planners to work well. It's fine if it sucks and runs into a wall as long as your plot for the intended path is correct.


# Question about Dealing with Singularity
**User:** Kai Xu
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4435680
 Hi!

I saw that in the project 2 release thread that one of the TA's says that we are not required to deal with singularities for this project. I'm running into a singularity problem with executing the point turn trajectory for the sinusoidal controller. I need to handle this singularity to actually run the trajectory, but from what the TA says, does it mean that it is ok for us to not handle this issue for this project? I am a bit confused about this. Thanks a lot!


### Answer 
**Name:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4435680?answer=10261209
Hi - if you're hitting a singularity and you're not able to fix it, what's important is that you can explain why you're hitting the singularity and explain how you would resolve the singularity in the report. You can try to fix it, but it is not required to implement/handle the singularity issue. 


# Nima's OH has been moved!
**User:** Nima Rahmanian
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4438055
 Hi students! I moved around my OH schedule in the hopes of boosting attendance. You can see the changes reflected on the course calendar.
# Runtime Error hw3 q2
**User:** Japinder Narula
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4446152
 I am having the following error:

Simulation Time Remaining:  5
---------------------------------------------------------------------------
RuntimeError                              Traceback (most recent call last)
<ipython-input-45-dcf39cdaaa6e> in <cell line: 30>()
     28 
     29 #run the simulation
---> 30 env.run()

6 frames
<ipython-input-32-a9018ad9398d> in run(self, N)
    125             while not self._is_done():
    126                 print("Simulation Time Remaining: ", self.TOTAL_SIM_TIME - self.t)
--> 127                 self.step() #step the environment while not done
    128             self.visualize() #render the result
    129 

<ipython-input-32-a9018ad9398d> in step(self)
     63 
     64         #solve for the control input using the observed state
---> 65         self.controller.eval_input(self.t)
     66 
     67         #Zero order hold over the controller frequency

<ipython-input-44-78803a8f24d3> in eval_input(self, t)
    341         """
    342         #set up the optimization at this step
--> 343         self.setup()
    344 
    345         #using the updated state, solve the problem

<ipython-input-44-78803a8f24d3> in setup(self)
    305             self.add_input_constraint()
    306         if obs_constr:
--> 307             self.add_obs_constraint()
    308 
    309         #Add obstacle constraint

<ipython-input-44-78803a8f24d3> in add_obs_constraint(self)
    164             #TODO: YOUR CODE HERE
    165             for i in range(N + 1):
--> 166                 self.opti.subject_to(((self.X[0, i] - center[0])**2 + (self.X[1, i] - center[1])**2)**0.5 >= radius)
    167 
    168 

/usr/local/lib/python3.10/dist-packages/casadi/casadi.py in subject_to(self, *args)
  49040           frame = {}
  49041       meta = {} if frame is None else {"stacktrace": {"file":os.path.abspath(frame.f_code.co_filename),"line":frame.f_lineno,"name":frame.f_code.co_name}}
> 49042       ret = self._subject_to(*args)
  49043       if len(meta)>0:
  49044           self.update_user_dict(args[0], meta)

/usr/local/lib/python3.10/dist-packages/casadi/casadi.py in _subject_to(self, *args)
  47620 
  47621         """
> 47622         return _casadi.Opti__subject_to(self, *args)
  47623 
  47624 

RuntimeError: Error in Opti::subject_to [OptiNode] at .../casadi/core/optistack.cpp:94:
.../casadi/core/optistack_internal.cpp:903: Assertion "!g.is_constant()" failed:
You passed a constant to `subject_to`. You need a symbol to form a constraint.



I have tried tweaking the line the error output references (166) but keep getting the same error no matter what. Could I get some guidance as to why this may be occurring? I can upload my code if necessary, I currently don't know what format I am supposed to upload code snippets in.


### Answer 
**Name:** Tarun Amarnath
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4446152?answer=10320457
resolved
# Possible typo in the A_Guide_to_Sinusoidal_Planning
**User:** Zekai Wang
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4446237
 Link: https://drive.google.com/file/d/1GkHz4mI22M_1EhaEiOeLFyV07uKuVDbp/view

On page 5 (steer_x4), I believe the underlined $\cos$ should be $\sin$ because $\dot{\phi} = v_2 = a_2 \cos(2 \omega t), \phi(t) = \phi(0) + \frac{a_2}{2\omega}\sin(2\omega t)$. 
# Week 7 Announcements!
**User:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4447211
 Hey everyone - I hope you had a good weekend! I spent yesterday holed up in the Vegas airport after my 7 am flight got canceled after my conference. I don't think I'm going back anytime soon lol. On the other hand, I got to see the sphere up close, which was very cool! 

A few announcements: 

There's an interesting talk happening tomorrow Tuesday 2/27 from 4-5 pm in 430 Soda Hall by MIT PhD Candidate Michael West, Jr. titled "All models are wrong, simple models are insightful: A study of human manipulation" -- goes hand in hand (haha get it) with all this grasping stuff. 

As you probably already know, Homework 3 is due tonight! By the way, (and I'm sorry I have to say it but) please make sure that you're turning in your own work. Going through the homework with your own effort (and additional help during OH/HW Party) is crucial to getting the most out of the course. 

Project 2 is due tomorrow night! Just a reminder that you shouldn't have to deal with any singularity-related implementations with the sinusoidal controller - please just discuss why it is occurring and the methodology with which you'd solve it in the report.

Projects 3 (Grasping) and 4 (State Estimation) will be released on Wednesday. We're updating them significantly from previous years, so it'll be fun. Plus, we'll have a special extra credit opportunity (more TBA).

Here's the best news that I hope will make your week: the midterm has been switched back to take-home since the problems we tackle in this class really push the envelope. It will be released on April 11th after class, and due April 13th. This is the final decision -- we won't switch it back. DSP students will be contacted for their accommodations soon. 

Make sure you're staying on top of your journal club presentations! And come to office hours for help! And lecture -- we've got some super exciting guest lectures lined up in the coming weeks! 

I'm going to update the website soon with all of this and lecture slides, probably tomorrow morning since I'm currently experiencing the compound interest of sleeping very little for the past five days: 
### Comment
**User:** Noah Adhikari
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4447211?comment=10268725
yay!!!!!!!!
**User:** Angel Aldaco
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4447211?comment=10268729
yayyyyyy
### Comment
**User:** Tarun Amarnath
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4447211?comment=10268745
i saw new weekly announcements and got hyped
### Comment
**User:** Matthew Sulistijowadi
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4447211?comment=10268754
yip
# Homework 2 solution
**User:** Sandip Maharaj
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4457555
 Hi when will homework 2 solution be released?

The class website said that it would be released 2 days after the deadline which was why a max of 2 slip days was allowed for a homework but it is still not available in the solutions mega thread. 
### Answer 
**Name:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4457555?answer=10300252
Hello! The reason why the solutions weren't released is that some folks had extenuating circumstances that required them to turn it in even later than the two days. Everyone's turned it in by now, so the solutions will be posted soon.
### Answer 
**Name:** Tarun Amarnath
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4457555?answer=10303723
#119 
# Homework 2 Solutions
**User:** Tarun Amarnath
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4461977
 
# Lecture Attendance Alternative
**User:** Derek Guo
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4465154
 I have an injured leg and have been advised to limit the amount I walk. Is there an alternative way to receive the lecture attendance extra credit?


### Answer 
**Name:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4465154?answer=10319955
Hi Derek, I'm sorry to hear that. Unfortunately, lecture extra credit is the incentive for people to come, but feel free to watch back the recordings. Not getting the extra credit won't count against you in any way of course. There are other ways to get extra credit in this class in the meantime (through EPA, extra journal club presentation, etc). I really hope your leg heals soon! 


# If you left a pair of headphones in Mulford, I'm bringing them up to Cory 105!!
**User:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4469984
 
### Comment
**User:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4469984?comment=10322404
didn't realize the heic wouldn't show up but they're like white and beige bose headphones!!


**User:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4469984?comment=10322900
I put them on the desk in the front!
### Comment
**User:** Zekai Wang
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4469984?comment=10358815
Emmmm I actually saw them there before the start of Thursday’s lecture. 
# Homework 4 Released!
**User:** Nima Rahmanian
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4473405
 Hot off the press. Enjoy.



Students, this is a reminder that using materials from previous iterations of EECS 106B to solve the homeworks is not allowed. While collaboration is encouraged, you may only reference materials introduced this semester.


### Comment
**User:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4473405?comment=10329921
Due 3/15!!! Website will be updated shortly:)
### Comment
**User:** Max Wilcoxson
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4473405?comment=10351041


The middle B matrix should be $B_{c_2}$, right?
**User:** Nima Rahmanian
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4473405?comment=10354753
Yes, good catch. I just realized there are similar typos in other parts of the homework. I updated the pdf.
### Comment
**User:** Max Wilcoxson
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4473405?comment=10359474
Are we allowed to use Sympy for question 6 as well, if we cite our code?
**User:** Nima Rahmanian
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4473405?comment=10416802
Sure.
### Comment
**User:** Jiachen Lian
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4473405?comment=10370749
I have a question about the wrench transformation. Take this figure from problem 2, for example. The reason we perform the transformation is to obtain the wrench in frame 0, which typically passes through the center of mass of the rigid body. Is this correct? If so, my confusion arises when considering that sometimes external torque might cause the rigid body to rotate about an axis that does not pass through the body's center of mass. In other words, the rotation axis might not intersect the center of mass. How do we handle such cases? 


**User:** Tarun Amarnath
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4473405?comment=10418590
That's perfectly ok! The transformation takes care of that by combining the force and torque terms as needed to account for the shifted center of mass. Now, this may not result in a grasp that is in force closure, as you may end up dropping your object. However, the process of finding the spatial wrench stays the same.
### Comment
**User:** Jiachen Lian
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4473405?comment=10370981
Also for this, three lines do not intersect right? (The other one is not central line)
**User:** Nima Rahmanian
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4473405?comment=10416851
The three lines do intersect.
### Comment
**User:** Jiachen Lian
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4473405?comment=10375678
For problem6 question2, I think it is obvious, curious what we need to show additionally
**User:** Nima Rahmanian
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4473405?comment=10416886
This problem is a bit more simple. You can detail how you arrive at the equation.
### Comment
**User:** Jiachen Lian
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4473405?comment=10376169
Also, can you please put submission entry in gradescope?
**User:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4473405?comment=10390480
Will create the submission portal soon, probably tomorrow!


### Comment
**User:** Matthew Sulistijowadi
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4473405?comment=10387631
Could the 3 slides for grasping please be posted on the website soon? Thank you in advance
**User:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4473405?comment=10390486
As soon as I get them from the prof, yes!
### Comment
**User:** Jacob Gottesman
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4473405?comment=10416184
Assuming it is, but to confirm, is the M in w_hat_oc the same as M_p in v_oc? And is T the same as T_p? Same with K?
**User:** Nima Rahmanian
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4473405?comment=10477233
Yes.
### Comment
**User:** Edward Chang
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4473405?comment=10565371
I have a conceptual (kind of) question about problem 5. The question describes that we parameterize a set of 3D coordinates into 2D coordinates. What I'm curious about is how much information the 2D coordinates retain. 

My current understanding is that the 2D coordinates do not describe a path/coordinates on the 3D surface, but rather can be converted by the chart to find the 3D path/coordinates.

In part 4, we are asked to convert the path p(t) into a(t) = (u(t), v(t)). Does this mean to find u(t) and v(t) that we can then use to convert p(t) to p(u(t), v(t))?

Thank you in advance!
**User:** Tarun Amarnath
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4473405?comment=10571905
Yup, that's exactly right, you're trying to find $p(u(t), v(t))$. You should be able to find $u(t)$ and $v(t)$ based on the structure of $p(t)$. 
### Comment
**User:** Kalie Ching
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4473405?comment=10565961
Should beta_i in 4.3 be > 0 instead of >= 0?
**User:** Tarun Amarnath
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4473405?comment=10571844
It should be $\geq$ because we wouldn't be able to make one of the basis vectors otherwise. 
# Reduced OH today for Michael
**User:** Michael Psenka
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4497216
 Hi all,

Due to a last-minute conflict, I will only be able to hold office hours today from 12-1. Apologies for the late notice.

I'll see you all tomorrow when I give a lecture on modern 3D vision!

Best,

Michael
# Homework 3 Solutions
**User:** Nima Rahmanian
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4501360
 
# Project 3: State Estimation Released!
**User:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4503020
 https://ucb-ee106.github.io/106b-sp24site/assets/proj/proj3.pdf

Due March 22nd 11:59 PM

Clarification on deliverables: #137
### Comment
**User:** Sandip Maharaj
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4503020?comment=10403788
We were told we will be given 4 weeks to do both project 3 and 4 but we are being given about 3 weeks only. 
**User:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4503020?comment=10404246
Hi, that must've been a miscommunication. We introduced the concepts of the projects last week (making the deadline 4 weeks out), but we've been pretty consistently thinking both projects would be due the Friday before spring break. If we extend the project beyond spring break, we'll just have less time for the final project, which is worth a lot more of people's grades and not good for anyone involved. Both of these projects are a lot more approachable than the previous ones. 
**User:** Nima Rahmanian
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4503020?comment=10417055
I'll also mention that these labs are less open-ended than previous labs. This means it should take less time to complete.

### Comment
**User:** Max Wilcoxson
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4503020?comment=10415994
The Github classroom page says spring 2023. Is that intended? Thanks! 



I currently think this is an accident, given the same hyperlink in the project 4 document instead goes to a Github classroom entry for Projects 3/4 Spring 24. I believe this is the correct link: https://classroom.github.com/assignment-invitations/cb8b41fc875400abbf067699c2b288de/team
### Comment
**User:** Max Wilcoxson
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4503020?comment=10417884
Should we expect a correct dead reckoning implementation to perfectly match the goal with zero error, or just be quite close? I feel like there should still potentially be some error due to the fact that the true trajectory is computed through integration of the dynamics, while we are still using a Euler discretization approximation, but I am not sure


Update: Empirically, there does seem to be a significant discretization error at dt = 0.1, but this goes away with dt = 0.01. 
**User:** Karim El-Refai
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4503020?comment=10458026
You should not expect it to perfectly match the goal with zero error for the drone or the turtlebot but it shouldnt be wildly off. Unless the data you are looking at for drone dead reckoning is the noisy data the final error should be noticeable but not egregious.
### Comment
**User:** Kev Wang
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4503020?comment=10465254
If yall are having errors running pip install -r -U requirements.txt in section 4.3 "Setup environment", try this command instead: pip3.8 install -U -r requirements.txt


**User:** Thien An Phan
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4503020?comment=10476394
Thanks this is super helpful! However there is still a few errors tho.
**User:** Kalie Ching
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4503020?comment=10484439
We also had to add the directory where pip and pip3.x were installed to the PATH to resolve additional issues. Maybe this will also work for you?



### Comment
**User:** Alejandro Marquez
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4503020?comment=10475741
What's the purpose of this condition inside the update function for the turtlebot in the starter code?


if len(self.x_hat) > 0 and self.x_hat[-1][0] < self.x[-1][0]:

**User:** Karim El-Refai
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4503020?comment=10477424
self.x and self.x_hat are lists of states where the the 0th element of the state is the time in which that state occurred (note: this is not the same for the drone simulator as self.t will contain the appropriate time). So the first part of the condition is checking that we have an initial estimate and the second part of the condition is checking that the time of the most recent estimate is newer than the time of the most recent actual state.
### Comment
**User:** Thien An Phan
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4503020?comment=10479061
I want to mention a typo in the starter code.

At line 19 and 20 in drone_estimator.py, u[i][1] should be u[i][0] and u[i][2] should be u[i][1].

Nothing major, just a classic off by 1 error.
### Comment
**User:** Max Wilcoxson
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4503020?comment=10483376
I am having trouble getting extended Kalman filter to work, it seems like the sensor just doesn't provide very rich information, so the estimation seems plausible under the sensor readings, but compounding error leads the end result to be very different in location, even though sensors have similar values. How might one deal with this? Is this expected?

Edit: Turns out my parameters not correct, and I did not have sufficient variance on measurement noise. 
**User:** Thien An Phan
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4503020?comment=10492599
Thanks for the hint!
### Comment
**User:** Noah Adhikari
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4503020?comment=10494404
For dead reckoning, how can we access $T$ (episode length) in the code?
**User:** Daniel Tan
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4503020?comment=10498912
I think we don't need T.
**User:** Noah Adhikari
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4503020?comment=10503131
Yeah, I realized this as I was going through - I thought we were supposed to recompute the entire trajectory using only x[0] (because the pseudocode algorithm has it in a while loop), but the update only performs one timestep which gets called by something else.

### Comment
**User:** Kaitlyn Lee
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4503020?comment=10495102
Hi, the website's probably wrong since it says the projects will be due 4/7, but would it be possible to get the project specs added to the website as well as the correct due dates? Thanks! 


**User:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4503020?comment=10513261
Oof yeah sorry working on it rn, will have it done by tn!! Been super swamped w midterms 😭 
### Comment
**User:** Vishnu Murali
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4503020?comment=10545322
Hi pulled the repo from the directions on the spec but I am not able to see all the files. I am not able to see the drone proj3 folder amongst many others?
**User:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4503020?comment=10554196
Make sure you're using this link: https://classroom.github.com/assignment-invitations/cb8b41fc875400abbf067699c2b288de/team
### Comment
**User:** Zekai Wang
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4503020?comment=10614541
For the extra credit, can we just implemented the Extended Kalman Filter for the Turtlebot in simulation (which simply means filling out the function in Estimator.py), or should we somehow implement this on a real turtlebot? 
**User:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4503020?comment=10780044
Extra credit is for implementing on the real turtlebots, only, unfortunately
### Comment
**User:** Max Wilcoxson
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4503020?comment=10664528
Deliverables b and c for section 3 Experimental Results seem to be the same, just one stated in math and one in English. Is that understanding accurate?
**User:** Karim El-Refai
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4503020?comment=10780431
Yes you're correct!
### Comment
**User:** Anish Dhanashekar
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4503020?comment=10793326
How are we supposed to get quantitative measurements on estimation accuracy and per-step computational running time as mentioned in part (a) of the experimental results section?
**User:** Karim El-Refai
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4503020?comment=10793815
You can get quantitative measurements on estimation accuracy by just taking the average error between $\hat x$ and $x$ across all the timesteps. For per-step computation you can just print out the difference in time at the beginning and end of the call to update() for each method
### Comment
**User:** Erick Brito
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4503020?comment=10803302
Anyone having trouble with imports remember you can always just run pip install --upgrade matplotlib etc. manually


# Project 4: Grasping Released!
**User:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4503027
 https://ucb-ee106.github.io/106b-sp24site/assets/proj/proj4.pdf

Due March 22nd 11:59 PM

Some hints: #151
### Comment
**User:** Minh Nguyen
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4503027?comment=10454813
I'm having a dependency issue trying to install vedo on my local virtual machine. I'm on Python3.8.10 with output:

ERROR: Could not find a version that satisfies the requirement vtk (from vedo) (from versions: none)
ERROR: No matching distribution found for vtk (from vedo)

**User:** Max Wilcoxson
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4503027?comment=10490877
There is no build wheel of VTK for python3.8, which is system python on Ubuntu 20.04. I would recommend creating a conda environment with a newer Python version, that resolves the issue

Edit: After turns out you actually need an older Python version. After a couple of hours of debugging, here is the sequence of actions that finally worked:
Create a Python 3.7 conda env: 
conda create --name proj4 python=3.7 (and activate it)

conda install -c conda-forge vtk==8.2.0 (install vtk version 8.2.0 through conda)

pip install vedo==2021.0.2 (use pip to install an older version of vedo that is compatible)


**User:** Cecilia Pham
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4503027?comment=10507108
Thank you!

### Comment
**User:** Max Wilcoxson
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4503027?comment=10489482
Are we supposed to implement compute_force_closure method in grasp.py? It doesn't seem to directly match the problem spec, though maybe I am missing something?
**User:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4503027?comment=10780091
Hi! I think I answered this question in OH, but try out looking at the hints + I can help further if you've still got this question:)
### Comment
**User:** Cecilia Pham
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4503027?comment=10517373
For the compute force closure, I thought that the force closure was a binary value so would it return a 0 or 1 and then the robust force closure would return somewhere in between? If I am wrong, where can I read more on determining the quality of a force closure?


**User:** Max Wilcoxson
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4503027?comment=10540564
This is part of why I was confused by this method 
**User:** Noah Adhikari
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4503027?comment=10768521
Based on the hints I think you can just return 0.0 for false and 1.0 for true. I don't think there's anything that prevents you from modifying it to return a boolean though...
**User:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4503027?comment=10780095
Yep it's supposed to be a boolean return value

### Comment
**User:** Max Wilcoxson
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4503027?comment=10541470
What does the notation mean here? I'm not sure what it means to have a subscript f before the norm. It does seem to mean something, since a standard norm would be a scalar, but we then index into f hat when computing J. Thanks!
**User:** Zekai Wang
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4503027?comment=10617947
I believe there is a typo. This is the doc from Spring 2022 (https://pages.github.berkeley.edu/EECS-106/sp22-site/assets/proj/2022__C106B_Project_3.pdf).
**User:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4503027?comment=10780097
Yep the f was an OCR issue! 

### Comment
**User:** Derek Guo
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4503027?comment=10566803
If custom_grasp_planner already receives the contact vertices, doesn't that already define our gripper pose up to one degree of freedom? Do we just choose a value for that last degree of freedom to avoid collision with the mesh, or do we disregard the vertices passed in and pick our own grasp?

Also, as I understand it, compute_robust_force_closure would require the mesh to project the perturbed contact points back onto the mesh surface and find the corresponding normal vectors, but the function does not accept a mesh as an argument. Is this an oversight or am I misunderstanding how the method works?
**User:** Noah Adhikari
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4503027?comment=10768398
I believe the starter code is pretty different from what you're supposed to do based on the "general workflow" portion of the spec (I guess it serves as an example assuming you already had the vertices). So I think you're supposed to modify the base code to run for a lot of trials and try a bunch of points on the given mesh, then find the best ones based on the three metrics.

Not sure about the angle of gripping though (maybe we just constrain it to straight down or straight out?)

For your second question I think you can add (default) arguments to the function if necessary.
**User:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4503027?comment=10781878
Noah's correct regarding the starter code and default arguments. For the angle of gripping - you can constrain it to straight down or straight out (either one should be okay). 



### Comment
**User:** Max Wilcoxson
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4503027?comment=10645077
What is the origin of the vertices provided from the dataset? Is z set at the bottom of the object?
**User:** Kev Wang
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4503027?comment=10792728
My group is still wondering about this. Does anyone know?
**User:** Max Wilcoxson
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4503027?comment=10809665
Try opening model file in Blender (installed on lab machines). Then you can see origin. It seems a bit random, so I’d recommend using Blender to visualize it and measure distances to sides of object.

### Comment
**User:** Matthew Sulistijowadi
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4503027?comment=10646258
how do we run the simulation (before attempting to use the sawyer)? atm just trying to run the code as "python grasping.py"
**User:** Minh Nguyen
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4503027?comment=10671500
Yeah, if you run grasping.py you can see what pose your grasp planner generates. There's not much else you can simulate though :( .
**User:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4503027?comment=10781890
Yeah Minh is correct, unfortunately there isn't something extra you can simulate prior to using the Sawyer.



### Comment
**User:** Minh Nguyen
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4503027?comment=10649923
Is there a method to close the gripper from python? The only way we've been able to do it is to launch a seperate node and use the intera_examples script.
**User:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4503027?comment=10822286
There should be! 

So if you have a gripper on a Sawyer you can do 

from intera_interface import gripper as robot_gripper

gripper = robot_gripper.Gripper('right_gripper')


and then from there you should be able to call

gripper.close() and gripper.open()
### Comment
**User:** Gurvir Kooner
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4503027?comment=10650916
i'm looking at the hints and i'm unsure about what b1 and b2 represent. could someone give a tip or explanation to help me understand their purpose and what they represent
**User:** Charles Paxson
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4503027?comment=10798608
Just bumping this, since I am very confused.  Is it supposed to represent the alpha terms from Section 1.1 on the spec?
**User:** Kaitlyn Lee
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4503027?comment=10807453
Not confident about this, but I think b1 and b2 are like the beta terms from the grasping homework question 4.4 for the force closure constraint $v = \beta_1 v_1 + \beta_2 v_2$, where $\beta \geq 0$. It's pretty similar to the alpha terms from Section 1.1. 




**User:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4503027?comment=10822308
Take a look at this thread -- please lmk if there's follow up questions!




### Comment
**User:** Noah Adhikari
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4503027?comment=10768516
In contrast to what has been said with projects 3-4 being more straightforward, our group has found this project to be much more involved compared to the first three projects, and I think other groups are noticing this as well.

For example, the directions in the spec often conflict with the starter code, and the hints given do not fully explain the gaps, often leading to more confusion than clarification. There are mistakes and missing information in the code. There is little information on how to use cvxpy and trimesh (we are simply told to look up documentation), and while the theory doesn't look as bad as previous projects, we are left on our own to figure out a lot of things. Add on that we have to go from simulation to a Sawyer and this project is pretty gnarly.

There's nothing wrong with some mistakes and/or a difficult project. However, I am frustrated because the support has been quite underwhelming. There hasn't been a single staff response in this thread. I understand that the course is understaffed and that staff members are busy, but some of the comments here have gone unresolved for weeks, and it feels like students are being hung out to dry. It should not be students' responsibility to answer other student questions.

I don't have a proposal or anything to reduce the scope of the project, I would just like to see a bit more (Ed) communication from the staff going forward.
**User:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4503027?comment=10780089
Hey Noah, your feedback is very felt and heard. This is part of why we extended this project further, and I'll be getting back to everyone on this thread + other Ed threads for this project ASAP. This project was definitely much harder to dev + support than I initially thought so it's great feedback for the future. Edited: we will also be grading the "real" portion of this project really gently. Mainly we just want you all to have the experience of trying out the grasping stuff IRL, but it doesn't have to work 100%. Will add this in during announcements today.
### Comment
**User:** Edward Chang
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4503027?comment=10797905
Once we have a transform matrix, how do we convert that into a quaternion for the sawyer bot?
**User:** Jacob Gottesman
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4503027?comment=10803608
tf.transformations.quaternion_from_matrix() :)


### Comment
**User:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4503027?comment=10798882
One idea that came up in project party today was that you could do the "real" part of the project without an AR tag. If you know the ideal contact point through the visualizations of the sim, then you could estimate a position to grasp an object using tf by directly positioning the gripper where it should be on the object. This is obviously not exact and not the most ideal, but will be accepted with full credit for the project given the time constraints. Edited: Just make sure you talk about exactly what you did in the report and why this might not be the most exact.

Shoutout to Hams for coming up with this idea!
### Comment
**User:** Akhil Vemuri
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4503027?comment=10824583
How do we get 18 grasps? Shouldn't it just be 3 grasp metrics each for 2 objects, so 6?


**User:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4503027?comment=10824959
3 trials for each one!
### Comment
**User:** Derek Guo
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4503027?comment=10835481
Here are some notes we recorded while doing this project:

Pawn has inverted normals, nozzle does not. When getting the normals using trimesh, flip the normals for one object.
Both models are about 1.5x larger than the real objects. Scale down the location of the origin and the grasp position.
AR tag localization is extremely inaccurate - we first moved the camera closer to the AR tag to get a more accurate reading, then calculated the intersection between the known table plane and the line between the usb cam and the detected AR tag position. You can get the camera position using tf frame 'usb_cam'
The example images in the project spec images show z=0.091 for pawn origin but it's more like 0.064 (in the model, scaled by 2/3 for real)
The object origins are not aligned with anything significant; use Blender to inspect origin locations and axes orientation


### Comment
**User:** Michael Campos
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4503027?comment=10846180
Once we get the pose in the object's reference frame from custom_grasp_planner, how do convert in a way that it is in the robot's base frame? 

Also, we used lab 5 from last semester, and we are having issues with moving the robot arm. It can move to any position when the gripper is facing down; however, when we use the Cartesian coordinates and quaternions of the gripper from tf echo, it times out, basically outputting an error saying that the jointspace is empty. Any help would be appreciated, thank you!
**User:** Karim El-Refai
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4503027?comment=10872777
What you’ll need to do is localize the object in some way relative to the robot’s base fram you can do this by using an ar tag or manually moving the gripper over the object and calling tf_echo. Note that the origins for the pawn and nozzle are not at the bottom of them so you’ll have to account for that
### Comment
**User:** Trinity Chung
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4503027?comment=10849110
for Ferrari Canny: the hint doc suggests that we create a "desired wrench" (Wg) and then perturb it for n=10 trials, while the project doc says to create random n=1000 w's from sampling unit sphere. Which are we supposed to follow?
**User:** Karim El-Refai
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4503027?comment=10872754
Our implementation requires only 10 however depending on your implementation you may need more. I recommend starting offwith the lower number of samples (since it’s easier to compute) and increase the number if you find your current results lacking
### Comment
**User:** Max Wilcoxson
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4503027?comment=10852463
How should we match pages? I am a bit confused by the way the sections are allocated compared to the project report deliverables. It feels like discussion of the individual metrics (broken down in results) is part of the methods section (and that the method section has more content than 5pts). Am I missing something? Thanks!
**User:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4503027?comment=10884905
Hi! You're right that methods is actually worth 10 points as stated on the project doc -- just updated the gradescope to reflect that. Hope that helps, let me know if you have a further question!


### Comment
**User:** Nithila Poongovan
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4503027?comment=10915134
We've had this problem all night where the right gripper will rarely close on any of the Sawyers even though we are using the gripper commands given in lab 5 from 106A. It only seems to work sporadically - does anyone have any solutions to this?
# Week 8 Announcements!
**User:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4503051
 Hi guys! It's too late so let's get right into it~

REALLY COOL lectures this week: Tuesday (today now) is Michael's lecture on vision !!! and Thursday is Prof. Ken Goldberg's guest lecture on grasping. They're both going to be really really good. You should come. 

Homework 4 (#122) due March 15th -- please be sure that you're turning in your own work:)

Project 3 (#125) and 4 (#126) both due March 22nd (Friday night before Spring Break). This uses Github Classroom so you must have the same team for 3 & 4 (and same repo, to make things easier for both of us). Make sure you pace yourselves well!

Make sure you're staying on top of your journal club presentations! And come to office hours for help! 

Will release the Gradescope for both shortly, as well as update the website soon too. COME TO LECTURE
### Comment
**User:** Bear Häon
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4503051?comment=10400378
So excited for the vision lecture tomorrow !! 
### Comment
**User:** Noah David
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4503051?comment=10400405
The GSI's told us during journal club that we would have 4 weeks to do both labs 3 and 4. So wouldn't we have at least until the week after break?
**User:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4503051?comment=10404237
Hi Noah, that must've been a miscommunication. We introduced the concepts of the projects last week (making the deadline 4 weeks out), but we've been pretty consistently thinking both projects would be due the Friday before spring break. If we extend the project beyond spring break, we'll just have less time for the final project, which is worth a lot more of people's grades and not good for anyone involved. Both of these projects are a lot more approachable than the previous ones. 
### Comment
**User:** Bear Häon
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4503051?comment=10409169
Are this weeks guest lectures in the standard class - or at SDH again? 
**User:** Tarun Amarnath
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4503051?comment=10410748
Should be in the standard classroom
### Comment
**User:** Trinity Chung
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4503051?comment=10469164
midterms are happening this week and next week 🙃 may we kindly request the project due date be Sunday
**User:** Matthew Sulistijowadi
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4503051?comment=10469258
i would like to concur also since we have the current hw and the next hw presumably 
**User:** Trinity Chung
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4503051?comment=10469520
I suppose if staff is planning to grade that weekend then Friday is reasonable.. staff has already been pretty nice (re: midterm) so thank you in general!!
**User:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4503051?comment=10513251
keep an eye out for the announcement tn;)

# Slides for today's lectures
**User:** Michael Psenka
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4503325
 Attached are draft slides for today's lectures. Most content will be talked through and written in during lecture today, but hopefully the slides can help introduce some of the topics we will discuss before lecture. See you then!

- Michael


# Running late to journal club
**User:** Karim El-Refai
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4513516
 Going to take me longer than I hoped to get to cory (damn you buses!). Feel free to start presenting at Berkeley time.
# Final Project Logistics Question
**User:** Max Wilcoxson
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4516488
 Hello, 

I just wanted to confirm two points about the logistics for the final project:

1) The last in-person event for this class is the final project presentation on May 3rd

2) The final project report/website (due May 10th) is a digital submission

Thanks!


### Answer 
**Name:** Tarun Amarnath
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4516488?answer=10451263
Yup, you won't have to be in town after the presentation date. 
# Come to lecture (Mulford 159)!!!! We have 2 special guests today!!!!!
**User:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4523386
 Professor Ken Goldberg, who'll talk about grasping & Yahav Avigal from Jacobi Robotics - a previous 106B student who converted his final project into a startup (or something like that)! Come to learn more! Very excited!! 


# 3D lecture annotated slides + makeup lecture
**User:** Michael Psenka
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4523555
 Hi all,


Attached are annotated slides from the 3D lecture.


Since we lost a good chunk of time on our 3D lecture and had to rush through the star of the show, I will be giving a makeup lecture next Monday 3/11, 3-3:30pm, in the lab room Cory 105 covering volumetric rendering, NeRFs, and gaussian splatting. Note this immediately follows my 12-3pm office hours in the same room, where I am of course also happy to answer any questions on 3D vision, or even just going over parts of lecture again.

This will not be recorded, so please come at any point 12pm-3:30pm if you would like to cover anything from the 3D lecture!

Best,

Michael
### Comment
**User:** Michael Psenka
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4523555?comment=10450105
Sorry, above doesn't have annotations. This one does.
# Makeup 3D vision lecture (NeRF, gaussian splatting)
**User:** Michael Psenka
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4523617
 (just for email announcement, see pinned comment. Text copied here)

Hi all,

Since we lost a good chunk of time on our 3D lecture and had to rush through the star of the show, I will be giving a makeup lecture next Monday 3/11, 3-3:30pm, in the lab room Cory 105 covering volumetric rendering, NeRFs, and gaussian splatting. Note this immediately follows my 12-3pm office hours in the same room, where I am of course also happy to answer any questions on 3D vision, or even just going over parts of lecture again.

This will not be recorded, so please come at any point 12pm-3:30pm if you would like to cover anything from the 3D lecture!

Best,

Michael
# Discussion 8 Worksheet
**User:** Nima Rahmanian
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4523623
 Very excited to see you all today.
# Laptop repair shops nearby recommended?
**User:** Matthew Sulistijowadi
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4526484
 Does anyone have recommendations for computer repair services nearby berkeley (besides student technology services) that can help a windows laptop (Dell XPS)? I tried a loy of things and it wont turn on. The lights go on and the laptop fan runs for a second then everything shuts off. Its been doing this cycle for the past 45 minutes.
What I suspect is maybe its a storage issue so if I can transfer some data (files) from the laptop to a different drive (while it’s turned off) I assume it should be fine
# Cancelled OH
**User:** Daniel Bostwick
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4528954
 Hi everyone,

I have to cancel my OH today, I can have a make up OH next week. Sorry for the last minute notification.

Daniel
# Expected Deliverables for Project 3
**User:** Karim El-Refai
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4531856
 There has been some confusion on what we're expecting for project 3. We are only requiring that for the turtlebot simulator you implement only dead reckoning and kalman filtering. For the drone simulator we are expecting only dead reckoning and extended kalman filtering (EKF). Implementing the EKF on a real turtlebot is extra credit and not expected.


# Opportunity from Prof. Ken Goldberg
**User:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4539377
 UC Berkeley MS and PhD students are invited to apply to this annual 2-week
summer robotics and entrepreurship school in Denmark (deadline 18 March).
It's essentally free and a great opportunity to meet other students and
get different perspectives. If you're interested, please discuss with
your advisor: as it occurs in the middle of August, it conflicts with PhD
prelims and ICRA paper writing. It is competitive; only two students from
Berkeley are accepted per year.

Two of Ken Goldberg's PhD students attended in past years and had positive
experiences.

=================================

The International Elite Summer School in Robotics and Entrepreneurship

Each year since 2021, 50 top PhD- and Master robotics students from
universities around the world are accepted to participate in an elite
2-week summer school in the beginning of August in Odense, Denmark
(home of Universal Robots). Students pay only for local
transportation and dinners -- flights, tuition, lunch, and lodging
expenses are covered by the program, which includes guest lectures on
motion planning, HRI, medical applications, simulation, and computer
vision and a team research project.

Applications are due on 18 March 2024. Please confirm with your research advisor before applying.

Details:
https://robotelite.sdu.dk/

https://robotelite.sdu.dk/governance/
# Discussion 8 Recording + Notes
**User:** Nima Rahmanian
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4541285
 Dear students,



I have posted my recording and notes on my website here: https://sites.google.com/berkeley.edu/nima-rahmanian/teaching-eecs-106b-sp24.
# Week 9 Announcements!
**User:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4551474
 Good evening everyone!! Hope you've been taking some time to yourself during this busy midterm season. As my little sister would say, I didn't eat on my exams, my exams ate me. I hope your midterms have been going at least a bit better lol! 

I hope you enjoyed the guest lectures last week:) Feel free to visit Michael in OH to learn more about vision, and reach out to Yahav if you have any questions about Jacobi. Lectures this week will be on SLAM! (Does someone smell extra credit cooking?)

Homework is due this Friday, and projects 3 and 4 are due next Friday. While the official due date will remain the same for the course to stay on track, you will be able to submit your reports with no penalty until Tuesday, April 2nd. However, you cannot apply slip days on this. It's just a (really long) grace period to submit the report, in case life gets in the way right before break (I get it). 

Consider yourself warned: please, do not slow down on project work because of this grace period. Project 5 will not have any extensions or grace periods, and it'll probably be nice to have these projects out of sight & mind over break. Also, the proposal for the final project will probably be due the week after the break as well. Don't say I didn't warn you.

Keep your final project ideas top of mind. Start thinking about your groups and remember 106B projects are going to be more research-oriented. Specifications for it will be released as soon as possible.

Make sure you're attending journal club on time & come out to OH with any questions. 

Finally, to brighten up your evening a bit, meet my roommate's dog, Jino. Jino is a menace, but very cute. 
### Comment
**User:** Cecilia Pham
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4551474?comment=10517404
thank you course staff!! <3


# User ee106b-abh Dead Machine
**User:** Daniel Tan
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4555927
 Hi user ee106b-abh. It seems like you were using the Mango machine in the lab and left something running after you left. When I started up the machine it's completely dead and I had to force a power cycle without saving your work.
# Deploy key for GH classroom
**User:** James Ni
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4557191
 Can I get a deploy key on the local machines added to the GH classroom repo for projects 3 and 4, so I can push/pull without granting full access to my GH account?


### Answer 
**Name:** Daniel Bostwick
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4557191?answer=10618563
When I took this class I was able to create a temporary personal access token on the 105 computers, I would speak to your lab GSI next lab meeting or during someone's OH and see if this is still allowed.


# Example presentation slides is gone
**User:** Ethan Pang
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4558890
 Hi I saw that the example presentation slides seems to be deleted/gone from this link https://docs.google.com/presentation/d/1Wk9FhEjvpV5dDpzYsDxFrlN_OdaRhqjCeQHlQ40M3C8/edit#slide=id.p

that's in the Journal Club Presentation Information post. Could this be fixed? Thanks in advance
### Answer 
**Name:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4558890?answer=10789519
Hey - sorry for the long long delay in my response - yeah for some reason it was deleted! I'll put up a diff presentation!
# 3/13/24 Presentation Slides
**User:** Joshua Tsai
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4561883
 Ethan & Josh's presentation slides: https://docs.google.com/presentation/d/1m5O5nUnnpHSFTB7Fr9GNtHXW_EwiGlO4r916OgrLiIY/edit?usp=sharing


### Comment
**User:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4561883?comment=10549846
Hey Josh! Make sure you're posting this link to gradescope 
**User:** Noah Adhikari
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4561883?comment=10550499
They submitted it here since the projector wasn't working today in 105 so the students in our journal club could see the slides


# Homework 3 Q3.1 Question
**User:** Noah Adhikari
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4565292
 I am struggling to understand the solution here. How do you know that $h(q)$ is separable in $q_n$? For example, something like $h(q) = q_1 + q_1 q_2 + q_2 \sin(q_1 + q_2) = 0$. 

Then I don't think you can decouple $q_n$ from $q_0, \dots, q_{n-1}$, so there is no way to uniquely define $q_n$ in terms of $q_0, \dots, q_{n-1}$ (and thus their time derivatives).

I didn't want to comment on the homework solutions thread since it isn't a megathread so it doesn't mark it as unresolved
### Answer 
**Name:** Tarun Amarnath
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4565292?answer=10551094
Just made it a megathread!

You're right that it's not a true statement for Pfaffian systems in general. However, the question does say that you can assume the constraint can be made a function of $q_n$.


**User:** Noah Adhikari
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4565292?comment=10564417
I see, so it's separable since the constraint is a function of $q_n$ and the other constraints are not dependent on $q_n$. Thanks Tarun!
**User:** Tarun Amarnath
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4565292?comment=10565850
Yup, exactly!

# Make sure you're using this repo for project 3 & 4
**User:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4567046
 This is the link: https://classroom.github.com/assignment-invitations/cb8b41fc875400abbf067699c2b288de/team

There was one spot in the project 3 doc where it was linked incorrectly. You should have both project 3 and 4 starter code in there. 


# Launch Issue with roslaunch proj3_pkg unicycle_bringup.launch on Personal Laptop
**User:** Thomas Kragerud
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4569616
 Hey

I tried running the following on my dual-boot MacBook Pro (Intel i7, Ubuntu 20.04) without success, though it works fine on the lab computer:

roslaunch proj3_pkg unicycle_bringup.launch 
 estimator_type:=dead_reckoning 
 noise_injection:=true 
 freeze_bearing:=false


I get the error: 

[roslaunch][ERROR] 2024-03-14 15:28:57,908: OSError(8, Exec format error) [roslaunch][ERROR] 2024-03-14 15:28:57,908: RLException: Unable to launch [unicycle_node-2].
If it is a script, you may be missing a '#!' declaration at the top. [roslaunch][ERROR] 2024-03-14 15:28:57,909: Traceback (most recent call last): ...


I was able to run the simulation in project 2, which suggests it might not be a ROS setup error. I'm considering if it could be related to a potential computer architecture mismatch between the machine that compiled the unicycle_node file and mine. Both my machine and the lab machines are x86_64, but there might be other differences. Looking through the code, it looks like unicycle_node is a compiled Python file. Would it be possible to get that file and try to compile it myself?

Or does anyone have any other suggestions?
### Answer 
**Name:** Karim El-Refai
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4569616?answer=10780443
So sorry for the late response but we cannot give you the unicycle_node source file as that does contain the source code and would allow you to reverse engineer the solution. I am surprised that this is an issue though as I believe the original file was compiled with an intel machine running x86_64. If this has still caused issues with you guys working on the lab please do feel free to make a private Ed post and we can try to accommodate.
# Running a bit late to OH be there soon!
**User:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4574473
 I'll be there by 1:30pm at the latest!
# HW & Project Grades Not Returned
**User:** Sunny ME
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4578839
 Its been quite while since we have submitted HWs and projects. It would be nice if we can get grades back for HWs and labs. 
### Answer 
**Name:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4578839?answer=10617468
We're trying our best! Got Daniel working very hard!
# Robust Force Closure Project 4
**User:** Cecilia Pham
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4580163
 Hi! Would we be able to get some more information on the Robust force closure metric method for Project 4? I and a few others are a little confused on how to proceed forward. I've been trying to figure this out for several days. https://edstem.org/us/courses/54072/discussion/4503027

Thank you so much!


### Answer 
**Name:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4580163?answer=10789466
Sorry about missing this thread - hopefully the hint doc was helpful to get started on this, and if not I'd love to answer any further questions you have!


# Hints for Project 4
**User:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4592191
 Here are some hints: https://docs.google.com/document/d/1cS1ny97Vc3Fk5ip9OsFLHQuguqDHml06Nfqb7lsfiEk/edit?usp=sharing 

Will add more on to the doc as I have time to!
### Comment
**User:** Saajid Ahmed
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4592191?comment=10630259
Compute Robust Force Closure

Create a desired wrench similar to the above

You’ll have a certain number of trials (we have ~500). For each trial, get new normals and new vertices by sampling around the vertices (how would you implement this?), and compute the force closure. Record if it was a success. 

Why would we need to create a desired wrench if we can just call compute_force_closure() on our new normals and vertices?  If we can find that our grasp is in force closure given these new sets of vectors, shouldn't that mean we can resist the effects of gravity?
**User:** Cecilia Pham
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4592191?comment=10630778
I also was unsure of why we would use the wrench function. I just sampled then called the force closure function to calculate the score. Additionally, how would we incorporate the torsional coefficient? I read through the textbook and resources and still don't quite understand.


**User:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4592191?comment=10789862
So, with robust force closure, the idea is to have a certain number of trials and for each trial, get a set of new_vertices and new_normals (you can implement the sample_around_vertices function for this, more on that below). If the new_vertices and new_normals returned by that function are usable, you can add to the total the result of the compute_force_closure function (in which you need to pass in a desired wrench). You'll return the total divided by the number of trials. The desired wrench isn't a separate function but simply the mass of the object times a 6x1 array that accounts for gravity. 

For sample around vertices, you can use np.random.normal and pass in some params to offset a new_vertices array that you can generate through find_grasp_vertices (a util function, you'll have to pass in the object mesh). You'll be able to generate some new_normal by applying normal_at_point (also in utils).

Not sure if any of that helped but please ask any follow-up questions!
### Comment
**User:** Jacob Gottesman
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4592191?comment=10664454
"See the bottom of page two for some of this written out mathematically!"

Where might I find these generously given mathematical expressions?  Sends helps :praige: 


**User:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4592191?comment=10789898
The entire 1.1 section is what's useful here! Lmk if you've got follow-up questions + happy to help outline the constraints we used!
### Comment
**User:** Matthew Sulistijowadi
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4592191?comment=10668461
in def main() at the bottom of grasping.py, we have a code snippet that says that we may or may not find that snippet helpful and that it was supposed to be deleted, was that necessary code we would've had to construct or like a bonus checker?
**User:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4592191?comment=10789919
It's helpful for testing/debugging but the extent to which you'd have had to construct it is hard to say exactly. Is there a particular question you have with respect to that portion of the code?
### Comment
**User:** Charles Paxson
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4592191?comment=10798636
What are A, b1, and b2 supposed to represent in contact_forces_exist?  My best guess currently is that b is analagous to alpha in the project spec, but I don't see why that would need a b1 and b2.
**User:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4592191?comment=10801841
A is representing the discretized friction cone. So it would be something like

[[cos(i*2pi/num_facets)], [sin(i*2pi/num_facets)]] <- this should be a 2x1 matrix

and you're going to want to loop through num_facets (which is what i is), and basically horizontally stack all of these 2x1's to end up with a 2xnum_facets matrix. So think about going through and splitting the friction cone into two halves which is basically what A is.

And then b1 and b2 - the way that I think about it is like the weights that you're applying to get it to hold the constraints that we want it to in this situation. Those are mostly talked about in section 1.1, but I'll also add something in the hints about the constraints for contact_forces_exist as well soon!
**User:** Charles Paxson
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4592191?comment=10803627
If A is the discretized friction cone, then what is fc supposed to be?
**User:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4592191?comment=10822220
Yeah so I guess when I said A is the discretized friction cone what I meant was it's the thing that applies the discretization (in section 1.1 it's what's referred to as F). We're using cvxpy to solve for the actual values of fc (in section 1.1 it's what's referred to as FC). And so b1 and b2 are the alphas that are referred to in section 1.1. Does that help?


**User:** Charles Paxson
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4592191?comment=10827124
That does help a lot, but I'm not sure I understand why there is a b1 and b2 vector, since alpha is just one vector.

Thanks for helping out.  I've been getting pretty confused because there are like three different notations in the resources we've been given.



### Comment
**User:** Matthew Sulistijowadi
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4592191?comment=10828654
in custom grasp planner, what do we use to create the desired 4x4 pose after testing each of the 3 grasps meeting the thresholds? (aka how do we make the 4x4 pose)
### Comment
**User:** Matthew Sulistijowadi
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4592191?comment=10900782
what should the quality be for the compute_ferrari_canny? should it be a decimal value between 0 and 1.0 or what is it
# Week 10 Announcements!
**User:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4594605
 Week 10 already???? I can't believe next week's already spring break, and then there's only like a month and a half before school's over. So wild!!

This week in 106B:

There's no discussion this week! Enjoy the break:)

Thursday's lecture is a guest lecture from Amay Saxena who's at Tesla and previously was a 106B TA. It'll be a great opportunity to get a glimpse into SLAM applied in industry. A great opportunity, so please come out to it!

Project 3 is due 3/22 (with a submission grace period until April 2nd). You may not use slip days for this project.

Project 4 is now extended to April 3rd (since it's a bit tougher than we anticipated). You may use up to 2 slip days on this project.

The policies page says that there will be homework due 3/22. Rest assured there is no homework due then, but we'll probably still have one or two more homework assignments (potentially released this week or the week after break). Keep an eye out for that, and the website will be updated with that information shortly! 

Final project guidelines will be released soon! You should be in groups of 4, mostly. You're encouraged to reach out to graduate students/professors in various groups to help source your project!

Finally, staff went to brunch on Sunday, which was super cute:)


### Comment
**User:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4594605?comment=10617478
Forgot to mention also that Project 1 grades were released! 
### Comment
**User:** Daniel Tan
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4594605?comment=10619100
Will the proj4 due date update be reflected on gradescope?
**User:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4594605?comment=10621099
Yep will do today! 
**User:** Daniel Tan
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4594605?comment=10624292
Thanks! btw will this week's lectures be uploaded during the break?
**User:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4594605?comment=10780050
Lectures should be uploaded!
**User:** Max Wilcoxson
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4594605?comment=10785421
Will Thursday's lecture be uploaded? I only see the lecture from Tuesday (March 19th)
**User:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4594605?comment=10785911
Thursday's lecture was a guest lecture by Amay from Tesla and due to the sensitive nature of the content he requested that it not be recorded.




**User:** Daniel Tan
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4594605?comment=10665933
Any update on this?
**User:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4594605?comment=10780051
Gradescope should be fixed now!

### Comment
**User:** Saajid Ahmed
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4594605?comment=10624934
Is project 4 the last project before the final project, or is there going to be a project 5 + a final project?
**User:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4594605?comment=10780057
Will have an update for you on this tomorrow after staff meeting, but right now it's looking like most likely there will not be a project 5 due to issues with project 4. Edit: we will have a mini project 5, more details about that will be released today.
### Comment
**User:** Nithila Poongovan
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4594605?comment=10657955
"Project 3 is due 3/22 (with a submission grace period until April 2nd)" -> is this grace period for project 3 without penalty? Does this mean that the latest we can submit our project 3 without penalty is April 2nd?


**User:** Emily Park
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4594605?comment=10677750
Bump on this! So we can submit project 3 by April 2nd without using slip days?


**User:** Max Wilcoxson
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4594605?comment=10678154
"Homework is due this Friday, and projects 3 and 4 are due next Friday. While the official due date will remain the same for the course to stay on track, you will be able to submit your reports with no penalty until Tuesday, April 2nd." -- Week 9 Announcements 

So, I think we do not need to use slip days 
**User:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4594605?comment=10780068
For Project 3, no slip days need to be applied. Everyone has a grace period to submit it by Tuesday (4/2). For Project 4, the new deadline is 4/3, and you can use slip days on this project. 




### Comment
**User:** Tianqi Zeng
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4594605?comment=10762709
Just want to make sure that Project 4 could be submitted on April 5th right? However on Gradescope it still shows April 3rd
**User:** Daniel Tan
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4594605?comment=10764666
I think they have not updated gradescope yet, it's supposed to normal due on 3rd and slip days extend to 5th. On gradescope it's still normal due on 22nd and late due on 2nd.
**User:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4594605?comment=10780069
Yup sorry about that just had a chance to update the gradescope!

# Ada gripper doesn't open and close when commanded
**User:** Eddie Shi
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4600187
 When working with Ada this afternoon, our group found that the gripper doesn't open and close when commanded. (Ex: when we run gripper_test.py, Ada only closes and opens for calibration 1 time while a working gripper on a different robot closes and opens 3 times).
### Answer 
**Name:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4600187?answer=10780026
I'll have ESG look into this this week!
**User:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4600187?comment=10784791
Apparently it's working fine now so let me know if you're still running into issues!


# Running Late to Journal Club - Karim
**User:** Karim El-Refai
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4604253
 Hi hi, i am currently traversing all of campus in this beautiful yet hot sun. Will be 6ish mins late. (Wish me luck against hearst hill)
# code clarification/conceptual proj4 compute force closure and get grasp map
**User:** Matthew Sulistijowadi
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4607513
 for both methods mentioned above in project 4,, is it ok and/or expected that we don't use all of the arguments? Looked at pages surrounding 219 for, get grasp map and don't know where I would involve friction or num_facets. compute force closure doesn't use num_facets (we think we can hardcode to just be 2, or object mass. Also wanted to ask if the code we have seems reasonable for the two methods (picture below)


### Answer 
**Name:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4607513?answer=10780108
Answer to your question: Yes - totally expected that you might not use some of the parameters. Num_facets isn't used in this function in our implementation.

I'll circle back to this thread to check your code!
**User:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4607513?comment=10789424
At a high level your code seems on the right track - nice! 
# Project 3 Logistics
**User:** Fahim Yousuf Choudhury
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4608988
 Is the grace period till 2nd April for project 3 meant only for the report ? Or can we also make changes to the code after 22nd March also?
### Comment
**User:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4608988?comment=10780034
Sorry for the late reply - you can make changes to the code after March 22nd!
# Homework 4 submission bug
**User:** Johnny Lau
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4615066
 Apparently, I was checking my homework 4 submission from last week, however my work could not show up except a single page. I handwrote and used camscanner (not able to upload pics to gradescope directly) for the work, and as you can see, it should include eight pages of work. This also happened to me once before. I am confused about what I should do in this situation.





### Comment
**User:** Johnny Lau
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4615066?comment=10664191
As you can see from image 1,  work on the front corresponds to page 6, and page 7 on the flip side. Nothing was changed after the work got scanned


### Comment
**User:** Johnny Lau
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4615066?comment=10664345
Would staff mind help accepting the above work for HW4? I sincerely apologize for the inconvenience. I should have been aware of this problem earlier.......... I've been swamped with other tasks, as I am working in only a group of two for the recent project. I literally went to the lab room for the project that morning, and asked my partner about the assignment question..
### Comment
**User:** Johnny Lau
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4615066?comment=10779714
Hi, may I ask for a follow-up after Spring break if possible? Ty!
### Answer 
**Name:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4615066?answer=10780035
Hey Johnny! Can you please email me a PDF of your homework? kirthi.kumar@berkeley.edu -- I'll upload it manually!
**User:** Johnny Lau
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4615066?comment=10847096
Okay, thank you so much! I will do that tonight.
# Late HW submission?..
**User:** Trinity Chung
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4620437
 hi staff, I know that per policy late work is not accepted.. but I wanted to try asking because I frustratingly did it on time but forgot to submit to gradescope 🫠 which I noticed as I was submitting for journal club presentation today
### Answer 
**Name:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4620437?answer=10780039
Hey Trinity no worries it happens! Can you email me a pdf of your homework? I can upload it manually. kirthi.kumar@berkeley.edu
# Midterm resources?
**User:** Kalie Ching
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4668765
 Are there any resources available for preparing for the midterm? Any tips for how to best prepare? I’ve just been going through the discussions so far.
### Answer 
**Name:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4668765?answer=10784930
Going through the discussions is good. We'll also have a review this week, and then release a list of question topics as well once we've finalized the content of the midterm.
# final project presentation week
**User:** Jiachen Lian
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4669798
 Hey I am just going to confirm when is final week presentation since I am planning for summer plan. Does it end on 5/3?



Thanks!
### Answer 
**Name:** Tarun Amarnath
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4669798?answer=10788739
Yup, you won’t have to be in person past 5/3 for this class 
# [Midterm] Midterm Assignment Released
**User:** Nima Rahmanian
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4669822
 Dear students,



After a long deliberation, we have decided to allow a take-home midterm this semester. I've attached the pdf below. You have three days to submit the assignment. You may NOT collaborate with other students, and you may not ask content questions in OH.



Best,

Nima



Clarification: this is an April Fool's joke. 
### Comment
**User:** Bill Zheng
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4669822?comment=10784885
Real
### Comment
**User:** Matthew Sulistijowadi
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4669822?comment=10784923
damn, we ball >:3




### Comment
**User:** SooHyuk Cho
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4669822?comment=10784976
:O
### Comment
**User:** Colin K.
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4669822?comment=10785020
is this an april fool's joke 
**User:** Jacob Gottesman
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4669822?comment=10785023
Check the last page of the midterm...
**User:** Colin K.
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4669822?comment=10785120
I read the last page of the midterm, I wasn't sure because it was an official announcement from course staff. I've had professors include jokes on exams in the past so I genuinely wasn't sure and was hoping to get clarification from course staff.


**User:** Vishnu Murali
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4669822?comment=10794161
Haha this was me too! Wasn't sure what to think.


**User:** Karim El-Refai
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4669822?comment=10785309
Yes it is a joke. The actual exam is take-home and will be released on April 11th and due the 14th
**User:** Jacob Gottesman
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4669822?comment=10786258
On a side note, how does the actual exam questions compare to this joke one? This one seemed pretty straight forward. 
**User:** Max Wilcoxson
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4669822?comment=10786613
The exam says that this was the midterm last year. I believe this detail is accurate (ie not a joke) 
**User:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4669822?comment=10787099
Yes this is last year's exam. We'll release the solutions soon so you all can use it as practice! 


**User:** Japinder Narula
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4669822?comment=10796388
can we get other practice exams too? sp22 or older that are relevant?
**User:** Tarun Amarnath
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4669822?comment=10796626
DNE unfortunately. Last year was the first time 106B had a midterm. Blame the head TA back then for that 





### Comment
**User:** Alejandro Marquez
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4669822?comment=10785193
Don’t play with my feelings like that😭
### Comment
**User:** Bear Häon
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4669822?comment=10785237
I just had a stroke
### Comment
**User:** Sunny ME
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4669822?comment=10785316
Got me lol
### Comment
**User:** John Vicino
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4669822?comment=10785541

**User:** Tarun Amarnath
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4669822?comment=10796632
this is fine.
### Comment
**User:** Karim El-Refai
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4669822?comment=10785558
Note: Nima still has his OH till noon. This is your chance to get some payback.
### Comment
**User:** Japinder Narula
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4669822?comment=10787730
thanks for the heart attack
### Comment
**User:** Nithila Poongovan
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4669822?comment=10788474
OIASJDFIPSJAFOIPAJSDFOPIDSAJFPOIDSAFJSPAODIJFSAODPJ THAT WAS SO SCARY
# Michael's OH today (Monday) shortened -- 2-3pm
**User:** Michael Psenka
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4669834
 Hi all,

I will only be able to have my OH today 2-3pm -- my apologies for the late notice.



Best,

Michael
# Karim OH Delayed Today, Extra OH Wednesday
**User:** Karim El-Refai
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4672988
 Hi sorry Im running late and anticipate I’ll get to cory around 6:00 but I’ll be hosting an extra OH from 12-1 Wednesday as a result. Really sorry!
# Project 4 Code not working.
**User:** Edward Chang
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4673613
 I can run the files using python, but not with rosrun. I have added file permissions as well as look through CMakeLists.txt and package.xml files for all of my packages.
### Answer 
**Name:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4673613?answer=10822093
Hi! Forwarding this to ESG rn as we talked about in OH:)
**User:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4673613?comment=10848799
Hi! Apparently there are line ending issues. 

That first line should be #!/usr/bin/env python\n (Unix style), but if its Windows style line endings the file will have #!/usr/bin/env python\r\n 

So, you need to save the file with Unix line endings, or convert it -- apparently c105 machines have dos2unix installed for this
# Disrupted Flight plans and request for minor extension
**User:** Vishnu Murali
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4673796
 Hi Course Staff,

I have completed project 3 but I have a few images and files on the lab computers that I need to finish my report. I should have been in Berkeley today to gather the files and submit the report but my flight got canceled today and the soonest flight they could put me on was early Wednesday morning. I was wondering if I could get a one-day extension. I realize that the announcement of the project said it was a hard deadline but under the circumstances, I am not really sure what else I can do.

Thanks,

Vishnu
### Answer 
**Name:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4673796?answer=10798831
This is fine, Vishnu! Please email me your final lab report with your team member's emails as well -- kirthi.kumar@berkeley.edu. Have a safe flight to berkeley:)
# [Homework 5] Problem 1: Image Filtering & Feature Detection
**User:** Tarun Amarnath
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4674734
 Post any questions here!
### Comment
**User:** Akhil Vemuri
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4674734?comment=10888337
For 1.1, if we have $i=-2$ and $j=-2$ let's say, then we would be computing $F(-2, -2)I(x + 2, y + 2)$ according to the formula. But shouldn't we be computing $F(-2, -2)I(x - 2, y - 2)$?


**User:** Tarun Amarnath
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4674734?comment=10909005
You're right, there seems to be a double negative in there, sorry about that
### Comment
**User:** Max Wilcoxson
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4674734?comment=10906025
I understand that convolution is cross correlation (sliding the filter over the image) with a flipped filter across the x and y axes. However, I am a bit confused by the indexing. In 1.1, we take both indexes from [-N, N]. But the underlying F is a Numpy array with positive indices. Do we assume that (-N, -N) in the filter math notation corresponds to (0, 0) in the Numpy array?
**User:** Tarun Amarnath
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4674734?comment=10909010
Yeah, see my answer below - it should be $I(x+i, y+j)$.
### Comment
**User:** Max Wilcoxson
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4674734?comment=10906734
How should we submit the code for the Collab section? Should we copy it into our latex writeup, or just attach the PDF at the end (and match pages appropriately?)?
**User:** Tarun Amarnath
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4674734?comment=10909023
Whatever is most convenient for you! Make sure your final results are integrated into your writeup directly though


# [Homework 5] Problem 2: Feature Extraction with RANSAC
**User:** Tarun Amarnath
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4674739
 Post any questions here! 
# [Homework 5] Problem 3: Least Squares State Estimation
**User:** Tarun Amarnath
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4674741
 Post any questions here! 
# [Homework 5] Problem 4: The Kalman Filter
**User:** Tarun Amarnath
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4674751
 Post any questions here! 
# Week 11 Announcements!
**User:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4675568
 Hey hey everyone! I hope you all had a good spring break & that you weren't tooooo jump scared by the prank this morning:)

A couple of announcements: 

Project 3 - the grace period for submission ends tomorrow night!

Project 4 - due Wednesday! 

Please keep an eye out for a few more hints + will also be responding over Ed tomorrow! Real implementation isn't going to be perfect (or even good -- we know that this project is challenging and we'll be grading it leniently). I'll clarify further on the Project 4 thread. 

The take-home midterm exam will be April 11th-14th. There'll be some review during the discussion this week and once we've finalized the exam we'll release a list of topics as well. In addition, journal club sections that week will be canceled & if you're signed up to present that week you'll just submit the presentation via Gradescope before your next journal club.

Homework 5 will be released soon and due April 16th. Please take a look at it before the exam and come to OH with questions as some of the content will be relevant to the exam. 

Mini Project 5 - we'll have an extremely small tiny project 5 just so you have the opportunity to try out something on the turtlebots this semester. There will be no writeup (only video submission) -- we just want you all to have the opportunity to try it out. It will be released on Thursday and due April 18th. 

Final project proposals will be accepted on a rolling basis, with an absolute final deadline of April 19th. We encourage you to submit sooner to get feedback sooner, as well. More on the proposals will be released later this week. 

These deadlines will be reflected on the website as soon as possible. 

P.S. I was in Pittsburgh for a bit during spring break and came across this cool art installation at the airport. 


### Comment
**User:** Anish Dhanashekar
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4675568?comment=10798975
Thanks! Where can we find the final project proposal guidelines?
**User:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4675568?comment=10801861
They will be released later this week!
# Looking for Final Project Team Member
**User:** Peter Julian
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4680318
 Hi, my group mate (Oscar) and I are looking for 2-3 persons for final project. Please email me at pjulian@berkeley.edu or oscarfx20@berkeley.edu if you are interested in joining our team. We still have not finalized our project yet, but we are open to new ideas also. Thank you
# Homework 4 Solutions Released
**User:** Nima Rahmanian
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4688660
 
# Positive Definite Function
**User:** Sunny ME
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4688908
 Can someone please explain what are positive definite function and matrices. 
### Answer 
**Name:** Tarun Amarnath
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4688908?answer=10831438
Positive definite functions are defined in homework 2

A positive definite matrix $M$ is defined such that for any nonzero vector $z$, $z^T M z$ is strictly greater than 0. 
# 1 day extension for project 4
**User:** Edward Chang
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4690689
 Hello,

I am writing to request a one-day extension for the submission of project 4. Due to a critical equipment malfunction (a faulty gripper on Amir), I was unable to complete the video recording and final testing phases of my work.

I sincerely apologize for the late notice.

Thank you for your understanding and consideration.

Sincerely, Edward
### Answer 
**Name:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4690689?answer=10835581
Hey Edward, you can use slip days to submit until Friday but if you need an extension beyond that please let me know!


**User:** Edward Chang
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4690689?comment=10836683
That's great to hear! Thank you :)
**User:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4690689?comment=10876637
Hi Edward - sorry for the reply, but yes you can have that 24-hour extension!

**User:** Edward Chang
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4690689?comment=10859725
Hi Kirthi, could I get a 24 hour extension? I just need a bit of time tomorrow to put everything together. Thanks!
# Submitting Project 3
**User:** Ethan Pang
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4690734
 Hello, my group and I (Ethan Pang, Joshua Tsai, Josh Kavilaveettil) mistakenly assumed that Projects 3 and 4 would be due at the same time given that the original due dates (3/22) were the same. I see that the Gradescope assignment for Project 3 is closed, but would we be able to still submit for this late?  Thanks in advance


### Answer 
**Name:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4690734?answer=10835589
Hi Ethan, we made it pretty clear that it would be due on the 2nd. However, I understand that mix-ups happen, so please email me your report by tonight, kirthi.kumar@berkeley.edu. Thank you! 
# Project 4 Integration with pick_place.py
**User:** Sergio Peterson
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4690805
 I know we have to use custom_grasp_planner(object_mesh, vertices) on pick_place from lab 5 of 106a, but how do we get the object_mesh and vertices of the object to pass through the custom_grasp_planner? 
### Answer 
**Name:** Karim El-Refai
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4690805?answer=10876860
The objects are the pawn and nozzle which have the object meshes and vertices you need. You can then find the pose for an ideal grasp and you’ll have to transform this grasp pose from the object frame to the robot’s base frame. This can be done by placing an AR tag a known distance away from the object and finding the transform between the ar tag and the robot base. Alternatively you can also move the gripper right above the object and then do a manual translation between the gripper and center of object.
# Wrong submission for gradescope
**User:** Eddie Shi
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4690925
 I accidently submitted a tax document to a gradescope assignment (sorry, was tired). I resubmitted the actual project report, however, I can still see and download the first submission (tax document). I don't want other people (aka course staff) to see my tax documents and I don't think I can delete the submission as a student. I would greatly appreciate if someone on course staff could delete the submission for me. Thanks.

Info: My first submission in Gradescope assignment for project 4 report


### Answer 
**Name:** Tarun Amarnath
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4690925?answer=10844952
So I deleted your group's entire submission for project 4. I hope that deleted your submission (it's the only button even we have access to for deletion). Could you resubmit your project 4 report for your whole group?

To be honest, we don't look at submission history, only the latest assignment. I doubt anybody will see it. 


**User:** Eddie Shi
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4690925?comment=10845826
Thank you. Although I still can see the first two submissions from my side lol. It should fine.

Just to clarify, this won't use a slip day (resubmitting the exact same report as the on-time submission)?
**User:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4690925?comment=10848806
Yep - this won't use a slip day

# Big Night for x06B Fam <3 !!
**User:** Bear Häon
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4691222
 
### Answer 
**Name:** Anish Dhanashekar
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4691222?answer=10836841
It really is one of the nights of all time.
### Answer 
**Name:** SooHyuk Cho
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4691222?answer=10841237
Cook it sawyer
**User:** Sameer Nayyar
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4691222?comment=10849753
project really got cooked
**User:** SooHyuk Cho
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4691222?comment=10849840
Fr

# "Rethink"-ing Robotics
**User:** Anish Dhanashekar
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4692008
 Roses are red

Violets are blue 

When sim-to-real isn’t doing it for you

Catch the sunrise with the 106B crew


### Comment
**User:** Anish Dhanashekar
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4692008?comment=10838503
Project 4 is tough but the true reward is the friends made along the way :)
### Comment
**User:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4692008?comment=10839101
omg 😭 
### Comment
**User:** Tarun Amarnath
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4692008?comment=10844987
The poem you provided is a variation of the classic "Roses are red" poem structure. It's often used for light-hearted or humorous purposes. This particular poem seems to be a playful nod to the popular "Roses are red" format, incorporating a reference to "sim-to-real" and "106B crew," which suggests a more technical or niche context, possibly related to a specific group or community.


**User:** Anish Dhanashekar
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4692008?comment=10845119
I see 189 wasn't capping about the LLMs on Ed.
**User:** Tarun Amarnath
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4692008?comment=10845206
You should sleep, it's good for you after this late-night grind!


**User:** Anish Dhanashekar
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4692008?comment=10845238
Unable to fall asleep, currently rethinking robotics as indicated above... Have you ever rethunk robotics Tarun?


**User:** Tarun Amarnath
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4692008?comment=10845253
Actively rethinking robotics right now
**User:** Anish Dhanashekar
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4692008?comment=10845309
Amazing, keep me posted about your conclusions - we're making it out of Cory 105 with this one.




# Project 4 Debugging help.
**User:** Edward Chang
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4696585
 I'm trying to use multithreading to calculate the grasp data, and then save it to a file so I don't have to recalculate every time. I keep getting this exception. Is this a cause for concern? Also, even though I ensure that my threads write to different locations, and that there are only 2 active at any given time, I keep getting segfaults. I'm confused but I don't want to give up on this
Edit: Added a picture of where I am threading.
### Comment
**User:** Edward Chang
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4696585?comment=10848933
correction: 1 main thread and 2 additional threads. so maximum of 3 threads at a time.
# Project 4 Extension
**User:** Japinder Narula
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4696905
 Hello,

Would it be possible for our group to get a 1-2 day extension on project 4? we used our last slip day today but we had a lot of inexplicable bugs (like utils.py not working??) and with limited time with sawyers, it seems impossible to finish in the next 5 hours. Also, one of our group members  (me) had to unexpectedly go to the health centre for a few hours yesterday which didn't help our progress...

It would be greatly appreciated if we could be granted a bit more type to deliver our best work (and preferably work under a bit less stress).

Thank you!
### Comment
**User:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4696905?comment=10849737
Hi Japinder, your group can have an extra day - is that okay?
**User:** Japinder Narula
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4696905?comment=10849804
Yes, I think that hopefully should be enough. Thank you!
### Comment
**User:** Japinder Narula
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4696905?comment=10894344
Just wanted to clarify, does my group need to do anything additional so we aren't marked late? We submitted Friday night, as per our extra 1 day beyond our 5 slip days,
**User:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4696905?comment=10910910
Nope - that's fine. We'll be sure to account for this! 
# Week 11.5 Announcements!
**User:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4696963
 Hey everyone! I'm back on a Thursday to say two things:

First, I'm super excited to announce that we're canceling the take-home midterm! We think the homework and projects are better ways to learn in this course than the midterm - so hopefully this helps alleviate some stress caused by Project 4 and the upcoming deadlines. (I promise this is not a late April fools joke or anything like that!) As a result, here are some administrative items related to that:

We will have a synchronous journal club next week (this is different from what was announced on Monday). If you're signed up to present, please be ready.

We will have a discussion next week - we'll be going through a special vision problem that was going to be on the midterm! You should plan to go to discussion next week, as we'll be taking attendance as well. 

We will be having a small homework 6 on optimal control and RL.

Secondly, HW 5 is released and due on April 16th, please use these threads for questions:

Problem 1

Problem 2

Problem 3

Problem 4

We'll release mini-project 5 and final project specifications tomorrow! The deadlines for these two assignments will remain unchanged. Hope you have a great rest of your evening:)
### Comment
**User:** Max Wilcoxson
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4696963?comment=10857907
Sorry if this is written somewhere else, but I didn't see it on the homework itself or website, and I couldn't find a "Homework 5 Released" post with the information. When is Homework 5 due? Thanks!
**User:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4696963?comment=10860412
This is the homework 5 released post! Homework 5 is due April 16th.
### Comment
**User:** Saajid Ahmed
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4696963?comment=10876976
Hi, have discussion 9 solutions been posted anywhere? Thanks!
**User:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4696963?comment=10912007
It'll be posted ASAP!
### Comment
**User:** Akhil Vemuri
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4696963?comment=10894388
Could the gradescope assignment be released for HW 5?
**User:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4696963?comment=10912004
I'll do it tomorrow!
# Left umbrella in lecture
**User:** Kevyn Ramirez
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4697195
 Hi I left my umbrella on the floor during lecture, is there any specific place I should go to pick it up?
# Unable to connect to move_group action server
**User:** Alejandro Marquez
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4698757
 Hello, my team and I are getting this weird error where we are able to run our code to move the robot sometimes but then randomly we will get the error “unable to connect to move_group action server” and we have to restart the computer for anything to work again. 

We have tried looking at the labs from 106a, other edstem posts, and previous projects to see if there is anything that mentions this but could not find anything. 

Once we get this we are not able to do anything with the robot including tucking and we have to run pkill
# Project 4 Extension
**User:** Sergio Peterson
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4703626
 Would it be possible to use an extra extension on project 4, my group an I are stuck on the code, and would like to not submit with what little we have.
### Answer 
**Name:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4703626?answer=10876654
Hi Sergio,

Please let me know if you need an extension beyond using up your remaining slip days!
# Potential Extension for Proj-4
**User:** Jiachen Lian
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4704244
 Dear instructors:

    We almost finished the proj-4. But we feel like we might need 2-3 more hours to wrap up the report. There are a couple of trivial reasons. I personally attended my conference in last two weeks, which makes my two other teammates have a harder time. Also, there are some issues for Sawyer, and most of time, there will be a "time-out" bug so that robot will lose connection. We have to restarted the desktop. We contacted GSI but it seems no reasons was found. But even though, we almost finish it, just want 2-3 more hours! We have three slip days left. Although it is only allowed that 2 slip days can be applied for each project, can we still possibly make a special request for our case? 

Best

     

    
### Answer 
**Name:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4704244?answer=10876665
Hi Jiachen,

We've now allowed everyone to apply more than 2 slip days, but please let me know if you need a further extension. 
# Three Slip Days for Proj 4?
**User:** Kai Xu
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4704325
 Hi, I heard that we can use 3 slip days on Proj 4, so we can submit it tomorrow. Can I get a confirmation on this? Thank you so much!
### Comment
**User:** Edward Lee
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4704325?comment=10867219
Please let this be true 🥺 
### Answer 
**Name:** Sergio Peterson
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4704325?answer=10867432
Please
### Answer 
**Name:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4704325?answer=10870316
The extension/slip day rumors that go around in this class are really interesting! But yes you can use all your remaining slip days on this project.
# proj4 extension
**User:** Matthew Sulistijowadi
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4704430
 I was wondering if our group could possibly have an option for an extension on project 4, we haven't used any of our slip days yet so maybe could use some of the other 3 for it? We've been stuck on getting the simulation to work on getting the 4x4 pose custom_grasp_planner returns. We've tried using look_at_general (problem is that it's built for 1x3 matricies), get_gripper_pose (for some reason have a shape error) and making our own rotation and transform matrix which is inconsistent at returning valid poses. 


### Answer 
**Name:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4704430?answer=10876674
Hi Matthew,

We've now allowed everyone to apply more than 2 slip days, but please let me know if you need a further extension. 
**User:** Matthew Sulistijowadi
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4704430?comment=10903383
I and one other groupmate were completely bundled by cs189 and even working on it over the weekend, couldn't fix anything without OH clarifications today so we wanted to ask if we could have a further extension
**User:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4704430?comment=10910296
We are having a hard stop to this project today. Please submit whatever you have.

# Can Use More Than 2 Slip Days For Project 4
**User:** Karim El-Refai
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4704576
 We’ve lifted the 2 slip day restriction for Project 4. This means that if you have 5 slip days you can submit the project on Monday night without penalty. Note: We can’t give any more extensions for this project. Hope it helps!
### Comment
**User:** Colin K.
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4704576?comment=10867608
How does this impact students with DSP assignment extension accommodations?
**User:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4704576?comment=10867724
DSP students will have their usual allotted extension, plus as many slip days as they have left.

For example, say you have all 5 slip days left, then you could turn it in on Wednesday. However, you’re encouraged to turn it in sooner rather than later. 

Sorry I realize I miswrote last night and we're trying to get all Project 4s submitted by Monday to keep the class on track. Here's the true follow up:

We usually have DSP extensions as 2 days for every project (slip days don't come into play). Instead of the slip days situation, for this assignment, DSP students will be allowed to submit up to the full slip day deadline, so essentially you're welcome to submit by Monday (4/8) by 11:59pm. 

Regardless, as always with students with DSP accommodations, if you need further extensions please email me and I’m happy to talk further. 
### Comment
**User:** Akhil Vemuri
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4704576?comment=10874082
Looks like the gradescope assignment is closed. Can it be re-opened?
**User:** Karim El-Refai
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4704576?comment=10874347
should be updated!
# Project 4 Debugging
**User:** Emily Park
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4708998
 Hello,

One of the consistent errors we're getting for our project is: 

raise error.SolverError(
cvxpy.error.SolverError: Solver 'ECOS' failed. Try another solver, or solve with verbose=True for more information.


I was wondering how to solve this bug? It comes up after the first 5 images of pawn/nozzle shows up. The code that is giving the error is:

prb = cvx.Problem(cvx.Minimize(cvx.norm(G @ f - desired_wrench, 2)), constraints)
prb.solve()

### Answer 
**Name:** Karim El-Refai
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4708998?answer=10908773
So sorry for the delay but the first 5 images are poses we give you that are supposed to be pretty good grasps. The reason why you're getting this error is because when you're trying to solve the optimization problem for contact_forces_exist() your optimization failed. This is likely due to an issue with how you've set up the system. Make sure to double check your constraints such that they aren't constraining the problem to be infeasible and that the function you're optimizing makes sense. feel free to comment with more questions!


# Final grades for the class
**User:** Harshika Jalan
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4720719
 Hi,

I was going through the final project guidelines and I noticed that the project report is due on 5/10. When will the final grades for the class be posted on Calcentral? I really need the grades to be posted by 5/12-5/15 at the latest. This is because I requested an expedited degree conferral to apply for my H1b work visa and my advisor said that they won't be able to confer the degree to my transcript till the final grades for all the classes that I am taking are available. Would it be possible to post my final grade by 5/12? 



Thanks,

Harshika
# Final Project Guidelines
**User:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4723645
 Final Project Guidelines

Major Deadlines

Proposal - 4/19*

Presentations - 5/2, 5/3

Final Report - 5/10

*These proposals will be accepted on a rolling basis. You're encouraged to submit sooner rather than later.

People to reach out to for resources/ideas for fifinal projects:

(Learning) Kaylene, kaylene@berkeley.edu

(UAVs) Daniel / Eric (daniel.k.bostwick@berkeley.edu, ekberndt@berkeley.edu)

(CBFs) Jason (jason.choi@berkeley.edu)

(Jacobi Robotics Resources) Yahav (yahav@jacobirobotics.com)

(ROAR) Chris (chrislai_502@berkeley.edu) / C.K. (ckwolfe@berkeley.edu)
### Comment
**User:** Matthew Sulistijowadi
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4723645?comment=11003941
could there be like a separate thread for people to see what other groups are doing for final project idea or look for more members for current group?
**User:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4723645?comment=11030823
Why not this thread?:)
### Comment
**User:** Jackson Hilton
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4723645?comment=11060225
What is the full grading breakdown for the project? The guidelines rubric only adds to 75%...


**User:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4723645?comment=11095967
Sorry about the confusion on this -- I took out a legacy component of the project and did not redistribute the weights.

The distribution will be as follows: 

Project Proposal - 10%

Final Presentation - 30%

Final Report - 60%
### Comment
**User:** Anish Dhanashekar
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4723645?comment=11100305
My final project group has five people in it - could the Gradescope submission for the proposal be modified to be able to add five people? Thanks!
**User:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4723645?comment=11109570
Done!
# Mini Project 5 - Released!
**User:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4723675
 Last one of the semester! Here's the lab doc! 

It's due 4/18 but you'll have 2 free slip days. You're encouraged to submit this project ASAP so you can fully focus on your final project. This project has been scaffolded a lot more than other projects, so I really hope everyone can have fun with the Turtlebots:)

Please remember not to book the robots during lecture time (Tu/Th 2-3:30pm) - this is our scheduled maintenance time and if you're using them then, you will be asked to save your work and log off the workstation. In addition, please clean up after yourself in the lab room (especially if you're eating - which you're not allowed to be - please clean up your trash, it's so not nice). 


### Comment
**User:** Max Wilcoxson
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4723675?comment=10941874
With respect to the linearization plots, is fine as long as the x/y match the demo plot, right? Since there are no constraints on the other variables? Our solution has a V curve that is roughly the negative flipped of the reference, so our heading is also rotated by about pi radians (pointing the other direction) 
**User:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4723675?comment=11094856
This should be fine -- as long as it ends up with a Turtlebot behavior that's what you would expect. But to be honest, it's a bit tricky to debug exactly what's going on without taking a look at your code.


### Comment
**User:** Zekai Wang
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4723675?comment=11010042
Just a reminder that the LiDAR on the turtlebot Apple sometimes doesn’t spin. I had to manually rotate it to make it start spinning. 
### Comment
**User:** Charles Paxson
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4723675?comment=11057853
Is there a reason that the Gradescope submission is limited to two people for submission?
**User:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4723675?comment=11084541
No sorry that was a mistake


### Comment
**User:** Jiachen Lian
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4723675?comment=11064699

Hey we are stuck here. All turtlebots convert first and diverge finally (all go up or down). Do you have any suggestions?  Also znorm is so large finally (200+).  and phi will go from 0 to 30000 suddenly after 9 seconds. 

Also to confirm, we don't need this constraint in the implementation right? 




**User:** Tianqi Zeng
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4723675?comment=11078409
Similar problem with us. We tried to tune k1 and k2, but the graph looks way different than the graph in the document
**User:** Jiachen Lian
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4723675?comment=11082508
We solved the problem by correcting the shape of A matrix in equation 6, which should be 2,2 and ours is 2,2,1
**User:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4723675?comment=11095308
If you're still having issues with this, can you make a private post? It would be easier to debug knowing the k1 and k2 y'all are using, we can give you more in-depth hints!

### Comment
**User:** Noah Adhikari
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4723675?comment=11086481
Sorry if we're missing something, but on this part of the spec:

How do you know what $v$ is from $z$? $z$ only gives you the accelerations $\ddot{x}$ and $\ddot{y}$, and $v$ is the input velocity, but we need to know the value of $v$ in our flowchart to determine that. So it feels a bit circular and we don't know what to do.
**User:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4723675?comment=11095615
You should be able to get these values from things like self.trajectory.get_state , self.observer.get_state , or self.observer.get_vel , or other similar methods.

I'd recommend looking at the starter code first because it outlines a lot of the implementation steps more in depth -- but lmk if you've got follow-up questions!


**User:** Noah Adhikari
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4723675?comment=11096386
It's okay we got it! For anyone else wondering, $v$ is the norm of the xy-velocity

### Comment
**User:** Noah Adhikari
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4723675?comment=11086492
Our control law really seems to want to circle around (5, 4) no matter what we try. Has anyone else run into this?


**User:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4723675?comment=11095618
If you make a private post I could take a look! 
**User:** Noah Adhikari
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4723675?comment=11096380
It's ok we got it!

### Comment
**User:** Enyang Zou
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4723675?comment=11096185
When we try to run python3 main.py on the bot, it returns this.

It's weird because we actually can see the odom frame in rviz.

The code generating this problem is not the part we have revised. It is the state_estimation.py

Has anyone run into the problem?
**User:** Enyang Zou
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4723675?comment=11096339
It's solved. If anyone run into this problem, you can open rviz and choose the odom frame there, and open a new terminal to run main.py.
# Week 13 (oops) Announcements!
**User:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4723715
 Hi everyone! I hope you had a great solar eclipse moment today (I was inside most of the day working on Project 5 so I missed it, but my mom sent me videos of the NASA livestream). 

Major things happening this week:

Tuesday and Thursday are both guest lectures, so please do come out! Tuesday's lecture is by Prof. Allen Yang and is titled "Bridging Artificial General Intelligence and Artificial General Autonomy via High-Precision Digital Twin Modeling and New Human Interaction Modalities" and Thursday we'll have the AI Racing Tech group come in to talk about SLAM (it's a great opportunity to talk about final project stuff with them)

Speaking of final projects... guidelines are released! See this thread for more. The proposal is due 4/19, but you're encouraged to submit it sooner for feedback. 

We have one last project -- a shorter one for some exposure to CBFs and Turtlebots. See this thread for more. This is due 4/18, with 2 free slip days since everyone probably used up their slip days on the last project. But, you're encouraged to submit sooner so you can focus your efforts on the final projects. 

Discussion this Friday will be on CBFs, and paper presentations/journal clubs are on for this week!

After all this Project 4, I hope that the cancellation of the midterm was at least some good news:) We said we'd have a small homework 6, but we are removing that assignment to give you all more time for final projects. Proposal and project gradescope assignments will be created tomorrow. 

P.S. This week's weather is gorgeous. Get some sun!!!
### Comment
**User:** Encheng Liu
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4723715?comment=10924297
Hi, just a quick question. Could our project be just on software sides? Because for ROAR project, it's hard to operate on a real vehicle especially for a short time. 
**User:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4723715?comment=10925188
Projects that are just software-based will be approved on a case-by-case basis. Most ROAR projects will most likely be approved anyways, but make sure you get your proposal in ASAP in case there are some improvements to be made before the final deliverable!
### Comment
**User:** Jacob Gottesman
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4723715?comment=10929518
Is discussion attendance still being taken this week (as per Week 11.5 Annoucements)/ are we required to go?


**User:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4723715?comment=10929943
No, we had a change of discussion topic. but you're encouraged to go! It's Tarun's last week of discussion ever:')


# Prof Sastry's Office Hours
**User:** Vishnu Murali
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4726221
 Hi Staff, a few students are at Cory 333B for Prof Sastry's office hours right now but the prof doesn't seem to be here. Are we in the wrong place?


### Answer 
**Name:** Tarun Amarnath
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4726221?answer=10920922
You were in the right place, but it's possible that he's unavailable today. Did he end up showing up?
### Answer 
**Name:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4726221?answer=10921478
I think he's out of town this week - sorry for the confusion!
# Guest Lecture by Prof. Allen Yang Right Now @ Mulford 159
**User:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4727699
 It's titled "Bridging AGI and Generative Autonomy via High-Precision Digital Twin Modeling and New User Interaction Modality" -- a super hot topic at the moment! Please come out! 


### Comment
**User:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4727699?comment=10922088
For those in the lecture, please use this link: https://berkeley.zoom.us/j/4530888248
**User:** Matthew Sulistijowadi
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4727699?comment=10922176
Is this being recorded still or nah 😅
**User:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4727699?comment=10922205
Unfortunately no - the audio will be though, we can release that

# Lab/HW Grading
**User:** Jacob Gottesman
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4731393
 Now that we don't have a midterm and the grade breakdown percentages have been rearranged to account for that, I'm a little worried about surprise/unexpected grade changes.

Currently, we (at least I) only have grades for HW 1 and Project 1. I'm a little worried that all the remaining HW/project grades will come out all at once and my grade will suddenly change. When can we expect these grades to be released?

On a side note, I do believe holding back our grades since Jan 24 (when Project 1 was graded) is a little unfair as I have no idea how well I'm doing in the class and will have no chance to work to change my grade at this rate. I'm curious what other students think.


### Answer 
**Name:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4731393?answer=10930064
I completely understand where you're coming from on this - and would feel absolutely the same way as a student (and did tbh when I took it as a student - this class is understaffed even with the addition of extra staff from last year). We're doing our best to try to balance student support in the moment and grading at the same time. I'm working with all of staff to have many of the pending grades released over the next few weeks.

For what it's worth, the midterm grade percentage was redistributed equally into the other grading bins (other than journal club), so in the end I'd say your final grade likely isn't impacted too much by this change. However, I completely understand where you're coming from and we're trying our absolute best to make grades come out ASAP so that everyone has a better sense of how they're doing!
# Discussion Moved to 7-8pm in Cory 293
**User:** Tarun Amarnath
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4743119
 Hi everyone,

I am running the EE/CS Town Hall today from 4-5pm, which conflicts with our normal discussion section. I'd highly encourage all of you to come out to it - it's your chance to have a conversation with EE/CS administrative leaders and faculty members about classes, research, enrollment, the new college (CDSS), a potential new ECE major, and any other questions you might have. More details are here (if you're not in EECS 101, here is the invite link). (Shameless self-plug I know but it's a wider EECS event that I hope will be fairly worthwhile)

Regardless, I will host an alternate section from 7-8pm in Cory 293. Feel free to come out to that instead! Tomorrow's 11am section will happen as normal. 


# Missing journal club
**User:** Thien An Phan
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4743847
 Hi, my journal club date is on Wednesday from 2-3:30 but I have been sick since Tuesday until today and I couldn't make it yesterday. May I ask if it's finr if I missed out on one journal club? Thank you.
### Answer 
**Name:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4743847?answer=11009840
Hi Thien, that's fine - please refer to #70 for how to make up attendance - let me know if you have any questions
# Few mins late to OH
**User:** Tarun Amarnath
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4743879
 I will stay over time!
# Come out to lecture @ Mulford 159 right now to get a final project!!!
**User:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4744454
 ROAR is presenting and offering some final projects - you should come out to hear about their team and all the work they're doing! 


### Comment
**User:** Nithila Poongovan
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4744454?comment=11056212
Is there a list of final projects offered by ROAR somewhere that we can see?
# Karim's 5-6 Journal Club Moved to Zoom Today
**User:** Karim El-Refai
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4752753
 Please join at this link: https://berkeley.zoom.us/j/9270698618


### Comment
**User:** Jiachen Lian
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4752753?comment=10978641
Hey. Thanks for informing. Does that mean only GSI is on zoom or all students will be on zoom during the session?
**User:** Karim El-Refai
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4752753?comment=10978704
The students who are presenting should be on Zoom at minimum (so I can see it lol). It's fine if the presenters want to do Zoom from somewhere outside of the lab and as a result there's no guarantee that the presenters will be in the lab so I recommend all students join on Zoom. If you want to still come by the lab and see if the presenters are doing it in person as well which would be great! 
**User:** Jiachen Lian
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4752753?comment=10978732
Oh sure. We will present on zoom in 105.

### Comment
**User:** Japinder Narula
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4752753?comment=10978860
I missed my journal club section on Wednesday (2pm) due to a doctor’s appointment. Can I attend today’s as a makeup?
**User:** Karim El-Refai
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4752753?comment=10978862
Sure


**User:** Japinder Narula
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4752753?comment=10978872
Great, thank you

# Project 5 Conceptual
**User:** Michael Campos
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4753848
 Inside the the TurtlebotBarrier, why does the eval function compute dot from the system dynamics and not from observerEgo.get_vel()? 
# Team Change for Project 5
**User:** Fahim Yousuf Choudhury
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4758846
 Hello,

I changed teams for project 5 but I don't know how to access the github repo for my new team. Could you add me to my new team's repo ? The name of the new team is "eclipse".

Thanks.
### Comment
**User:** Tarun Amarnath
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4758846?comment=11015031
Seems like the team eclipse has not been created for Project 5 yet. Could you do that first?
**User:** Fahim Yousuf Choudhury
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4758846?comment=11030010
Yeah the team repo for project 5 has been created now.
# Final Project Hardware Questions
**User:** Derek Guo
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4759105
 My final project group came up with a few different project ideas but each one of them brings a few questions about hardware:

Idea 1: Modifying DexNet, testing on Sawyer: We are concerned the Sawyer will not be able to run our model. Is it possible to run the model on a separate computer and send the results to the Sawyer to execute? Are there any instructions on how to do this?

Idea 2: Grasping with multiple turtlebots: Are we allowed to use multiple turtlebots? Is it possible to control multiple turtlebots from a single script?

Idea 3: RC car with shifting mass to take tighter corners: Do we have any hardware or a hardware budget for constructing a small, fast, wheeled robot?
### Answer 
**Name:** Karim El-Refai
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4759105?answer=11017661
For idea 1: you can do the same thing as project 4 right where all you really the sawyer for is to execute a given pose/trajectory of poses. you can totally run all the computation for this on another computer and then tell moveit to execute these poses for you

Idea 2: controlling multiple turtlebots with one scripts is trickier. I would ask David Au since Im not familiar with how the networking is set up for that. As for using multiple turtlebots to be honest in the past students havent really used the turtlebot much for their final projs so I feel like you may be able to get up to 2 but this is going to have to depend on how many teams want them. 

Idea 3: I believe the hardware budget is $50 but all the ordering needs to go through David Au @Kirthi can you verify?
### Answer 
**Name:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4759105?answer=11037910
#211 <- for details about hardware + software 
# Homework 4 Solutions
**User:** Tarun Amarnath
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4765273
 
### Comment
**User:** Noah Adhikari
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4765273?comment=11139670
Hi, what would constitute partial credit for question 3? I feel that my work deserves more than 0 points (which would be the same if someone had left the question entirely blank). I didn't calculate it for a frictionless point contact, but for a general wrench basis, oops, but I still feel like I should get some credit.
# Late to OH today
**User:** Nima Rahmanian
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4768250
 Hi students. I will be late to OH today. If you were not able to catch me, please email me and I will schedule a make up time with you!
# No OH today (Michael, 12-3pm)
**User:** Michael Psenka
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4768316
 Hi all,



There will be no OH today 12-3pm, apologies for the late notice. Feel free to send me any questions about final project proposals over email  -- psenka@eecs.berkeley.edu. 



(please start the email title with [eecs 106b] :) ) 



Best,

Michael
# Project Party Today Moved to Zoom
**User:** Karim El-Refai
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4768939
 I am unfortunately stuck in an airport during 6-8pm today so if you guys have any questions please email me during that time span and I can open up a zoom call and try to help out. I know it's not ideal so I'll be holding extra OH later in the week to help make up for it


### Comment
**User:** Bill Zheng
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4768939?comment=11026498
Do you have a zoom link so that we can join?
**User:** Karim El-Refai
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4768939?comment=11026548
https://berkeley.zoom.us/j/9270698618
# Looking for one final project group member
**User:** Kalie Ching
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4772293
 Hi everyone, my group and I are looking for one more person to join our group for the final project. Please email me at kalieching@berkeley.edu about finalizing the team!


# Week 14 Announcements!
**User:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4774733
 Hi everyone - hope you had a lovely Cal Day!

Tuesday’s lecture will be about soft robotics and Thursday’s will be a guest lecture from Prof. Sreenath. Come out!!

Homework 5 is due 4/16!

Mini Project 5 ( thread ) is due 4/18 with two free slip days

Final project guidelines thread - the proposal is due 4/19, but you’re encouraged to submit it as early as you can! 

Requests for final projects should be submitted as early as possible

Software: https://forms.gle/JWeFuk36G3w4V1J86

Due 4/26 @4:59p

Hardware: https://forms.gle/d1HB4yYhFZRGXibYA

Budget: $60/team

Due 4/22 @11:59PM 

Discussion this Friday will be an overview of the course content & the last one so you should come!!

Be sure to do your paper presentations!

More homework has been graded. We’ve released 2 & 3 (thread for sols + distros) - more to come later this week!

Good night!
### Comment
**User:** Daniel Tan
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4774733?comment=11040915
Is there a deadline on hw regrades?
**User:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4774733?comment=11043114
We'll have them open until the Friday of RRR week! 
### Comment
**User:** Max Wilcoxson
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4774733?comment=11042103
Will grade distributions for projects be released?
**User:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4774733?comment=11043115
Yeah as they're graded - I'll put them in that same thread! 
### Comment
**User:** Charles Paxson
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4774733?comment=11075741
Can the Project 5 Gradescope submission be opened to allow more than groups of 2?  I worked in a group of 3.
**User:** Anish Dhanashekar
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4774733?comment=11084031
Same!
**User:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4774733?comment=11084260
Fixed! Sorry about that!
**User:** Anish Dhanashekar
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4774733?comment=11084746
Thanks!


# Looking for Final Project Team Member
**User:** Minh Nguyen
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4777612
 Hi,

Our group is looking for one team member to join our final project group. We'll most likely be deploying FPGAs on small quadrotors so please reach out to minh02@berkeley.edu if you're interested.
# Tuning k1,k2 on project 5
**User:** Thien An Phan
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4780354
 Hi, i’m stuck on the deadlock for project 5 because it seems that the robot always turns around instead of returning to it’s original trajectories for some reason and we just hoping to get if this is a tuning issue or not. 

For the 1st image is k1: 4.75 and k2:11.5
For the 2nd image k1:4.55 and k2:0.55


### Comment
**User:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4780354?comment=11095941
Try a k1 and k2 of 4, 4 -- if that doesn't work there may be a small bug with your implementation and I'm happy to take a closer look
# Discussion @ Exactly 11 Tomorrow
**User:** Tarun Amarnath
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4797618
 Not 11:10 because I have a meeting to run to at 12!

(It will be recorded though! Also a super chill discussion.)


### Comment
**User:** Anish Dhanashekar
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4797618?comment=11084017

# Request from Prof. Ken Goldberg
**User:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4797831
 The terrific @IEEE Robots Guide with the latest news and thousands of robot
profiles is a finalist in the Science Category for @TheWebbyAwards : please
take 15 seconds to vote — today is the last day!

https://vote.webbyawards.com/PublicVoting#/2024/websites-and-mobile-sites/general-desktop-mobile-sites/science

(Please forward to your colleagues and students if possible!)

Here's the guide:
https://robotsguide.com/
# ROAR / UAV Final Project / Research Offering
**User:** Eric Berndt
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4798394
 I realize this is quite last minute, but if you are racing autonomous cars at 170mph or hyper maneuverable autonomous drones we have two research projects (potentially final projects) listed below

Interest form for the UAV (tilt-rotor - this is fullstack robotics research to build a neuro-symbolic drone and do sim to real transfer onto a drone that can tilt all four rotors to turn faster - we have several groups from perception, path planning, control, simulation, and hardware) project (Sastry Lab) 

ROAR (Indy Autonomous Challenge - https://www.airacingtech.com/ 170mph autonomous IndyCar racing - subteams are Simulation/Vehicle Dynamics /Localization/Planning/Controls/Perception) project (Sastry / Allen Yang).

We will get back to you as soon as possible via email as we are currently on a work trip to CMU.

Fill out the form here, and we will get back to you via email as soon as possible :D : https://docs.google.com/forms/d/e/1FAIpQLSfIyN0kR1H4pWfXJdi5HQW_GZ8ns4LLdEb-bAaPQwoTiep5ng/viewform

We will get back to you as soon as possible via email as we are currently on a work trip to CMU.

We do not have specific projects in mind to offer to 106/206B students but are willing to offer projects that fit your interest if the fit is right. For that reason, we can't guarantee spots but also are very open to eager roboticists :). Also even if we can't offer you a 106/206B project we are still considering applicants to either research project regaurdless :D.

We realize the project proposal deadline is soon and will try our best to accommodate that (if you submit this form after the project proposal deadline, please follow up with an email to ekberndt@berkeley.edu so we can respond promptly). If your group ends up switching to one of the above projects after the deadline we can work with the TAs to accommodate that switch (we won't allow you to abandon your teammates though!)

Thanks for taking interest :D
# ROAR / UAV Final Project / Research Offering
**User:** Karim El-Refai
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4798441
 Reposting Eric's msg:

I realize this is quite last minute, but if you are interested in racing autonomous cars at 170mph or hyper maneuverable autonomous drones we have two research projects (potentially final projects) listed below

Interest form for the UAV (tilt-rotor - this is fullstack robotics research to build a neuro-symbolic drone and do sim to real transfer onto a drone that can tilt all four rotors to turn faster - we have several groups from perception, path planning, control, simulation, and hardware) project (Sastry Lab) 

ROAR (Indy Autonomous Challenge - https://www.airacingtech.com/ 170mph autonomous IndyCar racing - subteams are Simulation/Vehicle Dynamics /Localization/Planning/Controls/Perception) project (Sastry / Allen Yang). If you have questions, reach out to ckwolfe@berkeley.edu and ekberndt@berkeley.edu.

Fill out the form here, and we will get back to you via email as soon as possible :D : https://docs.google.com/forms/d/e/1FAIpQLSfIyN0kR1H4pWfXJdi5HQW_GZ8ns4LLdEb-bAaPQwoTiep5ng/viewform

We will get back to you as soon as possible via email as we are currently on a work trip to CMU.

We do not have specific projects in mind to offer to 106/206B students but are willing to offer projects that fit your interest if the fit is right. For that reason, we can't guarantee spots but also are very open to eager roboticists :). Also even if we can't offer you a 106/206B project we are still considering applicants to either research project regaurdless :D.

We realize the project proposal deadline is soon and will try our best to accommodate that (if you submit this form after the project proposal deadline, please follow up with an email to ckwolfe@berkeley.edu and ekberndt@berkeley.edu so we can respond promptly). If your group ends up switching to one of the above projects after the deadline we can work with the TAs to accommodate that switch (we won't allow you to abandon your teammates though!)

Thanks for taking interest :D
### Comment
**User:** Eric Berndt
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4798441?comment=11087311
Thank you @Karim :D


# Look for 2 people for Final Project
**User:** Jiachen Lian
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4803725
 Hey. We are still looking for 2 more people to form the final project team. Here is my email: jiachenlian@berkeley.edu
# proj5 debug
**User:** Emily Park
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4807545
 After my turtlebot stops, it goes to the other direction then after a while starts coming back again. Is this normal? Also the velocity between the two directions are different (one is way slower than the other)
### Answer 
**Name:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4807545?answer=11109559
Yeah that's fine for both


# Project 5 Submission
**User:** Justin Im
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4810295
 Hello,

I thought the video was to be pushed to our GitHub classroom repository, and only just now noticed there was a Gradescope submission (which is closed). Is it okay if we pay an extra slip day including the 2 free ones to submit our video?

Since it's my fault, and not my partner's, I'd like it if both slip days came from my allocated ones.

Thank you for your consideration.


### Answer 
**Name:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4810295?answer=11128129
Hi Justin,

I've just extended the link submission for you, please submit directly to Gradescope by tomorrow (April 22) by 11:59pm.


**User:** Justin Im
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4810295?comment=11132258
Thank you!
# Project 5 - Dimensions of z and w increase with multiple iterations (hardware only)
**User:** Kevyn Ramirez
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4816028
 We receive the following error:


Traceback (most recent call last):
  File "/home/cc/ee106b/sp24/class/ee106b-act/ros_workspaces/project-5-patatas/src/proj5_pkg/src/main.py", line 76, in <module>
    task_controller()
  File "/home/cc/ee106b/sp24/class/ee106b-act/ros_workspaces/project-5-patatas/src/proj5_pkg/src/main.py", line 62, in task_controller
    controller.apply_input()
  File "/home/cc/ee106b/sp24/class/ee106b-act/ros_workspaces/project-5-patatas/src/proj5_pkg/src/controller.py", line 193, in apply_input
    msg.linear.x = self._u[0][0]
  File "/opt/ros/noetic/lib/python3/dist-packages/rospy/topics.py", line 886, in publish
    raise ROSSerializationException(str(e))
rospy.exceptions.ROSSerializationException: field linear.x must be float type

This happens after we run main.py to test our TurtlebotFBLin class. For some reason the method works for 1-3 iterations, and then errors with the above message. We have confirmed that self._u consists of only floats. We would appreciate any pointers as to finding what we should fix.

We also noticed that as iterations ran, our variables would have an additional set of brackets around them, indicating that another dimension is added. We do not know where this happens.

z [1.83292967e-09 3.29448337e-08] 

w [-1.89790313e-10 0.00000000e+00] 

_u [[-3.79580626e-12] [ 0.00000000e+00]] 

z [[1.83292967e-09 3.29448337e-08] [1.83292967e-09 3.29448337e-08]] 

w [[1.71713849e-09 3.08636184e-08] [5.11572068e+02 9.19492818e+03]] 

_u [[[3.05469635e-11 6.13476562e-10]]

[[5.11572068e+02 9.19492818e+03]]]

The above are our values right after running and before the function errors.


### Answer 
**Name:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4816028?answer=11128151
It might be because you're getting a type issue. This could be because of a dimension mismatch -- I notice your z and _u have different dimensions as the output continues. Double check to make sure your dimensions are consistent across the board -- it's a bit trickier to pinpoint this issue exactly without having the full scope of your code.
**User:** Kevyn Ramirez
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4816028?comment=11128378
we fixed our issue by reshaping aD, but now our controller does not work at all how it does in sim, also we noticed our aD is always 0, is that normal?:
 #get the trajectory states

 xD, vD, aD = self.trajectory.get_state(t)

 aD = aD.reshape(2,)




# mini proj 5 submission closed
**User:** Erick Brito
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4816501
 We got an extension until today (monday 4/22) but now the gradescope is closed. Here is our video:
https://youtu.be/yyfnXYdYYAU?si=MMnI7ZDVaJPMiRLH

### Answer 
**Name:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4816501?answer=11134219
Hi,

Please make sure the person who reaches out to me about the extension is the one who submits. Please have them submit directly to the gradescope.


# Forwarding a talk this Wednesday!
**User:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4818860
 Greetings from Berkeley~


The Berkeley Artificial Intelligence Research (BAIR) seminar series is hosted by the BAIR Lab and primarily showcases dissertations and job talks of BAIR Ph.D. students and postdoctoral researchers. However, this special seminar will feature a new talk by Prof. Ken Goldberg!

This talk will be held in person on the 8th floor of Berkeley Way West (BWW) at 11:10 am on Wednesday and will be followed by pizza in the kitchen of BWW8 for those who attend the talk. 

We encourage those of you who are local to attend in person. However, if you are unable to attend in person, you can join the seminar here (passcode 443084). 

This Wednesday, 4/24/24's speaker is Prof. Ken Goldberg, and he will present a new talk, only given once before at Stanford University. 

Title: Data is All You Need (?): Large Robot Action Models and Good Old Fashioned Engineering

Abstract: 2024 is off to an exciting start with enormous enthusiasm for humanoids and other robots based on recent advances in "end-to-end" large robot action models. Initial results are promising, and several collaborative efforts are underway to collect the needed demonstration data. But is data all you need? I'll present my view of the status quo in terms of manipulation task definition, data collection, and experimental evaluation. I'll then suggest that to reach expected performance levels, it may be helpful for the community to reconsider good old fashioned engineering in terms of modularity, metrics, and failure analysis. I'll present MANIP, a new framework for incorporating engineering that shows promise for tasks such as cable untangling, surgical suturing, and bagging.


I welcome feedback: this will be the second time I present this talk (I presented it on 19 April at Stanford), and I expect it to be a bit controversial ;)

Bio: Ken Goldberg (IEEE Fellow, 2005) is William S. Floyd Distinguished Chair of Engineering at UC Berkeley and Chief Scientist of Ambi Robotics and Jacobi Robotics. Ken leads research in robotics and automation: grasping, manipulation, and learning for applications in warehouses, industry, homes, agriculture, and robot-assisted surgery. He is Professor of IEOR with appointments in EECS and Art Practice. Ken is Chair of the Berkeley AI Research (BAIR) Steering Committee (60 faculty) and is co-founder and Editor-in-Chief emeritus of the IEEE Transactions on Automation Science and Engineering (T-ASE). He has published ten US patents and over 400 refereed papers. He has presented over 600 invited lectures to academic and corporate audiences. http://goldberg.berkeley.edu



Wednesday seminar links: (Please do not distribute the below links beyond your organization.)

For those of you who are unable to join in person, please join via Zoom here (passcode 443084) 
# Proj 5 gradescope concern
**User:** Sergio Peterson
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4819463
 Hi,

I just wanted to confirm that i will receive credit for proj 5. My team and I finished proj 5 on time and submitted but the gradescope submission only let my teammate put an extra teammate on the gradescope submission so on my gradescope it says i never submitted proj 5. My  name is the doc and everything. I was told by them that i was ok but i just making sure and double checking with course staff. 

Thank you.


### Answer 
**Name:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4819463?answer=11139343
Who were your group members? I can manually add you to the submission.
**User:** Sergio Peterson
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4819463?comment=11174321
My group members were 
- Jackson Hilton 
- Matthew Sulistijowadi
# Week 15 Announcements!
**User:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4821050
 Good afternoon everyone everyone! I hope you had a lovely weekend. It’s the last week of 106B!

Tuesday’s lecture will be a continuation of the guest lecture from Prof. Sreenath, and Thursday’s session will be about work in progress — hear from GSIs about the research they’ve been working on!

Final project proposals have been graded (you’ll find some comments under the experimental plan section) and hardware requests for final projects are due tonight! 

Software: https://forms.gle/JWeFuk36G3w4V1J86

Due 4/26 @4:59PM

Hardware: https://forms.gle/d1HB4yYhFZRGXibYA

Budget: $60/team

Due 4/22 @11:59PM

This is the last week of office hours! There’s no project or homework party tonight but more hours will be added later this week. Please feel free to stop by to talk about final projects or anything else on your mind!

It’s the last week of journal club as well — if you’ve signed up for a journal club next week, please be sure to still submit your presentation to gradescope, or email it to your Lab TA if it’s extra credit. Also, last week was the last week of discussions (so no discussion this week)!

Final Project Presentation Slot Sign Up! Please fill this out by Wednesday, 5/24 at 11:59pm. Only one person in the group needs to fill this out.

Here’s our end of semester feedback form… please complete it! You’ll get 1% of extra credit for filling it out by Monday, April 29th.

Good luck on your last week of classes this semester!
### Comment
**User:** Kev Wang
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4821050?comment=11141321
The request forms ask for a team number, how would we find our team number?

Also my team is looking to use the Intel RealSense cameras, which I believe the course already has in stock. Would we be able to borrow some?
**User:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4821050?comment=11144210
You can just put n/a for now -- team numbers haven't been assigned yet.

For the RealSense camera, you can email David to pick it up or come by during office hours, any TA should be able to grab one from the cabinet.
### Comment
**User:** Zekai Wang
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4821050?comment=11141838
Just to double check, we have a depth camera available right
**User:** Max Wilcoxson
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4821050?comment=11142254
I also have this question!
**User:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4821050?comment=11144193
Yep -- if you're referring to the RealSense camera! You can email David to pick it up or come by during office hours, any TA should be able to grab one from the cabinet.



### Comment
**User:** Zekai Wang
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4821050?comment=11146466
Also how do we fill out the software installation request form (say what do we fill in for Group Number & Package Number). I want to have torch & tensorflow installed on the workstations and am a bit unsure of how I should proceed. 
**User:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4821050?comment=11146936
For group number you can put N/A. For "Package Number" I'm assuming you're referring to package name, so you can put down both torch and tensorflow for that (especially if you have any specific version, if not you can just ask them to download the latest one) 


# Project 2 Grading
**User:** Akhil Vemuri
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4823280
 Hello,

It seems like the extra credit was factored into the grade for Project 2, so if we didn't do the extra credit, we got 10 points off the total score. Is this intentional, or could it be fixed if not?

Thanks!
### Answer 
**Name:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4823280?answer=11144240
Hey! So the way we graded it, this will be taken into account when grades are finalized on our end. So your score will be adjusted for it to be extra credit. Mostly because gradescope doesn't have a way for something to be out of zero (or if it does, I wasn't able to figure it out).
# March 7 Lecture Recording | Missing sound
**User:** Sunny ME
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4824759
 The March 7 lecture uploaded on the b-courses has no sound. Can a transcript or a lecture from previous year be provided please? 


# Final Project Casadi invalid number detected
**User:** Noah David
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4832097
 I am running an optimizer for my final project using Casadi like we did on proj2, but using the dynamics and constraints for my situation. However, I get an error saying invalid number detected solver failed. Does anyone know how to fix this or what is causing the problem?

This is Ipopt version 3.14.11, running with linear solver MUMPS 5.4.1.

Number of nonzeros in equality constraint Jacobian...: 18009 Number of nonzeros in inequality constraint Jacobian.: 8004 Number of nonzeros in Lagrangian Hessian.............: 22015

CasADi - 2024-04-23 18:25:50 WARNING("solver:nlp_jac_g failed: NaN detected for output jac_g_x, at nonzero index 8 (row 8006, col 3).") [.../casadi/core/oracle_function.cpp:377] Error evaluating Jacobian of equality constraints at user provided starting point. No scaling factors for equality constraints computed! CasADi - 2024-04-23 18:25:50 WARNING("solver:nlp_jac_g failed: NaN detected for output jac_g_x, at nonzero index 8 (row 8006, col 3).") [.../casadi/core/oracle_function.cpp:377] Error evaluating Jacobian of inequality constraints at user provided starting point. No scaling factors for inequality constraints computed! CasADi - 2024-04-23 18:25:50 WARNING("solver:nlp_g failed: NaN detected for output g, at (row 8006, col 0).") [.../casadi/core/oracle_function.cpp:377]

Number of Iterations....: 0

Number of objective function evaluations = 0 Number of objective gradient evaluations = 0 Number of equality constraint evaluations = 0 Number of inequality constraint evaluations = 1 Number of equality constraint Jacobian evaluations = 0 Number of inequality constraint Jacobian evaluations = 0 Number of Lagrangian Hessian evaluations = 0 Total seconds in IPOPT = 0.055

EXIT: Invalid number in NLP function or derivative detected. solver : t_proc (avg) t_wall (avg) n_eval nlp_g | 7.00ms ( 7.00ms) 4.68ms ( 4.68ms) 1 nlp_grad_f | 0 ( 0) 3.01ms ( 3.01ms) 1 nlp_jac_g | 47.00ms ( 23.50ms) 43.90ms ( 21.95ms) 2 total | 72.00ms ( 72.00ms) 57.61ms ( 57.61ms) 1 Solver failed: Error in Opti::solve [OptiNode] at .../casadi/core/optistack.cpp:157: .../casadi/core/optistack_internal.cpp:996: Assertion "return_success(accept_limit)" failed: Solver failed. You may use opti.debug.value to investigate the latest values of variables. return_status is 'Invalid_Number_Detected'
 <bound method Opti.value of Opti(Opti { instance #0 #variables: 2 (nx = 7005) #parameters: 0 (np = 0) #constraints: 1011 (ng = 13013) CasADi solver allocated. CasADi solver was called: Invalid_Number_Detected })>


### Answer 
**Name:** Max Wilcoxson
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4832097?answer=11182573
I am not necessarily sure without additional information or looking at your code. However, I asked GPT 4 for some potential areas to check, and this is what it said. Have you checked these? Might be helpful:

The error message you're encountering, "Invalid number in NLP function or derivative detected", typically indicates that there are NaN (Not a Number) or Inf (Infinity) values being produced during the evaluation of your optimization problem. This can happen due to several reasons related to the formulation of your problem or the numerical properties of the solver and model. Here are some steps and checks you can perform to troubleshoot and hopefully resolve this issue:

Initial Values: The error messages suggest that there's a NaN detected at a specific point during the Jacobian evaluation. Check your initial values to ensure they are feasible and within a reasonable range. NaN or Inf values can occur if the initial values lead to undefined operations (like division by zero).

Scaling of Variables: Sometimes, very large or very small values of variables or parameters can lead to numerical instability in solvers like Ipopt. Make sure your variables are scaled properly. This means that typical values used in calculations should be close to 1 if possible.

Bounds and Constraints: Review the bounds on your variables and the constraints in your model. Ensure that all constraints are correctly implemented and that there are no contradictions or impossible conditions. An unbounded variable or poorly defined constraint can lead to extreme values that result in numerical errors.

Derivatives and Jacobians: Since the error specifically mentions the Jacobian of equality and inequality constraints, make sure that these derivatives are correctly defined. If you are providing analytical derivatives, verify their correctness. Alternatively, consider using automatic differentiation if supported by CasADi, to avoid manual errors.

Solver Settings: Adjusting solver settings can sometimes help manage numerical difficulties. You can try changing parameters related to step size, tolerance, or even switching between exact and approximate derivative calculations in the Ipopt settings.

Model Simplification: Temporarily simplify your model by reducing the number of variables or constraints, or by solving a simplified version of the problem. This can help isolate the source of the numerical issues.

Use opti.debug.value: As suggested in the error message, use the opti.debug.value method to inspect the values of variables at the point where the solver fails. This can give you insight into which variables or constraints are causing the problem.

Check CasADi and Ipopt versions: Ensure that your versions of CasADi and Ipopt are compatible and that you are using versions without known bugs that might affect your problem.

Here's a brief example of how you might use opti.debug.value to inspect the problematic values:

# Assuming `opti` is your CasADi Opti instance
try:
    sol = opti.solve()
except Exception as e:
    print("Solver failed:", str(e))
    # Print debug information for all variables
    for v in opti.variable():
        print(v, opti.debug.value(v))


Implement these checks and modifications to see if the problem persists. If issues continue, you might want to consider consulting more detailed logs or discussions specific to CasADi and Ipopt for additional diagnostic strategies.
# Tarun OH Moved to 4pm
**User:** Tarun Amarnath
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4842619
 Sorry about the last-minute change. I have OH of my own that I must attend unfortunately. Please let me know if you can't make it anymore, will be happy to schedule a time with you.


# Come to Mulford 159 for the last lecture!!
**User:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4846453
 We'll have 3 guest speakers to wrap up the semester:)
# Possible Homework Drop or Revision?
**User:** Cecilia Pham
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4849089
 Hi! I was wondering if course staff will be willing to consider allowing a homework drop or revising our past homework for credit similar to EECS16b or alternatively, assigning homework 6 as extra credit in the homework category? Since we also did not have a midterm and only had five homework assignments, if we got a low score on one of the homework assignments it would impact our grade quite a bit. No worries if not, thank you!




# Final Project Logistics
**User:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4850133
 Hey everyone!

I hope you've had a good week so far and enjoyed the last lecture today! Here are a few reminders for the final project.

Just to clarify, presentations should be 10-15 minutes, with at least 5 minutes reserved for Q&A. In total, you won't spend more than 20 minutes presenting.

The final presentation schedule is here.  Please reach out to other groups to switch your presentation slot (and if you agree on a switch, please send me an email with both groups CC'd). If it's an extremely extenuating circumstance, please reach out to me directly, and I'll see what we can do, but the timing for presentations will be extremely tight. 

We'll also release a project peer evaluation form if you're experiencing any issues with your group (or have experienced issues with your past project groups).

[VERY IMPORTANT] We'll have refreshments for both days of the final project! Please fill out this form for food ordering. You may only order something for one slot (ideally your presentation slot). It's due Sunday, 4/28 at 11:59 pm. 

Finally, I just wanted to note that most of the weightage for this project will be on your final report, so please don't stress toooo much about the presentation (I know it can be very stressful)! We're just looking forward to seeing the results you have so far:) 

It's a rough part of the semester, so please be sure to take care of yourself! Here are some self-care and wellness resources -- we really do care about you, so please reach out sooner rather than later if you're facing any extenuating circumstances. 

Please let me know if you have any questions!
### Comment
**User:** Edward Lee
**Role:** student
**URL:** https://edstem.org/us/courses/54072/discussion/4850133?comment=11205073
Where does the food come from? Does course staff make the food? 🥺 
**User:** Kirthi Kumar
**Role:** admin
**URL:** https://edstem.org/us/courses/54072/discussion/4850133?comment=11205110
Abe's Cafe!! Prof. Sastry is very very kind to sponsor:)


