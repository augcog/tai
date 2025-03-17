
# **Lecture 1: Introduction to Python Programming**

**By Allen Y. Yang, PhD**

(c) Copyright Intelligent Racing Inc., 2021-2024. All rights reserved. Materials may NOT be distributed or used for any commercial purposes.

If you just start reading this website, you have found the coding site for the introductory Python programming course developed by Dr. Allen Y. Yang. At the time of creating this course, Dr. Yang is in the faculty of the Department of EECS at the University of California, Berkeley. You may find his academic website at: https://people.eecs.berkeley.edu/~yang/

Beyond a research career at Berkeley, Dr. Yang is immensely interested about innovation and entrepreneurship, partially thanks to the geolocation of Berkeley close to Silicon Valley, a famous place in the US and around the world for its technology innovation and many inspiring entrepreneurial stories. Intelligent Racing is just one of many such companies founded in Silicon Valley by Dr. Yang, with the singular purpose to deliver state-of-the-art science and technology learning curricula to students, who will become the next technology leader and successful entrepreneurs.

In this course, we will be learning about the programming language of Python. If this is your very first time hearing about Python programming,
language, then you are in the right place: This course is for you!

In fact, you are not alone wondering:
1. How can a computer language help you to communicate with a computer?
2. What is Python?
3. Is Python a suitable language to learn for beginners?

For the rest of the course in ten lectures, we will gradually learn about the in-and-out of Python programming that is rigorous and fun for beginners. You will learn about all the basic elements of Python to effectively use the language to start coding useful computer programs and solving intersting practical real-world problems. We hope you will find that Python is a quite user-friendly programming language whose use cases may only be limited by your imagination.

Of course, being a super user-friendly programming language that is easy to use for humans does not mean that Python is good for every occasion. One can quickly point out several obvious examples:

First, Python is not the fastest language to run on computers. There are more traditional languages that were designed to be very easy to be understood by the processors of the computer and hence can be executed faster than Python.

Second, Python is not supported by all computing platforms. For example, it is still relatively difficult to code up and run Python programs on iOS or Android systems natively. To best develop applications that run on mobile platforms, one is recommended to continue their programming journey to pick up some other languages.

## Keywords

In this interactive lecture nodes, written in the style of Jupyter Notebook, we will impose a prelude section for each lecture called *Keywords*. This is the place we will help the learner to highlight some new technical jargons that will be introduced more formally in the lecture note. Our goal is that the learner can take advantage of this structure to quickly familarize themselves with the upcoming new concepts, and the section together with the later Summary section may also help the learner to better review the content of each lecture after taking the classes.

* **Jupyter Notebook**: Jupyter Notebook is an open-source computer document format developed by Project Jupyter Foundation. The document can be hosted and interactively edited within a web browser similar to a webpage, and then the web browser may connect with several supported language kernels to interpret the code embedded within the document. A primary programming language used in Jupyter Notebook is Python.
* **Interpreted language**: Python is an interpreted language that can be understood and executed by a computer line by line.
* **Function**: a function in a programming language is a piece of stand-alone code that encaptulates a set of starting conditions set by its input arguments and a set of return results by its return arguments.
* **Data type**: Data when stored in computer memory must declare its data type, such as integer, floating point, text (as strings), etc.
* **Debug**: Debug is a programming jargon referring to the practice of finding programming errors when designing a computer code.

## Running First Python Code in Command Line
After we have done a brief motivation about the Python language, we arrive at the best part of our coding exercise. Below, we are going to see, for the first time, how Python executes some basic computing commands. But before we do that, let us understand how a typical Python language command is executed by computer that is dictated by its language standards.

The most important rule to remember for Python is that: *it is a high-level, interpreted language.* Let us further consider this statement below.
1. *Python is a high-level language*. This usually means two things: First, programming statements of a high-level language broadly adopt words in natural languages (mostly English) and math symbols that are fairly easy for humans to understand. Second, high-level statements are more descriptive about user's intent to solve a problem and more abstract about the execution of the statements by the computer, compared to low-level languages that typically are descriptive about computer processor's precise but sometimes rather tedious steps to execute the code.
2. *Python is an interpreted language*. It means the execution of language statements is more direct and freely, compared to compiled languages. A compiled language must go through a compiling process before a code can be executed. An interpreted language such as Python allows users to freely execute any one line of or a block of multiple lines of code without explicitly invoking a compiling process.

Let us immediately run an example to have Python solving a simple problem for us. In this example, we want to know the result of an arithmetic problem: 3+2 = ?

To get the result, this code site has prepared the following code block. On the left-hand side at the beginning of the code block, there is a triangle symbol that means asking Python to **RUN** this code. Please use your mouse to left click this triangle symbol and observe the result of your action.


```python
3+2
```




5



Congratulations! You have just run your first Python code. The result of 5 can be seen in an *output* block, which is usually called **the terminal output** or **the console output**. It is so called because, before the invention of graphical user interface (GUI), all computer intput/output interaction with users was done on text-based terminal or console environment. Therefore, in many programming languages, text output by default always displays in the terminal environment. In fact, you may find the terminal application in most popular operating systems including Windows, Mac OSX, and Linux.

We also see in this coding exercise, the Python statement includes three elements, namely, an integer number 3, an integer number 2, and an arithmetic operator +. Python adopts the same meanings of these math symbols and the proper order how an addition calculation should be denoted in arithmetic. Therefore, this is an example of a high-level programming.

By clicking the **RUN** button, Python immediately executes the statement and results its computation result in the line immediately following the statement. This is thanks to the fact that Python is an interpreted language.

Another important fact about running the above code is that the reader should be aware that we are in a special Python development environment. A development environment is itself a computer program, where the code of a program can be written and more importantly often be executed within the environment. The development environment implemented by Kaggle uses the development style of so-called Jupyter Notebook. This style of Python programming allows users to directly edit and run their code on web browsers, and the users can conveniently select to run any particular code block or run all the code blocks together on the same page. If you pay attention to the top of this Kaggle webpage, right under the menu bar, there is a double-triangle button that is marked as **RUN ALL**. This is the command to sequentially run all the code blocks on this page in one shot. You may try this function to see the result.

Next, let us see Python interprets and executes a block of code together (by clicking the **RUN** button of the next code block):


```python
print(3+2.0)
print(17/3)
```

5.0
5.666666666666667


In the above code block, we see the use of **print()** function. As a special rule of Jupyter Notebook environment, if a code block contains only arithmetic equations, only the last equation will output its result on the browser. In the above code block, we want a block to contain tasks to evaluate three equations. We then use a Python keyword **print()** to ask the environment to print out each of the three results.

A function in a high-level programming language takes in one or more input arguments, and then output one or more return values that are also called output. In Python, input arguments of a function such as **print()** are denoted using the pair of paratheses. For example, the input of the first print function is 3+2.0. After you **RUN** the code, its output result is 5.0. The input of the second print function is 17/3, and its output result is 5.666666666666667.

You may also notice from the above code block that the Python calculation results from two seemingly very similar expressions, one of which is 3+2 and the other is 3+2.0, also return similar but different values. The result for 3+2 is 5, and for 3+2.0 is 5.0

The difference lies in the way numeric numbers are represented in computer. For numbers 3, 2, and 5, they are called integers. But for 2.0 and 5.0, they are called floating numbers, or floats. Even when mathematically 5 and 5.0 are equal in magnitude, the use of fraction indicates one of them (i.e., 5) is an integer while the other that contains the fraction part is a float. We will discuss Python numeric type in more detail in the next lecture.


```python
print(type(5))
print(type(5.0))
```

<class 'int'>
<class 'float'>


The type of a number or a function return value can be explicitly checked using another useful Python function, with the keyword **type()**. We observe after running the above code block, that we print the type of number 5 and the type of number 5.0, respectively.

To correctly read the two statements, the results from nested functions are calculated in stages from right to the left. For example, the statement **print(type(5))** should be understood as firstly query the type of number 5 using the type() function, then the output of the type() function is fed into the second print() function as its input. The final result is the print out of the type of the integer 5.

Finally, let us observe the information from the output of the two statements. The first result indicates number 5 is an 'int' type, which is a text string representing the integer type. The second result indicates number 5.0 is a 'float' type, a text string representing the float type. The two results also indicate that both the int and float types are classes. In fact, all Python numeric or other more complex value types are implemented as "classes". The use of class concept signals the third attributes of Python language (the first two being that it is high-level and interpreted):

*Python is an object-oriented programming language.*

In later lectures, we will go over the principles of object-oriented programming in details. For now, we will conclude the first lecture.

## Summary

* Python is a high-level, interpreted programming language.
* Function print() outputs its input argument(s) as string output
* Python displays text output by print() in the terminal environment.
* In Jupyter Notebook or the terminal, Python will also evaluate and print out the value of the last numeric expression.
* Function type() outputs the type of the input argument.
* A class-type variable indicates the variable is created as a class type in object-oriented programming.

## Exercises

1. Create two integer variables: a = 10 and b = 5. Then use print() function to print out the division result of a / b.

2. Please pay attention to the format of the above result, which indicates that it is not an integer. To verify that, continue with the above program, and print out the output of the function type(a/b). What can you say about this result?

3. Print out the result of 10 factorial, that is, the product of all positive integers from 1 to 10.

4. Can you list a few criteria that set apart between high-level programming languages such as Python and low-level programming languages used in early days of the computer industry?

5. Debug: Please correct the code below so that it can be correctly run in Python


```python
a = 10
b = 5
print(a/b)
print(type(a/b))
print(a//b)
print(type(a//b))
```

2.0
<class 'float'>
2
<class 'int'>



```python
import math
print(1*2*3*4*5*6*7*8*9*10)

result = 1
for i in range(1, 11):
result = i*result
print(result)

print(math.factorial(10))
```

3628800
3628800
3628800


Python is easy to understand in english, and describes the users intent rather than the specific steps the computer takes to complete a task.


```python
## Print 3 + 2
print(3 + 2)
```

5

