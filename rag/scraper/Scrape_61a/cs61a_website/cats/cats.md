

# 

Project 2: CS 61A Autocorrected Typing Software

 * [cats.zip](cats.zip "cats.zip")

![](images/cats_typing_still.gif)

> 
>  Programmers dream of  
> 
>  Abstraction, recursion, and  
> 
>  Typing really fast.
> 

## Introduction

> **Important submission note:** For full credit:
> 
> 
> * Submit with Phases 1 and 2 complete by **Thursday, February 22**, worth 1 pt.
> * Submit with all phases complete by **Tuesday, February 27**.
> 
> 
> Try to attempt the problems in order, as some later problems will depend
> on earlier problems in their implementation and therefore also when
> running `ok` tests.
> 
> 
> The entire project can be completed with a partner.
> 
> 
> You can get 1 bonus point by submitting the entire project by
> **Monday, February 26**.
> 
> 

In this project, you will write a program that measures typing speed.
Additionally, you will implement typing autocorrect, which is a feature that
attempts to correct the spelling of a word after a user types it. This project
is inspired by [typeracer](https://play.typeracer.com/ "https://play.typeracer.com/").

## Final Product

Our staff solution to the project can be interacted with at
[cats.cs61a.org](https://cats.cs61a.org "https://cats.cs61a.org").
If you'd like, feel free to try it out now.
When you finish the project, you'll have implemented a significant part of
this yourself!

## Download Starter Files

You can download all of the project code as a [zip archive](cats.zip "cats.zip").
This project includes several files, but your changes will be made only to
`cats.py`. Here are the files included in the archive:

* `cats.py`: The typing test logic.
* `utils.py`: Utility functions for interacting with files and strings.
* `ucb.py`: Utility functions for CS 61A projects.
* `data/sample_paragraphs.txt`: Text samples to be typed.
 These are
 [scraped](https://github.com/kavigupta/wikivideos/blob/626de521e04ca643751ed85d549faca6ea528b1d/get_corpus.py "https://github.com/kavigupta/wikivideos/blob/626de521e04ca643751ed85d549faca6ea528b1d/get_corpus.py")
 Wikipedia articles about various subjects.
* `data/common_words.txt`: Common
 [English words in order of frequency](https://github.com/first20hours/google-10000-english/blob/master/google-10000-english-usa-no-swears.txt "https://github.com/first20hours/google-10000-english/blob/master/google-10000-english-usa-no-swears.txt").
* `data/words.txt`: Many more
 [English words in order of frequency](https://github.com/first20hours/google-10000-english/blob/master/google-10000-english-usa-no-swears.txt "https://github.com/first20hours/google-10000-english/blob/master/google-10000-english-usa-no-swears.txt").
* `data/final_diff_words.txt`: Even more English words!
* `data/testcases.out`: Test cases for the optional Final Diff extension.
* `cats_gui.py`: A web server for the web-based graphical user interface (GUI).
* `gui_files`: A directory of files needed for the graphical user interface
 (GUI).
* `multiplayer`: A directory of files needed to support multiplayer mode.
* `favicons`: A directory of icons.
* `images`: A directory of images.
* `ok`, `proj02.ok`, `tests`: Testing files.
* `score.py`: Part of the optional Final Diff extension.

## Logistics

The project is worth 20 points.
19 points are for correctness
and 1 point is for submitting Phases 1 & 2 by the checkpoint date.

You will turn in the following files:

* `cats.py`

You do not need to modify or turn in any other files to complete the
project. To submit the project,  **submit the required files to the appropriate Gradescope assignment.**

For the functions that we ask you to complete, there may be some
initial code that we provide. If you would rather not use that code,
feel free to delete it and start from scratch. You may also add new
function definitions as you see fit.

**However, please do not modify any other functions or edit any files not
listed above**. Doing so may result in your code failing our autograder tests.
Also, please do not change any function signatures (names, argument order, or
number of arguments).

Throughout this project, you should be testing the correctness of your code.
It is good practice to test often, so that it is easy to isolate any problems.
However, you should not be testing *too* often, to allow yourself time to
think through problems.

We have provided an **autograder** called `ok` to help you
with testing your code and tracking your progress. The first time you run the
autograder, you will be asked to **log in with your Ok account using your web
browser**. Please do so. Each time you run `ok`, it will back up
your work and progress on our servers.

The primary purpose of `ok` is to test your implementations.

If you want to test your code interactively, you can run

```
 python3 ok -q [question number] -i 
```

with the appropriate question number (e.g. `01`) inserted.
This will run the tests for that question until the first one you failed,
then give you a chance to test the functions you wrote interactively.

You can also use the debugging print feature in OK by writing

```
 print("DEBUG:", x) 
```

which will produce an output in your terminal without causing OK tests to fail
with extra output.

# Getting Started Videos

> To see these videos, you should be logged into your berkeley.edu email.
> 
> 

 Getting Started Videos (enable JavaScript)

 [YouTube link](https://youtu.be/playlist?list=PLx38hZJ5RLZdGL_xs5FwpFDeAxfM7lBGw "https://youtu.be/playlist?list=PLx38hZJ5RLZdGL_xs5FwpFDeAxfM7lBGw") 

# Phase 1: Typing

### Problem 1 (1 pt)

Throughout the project, we will be making changes to functions in `cats.py`.

Implement `pick`. This function selects which paragraph the user will type.
It takes three parameters:

* a list of paragraphs (strings) called `paragraphs`
* a `select` function, which returns `True` for paragraphs that can be selected
* a non-negative index `k`

The `pick` function returns the `k`th paragraph for which `select` returns
`True`. If no such paragraph exists (because `k` is too large), then `pick`
returns the empty string.

Before writing any code, unlock the tests to verify your understanding of the question:

```
python3 ok -q 01 -uCopy✂️
```

 document.getElementById("copy-code-python3ok-q01-u").onclick = () => copyCode('python3 ok -q 01 -u', "copy-code-python3ok-q01-u");

Once you are done unlocking, begin implementing your solution. You can check your correctness with:

```
python3 ok -q 01Copy✂️
```

 document.getElementById("copy-code-python3ok-q01").onclick = () => copyCode('python3 ok -q 01', "copy-code-python3ok-q01");

