

# Lab 0: Getting Started (Mac/Linux Setup)

## Setup

#### Terminal

The terminal is a program that allows you to interact with your computer
by entering commands.

You already have a program called `Terminal` or something similar on your computer. Open
that up and you should be good to go.

#### Python

Python 3 is the primary programming language used in this course. If you have an older version of Python installed,
please make sure to download and install Python 3.9. You can check your Python
version with `python3 ––version`.

Download and install [Python 3 (64-bit)](https://www.python.org/ftp/python/3.9.13/python-3.9.13-macos11.pkg "https://www.python.org/ftp/python/3.9.13/python-3.9.13-macos11.pkg"). You may need to
right-click the download icon and select "Open." After installing, please close
and reopen your terminal.

If you have Homebrew installed, you can also install Python3 by running
`brew install python3`.

**Verify**: We can use the terminal to check if your Python interpreter was installed
correctly. Try the following command:

```
python3
```

If the installation worked, you should see some text printed out about the
interpreter followed by `>>>` on its own line. This is where you can type in Python
code. Try typing some expressions you saw in lecture, or just play around to see
what happens! You can type `exit()` or `Cmd-D` to return to your command line.

> 
> If the `python3` command doesn't work, try using `python` or `py`.
> 

#### Text Editor

The **Python interpreter** that you just installed allows you to *run* Python
code. You will also need a **text editor**, where you will *write* Python code.

[Visual Studio Code (VS Code)](https://code.visualstudio.com/ "https://code.visualstudio.com/") is the most popular choice among the staff for
this course for writing Python. After installing it, you may find our [Visual Studio Code](/articles/vscode "/articles/vscode") helpful.

**We highly recommend using VS Code for this class.** This will help us support
you best since most of staff uses VS Code. Please do not use word processors such as Microsoft Word to edit programs. Word processors can add extra content to documents that will confuse the interpreter.

You can open a terminal directly on VS Code. Thus, when running terminal commands, you can manage everything in VS Code rather than navigating back and forth between VS Code and a separate
terminal application. You can open an embedded terminal by going to
`Terminal > New Terminal` in VS Code's navigation bar.

![Open the VS Code embedded terminal](assets/vscode-embedded-terminal.png)

**Other Text Editors**

For your reference, we've written some guides on other popular text
editors:

* [Atom](/articles/atom "/articles/atom"): A more lightweight desktop editor.
* [Vim](/articles/vim "/articles/vim"): A command-line editor.
* [Emacs](/articles/emacs "/articles/emacs"): A command-line editor.

A few other editors:

* [PyCharm](https://www.jetbrains.com/pycharm/ "https://www.jetbrains.com/pycharm/"): A desktop editor designed for Python.
* [Sublime Text](https://www.sublimetext.com/ "https://www.sublimetext.com/"): A text editor that works with code.

**Pair Programming**

Throughout this course, you'll have many chances to collaboratively code in labs and projects. We recommend you download these pair programming extensions now to use in the future.

For sharing code, you can follow the instructions for your editor of choice:

* [VS Code](/articles/vscode#pair-programming "/articles/vscode#pair-programming")
* [Atom](/articles/atom#pair-programming "/articles/atom#pair-programming")

## Walkthroughs & Reviews

### Walkthrough: Using the Terminal

First, open a terminal window.

![starting the terminal](assets/terminal.png)

#### Working Directory

Your terminal is always open in some folder, which is called the "working directory."
The name of the working directory is visible before the `$` in the terminal.

#### Home Directory

When you first open your terminal, you will start in the "home directory".
In other words, the home directory is the default working directory.
The **home directory** is represented by the `~` symbol, which you might see
at the prompt.

> Don't worry if your terminal window doesn't look exactly the same.
> The important part is that the prompt shows `$` (indicating Bash) or `%`
> (indicating zsh).
> 
> 

Try running `echo "$HOME"`. That command should display the full path to your home directory.
It should look something like this:

`/Users/OskiBear`

#### Path

A path is like an address: it tells both you and the computer where to find a particular file or
folder (also called a "directory"). For example, `/Users/mollynicholas/Projects/cs61a-example/labs`
is a path that references a particular folder on your computer: the folder `labs`, which is
contained in the folder `cs61a-example`, which is contained in the folder `Projects`, etc.

#### CLI vs GUI

Remember that you can access the files and directories (folders) on your computer in two different ways.
You can either use the terminal (which is a **c**ommand **l**ine **i**nterface or CLI) or you can use
Finder. Finder is an example of a **g**raphical **u**ser
**i**nterface (GUI). The techniques for navigating are different, but the files are the same. For example,
here's how my lab folder for CS 61A looks in my GUI:

![GUI folder example](assets/finder-path-example.png)

And here's how the exact same folder looks in terminal:

![CLI folder exmaple](assets/terminal-path-example.png)

Notice the yellow box shows you the path in both cases, and the purple ellipse shows you the contents of the "labs" folder.

#### Terminal vs Python Interpreter

Let's pause and think about the difference between the terminal and the Python interpreter.

![interpreter vs terminal](assets/terminal-vs-interpreter.png)

1. Which is the terminal?
2. Which one is the Python interpreter?
3. Which one is my code editor?
4. And how can you tell?

Both A and D are my terminal. This is where you can run bash commands like `cd` and `ls`. D is the terminal that is built-in to VS Code.

B is the Python interpreter. You can tell because of the `>>>` prompt that means you've started a Python interpreter. You can also tell because the command that started it is visible: `python3`. The `python3` command launches a Python interpreter. If you type a bash command into the Python interpreter, you'll probably get a syntax error! Here's an example:

![interpreter syntax error](assets/interpreter-syntax-error.png)

C is my code editor. This is where I can write Python code to be executed via my terminal.

### Walkthrough: Organizing Your Files

In this section, you will learn how to manage files using terminal commands.

> Make sure your prompt contains a `$` somewhere in it and does not begin with
> `>>>`. If it begins with `>>>` you are still in a Python shell, and you need
> to exit. See above for how.
> 
> 

#### Directories

The first command you'll use is `ls`. Try typing it in your terminal:

```
ls
```

The `ls` command **l**i**s**ts all the files and folders in the current
directory. A **directory** is another name for a folder (such as the
`Documents` folder).

Since you're in the home directory right now, after you type `ls` you should
see the contents of your home directory.

**Changing Directories**

To move into another directory, use the `cd` command (**c**hange **d**irectory).

Let's try moving into your `Desktop` directory. First, make sure you're in your
home directory (check for the `~` on your command line) and use `ls` to see if
the `Desktop` directory is present.

Try typing the following command into your terminal, which should move you into
that directory:

```
cd Desktop
```

If you're *not* already in your home directory, try `cd ~/Desktop`. This is telling
the terminal the path where you want to go.

**Making New Directories**

The next command is called `mkdir`, which **m**a**k**es a new
**dir**ectory. Let's make a directory called `cs61a` in your `Desktop` directory to
store all of the assignments for this class:

```
mkdir cs61a
```

A folder named `cs61a` will appear on your Desktop. You can verify this by
using the `ls` command again or by checking your Desktop using Finder (Mac).

At this point, let's create some more directories. First, make sure you are in
the `cs61a` directory (`~/Desktop/cs61a`).
Then, create two new folders, one called `projects` and the other called `lab`. Both
should be inside of your `cs61a` folder:

```
cd ~/Desktop/cs61a
mkdir projects
mkdir lab
```

Now if you list the contents of the directory (using `ls`), you'll see two
folders, `projects` and `lab`.

![cs61a directory](assets/terminal_commands.png)

**More Directory Changing**
There are a few ways to return to the home directory:

* `cd ..` (two dots). The `..` means "the parent directory", or one directory above your current directory.
* `cd ~` (the tilde). Remember that `~` means home directory, so
 this command will always change to your home directory.
* `cd` (`cd` on its own). Typing just `cd` is a shortcut for typing `cd ~`.

> You do not have to keep your files on your Desktop if you prefer otherwise.
> Where you keep your files locally will not affect your grade. Do whatever is
> easiest and most convenient for you!
> 
> 

#### Downloading the Assignment

If you haven't already, download the zip archive, [lab00.zip](/lab/lab00/lab00.zip "/lab/lab00/lab00.zip"), which
contains all the files that you'll need for this lab. Once you've done that, let's
find the downloaded file. On most computers, `lab00.zip` is probably located in a
directory called `Downloads` in your home directory. Use the `ls` command to
check:

```
ls ~/Downloads
```

If you don't see `lab00.zip`, ask for help on Piazza or in office hours. On some versions of Safari the file may get unzipped for you, in which case you would just see a new directory named `lab00`.

#### Extracting Starter Files

You must expand the zip archive before you can work on the lab files.
Different operating systems and different browsers have different ways of
unzipping. Clicking on a .zip file in Mac will automatically unzip. If you
run into trouble, you can search online for how to unzip a file.

Here's a way to unzip using the terminal:

> Using a terminal, you can unzip the zip file from the command line.
> First, `cd` into the directory that contains the zip file:
> 
> 
> 
> 
```
> cd ~/Downloads
> 
```
> 
> Now, run the `unzip` command with the name of the zip file:
> 
> 
> 
> 
```
> unzip lab00.zip
> 
```
> 

You only need to unzip the files once.

Once you unzip `lab00.zip`, you'll have a new folder called `lab00` which
contains the following files (check it out with `cd lab00` and `ls`):

* `lab00.py`: The template file you'll be adding your code to
* `ok`: A program used to test and submit assignments
* `lab00.ok`: A configuration file for `ok`

#### Moving Files

Move the lab files to the lab folder you created earlier:

```
mv ~/Downloads/lab00 ~/Desktop/cs61a/lab
```

The `mv` command will **m**o**v**e the `~/Downloads/lab00` folder
into the `~/Desktop/cs61a/lab` folder. If you prefer, you can also move the file
by dragging and dropping it into the correct folder in your graphic file
explorer, which is probably more familar and will have exactly the same result.

Now, go to the `lab00` folder that you just moved. Try using `cd` to navigate
your own way! If you get stuck, you can use the following command:

```
cd ~/Desktop/cs61a/lab/lab00
```

#### Summary

Here is a summary of the commands we just went over for your reference:

* `ls`: **l**i**s**ts all files in the current directory
* `cd <path to directory>`: **c**hange into the specified **d**irectory
* `mkdir <directory name>`: **m**a**k**e a new **dir**ectory with the given name
* `mv <source path> <destination path>`: **m**o**v**e the file at the given source to the
 given destination

Finally, you're ready to start editing the lab files! Don't worry if this seems
complicated—it will get much easier over time. Just keep practicing!
You can also take a look at our [UNIX tutorial](/articles/unix "/articles/unix") for a more
detailed explanation of terminal commands.

