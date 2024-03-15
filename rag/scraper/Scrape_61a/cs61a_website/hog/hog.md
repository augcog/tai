

# 

Project 1: The Game of Hog

 * [hog.zip](hog.zip "hog.zip")

> ![5-sided die](assets/icon.gif)
> 
> 
>  I know! I'll use my  
>  Higher-order functions to  
>  Order higher
> rolls. 

## Introduction

> **Important submission note:** For full credit:
> 
> 
> * Submit with Phase 1 complete by **Tuesday, January 30**, worth 1 pt.
> * Submit the complete project by **Wednesday, February 7**.
> 
> 
> Try to attempt the problems in order, as some later problems will depend on
> earlier problems in their implementation and therefore also when running `ok`
> tests.
> 
> 
> You may complete the project with a partner.
> 
> 
> You can get 1 bonus point by submitting the entire project by **Tuesday, February 6**.
> You can receive extensions on the project deadline and checkpoint
> deadline, but not on the early deadline, unless you're a DSP student with an
> accommodation for assignment extensions.
> 
> 

In this project, you will develop a simulator and multiple strategies for the
dice game Hog. You will need to use *control statements* and *higher-order
functions* together, as described in Sections 1.2 through 1.6 of [Composing
Programs](https://www.composingprograms.com "https://www.composingprograms.com"), the online textbook.

> When students in the past have tried to implement the functions without
> thoroughly reading the problem description, they’ve often run into issues.
> 😱
> **Read each description thoroughly before starting to code.**
> 
> 

### Rules

In Hog, two players alternate turns trying to be the first to end a turn with
at least `GOAL` total points, where `GOAL` defaults to 100. On each turn, the current player chooses some number
of dice to roll, up to 10. That player's score for the turn is the sum of the
dice outcomes. However, a player who rolls too many dice risks:

* **Sow Sad**. If any of the dice outcomes is a 1, the current player's score
 for the turn is `1`.

 Examples (enable JavaScript)

* *Example 1:* The current player rolls 7 dice, 5 of which are 1's. They
 score `1` point for the turn.
* *Example 2:* The current player rolls 4 dice, all of which are 3's. Since
 Sow Sad did not occur, they score `12` points for the turn.

In a normal game of Hog, those are all the rules. To spice up the game, we'll
include some special rules:

* **Boar Brawl**. A player who rolls zero dice scores three times
 the absolute difference between the tens digit of the opponent’s score and
 the ones digit of the current player’s score, or 1, whichever is higher.
 The ones digit refers to the rightmost digit and the tens digit refers to
 the second-rightmost digit. If a player's score is a single digit (less than
 10), the tens digit of that player's score is 0.

 Examples (enable JavaScript)

* *Example 1:*

	+ The current player has `21` points and the opponent has `46` points, and the current player
	 chooses to roll zero dice.
	+ The tens digit of the opponent's score is `4` and the ones digit of the current player's score is `1`.
	+ Therefore, the player gains `3 * abs(4 - 1) = 9` points.
* *Example 2:*

	+ The current player has `45` points and the opponent has `52` points, and the current player
	 chooses to roll zero dice.
	+ The tens digit of the opponent's score is `5` and the ones digit of the current player's score is `5`.
	+ Since `3 * abs(5 - 5) = 0`, the player gains `1` point.
* *Example 3:*

	+ The current player has `2` points and the opponent has `5` points, and the current player
	 chooses to roll zero dice.
	+ The tens digit of the opponent's score is `0` and the ones digit of the current player's score is `2`.
	+ Therefore, the player gains `3 * abs(0 - 2) = 6` points.

* **Sus Fuss**.
 We call a number [*sus*](https://en.wikipedia.org/wiki/Sus_%28genus%29 "https://en.wikipedia.org/wiki/Sus_%28genus%29") if it has exactly
 3 or 4 factors, including 1 and the number itself.
 If, after rolling, the current player's score is a sus number, they gain enough points
 such that their score instantly increases to the next prime number.

 Examples (enable JavaScript)

* *Example 1:*

	+ A player has 14 points and rolls 2 dice that total 7 points.
	 Their new score would be 21, which has 4 factors: 1, 3, 7, and 21.
	 Because 21 is sus, the score of the player is increased to 23, the
	 next prime number.
* *Example 2:*

	+ A player has 63 points and rolls 5 dice that total 1 point.
	 Their new score would be 64, which has 7 factors: 1, 2, 4, 8, 16,
	 32, and 64.
	 Since 64 is not sus, the score of the player is unchanged.
* *Example 3:*

	+ A player has 49 points and rolls 5 dice that total 18 points.
	 Their new score would be 67, which is prime and has 2 factors: 1 and 67.
	 Since 67 is not sus, the score of the player is unchanged.

## Download starter files

To get started, download all of the project code as a [zip archive](hog.zip "hog.zip").
Below is a list of all the files you will see in the archive once unzipped.
For the project, you'll only be making changes to `hog.py`.

* `hog.py`: A starter implementation of Hog
* `dice.py`: Functions for making and rolling dice
* `hog_gui.py`: A graphical user interface (GUI) for Hog (updated)
* `ucb.py`: Utility functions for CS 61A
* `hog_ui.py`: A text-based user interface (UI) for Hog
* `ok`: CS 61A autograder
* `tests`: A directory of tests used by `ok`
* `gui_files`: A directory of various things used by the web GUI

You may notice some files other than the ones listed above too—those are needed for making the autograder and portions of the GUI work. Please do not modify any files other than `hog.py`.

## Logistics

The project is worth 25 points, of which 1 point is for submitting
Phase 1 by the checkpoint date of Tuesday, January 30.

You will turn in the following files:

* `hog.py`

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

## Graphical User Interface

A **graphical user interface** (GUI, for short) is provided for you. At the moment, it doesn't work because you haven't implemented the game logic. Once you complete the play function, you will be able to play a fully interactive version of Hog!

Once you've done that, you can run the GUI from your terminal:

```
python3 hog_gui.py
```

## Getting Started Videos

These videos may provide some helpful direction for tackling the coding
problems on this assignment.

> To see these videos, you should be logged into your berkeley.edu email.
> 
> 

 [YouTube link](https://youtu.be/playlist?list=PLx38hZJ5RLZebgMROlbtGHlmAbOjDegj5 "https://youtu.be/playlist?list=PLx38hZJ5RLZebgMROlbtGHlmAbOjDegj5") 

## Phase 1: Rules of the Game

In the first phase, you will develop a simulator for the game of Hog.

### Problem 0 (0 pt)

The `dice.py` file represents dice using non-pure zero-argument functions.
These functions are non-pure because they may have different return values each
time they are called, and so a side-effect of calling the function is
changing what will be returned when the function is called again.

Here's the documentation from `dice.py` that you need to read in order to
simulate dice in this project.

```
A dice function takes no arguments and returns a number from 1 to n
(inclusive), where n is the number of sides on the dice.

Fair dice produce each possible outcome with equal probability.
Two fair dice are already defined, four_sided and six_sided,
and are generated by the make_fair_dice function.

Test dice are deterministic: they always cycles through a fixed
sequence of values that are passed as arguments.
Test dice are generated by the make_test_dice function.

def make_fair_dice(sides):
    """Return a die that returns 1 to SIDES with equal chance."""
    ...

four_sided = make_fair_dice(4)
six_sided = make_fair_dice(6)

def make_test_dice(...):
    """Return a die that cycles deterministically through OUTCOMES.

    >>> dice = make_test_dice(1, 2, 3)
    >>> dice()
    1
    >>> dice()
    2
    >>> dice()
    3
    >>> dice()
    1
    >>> dice()
    2
```

Check your understanding by unlocking the following tests.

```
python3 ok -q 00 -uCopy✂️
```

 document.getElementById("copy-code-python3ok-q00-u").onclick = () => copyCode('python3 ok -q 00 -u', "copy-code-python3ok-q00-u");

You can exit the unlocker by typing `exit()`.

**Typing Ctrl-C on Windows to exit out of the unlocker has been known to cause
problems, so avoid doing so.**

### Problem 1 (2 pt)

Implement the `roll_dice` function in `hog.py`. It takes two arguments: a
positive integer called `num_rolls` giving the number of times to roll a die and a
`dice` function. It returns the number of points scored by rolling the die
that number of times in a turn: either the sum of the outcomes or 1 *(Sow
Sad)*.

* **Sow Sad**. If any of the dice outcomes is a 1, the current player's score
 for the turn is `1`.

 Examples (enable JavaScript)

* *Example 1:* The current player rolls 7 dice, 5 of which are 1's. They
 score `1` point for the turn.
* *Example 2:* The current player rolls 4 dice, all of which are 3's. Since
 Sow Sad did not occur, they score `12` points for the turn.

To obtain a single outcome of a dice roll, call `dice()`. You should call
`dice()` **exactly `num_rolls` times** in the body of `roll_dice`.

Remember to call `dice()` exactly `num_rolls` times **even if Sow Sad happens
in the middle of rolling**. By doing so, you will correctly simulate rolling
all the dice together (and the user interface will work correctly).

> **Note:** The `roll_dice` function, and many other functions throughout the
> project, makes use of *default argument values*—you can see this in the
> function heading:
> 
> 
> 
> 
```
> def roll_dice(num_rolls, dice=six_sided): ...
> 
```
> 
> The argument `dice=six_sided` means that when `roll_dice` is called, the
> `dice` argument is **optional**. If no value for `dice` is provided, then
> `six_sided` is used by default.
> 
> 
> For example, calling `roll_dice(3, four_sided)`, or equivalently `roll_dice(3,
> dice=four_sided)`, simulates rolling 3 four-sided dice, while calling `roll_dice(3)`
> simulates rolling 3 six-sided dice.
> 
> 

**Understand the problem**:

Before writing any code, unlock the tests to verify your understanding of the question:

```
python3 ok -q 01 -uCopy✂️
```

 document.getElementById("copy-code-python3ok-q01-u").onclick = () => copyCode('python3 ok -q 01 -u', "copy-code-python3ok-q01-u");

> **Note:** You will not be able to test your code using `ok` until you unlock
> the test cases for the corresponding question.
> 
> 

**Write code and check your work**:

Once you are done unlocking, begin implementing your solution. You can check your correctness with:

```
python3 ok -q 01Copy✂️
```

 document.getElementById("copy-code-python3ok-q01").onclick = () => copyCode('python3 ok -q 01', "copy-code-python3ok-q01");

 Debugging Tips (enable JavaScript)

Check out the [Debugging Guide](/articles/debugging/ "/articles/debugging/")!

#### Debugging Tips

If the tests don't pass, it's time to debug. You can observe the behavior of
your function using Python directly. First, start the Python interpreter and
load the `hog.py` file.

```
python3 -i hog.py
```

Then, you can call your `roll_dice` function on any number of dice you want.

```
>>> roll_dice(4)
```

You will find that the previous expression may have a different result each
time you call it, since it is simulating random dice rolls. You can also use
test dice that fix the outcomes of the dice in advance. For example, rolling
twice when you know that the dice will come up 3 and 4 should give a total
outcome of 7.

```
>>> fixed_dice = make_test_dice(3, 4)
>>> roll_dice(2, fixed_dice)
7
```

> On most systems, you can evaluate the same expression again by pressing the
> up arrow, then pressing enter or return. To evaluate earlier commands, press
> the up arrow repeatedly.
> 
> 
> If you find a problem, you first need to change your `hog.py` file to fix the
> problem, and save the file. Then, to check whether your fix works, you'll
> have to quit the Python interpreter by either using `exit()` or `Ctrl^D`, and
> re-run the interpreter to test the changes you made. Pressing the up arrow in
> both the terminal and the Python interpreter should give you access to your
> previous expressions, even after restarting Python.
> 
> 
> Continue debugging your code and running the `ok` tests until they all pass.
> 
> 
> One more debugging tip: to start the interactive interpreter automatically
> upon failing an `ok` test, use `-i`. For example, `python3 ok -q 01 -i` will
> run the tests for question 1, then start an interactive interpreter with
> `hog.py` loaded if a test fails.
> 
> 

### Problem 2 (2 pt)

Implement `boar_brawl`, which takes the player's current score `player_score` and the
opponent's current score `opponent_score`, and returns the number of points scored by
Boar Brawl when the player rolls 0 dice.

* **Boar Brawl**. A player who rolls zero dice scores three times
 the absolute difference between the tens digit of the opponent’s score and
 the ones digit of the current player’s score, or 1, whichever is higher.
 The ones digit refers to the rightmost digit and the tens digit refers to
 the second-rightmost digit. If a player's score is a single digit (less than
 10), the tens digit of that player's score is 0.

 Examples (enable JavaScript)

* *Example 1:*

	+ The current player has `21` points and the opponent has `46` points, and the current player
	 chooses to roll zero dice.
	+ The tens digit of the opponent's score is `4` and the ones digit of the current player's score is `1`.
	+ Therefore, the player gains `3 * abs(4 - 1) = 9` points.
* *Example 2:*

	+ The current player has `45` points and the opponent has `52` points, and the current player
	 chooses to roll zero dice.
	+ The tens digit of the opponent's score is `5` and the ones digit of the current player's score is `5`.
	+ Since `3 * abs(5 - 5) = 0`, the player gains `1` point.
* *Example 3:*

	+ The current player has `2` points and the opponent has `5` points, and the current player
	 chooses to roll zero dice.
	+ The tens digit of the opponent's score is `0` and the ones digit of the current player's score is `2`.
	+ Therefore, the player gains `3 * abs(0 - 2) = 6` points.

> Don't assume that scores are below 100. Write your `boar_brawl` function so
> that it works correctly for any non-negative score.
> 
> 

> **Important:** Your implementation should **not** use `str`, lists, or
> contain square brackets `[` `]`. The test cases will check if those have
> been used.
> 
> 

Before writing any code, unlock the tests to verify your understanding of the question:

```
python3 ok -q 02 -uCopy✂️
```

 document.getElementById("copy-code-python3ok-q02-u").onclick = () => copyCode('python3 ok -q 02 -u', "copy-code-python3ok-q02-u");

 Once you are done unlocking, begin implementing your solution. You can check your correctness with:

```
python3 ok -q 02Copy✂️
```

 document.getElementById("copy-code-python3ok-q02").onclick = () => copyCode('python3 ok -q 02', "copy-code-python3ok-q02");

You can also test `boar_brawl` interactively by running `python3 -i hog.py`
from the terminal and calling `boar_brawl` on various inputs.

### Problem 3 (2 pt)

Implement the `take_turn` function, which returns the number of points scored
for a turn by rolling the given `dice` `num_rolls` times.

Your implementation of `take_turn` should call both `roll_dice` and
`boar_brawl` rather than repeating their implementations.

Before writing any code, unlock the tests to verify your understanding of the question:

```
python3 ok -q 03 -uCopy✂️
```

 document.getElementById("copy-code-python3ok-q03-u").onclick = () => copyCode('python3 ok -q 03 -u', "copy-code-python3ok-q03-u");

 Once you are done unlocking, begin implementing your solution. You can check your correctness with:

```
python3 ok -q 03Copy✂️
```

 document.getElementById("copy-code-python3ok-q03").onclick = () => copyCode('python3 ok -q 03', "copy-code-python3ok-q03");

### Problem 4 (2 pt)

First, implement `num_factors`, which takes in a positive integer `n` and determines the number
of factors that `n` has.

> 1 and `n` are both factors of `n`!
> 
> 

After, implement `sus_points` and `sus_update`.

* `sus_points` takes in a player's score and returns the player's
 new score after applying the Sus Fuss rule (for example, `sus_points(5)` should
 return `5` and `sus_points(21)` should return `23`). You should use `num_factors`
 and the provided `is_prime` function in your implementation.
* `sus_update` returns a player's total score after they roll `num_rolls` dice, taking
 both Boar Brawl and Sus Fuss into account. You should use `sus_points` in
 this function.

> **Hint:** You can look at the implementation of `simple_update` provided in `hog.py` and use that
>  as a starting point for your `sus_update` function.
> 
> 

* **Sus Fuss**.
 We call a number [*sus*](https://en.wikipedia.org/wiki/Sus_%28genus%29 "https://en.wikipedia.org/wiki/Sus_%28genus%29") if it has exactly
 3 or 4 factors, including 1 and the number itself.
 If, after rolling, the current player's score is a sus number, they gain enough points
 such that their score instantly increases to the next prime number.

 Examples (enable JavaScript)

* *Example 1:*

	+ A player has 14 points and rolls 2 dice that total 7 points.
	 Their new score would be 21, which has 4 factors: 1, 3, 7, and 21.
	 Because 21 is sus, the score of the player is increased to 23, the
	 next prime number.
* *Example 2:*

	+ A player has 63 points and rolls 5 dice that total 1 point.
	 Their new score would be 64, which has 7 factors: 1, 2, 4, 8, 16,
	 32, and 64.
	 Since 64 is not sus, the score of the player is unchanged.
* *Example 3:*

	+ A player has 49 points and rolls 5 dice that total 18 points.
	 Their new score would be 67, which is prime and has 2 factors: 1 and 67.
	 Since 67 is not sus, the score of the player is unchanged.

Before writing any code, unlock the tests to verify your understanding of the question:

```
python3 ok -q 04 -uCopy✂️
```

 document.getElementById("copy-code-python3ok-q04-u").onclick = () => copyCode('python3 ok -q 04 -u', "copy-code-python3ok-q04-u");

 Once you are done unlocking, begin implementing your solution. You can check your correctness with:

```
python3 ok -q 04Copy✂️
```

 document.getElementById("copy-code-python3ok-q04").onclick = () => copyCode('python3 ok -q 04', "copy-code-python3ok-q04");

