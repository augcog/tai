

# 

Discussion 3: Recursion

 * [disc03.pdf](disc03.pdf "disc03.pdf")

Pick someone in your group to [join Discord](https://cs61a.org/articles/discord "https://cs61a.org/articles/discord").
It's fine if multiple people join, but one is enough.

Now switch to Pensieve:

* **Everyone**: Go to [discuss.pensieve.co](http://discuss.pensieve.co "http://discuss.pensieve.co") and log in with your @berkeley.edu email, then enter your group number. (Your group number is the number of your Discord channel.)

Once you're on Pensieve, you don't need to return to this page; Pensieve has all the same content (but more features). If for some reason Penseive doesn't work, return to this page and continue with the discussion.

Post in the `#help` channel on [Discord](https://cs61a.org/articles/discord/ "https://cs61a.org/articles/discord/") if you have trouble.

# Getting Started

Say your name and share a food that you really liked as a child. (It's ok if you still like that food now.)

**Suggestion:** After Midterm 1, some students are looking for more effective
ways to study. One great option is to meet up with your discussion group outside
of class to review practice problems together. Now is a great time to schedule a
time and place for some extra group practice of old Midterm 1 questions. This is
optional and not everyone needs to come, but if there are Midterm 1 topics that
haven't totally clicked yet, this weekend is a perfect time to review them.

Everything in this course builds on prior topics, and it's going to be hard to
keep up if you don't have a solid understanding of Midterm 1 material.

Remember, it's ok if someone hasn't learned everything yet and needs more time
to master the course material. The whole point of the course is for students to
learn things they don't already know. Please support each other in the process.

# Recursion

Ok, time to discuss problems! Remember to work together. Everyone in the group
should understand a solution before the group moves on. Many students find this discussion challenging. Everything gets easier with practice.

**VERY IMPORTANT:** In this discussion, don't check your answers until your whole group is sure that the answer is right. Figure things out and check your work by *thinking* about what your code will do. Your goal should be to have all checks pass the first time you run them! If you need help, ask.

### Q1: Swipe

Implement `swipe`, which prints the digits of argument `n`, one per line, first backward then forward. The left-most digit is printed only once. Do not use `while` or `for` or `str`. (Use recursion, of course!)

**Your Answer**

[Run in 61A Code](javascript:void "javascript:void")

 $(() => activateEditor('def swipe(n):\n &quot;&quot;&quot;Print the digits of n, one per line, first backward then forward.\n\n &gt;&gt;&gt; swipe(2837)\n 7\n 3\n 8\n 2\n 8\n 3\n 7\n &quot;&quot;&quot;\n if n &lt; 10:\n print(n)\n else:\n "\*\*\* YOUR CODE HERE \*\*\*"\n\n', "python", "swipe-input"));

**Solution**

```
def swipe(n):
    """Print the digits of n, one per line, first backward then forward.

    >>> swipe(2837)
    7
    3
    8
    2
    8
    3
    7
    """
    if n < 10:
        print(n)
    else:
        print(n % 10)
        swipe(n // 10)
        print(n % 10)

```

 Hint (enable JavaScript)

First `print` the first line of the output, then make a recursive call, then `print` the last line of the output.

### Q2: Skip Factorial

Define the base case for the `skip_factorial` function, which returns the product of every other positive integer, starting with `n`.

**Your Answer**

[Run in 61A Code](javascript:void "javascript:void")

 $(() => activateEditor('def skip\_factorial(n):\n &quot;&quot;&quot;Return the product of positive integers n \* (n &#x2d; 2) \* (n &#x2d; 4) \* ...\n\n &gt;&gt;&gt; skip\_factorial(5) # 5 \* 3 \* 1\n 15\n &gt;&gt;&gt; skip\_factorial(8) # 8 \* 6 \* 4 \* 2\n 384\n &quot;&quot;&quot;\n if \_\_\_:\n return \_\_\_\n else:\n return n \* skip\_factorial(n &#x2d; 2)\n', "python", "skip\_factorial-input"));

**Solution**

```
def skip_factorial(n):
    """Return the product of positive integers n * (n - 2) * (n - 4) * ...

    >>> skip_factorial(5) # 5 * 3 * 1
    15
    >>> skip_factorial(8) # 8 * 6 * 4 * 2
    384
    """
    if n <= 2:
        return n
    else:
        return n * skip_factorial(n - 2)
```

 Hint (enable JavaScript)

If `n` is even, then the base case will be 2. If `n` is odd, then the base case will be 1. Try to write a condition that handles both possibilities.

### Q3: Is Prime

Implement `is_prime` that takes an integer `n` greater than 1. It returns `True`
if `n` is a prime number and `False` otherwise. Try following the approach
below, but implement it recursively without using a `while` (or `for`)
statement.

```
def is_prime(n):
    assert n > 1
    i = 2
    while i < n:
        if n % i == 0:
            return False
        i = i + 1
    return True
```

You will need to define another "helper" function (a function that exists just
to help implement this one). Does it matter whether you define it within
`is_prime` or as a separate function in the global frame? Try to define it to
take as few arguments as possible.

**Your Answer**

[Run in 61A Code](javascript:void "javascript:void")

 $(() => activateEditor('def is\_prime(n):\n &quot;&quot;&quot;Returns True if n is a prime number and False otherwise.\n &gt;&gt;&gt; is\_prime(2)\n True\n &gt;&gt;&gt; is\_prime(16)\n False\n &gt;&gt;&gt; is\_prime(521)\n True\n &quot;&quot;&quot;\n "\*\*\* YOUR CODE HERE \*\*\*"\n\n', "python", "is-prime-input"));

**Solution**

```
def is_prime(n):
    """Returns True if n is a prime number and False otherwise.
    >>> is_prime(2)
    True
    >>> is_prime(16)
    False
    >>> is_prime(521)
    True
    """
    def check_all(i):
        "Check whether no number from i up to n evenly divides n."
        if i == n:      # could be replaced with i > (n ** 0.5)
            return True
        elif n % i == 0:
            return False
        return check_all(i + 1)
    return check_all(2)

```

 Hint (enable JavaScript)

Define an inner function that checks whether some integer between `i` and `n` evenly divides `n`. Then you can call it starting with `i=2`:

```
def is_prime(n):
    def f(i):
        if n % i == 0:
            return ____
        elif ____:
            return ____
        else:
            return f(____)
    return f(2)
```

Come up with a one sentence docstring for the helper function that describes what it does.
Don't just write, "it helps implement `is_prime`." Instead, describe its
behavior. When you're done, paste the text of that docstring in your group's
[channel's text
chat](https://support.discord.com/hc/en-us/articles/4412085582359-Text-Channels-Text-Chat-In-Voice-Channels#h_01FMJT412WBX1MR4HDYNR8E95X "https://support.discord.com/hc/en-us/articles/4412085582359-Text-Channels-Text-Chat-In-Voice-Channels#h_01FMJT412WBX1MR4HDYNR8E95X").

### Q4: Recursive Hailstone

Recall the `hailstone` function from [Homework 1](/hw/hw01/ "/hw/hw01/").
First, pick a positive integer `n` as the start. If `n` is even, divide it by 2.
If `n` is odd, multiply it by 3 and add 1. Repeat this process until `n` is 1.
Complete this recursive version of `hailstone` that prints out the values of the
sequence and returns the number of steps.

**Your Answer**

[Run in 61A Code](javascript:void "javascript:void")

 $(() => activateEditor('def hailstone(n):\n &quot;&quot;&quot;Print out the hailstone sequence starting at n, \n and return the number of elements in the sequence.\n &gt;&gt;&gt; a = hailstone(10)\n 10\n 5\n 16\n 8\n 4\n 2\n 1\n &gt;&gt;&gt; a\n 7\n &gt;&gt;&gt; b = hailstone(1)\n 1\n &gt;&gt;&gt; b\n 1\n &quot;&quot;&quot;\n print(n)\n if n % 2 == 0:\n return even(n)\n else:\n return odd(n)\n\ndef even(n):\n return \_\_\_\_\n\ndef odd(n):\n "\*\*\* YOUR CODE HERE \*\*\*"\n\n', "python", "recursive-hailstone-input"));

**Solution**

```
def hailstone(n):
    """Print out the hailstone sequence starting at n, 
    and return the number of elements in the sequence.
    >>> a = hailstone(10)
    10
    5
    16
    8
    4
    2
    1
    >>> a
    7
    >>> b = hailstone(1)
    1
    >>> b
    1
    """
    print(n)
    if n % 2 == 0:
        return even(n)
    else:
        return odd(n)

def even(n):
    return 1 + hailstone(n // 2)

def odd(n):
    if n == 1:
        return 1
    else:
        return 1 + hailstone(3 * n + 1)

```

 Hint (enable JavaScript)

An even number is never a base case, so `even` always makes a recursive call to `hailstone` and returns one more than the length of the rest of the hailstone sequence.

An odd number might be 1 (the base case) or greater than one (the recursive case). Only the recursive case should call `hailstone`.

Once your group has converged on a solution, it's time to practice your ability
to describe your own code. Pick a presenter, then send a message to the
`discuss-queue` channel with the @discuss tag, your discussion group number, and
the message "Hailing all course staff!" and a member of the course staff will
join your voice channel to hear your description.

# Document the Occasion

Please all fill out the [attendance form](https://docs.google.com/forms/d/e/1FAIpQLSeqlK8l6WkScGr-RHR-kM4p5bnR9cllYrG95fDqPJspSlll7A/viewform "https://docs.google.com/forms/d/e/1FAIpQLSeqlK8l6WkScGr-RHR-kM4p5bnR9cllYrG95fDqPJspSlll7A/viewform") (one submission per person per week).

# Extra Challenge

You'll need your whole discussion group for this question. At least try it out. You might have fun. We'll review the question in lecture on Friday.

### Q5: Sevens

**The Game of Sevens**: Players in a circle count up from 1 in the clockwise
direction. (The starting player says 1, the player to their left says 2, etc.) If a
number is divisible by 7 or contains a 7 (or both), switch directions. Numbers
must be said on the beat at [60 beats per
minute](https://www.youtube.com/watch?v=ymJIXzvDvj4 "https://www.youtube.com/watch?v=ymJIXzvDvj4"). If someone says a number
when it's not their turn or someone misses the beat on their turn, the game
ends.

For example, 5 people would count to 20 like this:

```
Player 1 says 1
Player 2 says 2
Player 3 says 3
Player 4 says 4
Player 5 says 5
Player 1 says 6  # All the way around the circle
Player 2 says 7  # Switch to counterclockwise
Player 1 says 8
Player 5 says 9  # Back around the circle counterclockwise
Player 4 says 10
Player 3 says 11
Player 2 says 12
Player 1 says 13
Player 5 says 14 # Switch back to clockwise
Player 1 says 15
Player 2 says 16
Player 3 says 17 # Switch back to counterclockwise
Player 2 says 18
Player 1 says 19
Player 5 says 20
```

Play a few games. Post the highest score your group reached on Discord.

Then, implement `sevens` which takes a positive integer `n` and a number of
players `k`. It returns which of the `k` players says `n`. You may call
`has_seven`.

An effective approach to this problem is to simulate the game, stopping on turn
`n`. The implementation must keep track of the final number `n`, the current
number `i`, the player `who` will say `i`, and the current `direction` that
determines the next player (either increasing or decreasing). It works well to
use integers to represent all of these, with `direction` switching between `1`
(increase) and `-1` (decreasing).

**Your Answer**

[Run in 61A Code](javascript:void "javascript:void")

 $(() => activateEditor('def sevens(n, k):\n &quot;&quot;&quot;Return the (clockwise) position of who says n among k players.\n\n &gt;&gt;&gt; sevens(2, 5)\n 2\n &gt;&gt;&gt; sevens(6, 5)\n 1\n &gt;&gt;&gt; sevens(7, 5)\n 2\n &gt;&gt;&gt; sevens(8, 5)\n 1\n &gt;&gt;&gt; sevens(9, 5)\n 5\n &gt;&gt;&gt; sevens(18, 5)\n 2\n &quot;&quot;&quot;\n def f(i, who, direction):\n if i == n:\n return who\n "\*\*\* YOUR CODE HERE \*\*\*"\n return f(1, 1, 1)\n\ndef has\_seven(n):\n if n == 0:\n return False\n elif n % 10 == 7:\n return True\n else:\n return has\_seven(n // 10)\n', "python", "sevens-input"));

**Solution**

```
def sevens(n, k):
    """Return the (clockwise) position of who says n among k players.

    >>> sevens(2, 5)
    2
    >>> sevens(6, 5)
    1
    >>> sevens(7, 5)
    2
    >>> sevens(8, 5)
    1
    >>> sevens(9, 5)
    5
    >>> sevens(18, 5)
    2
    """
    def f(i, who, direction):
        if i == n:
            return who
        if i % 7 == 0 or has_seven(i):
            direction = -direction
        who = who + direction
        if who > k:
            who = 1
        if who < 1:
            who = k
        return f(i + 1, who, direction)
    return f(1, 1, 1)

def has_seven(n):
    if n == 0:
        return False
    elif n % 10 == 7:
        return True
    else:
        return has_seven(n // 10)
```

 Hint (enable JavaScript)

First check if `i` is a multiple of 7 or contains a 7, and if so, switch
directions. Then, add the direction to `who` and ensure that `who` has not
become smaller than 1 or greater than `k`.

### Q6: Karel the Robot

[Karel the
robot](https://compedu.stanford.edu/karel-reader/docs/python/en/chapter1.html "https://compedu.stanford.edu/karel-reader/docs/python/en/chapter1.html")
starts in the corner of an `n` by `n` square for some unknown
number `n`. Karel responds to only four functions:

* `move()` moves Karel one square forward if there is no wall in front of Karel and errors if there is.
* `turn_left()` turns Karel 90 degrees to the left.
* `front_is_blocked()` returns whether there is a wall in front of Karel.
* `front_is_clear()` returns whether there is no wall in front of Karel.

Implement a `main()` function that will leave Karel stopped halfway in the
middle of the bottom row. For example, if the square is 7 x 7 and Karel starts
in position (1, 1), the bottom left, then Karel should end in position (1, 4)
(three steps from either side on the bottom row). Karel can be facing in any
direction at the end. If the bottom row length is even, Karel can stop in either
position (1, `n // 2`) or (1, `n // 2 + 1`).

**Important** You can only write `if` or `if`/`else` statements and function
calls in the body of `main()`. You may not write assignment statements, def
statements, lambda expressions, or while/for statements.

 Hint (enable JavaScript)

For every two steps forward, take one step back to end up in the middle.

 $('.alwaystoggle').css('display', 'inline-block');
 $('.alwaystoggle').click(function() {
 var solution\_id = $(this).attr('id');
 $('div.' + solution\_id).slideToggle(600);
 });

