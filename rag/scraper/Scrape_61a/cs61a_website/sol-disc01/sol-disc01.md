

# 

Discussion 1: Control, Environment Diagrams

 * [disc01.pdf](disc01.pdf "disc01.pdf")

What everyone will need:

* A way to view this worksheet (on your phone is fine)
* A way to take notes (paper or a device)

Pick someone in your group to [join Discord](https://cs61a.org/articles/discord "https://cs61a.org/articles/discord"), find your group's channel, and post "Here we go!" in your group's Discord [text channel](https://support.discord.com/hc/en-us/articles/4412085582359-Text-Channels-Text-Chat-In-Voice-Channels#h_01FMJT412WBX1MR4HDYNR8E95X "https://support.discord.com/hc/en-us/articles/4412085582359-Text-Channels-Text-Chat-In-Voice-Channels#h_01FMJT412WBX1MR4HDYNR8E95X"). It's fine if multiple people join, but one is enough.

**Find your facilitator:** Now figure out who will be your facilitator for the rest of the discussion.

* If you know you're the facilitator, let your group know.
* If no one is the facilitator, then whoever has the brightest phone case is the facilitator.

Now switch to Pensieve:

* **Everyone**: Go to [discuss.pensieve.co](http://discuss.pensieve.co "http://discuss.pensieve.co") and log in with your @berkeley.edu email.
* **Facilitator**: Click *Create Room*, pick this discussion, and share the room code with your group.
* **Others**: Don't create a room; instead, click "Join Room" after typing in the room code that the facilitator shared with you.
* Post in the `#help` channel on Discord if you have trouble.

Once you're on Pensieve, you don't need to return to this page; Pensieve has all the same content (but more features). If for some reason Penseive doesn't work, return to this page and continue with the discussion.

# Expressions

Say your name and an expression that people say when they really like something, such as "that's awesome" or "nice one". Each person should try to come up with a different expression. Feel free to ask your group for help if you're stuck. You can even use other languages than English. Now you have things to say later in the discussion when the group solves a problem.

# While and If

Learning to use `if` and `while` is an essential skill. During this discussion,
focus on what we've studied in the first three lectures: `if`, `while`,
assignment (`=`), comparison (`<`, `>`, `==`, ...), and arithmetic. Please don't
use features of Python that we haven't discussed in class yet, such as `for`,
`range`, and lists. We'll have plenty of time for those later in the course, but
now is the time to practice the use of `if` (textbook section
[1.5.4](https://www.composingprograms.com/pages/15-control.html#conditional-statements "https://www.composingprograms.com/pages/15-control.html#conditional-statements"))
and `while` (textbook section
[1.5.5](https://www.composingprograms.com/pages/15-control.html#conditional-statements "https://www.composingprograms.com/pages/15-control.html#conditional-statements")).

### Q1: Race

The `race` function below sometimes returns the wrong value and sometimes runs forever.

**Your Answer**

[Run in 61A Code](javascript:void "javascript:void")

 $(() => activateEditor('def race(x, y):\n &quot;&quot;&quot;The tortoise always walks x feet per minute, while the hare repeatedly\n runs y feet per minute for 5 minutes, then rests for 5 minutes. Return how\n many minutes pass until the tortoise first catches up to the hare.\n\n &gt;&gt;&gt; race(5, 7) # After 7 minutes, both have gone 35 steps\n 7\n &gt;&gt;&gt; race(2, 4) # After 10 minutes, both have gone 20 steps\n 10\n &quot;&quot;&quot;\n assert y &gt; x and y &lt;= 2 \* x, &#x27;the hare must be fast but not too fast&#x27;\n tortoise, hare, minutes = 0, 0, 0\n while minutes == 0 or tortoise &#x2d; hare:\n tortoise += x\n if minutes % 10 &lt; 5:\n hare += y\n minutes += 1\n return minutes\n', "python", "race-input"));

**Solution**

```
def race(x, y):
    """The tortoise always walks x feet per minute, while the hare repeatedly
    runs y feet per minute for 5 minutes, then rests for 5 minutes. Return how
    many minutes pass until the tortoise first catches up to the hare.

    >>> race(5, 7)  # After 7 minutes, both have gone 35 steps
    7
    >>> race(2, 4) # After 10 minutes, both have gone 20 steps
    10
    """
    assert y > x and y <= 2 * x, 'the hare must be fast but not too fast'
    tortoise, hare, minutes = 0, 0, 0
    while minutes == 0 or tortoise - hare:
        tortoise += x
        if minutes % 10 < 5:
            hare += y
        minutes += 1
    return minutes
```

Find positive integers `x` and `y` (with `y` larger than `x` but
not larger than `2 * x`) for which either:

* `race(x, y)` returns the wrong value or
* `race(x, y)` runs forever

You just need to find one pair of numbers that satisfies either of these
conditions to finish the question, but if you want to think of more you can.

Notes:

* `x += 1` is the same as `x = x + 1` when `x` is assigned to a number.
* 0 is a false value and all other numbers are true values.

 Hint (enable JavaScript)

The value of `race(x, y)` is incorrect when it is not the **first** time the
tortoise passes the hare. Try some small numbers (below 5) to see if you can
find a case where `tortoise` has become larger than `hare`, but the expression
`tortoise - hare` was not zero when it happened.

The return value is incorrect when the time that the tortoise first passes the
hare is not an integer number of minutes (e.g., for `race(2, 3)` the tortoise
passes the hare after 7.5 minutes), but there is some (larger) integer number of
minutes after which both animals have gone the same distance.

The `race` function will run forever if the only times that the tortoise and
hare have gone the same distance are not integers (e.g., for `race(4, 5)` the
tortoise passes the hare after 6.2 minutes, and the hare never catches up).

Put your `(x, y)` examples in the [text chat](https://support.discord.com/hc/en-us/articles/4412085582359-Text-Channels-Text-Chat-In-Voice-Channels#h_01FMJT412WBX1MR4HDYNR8E95X "https://support.discord.com/hc/en-us/articles/4412085582359-Text-Channels-Text-Chat-In-Voice-Channels#h_01FMJT412WBX1MR4HDYNR8E95X") of your group's voice channel. If you get
stuck for more than 10 minutes, ask for help!

### Q2: Fizzbuzz

Implement the classic [*Fizz Buzz*
sequence](https://en.wikipedia.org/wiki/Fizz_buzz "https://en.wikipedia.org/wiki/Fizz_buzz"). The `fizzbuzz` function
takes a positive integer `n` and prints out a *single line* for each integer
from 1 to `n`. For each `i`:

* If `i` is divisible by both 3 and 5, print `fizzbuzz`.
* If `i` is divisible by 3 (but not 5), print `fizz`.
* If `i` is divisible by 5 (but not 3), print `buzz`.
* Otherwise, print the number `i`.

Try to make your implementation of `fizzbuzz` concise.

**Your Answer**

[Run in 61A Code](javascript:void "javascript:void")

 $(() => activateEditor('def fizzbuzz(n):\n &quot;&quot;&quot;\n &gt;&gt;&gt; result = fizzbuzz(16)\n 1\n 2\n fizz\n 4\n buzz\n fizz\n 7\n 8\n fizz\n buzz\n 11\n fizz\n 13\n 14\n fizzbuzz\n 16\n &gt;&gt;&gt; print(result)\n None\n &quot;&quot;&quot;\n "\*\*\* YOUR CODE HERE \*\*\*"\n\n', "python", "fizzbuzz-input"));

**Solution**

```
def fizzbuzz(n):
    """
    >>> result = fizzbuzz(16)
    1
    2
    fizz
    4
    buzz
    fizz
    7
    8
    fizz
    buzz
    11
    fizz
    13
    14
    fizzbuzz
    16
    >>> print(result)
    None
    """
    i = 1
    while i <= n:
        if i % 3 == 0 and i % 5 == 0:
            print('fizzbuzz')
        elif i % 3 == 0:
            print('fizz')
        elif i % 5 == 0:
            print('buzz')
        else:
            print(i)
        i += 1

```

 Hint (enable JavaScript)

Be careful about the order of your `if` and `elif` clauses: try first checking if the current number is divisible by both 3 and 5, then check for just divisibility by 3 and just divisibility by 5.

[Video walkthrough](https://youtu.be/-Y2JdHG1aK8 "https://youtu.be/-Y2JdHG1aK8")

# Problem Solving

A useful approach to implementing a function is to:

1. Pick an example input and corresponding output.
2. Describe a process (in English) that computes the output from the input using simple steps.
3. Figure out what additional names you'll need to carry out this process.
4. Implement the process in code using those additional names.
5. Determine whether the implementation really works on your original example.
6. Determine whether the implementation really works on other examples. (If not, you might need to revise step 2.)

Importantly, this approach doesn't go straight from reading a question to writing code.

For example, in the `is_prime` problem below, you could:

1. Pick `n` is 9 as the input and `False` as the output.
2. Here's a process: Check that `9` (`n`) is not a multiple of any integers between 1 and `9` (`n`).
3. Introduce `i` to represent each number between 1 and 9 (`n`).
4. Implement `is_prime` (you get to do this part with your group).
5. Check that `is_prime(9)` will return `False` by thinking through the execution of the code.
6. Check that `is_prime(3)` will return `True` and `is_prime(1)` will return `False`.

Try this approach together on the next two problems.

**Important:** It's highly recommended that you **don't** check your work using
a computer right away. Instead, talk to your group and think to try to figure
out if an answer is correct. On exams, you won't be able to guess and check
because you won't have a Python interpreter. Now is a great time to practice
checking your work by thinking through examples. You could even draw an
environment diagram!

If you're not sure about how something works or get stuck, ask for help from the course staff.

### Q3: Is Prime?

Write a function that returns `True` if a positive integer `n` is a prime
number and `False` otherwise.

A prime number n is a number that is not divisible by any numbers other than
1 and n itself. For example, 13 is prime, since it is only divisible by 1 and
13, but 14 is not, since it is divisible by 1, 2, 7, and 14.

Use the `%` operator: `x % y` returns the remainder of `x` when divided by `y`.

 Hint (enable JavaScript)

Here's a `while` statement that goes through all numbers above 1 and below `n` :

```
i = 2
while i < n:
    ...
    i = i + 1
```

You can use `n % i == 0` to check whether `i` is a factor of `n`. If it is, `return False`.

**Your Answer**

[Run in 61A Code](javascript:void "javascript:void")

 $(() => activateEditor('def is\_prime(n):\n &quot;&quot;&quot;\n &gt;&gt;&gt; is\_prime(10)\n False\n &gt;&gt;&gt; is\_prime(7)\n True\n &gt;&gt;&gt; is\_prime(1) # one is not a prime number!!\n False\n &quot;&quot;&quot;\n "\*\*\* YOUR CODE HERE \*\*\*"\n\n', "python", "is\_prime-input"));

**Solution**

```
def is_prime(n):
    """
    >>> is_prime(10)
    False
    >>> is_prime(7)
    True
    >>> is_prime(1) # one is not a prime number!!
    False
    """
    if n == 1:
        return False
    k = 2
    while k < n:
        if n % k == 0:
            return False
        k += 1
    return True

```

**Presentation Time**: Come up with a **one sentence description** of the process you implemented to solve `is_prime` that you think someone could understand without looking at your code. Pick someone to present, then send a message to the `discuss-queue` channel with the `@discuss` tag, your discussion group number, and the message "Prime Time!" and a member of the course staff will join your voice channel to hear your description and give feedback.

### Q4: Unique Digits

Write a function that returns the number of unique digits in a positive integer.

> **Hints:** You can use `//` and `%` to separate a positive integer into
> its one's digit and the rest of its digits.
> 
> 
> You may find it helpful to first define a function
> `has_digit(n, k)`, which determines whether a number `n` has digit `k`.
> 
> 

**Your Answer**

[Run in 61A Code](javascript:void "javascript:void")

 $(() => activateEditor('def unique\_digits(n):\n &quot;&quot;&quot;Return the number of unique digits in positive integer n.\n\n &gt;&gt;&gt; unique\_digits(8675309) # All are unique\n 7\n &gt;&gt;&gt; unique\_digits(13173131) # 1, 3, and 7\n 3\n &gt;&gt;&gt; unique\_digits(101) # 0 and 1\n 2\n &quot;&quot;&quot;\n "\*\*\* YOUR CODE HERE \*\*\*"\n\ndef has\_digit(n, k):\n &quot;&quot;&quot;Returns whether k is a digit in n.\n\n &gt;&gt;&gt; has\_digit(10, 1)\n True\n &gt;&gt;&gt; has\_digit(12, 7)\n False\n &quot;&quot;&quot;\n assert k &gt;= 0 and k &lt; 10\n "\*\*\* YOUR CODE HERE \*\*\*"\n\n', "python", "unique\_digits-input"));

**Solution**

```
def unique_digits(n):
    """Return the number of unique digits in positive integer n.

    >>> unique_digits(8675309) # All are unique
    7
    >>> unique_digits(13173131) # 1, 3, and 7
    3
    >>> unique_digits(101) # 0 and 1
    2
    """
    unique = 0
    while n > 0:
        last = n % 10
        n = n // 10
        if not has_digit(n, last):
            unique += 1
    return unique

# Alternate solution
def unique_digits_alt(n):
    unique = 0
    i = 0
    while i < 10:
        if has_digit(n, i):
            unique += 1
        i += 1
    return unique

def has_digit(n, k):
    """Returns whether k is a digit in n.

    >>> has_digit(10, 1)
    True
    >>> has_digit(12, 7)
    False
    """
    assert k >= 0 and k < 10
    while n > 0:
        last = n % 10
        n = n // 10
        if last == k:
            return True
    return False

```

 Hint (enable JavaScript)

One approach is to loop through every digit from 0 to 9 and check whether `n` has the digit. Count up the ones it has.

We have provided two solutions:

* In one solution, we look at the current digit, and check if the rest of the
 number contains that digit or not. We only say it's unique if the digit
 doesn't exist in the rest. We do this for every digit.
* In the other, we loop through the numbers 0-9 and just call `has_digit` on
 each one. If it returns true then we know the entire number contains that
 digit and we can one to our unique count.

# Environment Diagrams

An **environment diagram** keeps track of names and their values in frames,
which are drawn as boxes.

### Q5: Bottles

Answer the following questions with your group. Step
through the diagram to check your answers.

1) What determines how many different frames appear in an environment diagram?

* The number of functions defined in the code
* The number of call expressions in the code
* The number of return statements in the code
* The number of times user-defined functions are called when running the code

2) What happens to the return value of `pass(bottles)`?

* It is used as the new value of `remaining` in the global frame
* It is used as the new value of `bottles` in the global frame
* It is used as the new value of `pass` in the global frame
* None of the above

3) What effect does the line `bottles = 98` have on the global frame?

* It temporarily changes the value bound to `bottles` in the global frame.
* It permanently changes the value bound to `bottles` in the global frame.
* It has no effect on the global frame.

### Q6: Double Trouble

Draw the environment diagram on paper or a whiteboard (without having the
computer draw it for you)! Then, check your work by stepping through the
diagram.

We first define the two functions `double` and `triple`, each bound to their
corresponding name. In the next line, we assign the name `hat` to the function
object that `double` refers to. Finally, we assign the name `double` to the
function object that `triple` refers to.

[Video walkthrough](https://youtu.be/EuygSqH8nTk "https://youtu.be/EuygSqH8nTk")

# Document the occasion

If you'd like, take another group selfie and add it to the text chat of your group's Discord channel.
Then, please all fill out the [attendance form](https://docs.google.com/forms/d/e/1FAIpQLSeqlK8l6WkScGr-RHR-kM4p5bnR9cllYrG95fDqPJspSlll7A/viewform "https://docs.google.com/forms/d/e/1FAIpQLSeqlK8l6WkScGr-RHR-kM4p5bnR9cllYrG95fDqPJspSlll7A/viewform") (one submission per person per week).

 $('.alwaystoggle').css('display', 'inline-block');
 $('.alwaystoggle').click(function() {
 var solution\_id = $(this).attr('id');
 $('div.' + solution\_id).slideToggle(600);
 });

