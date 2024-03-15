

# 

Lab 3 Solutions

 * [lab03.zip](lab03.zip "lab03.zip")

## Solution Files

## Topics

Consult this section if you need a refresher on the material for this lab. It's
okay to skip directly to [the questions](#required-questions "#required-questions") and refer back
here should you get stuck.

 Lists (enable JavaScript)

## Lists

A list is a data structure that can hold an ordered collection of items.
These items, known as elements, can be of any data type, including numbers, strings, or even other lists.
A comma-separated list of expressions in square brackets creates a list:

```
>>> list_of_values = [2, 1, 3, True, 3]
>>> nested_list = [2, [1, 3], [True, [3]]]
```

Each position in a list has an index, with the left-most element indexed `0`.

```
>>> list_of_values[0]
2
>>> nested_list[1]
[1, 3]
```

A negative index counts from the end, with the right-most element indexed `-1`.

```
>>> nested_list[-1]
[True, [3]]
```

Adding lists creates a longer list containing the elements of the added lists.

```
>>> [1, 2] + [3] + [4, 5]
[1, 2, 3, 4, 5]
```

 List Comprehensions (enable JavaScript)

## List Comprehensions

A list comprehension describes the elements in a list and evaluates to a new list
containing those elements.

There are two forms:

```
[<expression> for <element> in <sequence>]
[<expression> for <element> in <sequence> if <conditional>]
```

Here's an example that starts with `[1, 2, 3, 4]`, picks out the even elements
`2` and `4` using `if i % 2 == 0`, then squares each of these using `i*i`. The
purpose of `for i` is to give a name to each element in `[1, 2, 3, 4]`.

```
>>> [i*i for i in [1, 2, 3, 4] if i % 2 == 0]
[4, 16]
```

This list comprehension evaluates to a list of:

* The value of `i*i`
* For each element `i` in the sequence `[1, 2, 3, 4]`
* For which `i % 2 == 0`

In other words, this list comprehension will create a new list that contains
the square of every even element of the original list `[1, 2, 3, 4]`.

We can also rewrite a list comprehension as an equivalent `for` statement,
such as for the example above:

```
>>> result = []
>>> for i in [1, 2, 3, 4]:
...     if i % 2 == 0:
...         result = result + [i*i]
>>> result
[4, 16]
```

 For Loops (enable JavaScript)

## For Statements

A `for` statement executes code for each element of a sequence, such as a list or range. Each time the code is executed, the name right after `for` is bound to a different element of the sequence.

```
for <name> in <expression>:
    <suite>
```

First, `<expression>` is evaluated. It must evaluate to a sequence. Then, for each element in the sequence in order,

1. `<name>` is bound to the element.
2. `<suite>` is executed.

Here is an example:

```
for x in [-1, 4, 2, 0, 5]:
    print("Current elem:", x)
```

This would display the following:

```
Current elem: -1
Current elem: 4
Current elem: 2
Current elem: 0
Current elem: 5
```

 Ranges (enable JavaScript)

## Ranges

A range is a data structure that holds integer sequences. A range can be created by:

* `range(stop)` contains 0, 1, ..., `stop` - 1
* `range(start, stop)` contains `start`, `start` + 1, ..., `stop` - 1

Notice how the range function doesn't include the `stop` value; it generates numbers up to, but not including, the `stop` value.

For example:

```
>>> for i in range(3):
...     print(i)
...
0
1
2
```

While ranges and lists are both [sequences](https://en.wikibooks.org/wiki/Python_Programming/Sequences "https://en.wikibooks.org/wiki/Python_Programming/Sequences"), a range object is different from a list. A range can be converted to a list by calling `list()`:

```
>>> range(3, 6)
range(3, 6)  # this is a range object
>>> list(range(3, 6))
[3, 4, 5]  # list() converts the range object to a list
>>> list(range(5))
[0, 1, 2, 3, 4]
>>> list(range(1, 6))
[1, 2, 3, 4, 5]
```

# Required Questions

## Lists

### Q1: WWPD: Lists & Ranges

> Use Ok to test your knowledge with the following "What Would Python Display?" questions:
> 
> 
> 
```
> python3 ok -q lists-wwpd -uCopy✂️
> 
```
> 
> 
>  document.getElementById("copy-code-python3ok-qlists-wwpd-u").onclick = () => copyCode('python3 ok -q lists-wwpd -u', "copy-code-python3ok-qlists-wwpd-u");
>  
>   
> 

Predict what Python will display when you type the following into the
interactive interpreter. Then try it to check your answers.

```
>>> s = [7//3, 5, [4, 0, 1], 2]
>>> s[0]
\_\_\_\_\_\_2
>>> s[2]
\_\_\_\_\_\_[4, 0, 1]
>>> s[-1]
\_\_\_\_\_\_2
>>> len(s)
\_\_\_\_\_\_4
>>> 4 in s
\_\_\_\_\_\_False
>>> 4 in s[2]
\_\_\_\_\_\_True
>>> s[2] + [3 + 2]
\_\_\_\_\_\_[4, 0, 1, 5]
>>> 5 in s[2]
\_\_\_\_\_\_False
>>> s[2] * 2
\_\_\_\_\_\_[4, 0, 1, 4, 0, 1]
>>> list(range(3, 6))
\_\_\_\_\_\_[3, 4, 5]
>>> range(3, 6)
\_\_\_\_\_\_range(3, 6)
>>> r = range(3, 6)
>>> [r[0], r[2]]
\_\_\_\_\_\_[3, 5]
>>> range(4)[-1]
\_\_\_\_\_\_3
```

 Toggle Solution (enable JavaScript)

### Q2: Print If

Implement `print_if`, which takes a list `s` and a one-argument function `f`. It prints each element `x` of `s` for which `f(x)` returns a true value.

```
def print_if(s, f):
    """Print each element of s for which f returns a true value.

    >>> print_if([3, 4, 5, 6], lambda x: x > 4)
    5
    6
    >>> result = print_if([3, 4, 5, 6], lambda x: x % 2 == 0)
    4
    6
    >>> print(result)  # print_if should return None
    None
    """
    for x in s:
 if f(x):
 print(x)
```

Use Ok to test your code:

```
python3 ok -q print_ifCopy✂️
```

 document.getElementById("copy-code-python3ok-qprint\_if").onclick = () => copyCode('python3 ok -q print\_if', "copy-code-python3ok-qprint\_if");

### Q3: Close

Implement `close`, which takes a list of numbers `s` and a non-negative integer `k`. It returns how many of the elements of `s` are within `k` of their index. That is, the absolute value of the difference between the element and its index is less than or equal to `k`.

> Remember that list is "zero-indexed"; the index of the first element is `0`.
> 
> 

```
def close(s, k):
    """Return how many elements of s that are within k of their index.

    >>> t = [6, 2, 4, 3, 5]
    >>> close(t, 0)  # Only 3 is equal to its index
    1
    >>> close(t, 1)  # 2, 3, and 5 are within 1 of their index
    3
    >>> close(t, 2)  # 2, 3, 4, and 5 are all within 2 of their index
    4
    >>> close(list(range(10)), 0)
    10
    """
    count = 0
    for i in range(len(s)):  # Use a range to loop over indices
 if abs(i - s[i]) <= k:
 count += 1    return count
```

Use Ok to test your code:

```
python3 ok -q closeCopy✂️
```

 document.getElementById("copy-code-python3ok-qclose").onclick = () => copyCode('python3 ok -q close', "copy-code-python3ok-qclose");

