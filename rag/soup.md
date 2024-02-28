::: {.mobile-nav}
[ [![Python
logo](../_static/py.svg)](https://www.python.org/){.nav-logo} [
]{.version_switcher_placeholder} ]{.nav-items-wrapper}

::: {.menu-wrapper}
::: {.language_switcher_placeholder}
:::

Theme Auto Light Dark

<div>

### [Table of Contents](../contents.html)

-   [`                       random                     `{.xref .py
    .py-mod .docutils .literal .notranslate} â€" Generate pseudo-random
    numbers](#){.reference .internal}
    -   [Bookkeeping functions](#bookkeeping-functions){.reference
        .internal}
    -   [Functions for bytes](#functions-for-bytes){.reference
        .internal}
    -   [Functions for integers](#functions-for-integers){.reference
        .internal}
    -   [Functions for sequences](#functions-for-sequences){.reference
        .internal}
    -   [Discrete distributions](#discrete-distributions){.reference
        .internal}
    -   [Real-valued
        distributions](#real-valued-distributions){.reference .internal}
    -   [Alternative Generator](#alternative-generator){.reference
        .internal}
    -   [Notes on Reproducibility](#notes-on-reproducibility){.reference
        .internal}
    -   [Examples](#examples){.reference .internal}
    -   [Recipes](#recipes){.reference .internal}

</div>

<div>

#### Previous topic

[`                     fractions                   `{.xref .py .py-mod
.docutils .literal .notranslate} â€" Rational
numbers](fractions.html "previous chapter")

</div>

<div>

#### Next topic

[`                     statistics                   `{.xref .py .py-mod
.docutils .literal .notranslate} â€" Mathematical statistics
functions](statistics.html "next chapter")

</div>

::: {aria-label="source link" role="note"}
### This Page

-   [Report a Bug](../bugs.html)
-   [Show
    Source](https://github.com/python/cpython/blob/main/Doc/library/random.rst)
:::
:::
:::

::: {.related aria-label="related navigation" role="navigation"}
### Navigation

-   [index](../genindex.html "General Index")
-   [modules](../py-modindex.html "Python Module Index") \|
-   [next](statistics.html "statistics â Mathematical statistics functions")
    \|
-   [previous](fractions.html "fractions â Rational numbers") \|
-   ![Python logo](../_static/py.svg)
-   [Python](https://www.python.org/) »
-   ::: {.language_switcher_placeholder}
    :::

    ::: {.version_switcher_placeholder}
    :::

-   -   [[3.12.2 Documentation](../index.html)
    »]{#cpython-language-and-version}
-   [The Python Standard Library](index.html) »
-   [Numeric and Mathematical Modules](numeric.html) »
-   [`                 random               `{.xref .py .py-mod
    .docutils .literal .notranslate} â€" Generate pseudo-random
    numbers]()
-   ::: {.inline-search role="search"}
    :::

    \|
-   Theme Auto Light Dark \|
:::

::: {.document}
::: {.documentwrapper}
::: {.bodywrapper}
::: {.body role="main"}
::: {#module-random .section}
[ ]{#random-generate-pseudo-random-numbers}

[`                       random                     `{.xref .py .py-mod .docutils .literal .notranslate}](#module-random "random: Generate pseudo-random numbers with various common distributions."){.reference .internal} â€" Generate pseudo-random numbers [Â¶](#module-random "Link to this heading"){.headerlink}
=======================================================================================================================================================================================================================================================================================================================

**Source code:**
[Lib/random.py](https://github.com/python/cpython/tree/3.12/Lib/random.py){.reference
.external}

------------------------------------------------------------------------

This module implements pseudo-random number generators for various
distributions.

For integers, there is uniform selection from a range. For sequences,
there is uniform selection of a random element, a function to generate a
random permutation of a list in-place, and a function for random
sampling without replacement.

On the real line, there are functions to compute uniform, normal
(Gaussian), lognormal, negative exponential, gamma, and beta
distributions. For generating distributions of angles, the von Mises
distribution is available.

Almost all module functions depend on the basic function
[`                       random()                     `{.xref .py
.py-func .docutils .literal
.notranslate}](#random.random "random.random"){.reference .internal} ,
which generates a random float uniformly in the half-open range
`                     0.0                               <=                               X                               <                               1.0                   `{.docutils
.literal .notranslate} . Python uses the Mersenne Twister as the core
generator. It produces 53-bit precision floats and has a period of
2\*\*19937-1. The underlying implementation in C is both fast and
threadsafe. The Mersenne Twister is one of the most extensively tested
random number generators in existence. However, being completely
deterministic, it is not suitable for all purposes, and is completely
unsuitable for cryptographic purposes.

The functions supplied by this module are actually bound methods of a
hidden instance of the
[`                       random.Random                     `{.xref .py
.py-class .docutils .literal
.notranslate}](#random.Random "random.Random"){.reference .internal}
class. You can instantiate your own instances of
[`                       Random                     `{.xref .py
.py-class .docutils .literal
.notranslate}](#random.Random "random.Random"){.reference .internal} to
get generators that donâ€™t share state.

Class [`                       Random                     `{.xref .py
.py-class .docutils .literal
.notranslate}](#random.Random "random.Random"){.reference .internal} can
also be subclassed if you want to use a different basic generator of
your own devising: see the documentation on that class for more details.

The [`                       random                     `{.xref .py
.py-mod .docutils .literal
.notranslate}](#module-random "random: Generate pseudo-random numbers with various common distributions."){.reference
.internal} module also provides the
[`                       SystemRandom                     `{.xref .py
.py-class .docutils .literal
.notranslate}](#random.SystemRandom "random.SystemRandom"){.reference
.internal} class which uses the system function
[`                       os.urandom()                     `{.xref .py
.py-func .docutils .literal
.notranslate}](os.html#os.urandom "os.urandom"){.reference .internal} to
generate random numbers from sources provided by the operating system.

::: {.admonition .warning}
Warning

The pseudo-random generators of this module should not be used for
security purposes. For security or cryptographic uses, see the
[`                         secrets                       `{.xref .py
.py-mod .docutils .literal
.notranslate}](secrets.html#module-secrets "secrets: Generate secure random numbers for managing secrets."){.reference
.internal} module.
:::

::: {.admonition .seealso}
See also

M. Matsumoto and T. Nishimura, â€œMersenne Twister: A 623-dimensionally
equidistributed uniform pseudorandom number generatorâ€?, ACM
Transactions on Modeling and Computer Simulation Vol. 8, No. 1, January
pp.3â€"30 1998.

[Complementary-Multiply-with-Carry
recipe](https://code.activestate.com/recipes/576707/){.reference
.external} for a compatible alternative random number generator with a
long period and comparatively simple update operations.
:::

::: {#bookkeeping-functions .section}
Bookkeeping functions [Â¶](#bookkeeping-functions "Link to this heading"){.headerlink}
--------------------------------------------------------------------------------------

 [ [ random. ]{.pre} ]{.sig-prename .descclassname} [ [ seed ]{.pre} ]{.sig-name .descname} [ ( ]{.sig-paren} *[ [ a ]{.pre} ]{.n} [ [ = ]{.pre} ]{.o} [ [ None ]{.pre} ]{.default_value}* , *[ [ version ]{.pre} ]{.n} [ [ = ]{.pre} ]{.o} [ [ 2 ]{.pre} ]{.default_value}* [ ) ]{.sig-paren} [Â¶](#random.seed "Link to this definition"){.headerlink} 

:   Initialize the random number generator.

    If *a* is omitted or
    `                           None                         `{.docutils
    .literal .notranslate} , the current system time is used. If
    randomness sources are provided by the operating system, they are
    used instead of the system time (see the
    [`                             os.urandom()                           `{.xref
    .py .py-func .docutils .literal
    .notranslate}](os.html#os.urandom "os.urandom"){.reference
    .internal} function for details on availability).

    If *a* is an int, it is used directly.

    With version 2 (the default), a
    [`                             str                           `{.xref
    .py .py-class .docutils .literal
    .notranslate}](stdtypes.html#str "str"){.reference .internal} ,
    [`                             bytes                           `{.xref
    .py .py-class .docutils .literal
    .notranslate}](stdtypes.html#bytes "bytes"){.reference .internal} ,
    or
    [`                             bytearray                           `{.xref
    .py .py-class .docutils .literal
    .notranslate}](stdtypes.html#bytearray "bytearray"){.reference
    .internal} object gets converted to an
    [`                             int                           `{.xref
    .py .py-class .docutils .literal
    .notranslate}](functions.html#int "int"){.reference .internal} and
    all of its bits are used.

    With version 1 (provided for reproducing random sequences from older
    versions of Python), the algorithm for
    [`                             str                           `{.xref
    .py .py-class .docutils .literal
    .notranslate}](stdtypes.html#str "str"){.reference .internal} and
    [`                             bytes                           `{.xref
    .py .py-class .docutils .literal
    .notranslate}](stdtypes.html#bytes "bytes"){.reference .internal}
    generates a narrower range of seeds.

    ::: {.versionchanged}
    [ Changed in version 3.2: ]{.versionmodified .changed} Moved to the
    version 2 scheme which uses all of the bits in a string seed.
    :::

    ::: {.versionchanged}
    [ Changed in version 3.11: ]{.versionmodified .changed} The *seed*
    must be one of the following types:
    `                             None                           `{.docutils
    .literal .notranslate} ,
    [`                               int                             `{.xref
    .py .py-class .docutils .literal
    .notranslate}](functions.html#int "int"){.reference .internal} ,
    [`                               float                             `{.xref
    .py .py-class .docutils .literal
    .notranslate}](functions.html#float "float"){.reference .internal} ,
    [`                               str                             `{.xref
    .py .py-class .docutils .literal
    .notranslate}](stdtypes.html#str "str"){.reference .internal} ,
    [`                               bytes                             `{.xref
    .py .py-class .docutils .literal
    .notranslate}](stdtypes.html#bytes "bytes"){.reference .internal} ,
    or
    [`                               bytearray                             `{.xref
    .py .py-class .docutils .literal
    .notranslate}](stdtypes.html#bytearray "bytearray"){.reference
    .internal} .
    :::

<!-- -->

 [ [ random. ]{.pre} ]{.sig-prename .descclassname} [ [ getstate ]{.pre} ]{.sig-name .descname} [ ( ]{.sig-paren} [ ) ]{.sig-paren} [Â¶](#random.getstate "Link to this definition"){.headerlink} 

:   Return an object capturing the current internal state of the
    generator. This object can be passed to
    [`                             setstate()                           `{.xref
    .py .py-func .docutils .literal
    .notranslate}](#random.setstate "random.setstate"){.reference
    .internal} to restore the state.

<!-- -->

 [ [ random. ]{.pre} ]{.sig-prename .descclassname} [ [ setstate ]{.pre} ]{.sig-name .descname} [ ( ]{.sig-paren} *[ [ state ]{.pre} ]{.n}* [ ) ]{.sig-paren} [Â¶](#random.setstate "Link to this definition"){.headerlink} 

:   *state* should have been obtained from a previous call to
    [`                             getstate()                           `{.xref
    .py .py-func .docutils .literal
    .notranslate}](#random.getstate "random.getstate"){.reference
    .internal} , and
    [`                             setstate()                           `{.xref
    .py .py-func .docutils .literal
    .notranslate}](#random.setstate "random.setstate"){.reference
    .internal} restores the internal state of the generator to what it
    was at the time
    [`                             getstate()                           `{.xref
    .py .py-func .docutils .literal
    .notranslate}](#random.getstate "random.getstate"){.reference
    .internal} was called.
:::

::: {#functions-for-bytes .section}
Functions for bytes [Â¶](#functions-for-bytes "Link to this heading"){.headerlink}
----------------------------------------------------------------------------------

 [ [ random. ]{.pre} ]{.sig-prename .descclassname} [ [ randbytes ]{.pre} ]{.sig-name .descname} [ ( ]{.sig-paren} *[ [ n ]{.pre} ]{.n}* [ ) ]{.sig-paren} [Â¶](#random.randbytes "Link to this definition"){.headerlink} 

:   Generate *n* random bytes.

    This method should not be used for generating security tokens. Use
    [`                             secrets.token_bytes()                           `{.xref
    .py .py-func .docutils .literal
    .notranslate}](secrets.html#secrets.token_bytes "secrets.token_bytes"){.reference
    .internal} instead.

    ::: {.versionadded}
    [ New in version 3.9. ]{.versionmodified .added}
    :::
:::

::: {#functions-for-integers .section}
Functions for integers [Â¶](#functions-for-integers "Link to this heading"){.headerlink}
----------------------------------------------------------------------------------------

 [ [ random. ]{.pre} ]{.sig-prename .descclassname} [ [ randrange ]{.pre} ]{.sig-name .descname} [ ( ]{.sig-paren} *[ [ stop ]{.pre} ]{.n}* [ ) ]{.sig-paren} [Â¶](#random.randrange "Link to this definition"){.headerlink}\
[ [ random. ]{.pre} ]{.sig-prename .descclassname} [ [ randrange ]{.pre} ]{.sig-name .descname} [ ( ]{.sig-paren} *[ [ start ]{.pre} ]{.n}* , *[ [ stop ]{.pre} ]{.n}* [ \[ ]{.optional} , *[ [ step ]{.pre} ]{.n}* [ \] ]{.optional} [ ) ]{.sig-paren} 

:   Return a randomly selected element from
    `                           range(start,                                        stop,                                        step)                         `{.docutils
    .literal .notranslate} .

    This is roughly equivalent to
    `                           choice(range(start,                                        stop,                                        step))                         `{.docutils
    .literal .notranslate} but supports arbitrarily large ranges and is
    optimized for common cases.

    The positional argument pattern matches the
    [`                             range()                           `{.xref
    .py .py-func .docutils .literal
    .notranslate}](stdtypes.html#range "range"){.reference .internal}
    function.

    Keyword arguments should not be used because they can be interpreted
    in unexpected ways. For example
    `                           randrange(start=100)                         `{.docutils
    .literal .notranslate} is interpreted as
    `                           randrange(0,                                        100,                                        1)                         `{.docutils
    .literal .notranslate} .

    ::: {.versionchanged}
    [ Changed in version 3.2: ]{.versionmodified .changed}
    [`                               randrange()                             `{.xref
    .py .py-meth .docutils .literal
    .notranslate}](#random.randrange "random.randrange"){.reference
    .internal} is more sophisticated about producing equally distributed
    values. Formerly it used a style like
    `                             int(random()*n)                           `{.docutils
    .literal .notranslate} which could produce slightly uneven
    distributions.
    :::

    ::: {.versionchanged}
    [ Changed in version 3.12: ]{.versionmodified .changed} Automatic
    conversion of non-integer types is no longer supported. Calls such
    as
    `                             randrange(10.0)                           `{.docutils
    .literal .notranslate} and
    `                             randrange(Fraction(10,                                           1))                           `{.docutils
    .literal .notranslate} now raise a
    [`                               TypeError                             `{.xref
    .py .py-exc .docutils .literal
    .notranslate}](exceptions.html#TypeError "TypeError"){.reference
    .internal} .
    :::

<!-- -->

 [ [ random. ]{.pre} ]{.sig-prename .descclassname} [ [ randint ]{.pre} ]{.sig-name .descname} [ ( ]{.sig-paren} *[ [ a ]{.pre} ]{.n}* , *[ [ b ]{.pre} ]{.n}* [ ) ]{.sig-paren} [Â¶](#random.randint "Link to this definition"){.headerlink} 

:   Return a random integer *N* such that
    `                           a                                        <=                                        N                                        <=                                        b                         `{.docutils
    .literal .notranslate} . Alias for
    `                           randrange(a,                                        b+1)                         `{.docutils
    .literal .notranslate} .

<!-- -->

 [ [ random. ]{.pre} ]{.sig-prename .descclassname} [ [ getrandbits ]{.pre} ]{.sig-name .descname} [ ( ]{.sig-paren} *[ [ k ]{.pre} ]{.n}* [ ) ]{.sig-paren} [Â¶](#random.getrandbits "Link to this definition"){.headerlink} 

:   Returns a non-negative Python integer with *k* random bits. This
    method is supplied with the Mersenne Twister generator and some
    other generators may also provide it as an optional part of the API.
    When available,
    [`                             getrandbits()                           `{.xref
    .py .py-meth .docutils .literal
    .notranslate}](#random.getrandbits "random.getrandbits"){.reference
    .internal} enables
    [`                             randrange()                           `{.xref
    .py .py-meth .docutils .literal
    .notranslate}](#random.randrange "random.randrange"){.reference
    .internal} to handle arbitrarily large ranges.

    ::: {.versionchanged}
    [ Changed in version 3.9: ]{.versionmodified .changed} This method
    now accepts zero for *k* .
    :::
:::

::: {#functions-for-sequences .section}
Functions for sequences [Â¶](#functions-for-sequences "Link to this heading"){.headerlink}
------------------------------------------------------------------------------------------

 [ [ random. ]{.pre} ]{.sig-prename .descclassname} [ [ choice ]{.pre} ]{.sig-name .descname} [ ( ]{.sig-paren} *[ [ seq ]{.pre} ]{.n}* [ ) ]{.sig-paren} [Â¶](#random.choice "Link to this definition"){.headerlink} 

:   Return a random element from the non-empty sequence *seq* . If *seq*
    is empty, raises
    [`                             IndexError                           `{.xref
    .py .py-exc .docutils .literal
    .notranslate}](exceptions.html#IndexError "IndexError"){.reference
    .internal} .

<!-- -->

 [ [ random. ]{.pre} ]{.sig-prename .descclassname} [ [ choices ]{.pre} ]{.sig-name .descname} [ ( ]{.sig-paren} *[ [ population ]{.pre} ]{.n}* , *[ [ weights ]{.pre} ]{.n} [ [ = ]{.pre} ]{.o} [ [ None ]{.pre} ]{.default_value}* , *[ [ \* ]{.pre} ]{.o}* , *[ [ cum\_weights ]{.pre} ]{.n} [ [ = ]{.pre} ]{.o} [ [ None ]{.pre} ]{.default_value}* , *[ [ k ]{.pre} ]{.n} [ [ = ]{.pre} ]{.o} [ [ 1 ]{.pre} ]{.default_value}* [ ) ]{.sig-paren} [Â¶](#random.choices "Link to this definition"){.headerlink} 

:   Return a *k* sized list of elements chosen from the *population*
    with replacement. If the *population* is empty, raises
    [`                             IndexError                           `{.xref
    .py .py-exc .docutils .literal
    .notranslate}](exceptions.html#IndexError "IndexError"){.reference
    .internal} .

    If a *weights* sequence is specified, selections are made according
    to the relative weights. Alternatively, if a *cum\_weights* sequence
    is given, the selections are made according to the cumulative
    weights (perhaps computed using
    [`                             itertools.accumulate()                           `{.xref
    .py .py-func .docutils .literal
    .notranslate}](itertools.html#itertools.accumulate "itertools.accumulate"){.reference
    .internal} ). For example, the relative weights
    `                           [10,                                        5,                                        30,                                        5]                         `{.docutils
    .literal .notranslate} are equivalent to the cumulative weights
    `                           [10,                                        15,                                        45,                                        50]                         `{.docutils
    .literal .notranslate} . Internally, the relative weights are
    converted to cumulative weights before making selections, so
    supplying the cumulative weights saves work.

    If neither *weights* nor *cum\_weights* are specified, selections
    are made with equal probability. If a weights sequence is supplied,
    it must be the same length as the *population* sequence. It is a
    [`                             TypeError                           `{.xref
    .py .py-exc .docutils .literal
    .notranslate}](exceptions.html#TypeError "TypeError"){.reference
    .internal} to specify both *weights* and *cum\_weights* .

    The *weights* or *cum\_weights* can use any numeric type that
    interoperates with the
    [`                             float                           `{.xref
    .py .py-class .docutils .literal
    .notranslate}](functions.html#float "float"){.reference .internal}
    values returned by
    [`                             random()                           `{.xref
    .py .py-func .docutils .literal
    .notranslate}](#module-random "random: Generate pseudo-random numbers with various common distributions."){.reference
    .internal} (that includes integers, floats, and fractions but
    excludes decimals). Weights are assumed to be non-negative and
    finite. A
    [`                             ValueError                           `{.xref
    .py .py-exc .docutils .literal
    .notranslate}](exceptions.html#ValueError "ValueError"){.reference
    .internal} is raised if all weights are zero.

    For a given seed, the
    [`                             choices()                           `{.xref
    .py .py-func .docutils .literal
    .notranslate}](#random.choices "random.choices"){.reference
    .internal} function with equal weighting typically produces a
    different sequence than repeated calls to
    [`                             choice()                           `{.xref
    .py .py-func .docutils .literal
    .notranslate}](#random.choice "random.choice"){.reference .internal}
    . The algorithm used by
    [`                             choices()                           `{.xref
    .py .py-func .docutils .literal
    .notranslate}](#random.choices "random.choices"){.reference
    .internal} uses floating point arithmetic for internal consistency
    and speed. The algorithm used by
    [`                             choice()                           `{.xref
    .py .py-func .docutils .literal
    .notranslate}](#random.choice "random.choice"){.reference .internal}
    defaults to integer arithmetic with repeated selections to avoid
    small biases from round-off error.

    ::: {.versionadded}
    [ New in version 3.6. ]{.versionmodified .added}
    :::

    ::: {.versionchanged}
    [ Changed in version 3.9: ]{.versionmodified .changed} Raises a
    [`                               ValueError                             `{.xref
    .py .py-exc .docutils .literal
    .notranslate}](exceptions.html#ValueError "ValueError"){.reference
    .internal} if all weights are zero.
    :::

<!-- -->

 [ [ random. ]{.pre} ]{.sig-prename .descclassname} [ [ shuffle ]{.pre} ]{.sig-name .descname} [ ( ]{.sig-paren} *[ [ x ]{.pre} ]{.n}* [ ) ]{.sig-paren} [Â¶](#random.shuffle "Link to this definition"){.headerlink} 

:   Shuffle the sequence *x* in place.

    To shuffle an immutable sequence and return a new shuffled list, use
    `                           sample(x,                                        k=len(x))                         `{.docutils
    .literal .notranslate} instead.

    Note that even for small
    `                           len(x)                         `{.docutils
    .literal .notranslate} , the total number of permutations of *x* can
    quickly grow larger than the period of most random number
    generators. This implies that most permutations of a long sequence
    can never be generated. For example, a sequence of length 2080 is
    the largest that can fit within the period of the Mersenne Twister
    random number generator.

    ::: {.versionchanged}
    [ Changed in version 3.11: ]{.versionmodified .changed} Removed the
    optional parameter *random* .
    :::

<!-- -->

 [ [ random. ]{.pre} ]{.sig-prename .descclassname} [ [ sample ]{.pre} ]{.sig-name .descname} [ ( ]{.sig-paren} *[ [ population ]{.pre} ]{.n}* , *[ [ k ]{.pre} ]{.n}* , *[ [ \* ]{.pre} ]{.o}* , *[ [ counts ]{.pre} ]{.n} [ [ = ]{.pre} ]{.o} [ [ None ]{.pre} ]{.default_value}* [ ) ]{.sig-paren} [Â¶](#random.sample "Link to this definition"){.headerlink} 

:   Return a *k* length list of unique elements chosen from the
    population sequence. Used for random sampling without replacement.

    Returns a new list containing elements from the population while
    leaving the original population unchanged. The resulting list is in
    selection order so that all sub-slices will also be valid random
    samples. This allows raffle winners (the sample) to be partitioned
    into grand prize and second place winners (the subslices).

    Members of the population need not be [[ hashable ]{.xref .std
    .std-term}](../glossary.html#term-hashable){.reference .internal} or
    unique. If the population contains repeats, then each occurrence is
    a possible selection in the sample.

    Repeated elements can be specified one at a time or with the
    optional keyword-only *counts* parameter. For example,
    `                           sample(['red',                                        'blue'],                                        counts=[4,                                        2],                                        k=5)                         `{.docutils
    .literal .notranslate} is equivalent to
    `                           sample(['red',                                        'red',                                        'red',                                        'red',                                        'blue',                                        'blue'],                                        k=5)                         `{.docutils
    .literal .notranslate} .

    To choose a sample from a range of integers, use a
    [`                             range()                           `{.xref
    .py .py-func .docutils .literal
    .notranslate}](stdtypes.html#range "range"){.reference .internal}
    object as an argument. This is especially fast and space efficient
    for sampling from a large population:
    `                           sample(range(10000000),                                        k=60)                         `{.docutils
    .literal .notranslate} .

    If the sample size is larger than the population size, a
    [`                             ValueError                           `{.xref
    .py .py-exc .docutils .literal
    .notranslate}](exceptions.html#ValueError "ValueError"){.reference
    .internal} is raised.

    ::: {.versionchanged}
    [ Changed in version 3.9: ]{.versionmodified .changed} Added the
    *counts* parameter.
    :::

    ::: {.versionchanged}
    [ Changed in version 3.11: ]{.versionmodified .changed} The
    *population* must be a sequence. Automatic conversion of sets to
    lists is no longer supported.
    :::
:::

::: {#discrete-distributions .section}
Discrete distributions [Â¶](#discrete-distributions "Link to this heading"){.headerlink}
----------------------------------------------------------------------------------------

The following function generates a discrete distribution.

 [ [ random. ]{.pre} ]{.sig-prename .descclassname} [ [ binomialvariate ]{.pre} ]{.sig-name .descname} [ ( ]{.sig-paren} *[ [ n ]{.pre} ]{.n} [ [ = ]{.pre} ]{.o} [ [ 1 ]{.pre} ]{.default_value}* , *[ [ p ]{.pre} ]{.n} [ [ = ]{.pre} ]{.o} [ [ 0.5 ]{.pre} ]{.default_value}* [ ) ]{.sig-paren} [Â¶](#random.binomialvariate "Link to this definition"){.headerlink} 

:   [Binomial
    distribution](https://mathworld.wolfram.com/BinomialDistribution.html){.reference
    .external} . Return the number of successes for *n* independent
    trials with the probability of success in each trial being *p* :

    Mathematically equivalent to:

    ::: {.highlight-python3 .notranslate}
    ::: {.highlight}
        sum(random() < p for i in range(n))
    :::
    :::

    The number of trials *n* should be a non-negative integer. The
    probability of success *p* should be between
    `                           0.0                                        <=                                        p                                        <=                                        1.0                         `{.docutils
    .literal .notranslate} . The result is an integer in the range
    `                           0                                        <=                                        X                                        <=                                        n                         `{.docutils
    .literal .notranslate} .

    ::: {.versionadded}
    [ New in version 3.12. ]{.versionmodified .added}
    :::
:::

::: {#real-valued-distributions .section}
[ ]{#id1}

Real-valued distributions [Â¶](#real-valued-distributions "Link to this heading"){.headerlink}
----------------------------------------------------------------------------------------------

The following functions generate specific real-valued distributions.
Function parameters are named after the corresponding variables in the
distributionâ€™s equation, as used in common mathematical practice; most
of these equations can be found in any statistics text.

 [ [ random. ]{.pre} ]{.sig-prename .descclassname} [ [ random ]{.pre} ]{.sig-name .descname} [ ( ]{.sig-paren} [ ) ]{.sig-paren} [Â¶](#random.random "Link to this definition"){.headerlink} 

:   Return the next random floating point number in the range
    `                           0.0                                        <=                                        X                                        <                                        1.0                         `{.docutils
    .literal .notranslate}

<!-- -->

 [ [ random. ]{.pre} ]{.sig-prename .descclassname} [ [ uniform ]{.pre} ]{.sig-name .descname} [ ( ]{.sig-paren} *[ [ a ]{.pre} ]{.n}* , *[ [ b ]{.pre} ]{.n}* [ ) ]{.sig-paren} [Â¶](#random.uniform "Link to this definition"){.headerlink} 

:   Return a random floating point number *N* such that
    `                           a                                        <=                                        N                                        <=                                        b                         `{.docutils
    .literal .notranslate} for
    `                           a                                        <=                                        b                         `{.docutils
    .literal .notranslate} and
    `                           b                                        <=                                        N                                        <=                                        a                         `{.docutils
    .literal .notranslate} for
    `                           b                                        <                                        a                         `{.docutils
    .literal .notranslate} .

    The end-point value
    `                           b                         `{.docutils
    .literal .notranslate} may or may not be included in the range
    depending on floating-point rounding in the equation
    `                           a                                        +                                        (b-a)                                        *                                        random()                         `{.docutils
    .literal .notranslate} .

<!-- -->

 [ [ random. ]{.pre} ]{.sig-prename .descclassname} [ [ triangular ]{.pre} ]{.sig-name .descname} [ ( ]{.sig-paren} *[ [ low ]{.pre} ]{.n}* , *[ [ high ]{.pre} ]{.n}* , *[ [ mode ]{.pre} ]{.n}* [ ) ]{.sig-paren} [Â¶](#random.triangular "Link to this definition"){.headerlink} 

:   Return a random floating point number *N* such that
    `                           low                                        <=                                        N                                        <=                                        high                         `{.docutils
    .literal .notranslate} and with the specified *mode* between those
    bounds. The *low* and *high* bounds default to zero and one. The
    *mode* argument defaults to the midpoint between the bounds, giving
    a symmetric distribution.

<!-- -->

 [ [ random. ]{.pre} ]{.sig-prename .descclassname} [ [ betavariate ]{.pre} ]{.sig-name .descname} [ ( ]{.sig-paren} *[ [ alpha ]{.pre} ]{.n}* , *[ [ beta ]{.pre} ]{.n}* [ ) ]{.sig-paren} [Â¶](#random.betavariate "Link to this definition"){.headerlink} 

:   Beta distribution. Conditions on the parameters are
    `                           alpha                                        >                                        0                         `{.docutils
    .literal .notranslate} and
    `                           beta                                        >                                        0                         `{.docutils
    .literal .notranslate} . Returned values range between 0 and 1.

<!-- -->

 [ [ random. ]{.pre} ]{.sig-prename .descclassname} [ [ expovariate ]{.pre} ]{.sig-name .descname} [ ( ]{.sig-paren} *[ [ lambd ]{.pre} ]{.n} [ [ = ]{.pre} ]{.o} [ [ 1.0 ]{.pre} ]{.default_value}* [ ) ]{.sig-paren} [Â¶](#random.expovariate "Link to this definition"){.headerlink} 

:   Exponential distribution. *lambd* is 1.0 divided by the desired
    mean. It should be nonzero. (The parameter would be called
    â€œlambdaâ€?, but that is a reserved word in Python.) Returned
    values range from 0 to positive infinity if *lambd* is positive, and
    from negative infinity to 0 if *lambd* is negative.

    ::: {.versionchanged}
    [ Changed in version 3.12: ]{.versionmodified .changed} Added the
    default value for
    `                             lambd                           `{.docutils
    .literal .notranslate} .
    :::

<!-- -->

 [ [ random. ]{.pre} ]{.sig-prename .descclassname} [ [ gammavariate ]{.pre} ]{.sig-name .descname} [ ( ]{.sig-paren} *[ [ alpha ]{.pre} ]{.n}* , *[ [ beta ]{.pre} ]{.n}* [ ) ]{.sig-paren} [Â¶](#random.gammavariate "Link to this definition"){.headerlink} 

:   Gamma distribution. ( *Not* the gamma function!) The shape and scale
    parameters, *alpha* and *beta* , must have positive values. (Calling
    conventions vary and some sources define â€˜betaâ€™ as the inverse
    of the scale).

    The probability distribution function is:

    ::: {.highlight-python3 .notranslate}
    ::: {.highlight}
                  x ** (alpha - 1) * math.exp(-x / beta)
        pdf(x) =  --------------------------------------
                    math.gamma(alpha) * beta ** alpha
    :::
    :::

<!-- -->

 [ [ random. ]{.pre} ]{.sig-prename .descclassname} [ [ gauss ]{.pre} ]{.sig-name .descname} [ ( ]{.sig-paren} *[ [ mu ]{.pre} ]{.n} [ [ = ]{.pre} ]{.o} [ [ 0.0 ]{.pre} ]{.default_value}* , *[ [ sigma ]{.pre} ]{.n} [ [ = ]{.pre} ]{.o} [ [ 1.0 ]{.pre} ]{.default_value}* [ ) ]{.sig-paren} [Â¶](#random.gauss "Link to this definition"){.headerlink} 

:   Normal distribution, also called the Gaussian distribution. *mu* is
    the mean, and *sigma* is the standard deviation. This is slightly
    faster than the
    [`                             normalvariate()                           `{.xref
    .py .py-func .docutils .literal
    .notranslate}](#random.normalvariate "random.normalvariate"){.reference
    .internal} function defined below.

    Multithreading note: When two threads call this function
    simultaneously, it is possible that they will receive the same
    return value. This can be avoided in three ways. 1) Have each thread
    use a different instance of the random number generator. 2) Put
    locks around all calls. 3) Use the slower, but thread-safe
    [`                             normalvariate()                           `{.xref
    .py .py-func .docutils .literal
    .notranslate}](#random.normalvariate "random.normalvariate"){.reference
    .internal} function instead.

    ::: {.versionchanged}
    [ Changed in version 3.11: ]{.versionmodified .changed} *mu* and
    *sigma* now have default arguments.
    :::

<!-- -->

 [ [ random. ]{.pre} ]{.sig-prename .descclassname} [ [ lognormvariate ]{.pre} ]{.sig-name .descname} [ ( ]{.sig-paren} *[ [ mu ]{.pre} ]{.n}* , *[ [ sigma ]{.pre} ]{.n}* [ ) ]{.sig-paren} [Â¶](#random.lognormvariate "Link to this definition"){.headerlink} 

:   Log normal distribution. If you take the natural logarithm of this
    distribution, youâ€™ll get a normal distribution with mean *mu* and
    standard deviation *sigma* . *mu* can have any value, and *sigma*
    must be greater than zero.

<!-- -->

 [ [ random. ]{.pre} ]{.sig-prename .descclassname} [ [ normalvariate ]{.pre} ]{.sig-name .descname} [ ( ]{.sig-paren} *[ [ mu ]{.pre} ]{.n} [ [ = ]{.pre} ]{.o} [ [ 0.0 ]{.pre} ]{.default_value}* , *[ [ sigma ]{.pre} ]{.n} [ [ = ]{.pre} ]{.o} [ [ 1.0 ]{.pre} ]{.default_value}* [ ) ]{.sig-paren} [Â¶](#random.normalvariate "Link to this definition"){.headerlink} 

:   Normal distribution. *mu* is the mean, and *sigma* is the standard
    deviation.

    ::: {.versionchanged}
    [ Changed in version 3.11: ]{.versionmodified .changed} *mu* and
    *sigma* now have default arguments.
    :::

<!-- -->

 [ [ random. ]{.pre} ]{.sig-prename .descclassname} [ [ vonmisesvariate ]{.pre} ]{.sig-name .descname} [ ( ]{.sig-paren} *[ [ mu ]{.pre} ]{.n}* , *[ [ kappa ]{.pre} ]{.n}* [ ) ]{.sig-paren} [Â¶](#random.vonmisesvariate "Link to this definition"){.headerlink} 

:   *mu* is the mean angle, expressed in radians between 0 and 2\* *pi*
    , and *kappa* is the concentration parameter, which must be greater
    than or equal to zero. If *kappa* is equal to zero, this
    distribution reduces to a uniform random angle over the range 0 to
    2\* *pi* .

<!-- -->

 [ [ random. ]{.pre} ]{.sig-prename .descclassname} [ [ paretovariate ]{.pre} ]{.sig-name .descname} [ ( ]{.sig-paren} *[ [ alpha ]{.pre} ]{.n}* [ ) ]{.sig-paren} [Â¶](#random.paretovariate "Link to this definition"){.headerlink} 

:   Pareto distribution. *alpha* is the shape parameter.

<!-- -->

 [ [ random. ]{.pre} ]{.sig-prename .descclassname} [ [ weibullvariate ]{.pre} ]{.sig-name .descname} [ ( ]{.sig-paren} *[ [ alpha ]{.pre} ]{.n}* , *[ [ beta ]{.pre} ]{.n}* [ ) ]{.sig-paren} [Â¶](#random.weibullvariate "Link to this definition"){.headerlink} 

:   Weibull distribution. *alpha* is the scale parameter and *beta* is
    the shape parameter.
:::

::: {#alternative-generator .section}
Alternative Generator [Â¶](#alternative-generator "Link to this heading"){.headerlink}
--------------------------------------------------------------------------------------

 *[ class ]{.pre} [ ]{.w}* [ [ random. ]{.pre} ]{.sig-prename .descclassname} [ [ Random ]{.pre} ]{.sig-name .descname} [ ( ]{.sig-paren} [ \[ ]{.optional} *[ [ seed ]{.pre} ]{.n}* [ \] ]{.optional} [ ) ]{.sig-paren} [Â¶](#random.Random "Link to this definition"){.headerlink} 

:   Class that implements the default pseudo-random number generator
    used by the
    [`                             random                           `{.xref
    .py .py-mod .docutils .literal
    .notranslate}](#module-random "random: Generate pseudo-random numbers with various common distributions."){.reference
    .internal} module.

    ::: {.versionchanged}
    [ Changed in version 3.11: ]{.versionmodified .changed} Formerly the
    *seed* could be any hashable object. Now it is limited to:
    `                             None                           `{.docutils
    .literal .notranslate} ,
    [`                               int                             `{.xref
    .py .py-class .docutils .literal
    .notranslate}](functions.html#int "int"){.reference .internal} ,
    [`                               float                             `{.xref
    .py .py-class .docutils .literal
    .notranslate}](functions.html#float "float"){.reference .internal} ,
    [`                               str                             `{.xref
    .py .py-class .docutils .literal
    .notranslate}](stdtypes.html#str "str"){.reference .internal} ,
    [`                               bytes                             `{.xref
    .py .py-class .docutils .literal
    .notranslate}](stdtypes.html#bytes "bytes"){.reference .internal} ,
    or
    [`                               bytearray                             `{.xref
    .py .py-class .docutils .literal
    .notranslate}](stdtypes.html#bytearray "bytearray"){.reference
    .internal} .
    :::

    Subclasses of
    `                           Random                         `{.xref
    .py .py-class .docutils .literal .notranslate} should override the
    following methods if they wish to make use of a different basic
    generator:

     [ [ seed ]{.pre} ]{.sig-name .descname} [ ( ]{.sig-paren} *[ [ a ]{.pre} ]{.n} [ [ = ]{.pre} ]{.o} [ [ None ]{.pre} ]{.default_value}* , *[ [ version ]{.pre} ]{.n} [ [ = ]{.pre} ]{.o} [ [ 2 ]{.pre} ]{.default_value}* [ ) ]{.sig-paren} [Â¶](#random.Random.seed "Link to this definition"){.headerlink} 

    :   Override this method in subclasses to customise the
        [`                                 seed()                               `{.xref
        .py .py-meth .docutils .literal
        .notranslate}](#random.seed "random.seed"){.reference .internal}
        behaviour of
        `                               Random                             `{.xref
        .py .py-class .docutils .literal .notranslate} instances.

     [ [ getstate ]{.pre} ]{.sig-name .descname} [ ( ]{.sig-paren} [ ) ]{.sig-paren} [Â¶](#random.Random.getstate "Link to this definition"){.headerlink} 

    :   Override this method in subclasses to customise the
        [`                                 getstate()                               `{.xref
        .py .py-meth .docutils .literal
        .notranslate}](#random.getstate "random.getstate"){.reference
        .internal} behaviour of
        `                               Random                             `{.xref
        .py .py-class .docutils .literal .notranslate} instances.

     [ [ setstate ]{.pre} ]{.sig-name .descname} [ ( ]{.sig-paren} *[ [ state ]{.pre} ]{.n}* [ ) ]{.sig-paren} [Â¶](#random.Random.setstate "Link to this definition"){.headerlink} 

    :   Override this method in subclasses to customise the
        [`                                 setstate()                               `{.xref
        .py .py-meth .docutils .literal
        .notranslate}](#random.setstate "random.setstate"){.reference
        .internal} behaviour of
        `                               Random                             `{.xref
        .py .py-class .docutils .literal .notranslate} instances.

     [ [ random ]{.pre} ]{.sig-name .descname} [ ( ]{.sig-paren} [ ) ]{.sig-paren} [Â¶](#random.Random.random "Link to this definition"){.headerlink} 

    :   Override this method in subclasses to customise the
        [`                                 random()                               `{.xref
        .py .py-meth .docutils .literal
        .notranslate}](#random.random "random.random"){.reference
        .internal} behaviour of
        `                               Random                             `{.xref
        .py .py-class .docutils .literal .notranslate} instances.

    Optionally, a custom generator subclass can also supply the
    following method:

     [ [ getrandbits ]{.pre} ]{.sig-name .descname} [ ( ]{.sig-paren} *[ [ k ]{.pre} ]{.n}* [ ) ]{.sig-paren} [Â¶](#random.Random.getrandbits "Link to this definition"){.headerlink} 

    :   Override this method in subclasses to customise the
        [`                                 getrandbits()                               `{.xref
        .py .py-meth .docutils .literal
        .notranslate}](#random.getrandbits "random.getrandbits"){.reference
        .internal} behaviour of
        `                               Random                             `{.xref
        .py .py-class .docutils .literal .notranslate} instances.

<!-- -->

 *[ class ]{.pre} [ ]{.w}* [ [ random. ]{.pre} ]{.sig-prename .descclassname} [ [ SystemRandom ]{.pre} ]{.sig-name .descname} [ ( ]{.sig-paren} [ \[ ]{.optional} *[ [ seed ]{.pre} ]{.n}* [ \] ]{.optional} [ ) ]{.sig-paren} [Â¶](#random.SystemRandom "Link to this definition"){.headerlink} 

:   Class that uses the
    [`                             os.urandom()                           `{.xref
    .py .py-func .docutils .literal
    .notranslate}](os.html#os.urandom "os.urandom"){.reference
    .internal} function for generating random numbers from sources
    provided by the operating system. Not available on all systems. Does
    not rely on software state, and sequences are not reproducible.
    Accordingly, the
    [`                             seed()                           `{.xref
    .py .py-meth .docutils .literal
    .notranslate}](#random.seed "random.seed"){.reference .internal}
    method has no effect and is ignored. The
    [`                             getstate()                           `{.xref
    .py .py-meth .docutils .literal
    .notranslate}](#random.getstate "random.getstate"){.reference
    .internal} and
    [`                             setstate()                           `{.xref
    .py .py-meth .docutils .literal
    .notranslate}](#random.setstate "random.setstate"){.reference
    .internal} methods raise
    [`                             NotImplementedError                           `{.xref
    .py .py-exc .docutils .literal
    .notranslate}](exceptions.html#NotImplementedError "NotImplementedError"){.reference
    .internal} if called.
:::

::: {#notes-on-reproducibility .section}
Notes on Reproducibility [Â¶](#notes-on-reproducibility "Link to this heading"){.headerlink}
--------------------------------------------------------------------------------------------

Sometimes it is useful to be able to reproduce the sequences given by a
pseudo-random number generator. By reusing a seed value, the same
sequence should be reproducible from run to run as long as multiple
threads are not running.

Most of the random moduleâ€™s algorithms and seeding functions are
subject to change across Python versions, but two aspects are guaranteed
not to change:

-   If a new seeding method is added, then a backward compatible seeder
    will be offered.

-   The generatorâ€™s
    [`                             random()                           `{.xref
    .py .py-meth .docutils .literal
    .notranslate}](#random.Random.random "random.Random.random"){.reference
    .internal} method will continue to produce the same sequence when
    the compatible seeder is given the same seed.
:::

::: {#examples .section}
[ ]{#random-examples}

Examples [Â¶](#examples "Link to this heading"){.headerlink}
------------------------------------------------------------

Basic examples:

::: {.highlight-python3 .notranslate}
::: {.highlight}
    >>> random()                          # Random float:  0.0 <= x < 1.0
    0.37444887175646646

    >>> uniform(2.5, 10.0)                # Random float:  2.5 <= x <= 10.0
    3.1800146073117523

    >>> expovariate(1 / 5)                # Interval between arrivals averaging 5 seconds
    5.148957571865031

    >>> randrange(10)                     # Integer from 0 to 9 inclusive
    7

    >>> randrange(0, 101, 2)              # Even integer from 0 to 100 inclusive
    26

    >>> choice(['win', 'lose', 'draw'])   # Single random element from a sequence
    'draw'

    >>> deck = 'ace two three four'.split()
    >>> shuffle(deck)                     # Shuffle a list
    >>> deck
    ['four', 'two', 'ace', 'three']

    >>> sample([10, 20, 30, 40, 50], k=4) # Four samples without replacement
    [40, 10, 50, 30]
:::
:::

Simulations:

::: {.highlight-python3 .notranslate}
::: {.highlight}
    >>> # Six roulette wheel spins (weighted sampling with replacement)
    >>> choices(['red', 'black', 'green'], [18, 18, 2], k=6)
    ['red', 'green', 'black', 'black', 'red', 'black']

    >>> # Deal 20 cards without replacement from a deck
    >>> # of 52 playing cards, and determine the proportion of cards
    >>> # with a ten-value:  ten, jack, queen, or king.
    >>> deal = sample(['tens', 'low cards'], counts=[16, 36], k=20)
    >>> deal.count('tens') / 20
    0.15

    >>> # Estimate the probability of getting 5 or more heads from 7 spins
    >>> # of a biased coin that settles on heads 60% of the time.
    >>> sum(binomialvariate(n=7, p=0.6) >= 5 for i in range(10_000)) / 10_000
    0.4169

    >>> # Probability of the median of 5 samples being in middle two quartiles
    >>> def trial():
    ...     return 2_500 <= sorted(choices(range(10_000), k=5))[2] < 7_500
    ...
    >>> sum(trial() for i in range(10_000)) / 10_000
    0.7958
:::
:::

Example of [statistical
bootstrapping](https://en.wikipedia.org/wiki/Bootstrapping_(statistics)){.reference
.external} using resampling with replacement to estimate a confidence
interval for the mean of a sample:

::: {.highlight-python3 .notranslate}
::: {.highlight}
    # https://www.thoughtco.com/example-of-bootstrapping-3126155
    from statistics import fmean as mean
    from random import choices

    data = [41, 50, 29, 37, 81, 30, 73, 63, 20, 35, 68, 22, 60, 31, 95]
    means = sorted(mean(choices(data, k=len(data))) for i in range(100))
    print(f'The sample mean of {mean(data):.1f} has a 90% confidence '
          f'interval from {means[5]:.1f} to {means[94]:.1f}')
:::
:::

Example of a [resampling permutation
test](https://en.wikipedia.org/wiki/Resampling_(statistics)#Permutation_tests){.reference
.external} to determine the statistical significance or
[p-value](https://en.wikipedia.org/wiki/P-value){.reference .external}
of an observed difference between the effects of a drug versus a
placebo:

::: {.highlight-python3 .notranslate}
::: {.highlight}
    # Example from "Statistics is Easy" by Dennis Shasha and Manda Wilson
    from statistics import fmean as mean
    from random import shuffle

    drug = [54, 73, 53, 70, 73, 68, 52, 65, 65]
    placebo = [54, 51, 58, 44, 55, 52, 42, 47, 58, 46]
    observed_diff = mean(drug) - mean(placebo)

    n = 10_000
    count = 0
    combined = drug + placebo
    for i in range(n):
        shuffle(combined)
        new_diff = mean(combined[:len(drug)]) - mean(combined[len(drug):])
        count += (new_diff >= observed_diff)

    print(f'{n} label reshufflings produced only {count} instances with a difference')
    print(f'at least as extreme as the observed difference of {observed_diff:.1f}.')
    print(f'The one-sided p-value of {count / n:.4f} leads us to reject the null')
    print(f'hypothesis that there is no difference between the drug and the placebo.')
:::
:::

Simulation of arrival times and service deliveries for a multiserver
queue:

::: {.highlight-python3 .notranslate}
::: {.highlight}
    from heapq import heapify, heapreplace
    from random import expovariate, gauss
    from statistics import mean, quantiles

    average_arrival_interval = 5.6
    average_service_time = 15.0
    stdev_service_time = 3.5
    num_servers = 3

    waits = []
    arrival_time = 0.0
    servers = [0.0] * num_servers  # time when each server becomes available
    heapify(servers)
    for i in range(1_000_000):
        arrival_time += expovariate(1.0 / average_arrival_interval)
        next_server_available = servers[0]
        wait = max(0.0, next_server_available - arrival_time)
        waits.append(wait)
        service_duration = max(0.0, gauss(average_service_time, stdev_service_time))
        service_completed = arrival_time + wait + service_duration
        heapreplace(servers, service_completed)

    print(f'Mean wait: {mean(waits):.1f}   Max wait: {max(waits):.1f}')
    print('Quartiles:', [round(q, 1) for q in quantiles(waits)])
:::
:::

::: {.admonition .seealso}
See also

[Statistics for
Hackers](https://www.youtube.com/watch?v=Iq9DzN6mvYA){.reference
.external} a video tutorial by [Jake
Vanderplas](https://us.pycon.org/2016/speaker/profile/295/){.reference
.external} on statistical analysis using just a few fundamental concepts
including simulation, sampling, shuffling, and cross-validation.

[Economics
Simulation](https://nbviewer.org/url/norvig.com/ipython/Economics.ipynb){.reference
.external} a simulation of a marketplace by [Peter
Norvig](https://norvig.com/bio.html){.reference .external} that shows
effective use of many of the tools and distributions provided by this
module (gauss, uniform, sample, betavariate, choice, triangular, and
randrange).

[A Concrete Introduction to Probability (using
Python)](https://nbviewer.org/url/norvig.com/ipython/Probability.ipynb){.reference
.external} a tutorial by [Peter
Norvig](https://norvig.com/bio.html){.reference .external} covering the
basics of probability theory, how to write simulations, and how to
perform data analysis using Python.
:::
:::

::: {#recipes .section}
Recipes [Â¶](#recipes "Link to this heading"){.headerlink}
----------------------------------------------------------

These recipes show how to efficiently make random selections from the
combinatoric iterators in the
[`                         itertools                       `{.xref .py
.py-mod .docutils .literal
.notranslate}](itertools.html#module-itertools "itertools: Functions creating iterators for efficient looping."){.reference
.internal} module:

::: {.highlight-python .notranslate}
::: {.highlight}
    def random_product(*args, repeat=1):
        "Random selection from itertools.product(*args, **kwds)"
        pools = [tuple(pool) for pool in args] * repeat
        return tuple(map(random.choice, pools))

    def random_permutation(iterable, r=None):
        "Random selection from itertools.permutations(iterable, r)"
        pool = tuple(iterable)
        r = len(pool) if r is None else r
        return tuple(random.sample(pool, r))

    def random_combination(iterable, r):
        "Random selection from itertools.combinations(iterable, r)"
        pool = tuple(iterable)
        n = len(pool)
        indices = sorted(random.sample(range(n), r))
        return tuple(pool[i] for i in indices)

    def random_combination_with_replacement(iterable, r):
        "Choose r elements with replacement.  Order the result to match the iterable."
        # Result will be in set(itertools.combinations_with_replacement(iterable, r)).
        pool = tuple(iterable)
        n = len(pool)
        indices = sorted(random.choices(range(n), k=r))
        return tuple(pool[i] for i in indices)
:::
:::

The default
[`                         random()                       `{.xref .py
.py-func .docutils .literal
.notranslate}](#random.random "random.random"){.reference .internal}
returns multiples of 2â?»â?µÂ³ in the range *0.0 â‰¤ x \< 1.0* . All
such numbers are evenly spaced and are exactly representable as Python
floats. However, many other representable floats in that interval are
not possible selections. For example,
`                       0.05954861408025609                     `{.docutils
.literal .notranslate} isnâ€™t an integer multiple of 2â?»â?µÂ³.

The following recipe takes a different approach. All floats in the
interval are possible selections. The mantissa comes from a uniform
distribution of integers in the range *2â?µÂ² â‰¤ mantissa \< 2â?µÂ³* .
The exponent comes from a geometric distribution where exponents smaller
than *-53* occur half as often as the next larger exponent.

::: {.highlight-python3 .notranslate}
::: {.highlight}
    from random import Random
    from math import ldexp

    class FullRandom(Random):

        def random(self):
            mantissa = 0x10_0000_0000_0000 | self.getrandbits(52)
            exponent = -53
            x = 0
            while not x:
                x = self.getrandbits(32)
                exponent += x.bit_length() - 32
            return ldexp(mantissa, exponent)
:::
:::

All [[ real valued distributions ]{.std
.std-ref}](#real-valued-distributions){.reference .internal} in the
class will use the new method:

::: {.highlight-python3 .notranslate}
::: {.highlight}
    >>> fr = FullRandom()
    >>> fr.random()
    0.05954861408025609
    >>> fr.expovariate(0.25)
    8.87925541791544
:::
:::

The recipe is conceptually equivalent to an algorithm that chooses from
all the multiples of 2â?»Â¹â?°â?·â?´ in the range *0.0 â‰¤ x \< 1.0* .
All such numbers are evenly spaced, but most have to be rounded down to
the nearest representable Python float. (The value 2â?»Â¹â?°â?·â?´ is
the smallest positive unnormalized float and is equal to
`                       math.ulp(0.0)                     `{.docutils
.literal .notranslate} .)

::: {.admonition .seealso}
See also

[Generating Pseudo-random Floating-Point
Values](https://allendowney.com/research/rand/downey07randfloat.pdf){.reference
.external} a paper by Allen B. Downey describing ways to generate more
fine-grained floats than normally generated by
[`                           random()                         `{.xref
.py .py-func .docutils .literal
.notranslate}](#random.random "random.random"){.reference .internal} .
:::
:::
:::

::: {.clearer}
:::
:::
:::
:::

::: {.sphinxsidebar aria-label="main navigation" role="navigation"}
::: {.sphinxsidebarwrapper}
<div>

### [Table of Contents](../contents.html)

-   [`                       random                     `{.xref .py
    .py-mod .docutils .literal .notranslate} â€" Generate pseudo-random
    numbers](#){.reference .internal}
    -   [Bookkeeping functions](#bookkeeping-functions){.reference
        .internal}
    -   [Functions for bytes](#functions-for-bytes){.reference
        .internal}
    -   [Functions for integers](#functions-for-integers){.reference
        .internal}
    -   [Functions for sequences](#functions-for-sequences){.reference
        .internal}
    -   [Discrete distributions](#discrete-distributions){.reference
        .internal}
    -   [Real-valued
        distributions](#real-valued-distributions){.reference .internal}
    -   [Alternative Generator](#alternative-generator){.reference
        .internal}
    -   [Notes on Reproducibility](#notes-on-reproducibility){.reference
        .internal}
    -   [Examples](#examples){.reference .internal}
    -   [Recipes](#recipes){.reference .internal}

</div>

<div>

#### Previous topic

[`                     fractions                   `{.xref .py .py-mod
.docutils .literal .notranslate} â€" Rational
numbers](fractions.html "previous chapter")

</div>

<div>

#### Next topic

[`                     statistics                   `{.xref .py .py-mod
.docutils .literal .notranslate} â€" Mathematical statistics
functions](statistics.html "next chapter")

</div>

::: {aria-label="source link" role="note"}
### This Page

-   [Report a Bug](../bugs.html)
-   [Show
    Source](https://github.com/python/cpython/blob/main/Doc/library/random.rst)
:::
:::

::: {#sidebarbutton title="Collapse sidebar"}
Â«
:::
:::

::: {.clearer}
:::
:::

::: {.related aria-label="related navigation" role="navigation"}
### Navigation

-   [index](../genindex.html "General Index")
-   [modules](../py-modindex.html "Python Module Index") \|
-   [next](statistics.html "statistics â Mathematical statistics functions")
    \|
-   [previous](fractions.html "fractions â Rational numbers") \|
-   ![Python logo](../_static/py.svg)
-   [Python](https://www.python.org/) »
-   ::: {.language_switcher_placeholder}
    :::

    ::: {.version_switcher_placeholder}
    :::

-   -   [[3.12.2 Documentation](../index.html)
    »]{#cpython-language-and-version}
-   [The Python Standard Library](index.html) »
-   [Numeric and Mathematical Modules](numeric.html) »
-   [`                 random               `{.xref .py .py-mod
    .docutils .literal .notranslate} â€" Generate pseudo-random
    numbers]()
-   ::: {.inline-search role="search"}
    :::

    \|
-   Theme Auto Light Dark \|
:::

::: {.footer}
© [Copyright](../copyright.html) 2001-2024, Python Software Foundation.\
This page is licensed under the Python Software Foundation License
Version 2.\
Examples, recipes, and other code in the documentation are additionally
licensed under the Zero Clause BSD License.\
See [History and License](/license.html) for more information.\
\
The Python Software Foundation is a non-profit corporation. [Please
donate.](https://www.python.org/psf/donations/)\
\
Last updated on Feb 21, 2024 (07:21 UTC). [Found a bug](/bugs.html) ?\
Created using [Sphinx](https://www.sphinx-doc.org/) 7.2.6.
:::
