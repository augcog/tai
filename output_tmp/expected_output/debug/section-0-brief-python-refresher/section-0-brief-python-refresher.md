[![Open In Colab](../images/colab-badge.svg)](https://colab.research.google.com/github/MonashDataFluency/python-web-scraping/blob/master/notebooks/section-0-brief-python-refresher.ipynb)

<center><img src="../images/xkcd_python.png"></center>
<center>https://xkcd.com/license.html (CC BY-NC 2.5)</center>

### Jupyter-style notebooks on Google Colaboratory - A quick tour
---

1. Go to [this](https://colab.research.google.com) link  and login with your Google account.
2. Select **NEW NOTEBOOK** - a new Python3 notebook will be created.
3. Type some Python code in the top cell, eg:


```python
print("Hello world!!")
```

    Hello world!!
    

4. **Shift-Enter** to run the contents of the cell.

In this section we will take a quick tour of some of the concepts that will be used for the rest of our web scraping workshop.


### Dataframes
---

One of the most powerful data structures in Python is the Pandas `DataFrame`. It allows tabular data, including `csv` (comma seperated values) and `tsv` (tab seperated values), to be processed and manipulated.  

People familiar with Excel will no doubt find it intuitive and easy to grasp. Since most `csv` (or `tsv`) has become the de facto standard for sharing datasets both large and small, Pandas dataframe is the way to go.


```python
import pandas as pd # importing the package and using `pd` as the alias 
print('Pandas version : {}'.format(pd.__version__))
```

    Pandas version : 1.0.1
    

Suppose we wanted to create a dataframe as follows,

| name | title     |
|------|-----------|
| sam  | physicist |
| rob  | economist |

Let's create a dictionary with the headers as keys and their corresponding values as a list as follows,


```python
data = {'name': ['sam', 'rob'], 'title': ['physicist', 'economist']}
```

Converting the same to a dataframe,


```python
df = pd.DataFrame(data)
# Note: converting to markdown for ease of display on site
# print(df._to_markdown()) 
df
```

|    | name   | title     |
|---:|:-------|:----------|
|  0 | sam    | physicist |
|  1 | rob    | economist |


Now lets create a bigger dataframe and learn some useful functions that can be performed on them.


```python
data = {'Name': ['Sam', 'Rob', 'Jack', 'Jill', 'Dave', 'Alex', 'Steve'],\
        'Title': ['Physicist', 'Economist', 'Statistician', 'Data Scientist', 'Designer', 'Architect', 'Doctor'], \
        'Age': [59, 66, 42, 28, 24, 39, 52],\
        'City': ['Melbourne', 'Melbourne', 'Sydney', 'Sydney', 'Melbourne', 'Perth', 'Brisbane'],\
        'University': ['Monash', 'Monash', 'UNSW', 'UTS', 'Uni Mel', 'UWA', 'UQ']}
```


```python
df = pd.DataFrame(data)
df
```

|    | Name   | Title          |   Age | City      | University   |
|---:|:-------|:---------------|------:|:----------|:-------------|
|  0 | Sam    | Physicist      |    59 | Melbourne | Monash       |
|  1 | Rob    | Economist      |    66 | Melbourne | Monash       |
|  2 | Jack   | Statistician   |    42 | Sydney    | UNSW         |
|  3 | Jill   | Data Scientist |    28 | Sydney    | UTS          |
|  4 | Dave   | Designer       |    24 | Melbourne | Uni Mel      |
|  5 | Alex   | Architect      |    39 | Perth     | UWA          |
|  6 | Steve  | Doctor         |    52 | Brisbane  | UQ           |


We can also take a quick glance at its contents by using :

- `df.head()` : To display the first 5 rows

- `df.tail()` : To display the last 5 rows


```python
df.head()
```

|    | Name   | Title          |   Age | City      | University   |
|---:|:-------|:---------------|------:|:----------|:-------------|
|  0 | Sam    | Physicist      |    59 | Melbourne | Monash       |
|  1 | Rob    | Economist      |    66 | Melbourne | Monash       |
|  2 | Jack   | Statistician   |    42 | Sydney    | UNSW         |
|  3 | Jill   | Data Scientist |    28 | Sydney    | UTS          |
|  4 | Dave   | Designer       |    24 | Melbourne | Uni Mel      |


```python
df.tail()
```

|    | Name   | Title          |   Age | City      | University   |
|---:|:-------|:---------------|------:|:----------|:-------------|
|  2 | Jack   | Statistician   |    42 | Sydney    | UNSW         |
|  3 | Jill   | Data Scientist |    28 | Sydney    | UTS          |
|  4 | Dave   | Designer       |    24 | Melbourne | Uni Mel      |
|  5 | Alex   | Architect      |    39 | Perth     | UWA          |
|  6 | Steve  | Doctor         |    52 | Brisbane  | UQ           |

Lets say we want to fiter out all the people from `Sydney`.


```python
df[df['City'] == 'Sydney']
```

|    | Name   | Title          |   Age | City   | University   |
|---:|:-------|:---------------|------:|:-------|:-------------|
|  2 | Jack   | Statistician   |    42 | Sydney | UNSW         |
|  3 | Jill   | Data Scientist |    28 | Sydney | UTS          |


Now, lets say we want to look at all the people in `Melbourne` and in `Monash`. Notice the usage of `()` and `&`.


```python
df[(df['City'] == 'Melbourne') & (df['University'] == 'Monash')]
```

|    | Name   | Title     |   Age | City      | University   |
|---:|:-------|:----------|------:|:----------|:-------------|
|  0 | Sam    | Physicist |    59 | Melbourne | Monash       |
|  1 | Rob    | Economist |    66 | Melbourne | Monash       |


#### Challenge
---

How can we filter people from `Melbourne` and above the Age of `50`?

---

We can also fetch specific rows based on their indexes as well.


```python
df.iloc[1:3]
```

|    | Name   | Title        |   Age | City      | University   |
|---:|:-------|:-------------|------:|:----------|:-------------|
|  1 | Rob    | Economist    |    66 | Melbourne | Monash       |
|  2 | Jack   | Statistician |    42 | Sydney    | UNSW         |


Lets try changing Jack's age to 43, because today is his Birthday and he has now turned 43.


```python
df.loc[2, 'Age'] = 43
```

The above is just one way to do this. Some of the other ways are as follows:  

- `df.at[2, 'Age'] = 43`

- `df.loc[df[df['Name'] == 'Jack'].index, 'Age'] = 43`

Lets look at the updated data frame.


```python
df
```

|    | Name   | Title          |   Age | City      | University   |
|---:|:-------|:---------------|------:|:----------|:-------------|
|  0 | Sam    | Physicist      |    59 | Melbourne | Monash       |
|  1 | Rob    | Economist      |    66 | Melbourne | Monash       |
|  2 | Jack   | Statistician   |    43 | Sydney    | UNSW         |
|  3 | Jill   | Data Scientist |    28 | Sydney    | UTS          |
|  4 | Dave   | Designer       |    24 | Melbourne | Uni Mel      |
|  5 | Alex   | Architect      |    39 | Perth     | UWA          |
|  6 | Steve  | Doctor         |    52 | Brisbane  | UQ           |

For exporting a Pandas dataframe to a `csv` file, we can use `to_csv()` as follows
```python
df.to_csv(filename, index=False)
```

Lets try writing our data frame to a file.


```python
df.to_csv('researchers.csv', index=False)
```

In order to read external files we use `read_csv()` function,
```python
pd.read_csv(filename, sep=',')
```

We can read back the file that we just created.


```python
df_res = pd.read_csv('researchers.csv', sep=',')
df_res
```

|    | Name   | Title          |   Age | City      | University   |
|---:|:-------|:---------------|------:|:----------|:-------------|
|  0 | Sam    | Physicist      |    59 | Melbourne | Monash       |
|  1 | Rob    | Economist      |    66 | Melbourne | Monash       |
|  2 | Jack   | Statistician   |    43 | Sydney    | UNSW         |
|  3 | Jill   | Data Scientist |    28 | Sydney    | UTS          |
|  4 | Dave   | Designer       |    24 | Melbourne | Uni Mel      |
|  5 | Alex   | Architect      |    39 | Perth     | UWA          |
|  6 | Steve  | Doctor         |    52 | Brisbane  | UQ           |


### JSON
---

JSON stands for *JavaScript Object Notation*.

When exchanging data between a browser and a server, the data can only be text.

Python has a built-in package called `json`, which can be used to work with JSON data.


```python
import json
```

Once we imported the library, now lets look at how we can obtain a JSON object from a string (or more accurately a JSON string). This process is also know as deserialization where we convert a string to an object.

JSON is a string that can be turned into ('deserialized') a Python dictionary containing just primitive types (floats, strings, bools, lists, dicts and None).


```python
# Convert from JSON to Python:

# some JSON:
x =  '{ "name":"John", "age":30, "city":"New York"}'

# parse x:
y = json.loads(x)

# the result is a Python dictionary:
print(y["age"])
```

    30
    

Lets take a look at `y` as follows,


```python
print(y)
```

    {
        "name": "John",
        "age": 30,
        "city": "New York"
    }
    

We can obtain the exact same JSON string we defined earlier from a Python dictionary as follows,


```python
# Convert from Python to JSON

# a Python object (dict):
x = {
  "name": "John",
  "age": 30,
  "city": "New York"
}

# convert into JSON:
y = json.dumps(x)

# the result is a JSON string:
print(y)
```

    {"name": "John", "age": 30, "city": "New York"}
    

For better formatting we can indent the same as,


```python
# Indentation
y = json.dumps(x, indent=4)
print(y)

```

    {
        "name": "John",
        "age": 30,
        "city": "New York"
    }
    

### Regex
---

Regular expressions or regex are a powerful tool to extract key pieces of data from raw text.

This is a whirlwind tour of regular expressions so you have some familiarity, but becoming proficient will require further study and practise. Regex is deep enough to have a whole workshop of it's own

You can try your regex expressions in : 

- [pythex](https://pythex.org/) for a python oriented regex editor
- [regexr](https://regexr.com/) for a more visual explanation behind the expressions (good for getting started)


```python
import re # regex package in python is named 're'
```


```python
my_str = 'python123good'
re.search('123', my_str)
```




    <re.Match object; span=(6, 9), match='123'>




```python
if re.search('123', my_str):
    print("Found")
else:
    print("Not found")
```

    Found
    

We can use `[0-9]` in the regular expression to identify any one number in the string.


```python
my_str = 'python123good'
re.search('[0-9]', my_str)
```




    <re.Match object; span=(6, 7), match='1'>



Notice that it matches the first occurance only.

Now, the above regex can be modified to match any three numbers in a string.


```python
my_str = 'python123good'
re.search('[0-9][0-9][0-9]', my_str)
```




    <re.Match object; span=(6, 9), match='123'>




```python
print(re.search('[0-9][0-9][0-9]','hello123')) # matches 123
print(re.search('[0-9][0-9][0-9]','great678python')) # matches 678
print(re.search('[0-9][0-9][0-9]','01234webscraing')) # matches 012
print(re.search('[0-9][0-9][0-9]','01web5678scraing')) # matches 567
print(re.search('[0-9][0-9][0-9]','hello world')) # matches nothing
```

    <re.Match object; span=(5, 8), match='123'>
    <re.Match object; span=(5, 8), match='678'>
    <re.Match object; span=(0, 3), match='012'>
    <re.Match object; span=(5, 8), match='567'>
    None
    

As seen above, it matches the first occurance of three digits occuring together.

The above example can be extended to match any number of numbers using the wild character `*` which matches `zero or more repetitions`.


```python
print(re.search('[a-z]*[0-9]*','hello123@@')) # matches hello123
```

    <re.Match object; span=(0, 8), match='hello123'>
    

What if we just want to capture only the numbers? `Capture group` is the answer.


```python
num_regex = re.compile('[a-z]*([0-9]*)[a-z]*')
my_str = 'python123good'
num_regex.findall(my_str)
```




    ['123', '']



We see that it matchs an empty string as well because `*` matches zero or more occurances. 

To avoid this, we can use `+` which matches one or more occurances.


```python
num_regex = re.compile('[a-z]*([0-9]+)[a-z]*')
my_str = 'python123good'
num_regex.findall(my_str)
```




    ['123']



We can use `^` and `$` to match at the beginning and end of string.

As shown in the below 2 examples, we use `^` to get the numbers by which the string is starting.


```python
num_regex = re.compile('^([0-9]+)[a-z]*')
my_str = '123good'
num_regex.findall(my_str)
```




    ['123']




```python
my_str = 'hello123good'
num_regex.findall(my_str)
```




    []



#### Challenge
---

What regular expression can be used to match the numbers **only** at the end of the string:

- `'[a-z]*([0-9]+)[a-z]+'`
- `'[a-z]*([0-9]+)$'`
- `'$[a-z]*([0-9]+)'`
- `'([0-9]+)'`

---

Now, having learnt the regular expressions on basic strings, the same concept can be applied to an html as well as shown below:


```python
html = r'''
         <!DOCTYPE html>
        <html>
        <head>
        <title>site</title>
        </head>
        <body>

        <h1>Sam</h1>
        <h2>Physicist</h2>
        <p>Lorem ipsum dolor sit amet, consectetur adipiscing elit, sed do eiusmod.</p>

        <h1>Rob</h1>
        <h2>Economist</h2>
        <p>Lorem ipsum dolor sit amet, consectetur adipiscing elit, sed do eiusmod tempor incididunt ut labore et.</p>


        </body>
        </html> 
        '''

print(html)
```

    
             <!DOCTYPE html>
            <html>
            <head>
            <title>site</title>
            </head>
            <body>
    
            <h1>Sam</h1>
            <h2>Physicist</h2>
            <p>Lorem ipsum dolor sit amet, consectetur adipiscing elit, sed do eiusmod.</p>
    
            <h1>Rob</h1>
            <h2>Economist</h2>
            <p>Lorem ipsum dolor sit amet, consectetur adipiscing elit, sed do eiusmod tempor incididunt ut labore et.</p>
    
    
            </body>
            </html> 
            
    

Now, if we are only interested in : 
- names i.e. the data inside the `<h1></h1>` tags, and
- title i.e. the data inside the `<h2></h2>` tags
we can extract the same using regex.


Lets define the expressions (or patterns) to capture all text between the tags as follows :

- `<h1>(.*?)</h1>` : capture all text contained within `<h1></h1>` tags
- `<h2>(.*?)</h2>` : capture all text contained within `<h2></h2>` tags




```python
regex_h1 = re.compile('<h1>(.*?)</h1>')
regex_h2 = re.compile('<h2>(.*?)</h2>')
```

and use `findall()` to return all the instances that match with our pattern,


```python
names = regex_h1.findall(html)
titles = regex_h2.findall(html)

print(names, titles)
```

    ['Sam', 'Rob'] ['Physicist', 'Economist']
    

### From a web scraping perspective
- `JSON` and `XML` are the most widely used formats to carry data all over the internet.
- To work with `CSV`s (or `TSV`s), Pandas DataFrames are the de facto standard.
- Regexes help us extract key pieces of information (sub-strings) from raw, messy and unstructured text (strings).

### References

- https://xkcd.com/353/
- https://pandas.pydata.org/pandas-docs/stable/reference/api/pandas.DataFrame.html
- https://realpython.com/regex-python/