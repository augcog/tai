# "Converting Recursive Functions to Iterative Implementations"

##  Now we'll talk about the relationship between recursion and iteration. So there are cases when you want to convert a recursive function into an iterative implementation. And that can be tricky, because iteration is a special case of recursion. But for many functions there is a straightforward conversion into iteration. And the example we looked at before is one of those. So the idea is that you have to figure out what state needs to be maintained by the iterative function across each pass through the while statements. And so if we look at the sum digits function that we implemented earlier, we can look at what gets passed into some digits in each recursive call and what gets returned. And those are clues as to what we might need to give names to when we write an iterative version. So what gets passed in is what's left to some, which we call n here as a formal parameter. And what gets returned is a partial sum, the sum of the digits so far. So let's try to write an iterative version.

# "A Method for Iterative Digit Summation"

##  Some digits iterative. Well, we're going to store the partial sum so far. And then, while n is greater than zero, meaning there are digits left to some, we will rebind n and last to the split up version of it. So n will become smaller than it was before. It will contain all but the last digit of what n was. And last, we'll contain the last digit. Then we can update digits sum to be whatever it was before plus last. And we'll eventually return digit sum. So we can still sum digits and we can now sum digits, it's an get the same answer.

# "Finding a Solution Through Inspection"

##  So by inspection, we figured out how to do that.

# "Converting Iterative Implementations to Recursion"

##  It turns out that converting an iterative implementation using a wild statement to recursion is quite a bit more straightforward, precisely because iteration is a special case of recursion. So here's the story. When you look at an iterative implementation, you look for the state that is maintained across different iterations. And you just pass those in as arguments. So here's our iterative implementation of some digits. And the state that's maintained across each pass for the wild suite here is n, which changes to be all but the last digit of n, and the digit sum, which contains the partial sum of digits so far. So when we write a recursive version of the same thing, we pass in exactly that n and the digit sum so far. Instead of a wild statement that says, well n is greater than zero, we now have a base case that is exactly the opposite when one n equals zero, then we just return the digit sum. Otherwise, we execute the same thing as the suite of the wild statement. Except that we pass in the new values of n and the updated digit sum as arguments through a recursive call. The recursive call sum digits rec, on n, which is bound to all but the last digit of what was passed in, and then digit sum plus last is the sum so far. So updates via assignment become arguments to a recursive call. And this can be done quite generally for every iterative implementation.

