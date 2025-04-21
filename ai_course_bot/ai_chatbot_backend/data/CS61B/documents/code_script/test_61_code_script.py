"""61B Code Script Example
"""

def factorial(n):
    """Compute the factorial of n."""
    if n <= 1:
        return 1
    return n * factorial(n-1)
