from tests.utils import *
import pytest

from rag.file_conversion_router.classes.chunk import Chunk
from tests.test_rag.conftest import load_test_cases_config


@pytest.mark.parametrize(
    "expected_output_path",
    load_test_cases_config("pkl_structure_tests")
)
def test_pkl(expected_output_path):
    """ Shows the internal structure of the pkl file. """

    # load the actual pkl file into expected_content
    expected_content = read_file_contents(Path(expected_output_path[0]), True)

    # the content is a list of Chunk objects, each has three attributes
    chunk_1_url = ['URL_NOT_AVAILABLE#', 'URL_NOT_AVAILABLE#3-(8.0-points)-nearly-square']
    chunk_1_titles = ' (h1) > 3 (8.0 points) Nearly Square (h2) (0)'
    chunk_1_content = """(Segment 1)
# (h1)
[]
##3 (8.0 points) Nearly Square (h2)

Implement near_square, which takes positive integer n and non-negative integer k. It returns the largest integer less than or equal to n which is the product of two positive integers that differ by k or less. You may use solve, which is provided.

def near_square(n, k):  \"\"\"Return the largest integer that is less than or equal to n and  equals a * b for some positive integers a and b where abs(a - b) <= k.

 >>> near_square(125, 0) # 11 * 11 = 121 and abs(11 - 11) = 0  121  >>> near_square(120, 3) # 10 * 12 = 120 and abs(10 - 12) = 2  120  >>> near_square(120, 1) # 10 * 11 = 110 and abs(10 - 11) = 1  110  \"\"\"  while True:

 gap = k

 while ------:  (a)  x = ------  (b)  if ------: # Check if x is a whole number  (c)

 return ------  (d)  ------  (e)  ------  (f)

 def solve(b, c):  \"\"\"Returns the largest x for which x * (x + b) = c

 >>> solve(2, 120) # x=10 solves x * (x + 2) = 120  10.0  >>> solve(2, 121) # x=10.045... solves x * (x + 2) = 121  10.045361017187261  \"\"\"  return (b*b/4 + c) ** 0.5 - b/2"""

    chunk_2_url = ['URL_NOT_AVAILABLE#', 'URL_NOT_AVAILABLE#3-(8.0-points)-nearly-square']
    chunk_2_titles = ' (h1) > 3 (8.0 points) Nearly Square (h2) (1)'
    chunk_2_content = """(a) (2.0 pt) Fill in blank (a). Select **all** that apply.

 gap  gap!= 0

 gap? 0

 gap?= 0"""

    chunk_1 = Chunk(chunk_1_titles, chunk_1_content, chunk_1_url)
    chunk_2 = Chunk(chunk_2_titles, chunk_2_content, chunk_2_url)

    assert expected_content == [chunk_1, chunk_2]
