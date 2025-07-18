import pytest

from rag.file_conversion_router.classes.chunk import Chunk
from tests.test_rag.conftest import load_test_cases_config
from tests.utils import *


@pytest.mark.parametrize(
    "expected_output_path", load_test_cases_config("pkl_structure_tests")
)
def test_pkl_structure(expected_output_path):
    """Shows the internal structure of the pkl file."""
    # load the actual pkl file into expected_content
    expected_content = read_file_contents(Path(expected_output_path[0]), True)
    # TODO: Revise the Chunk design here. Is it necessary to have titles, and chunk_url as separate fields?
    #  Can they be combined into a single metadata field?
    chunk_0 = Chunk(
        content='(Segment 1)\n##3 (8.0 points) Nearly Square (h2)\nImplement near_square, which takes positive integer n and non-negative integer k. It returns the largest integer less than or equal to n which is the product of two positive integers that differ by k or less. You may use solve, which is provided.\n\ndef near_square(n, k):  """Return the largest integer that is less than or equal to n and  equals a * b for some positive integers a and b where abs(a - b) <= k.\n\n >>> near_square(125, 0) # 11 * 11 = 121 and abs(11 - 11) = 0  121  >>> near_square(120, 3) # 10 * 12 = 120 and abs(10 - 12) = 2  120  >>> near_square(120, 1) # 10 * 11 = 110 and abs(10 - 11) = 1  110  """  while True:\n\n gap = k\n\n while ------:  (a)  x = ------  (b)  if ------: # Check if x is a whole number  (c)\n\n return ------  (d)  ------  (e)  ------  (f)\n\n def solve(b, c):  """Returns the largest x for which x * (x + b) = c\n\n >>> solve(2, 120) # x=10 solves x * (x + 2) = 120  10.0  >>> solve(2, 121) # x=10.045... solves x * (x + 2) = 121  10.045361017187261  """  return (b*b/4 + c) ** 0.5 - b/2',
        titles="3 (8.0 points) Nearly Square",
        chunk_url="URL_NOT_AVAILABLE#page=2",
        metadata={
            "titles": "3 (8.0 points) Nearly Square",
            "chunk_url": "URL_NOT_AVAILABLE#page=2",
            "page_num": 2,
            "enhanced": True,
            "original_chunk_url": "URL_NOT_AVAILABLE#page=2",
        },
        page_num=2,
    )

    chunk_1 = Chunk(
        content="(a) (2.0 pt) Fill in blank (a). Select **all** that apply.\n\n gap  gap!= 0\n\n gap? 0\n\n gap?= 0",
        titles="3 (8.0 points) Nearly Square",
        chunk_url="URL_NOT_AVAILABLE#page=2",
        metadata={
            "titles": "3 (8.0 points) Nearly Square",
            "chunk_url": "URL_NOT_AVAILABLE#page=2",
            "page_num": 2,
            "enhanced": True,
            "original_chunk_url": "URL_NOT_AVAILABLE#page=2",
        },
        page_num=2,
    )

    chunk_2 = Chunk(
        content='(Segment 2)\n##3 (8.0 points) Nearly Square (h2, Page 2)\nImplement near_square, which takes positive integer n and non-negative integer k. It returns the largest integer less than or equal to n which is the product of two positive integers that differ by k or less. You may use solve, which is provided.\n\ndef near_square(n, k):  """Return the largest integer that is less than or equal to n and  equals a * b for some positive integers a and b where abs(a - b) <= k.\n\n >>> near_square(125, 0) # 11 * 11 = 121 and abs(11 - 11) = 0  121  >>> near_square(120, 3) # 10 * 12 = 120 and abs(10 - 12) = 2  120  >>> near_square(120, 1) # 10 * 11 = 110 and abs(10 - 11) = 1  110  """  while True:\n\n gap = k\n\n while ------:  (a)  x = ------  (b)  if ------: # Check if x is a whole number  (c)\n\n return ------  (d)  ------  (e)  ------  (f)\n\n def solve(b, c):  """Returns the largest x for which x * (x + b) = c\n\n >>> solve(2, 120) # x=10 solves x * (x + 2) = 120  10.0  >>> solve(2, 121) # x=10.045... solves x * (x + 2) = 121  10.045361017187261  """  return (b*b/4 + c) ** 0.5 - b/2',
        titles="3 (8.0 points) Nearly Square",
        chunk_url="URL_NOT_AVAILABLE#page=2",
        metadata={
            "titles": "3 (8.0 points) Nearly Square",
            "chunk_url": "URL_NOT_AVAILABLE#page=2",
            "page_num": 2,
            "enhanced": True,
            "original_chunk_url": "URL_NOT_AVAILABLE#page=2",
        },
        page_num=2,
    )

    chunk_3 = Chunk(
        content="(a) (2.0 pt) Fill in blank (a). Select **all** that apply.\n\n gap  gap!= 0\n\n gap? 0\n\n gap?= 0",
        titles="3 (8.0 points) Nearly Square",
        chunk_url="URL_NOT_AVAILABLE#page=2",
        metadata={
            "chunk_url": "URL_NOT_AVAILABLE#page=2",
            "enhanced": True,
            "original_chunk_url": "URL_NOT_AVAILABLE#page=2",
            "page_num": 2,
            "titles": "3 (8.0 points) Nearly Square",
        },
        page_num=2,
    )

    expected_content = [chunk_0, chunk_1, chunk_2, chunk_3]

    assert expected_content == [chunk_0, chunk_1, chunk_2, chunk_3]
