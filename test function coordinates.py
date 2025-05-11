"""
testscript for function coordinates
This scripts tests whether the right middle pixel is found, if it selects only connected pixels 
"""
import numpy as np
from unittest import mock
from io import StringIO
from functions import coordinates

image1 = np.zeros((2000, 2000))
image1[950:1050, 950:1050] = 255  # Block from (950,950) to (1049,1049)

# Expected center: mean of rows = 999.5 -> int(999.5) = 999
# Same for columns -> expected (999, 999)
with mock.patch("builtins.open", mock.mock_open()) as mocked_file:
    result1 = coordinates(image1, 1, 2, 3, 4)
    assert result1 == (999, 999), f"Expected (999, 999), got {result1}"
    mocked_file().write.assert_called_once_with("(1, 2, 3, 4):999, 999\n")

print("Test passed with large block and image.")

print("Running test 2: isolated max pixel...")

image2 = np.zeros((5, 5))
image2[0, 0] = 255  # Only one max pixel, no neighbor

with mock.patch("builtins.open", mock.mock_open()):
    result2 = coordinates(image2, 0, 0, 0, 0)
    assert result2 is None, f"Expected None, got {result2}"

print("Test 2 passed.")

print("Running test 3: edge-connected max pixels...")

image3 = np.zeros((5, 5))
image3[0, 0] = 255
image3[0, 1] = 255

with mock.patch("builtins.open", mock.mock_open()):
    result3 = coordinates(image3, 9, 8, 7, 6)
    assert result3 == (0, 0), f"Expected (0,0), got {result3}"

print("Test 3 passed.")
print("All tests completed successfully.")