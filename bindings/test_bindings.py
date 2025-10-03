import sys
import importlib

try:
    import cora # type: ignore
except ImportError as e:
    print("ERROR: Could not import the 'cora' module. Make sure the bindings are built and the .so file is in your PYTHONPATH or current directory.")
    sys.exit(1)

print("Successfully imported 'cora' Python bindings!")

# Example: test for a function or class that should always exist
if hasattr(cora, "__doc__"):
    print("Module docstring:", cora.__doc__)

# Try to call a known function or instantiate a class if available
if hasattr(cora, "Problem"):
    print("Problem(3,3) =", cora.Problem(3,3))  # Example call, adjust as per actual API
else:
    print("'setRank' function not found in cora bindings (this is just an example)")
