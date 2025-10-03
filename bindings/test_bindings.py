import sys
import importlib

try:
    cora = importlib.import_module("cora")
except ImportError as e:
    print("ERROR: Could not import the 'cora' module. Make sure the bindings are built and the .so file is in your PYTHONPATH or current directory.")
    sys.exit(1)

print("Successfully imported 'cora' Python bindings!")

# Example: test for a function or class that should always exist
if hasattr(cora, "__doc__"):
    print("Module docstring:", cora.__doc__)

# Try to call a known function or instantiate a class if available
if hasattr(cora, "add"):
    print("add(2, 3) =", cora.add(2, 3))
else:
    print("'add' function not found in cora bindings (this is just an example)")

if hasattr(cora, "Problem"):
    try:
        problem = cora.Problem()
        print("Successfully instantiated cora.Problem")
        if hasattr(problem, "num_variables"):
            print("num_variables:", problem.num_variables())
    except Exception as e:
        print("Could not instantiate cora.Problem or call num_variables:", e)
else:
    print("'Problem' class not found in cora bindings (this is just an example)")
