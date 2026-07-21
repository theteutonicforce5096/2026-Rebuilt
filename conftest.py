import os
import sys

# Ensure the project root is importable so tests can `import constants...`
# regardless of whether they are run via `python -m pytest` or `robotpy test`.
sys.path.insert(0, os.path.dirname(__file__))
