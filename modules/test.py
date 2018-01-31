import os
from pathlib import Path

p = Path(os.getcwd()).parents[0]

print(str(p))
print(os.getcwd())
