"""TRL 7 Operational Environment Demonstration Page."""
import os
import sys
import runpy

ROOT_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if ROOT_DIR not in sys.path:
    sys.path.insert(0, ROOT_DIR)

target = os.path.join(ROOT_DIR, "apps_trl7_dashboard.py")
runpy.run_path(target, run_name="__main__")
