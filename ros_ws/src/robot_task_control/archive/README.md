# Historical Task Fixtures

These recipes are retained for reference and regression tests only. They are
not installed by setup.py and are not production entry points.

Use ../launch/robot_task.launch.py and ../config/robot_task.yaml instead.
The old line_robot_test and line_robot_camer_detect_test variants now share
that single workflow. To omit detection, edit its workflow.steps, not Python.
