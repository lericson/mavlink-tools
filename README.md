# MAVLink Tools

These are all heavily overlapping with QGroundControl capabilities, but easier
to work with, and much more hackable.

## `mavlink_shell.py` improved

Among the features:

- Proper line editing with the usual stuff, `^W, ^A, ^E, ^Y` etc. working
  properly.

- Prints log messages aka MAVLink `STATUSTEXT` messages (while still supporting
  line editing)

## `mavplot.py`

Show the current pose in terminal, with a GUI for showing the last N positions.
Very useful for debugging the position estimator.

## `logdl.py`

Like a crappy rsync for logs. Useful for examining flight data.
