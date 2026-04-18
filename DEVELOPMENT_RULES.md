# DEVELOPMENT_RULES

## Naming rules
- Package names use lowercase snake_case.
- Topics use lowercase slash-separated names such as `/cmd_vel`, `/odom`, and `/scan`.
- Frames use lowercase names with underscores such as `map`, `odom`, `base_footprint`, and `base_link`.
- Launch filenames should describe the feature they start, for example `navigation.launch` or `display.launch`.

## Commit rules
- Keep commits focused on one concern when possible.
- Use concise commit subjects that explain the change intent.
- Build the workspace before creating a commit that changes package structure, launch files, or dependencies.

## Launch, config, and script rules
- Prefer launch files for composition and YAML for tunable parameters.
- Keep scripts small and phase-appropriate; avoid dropping complex logic into early skeleton packages.
- Document every new launch file, config file, and executable script in the package README.
- Avoid hard-coding machine-specific absolute paths.

## Test rules after each change
- Run `catkin_make` from `~/catkin_ws`.
- Source `~/catkin_ws/devel/setup.bash` before runtime checks.
- For launch changes, verify at least the XML structure and package references.
- For scripts, ensure the shebang is correct and executable permissions are set when needed.
