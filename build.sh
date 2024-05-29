source_setup_script() {
	case $1 in
		"bash")
			.  install/setup.bash
			;;

		"zsh")
			. install/setup.zsh
			;;

		*)
			echo "Unsupported shell: $1"
			exit 1
			;;
	esac
}

# Determine current shell
current_shell=$(basename "$SHELL")

# Build and source package without converting xacro to urdf
colcon build
source_setup_script "$current_shell"

# Convert xacro model to urdf file
echo "\nConvert XACRO model into a URDF file.....\n"
model_dir="$(dirname "$0")/src/robot_description/models/myCar"
xacro "$model_dir/BMW.urdf.xacro" > "$model_dir/BMW.urdf"

# Re-build package
colcon build
source_setup_script "$current_shell"


