source_setup_script() {
  case $1 in
    "bash")
      echo -e "\nFound bash Shell ... Sourcing install/setup.bash\n"
      source "$(dirname "$0")/install/setup.bash"
      ;;

    "zsh")
      echo -e "\nFound zsh Shell ... Sourcing install/setup.zsh\n"
      source "$(dirname "$0")/install/setup.zsh"
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

# Ensure the setup script is sourced
source_setup_script "$current_shell"

# Convert xacro model to urdf file
echo -e "\nConvert XACRO model into a URDF file...\n"
model_dir="$(dirname "$0")/src/robot_description/models/myCar"
xacro "$model_dir/BMW.urdf.xacro" > "$model_dir/BMW.urdf"

# Check if the URDF file was created successfully
if [ -f "$model_dir/BMW.urdf" ]; then
  echo "URDF file created successfully."
else
  echo "Failed to create URDF file."
  exit 1
fi

# Re-build package
colcon build

# Ensure the setup script is sourced again
source_setup_script "$current_shell"

