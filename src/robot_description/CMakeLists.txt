cmake_minimum_required(VERSION 3.8)
project(robot_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(xacro REQUIRED)

# Make the custom command depend on a file that will never be created so that
# the command will run every time we build regardless of the BMW.urdf.xacro state
# Add custom command to generate URDF from Xacro
add_custom_command(
  OUTPUT ${CMAKE_CURRENT_SOURCE_DIR}/models/myCar/BMW.urdf
  OUTPUT ${CMAKE_CURRENT_SOURCE_DIR}/a_file_that_should_never_be_created.C
  COMMAND xacro ${CMAKE_CURRENT_SOURCE_DIR}/models/myCar/BMW.urdf.xacro -o ${CMAKE_CURRENT_SOURCE_DIR}/models/myCar/BMW.urdf
  DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/models/myCar/BMW.urdf.xacro
  )

# Add custom target to ensure the command runs
add_custom_target(
  generate_urdf ALL
  DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/a_file_that_should_never_be_created.C
  COMMENT "Generating urdf from xacro"
)

# Install launch, models, and rviz directories
install(
  DIRECTORY launch models rviz config
  DESTINATION share/${PROJECT_NAME}
)

# Add testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
