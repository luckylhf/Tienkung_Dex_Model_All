cmake_minimum_required(VERSION 3.10)
project(tiangong3_urdf)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()


# 安装本包自身的运行时资源目录，列表中的目录必须实际存在。
# - 未列出 BrainCo_revo2/ 与 Inspire_RH5DG2/：两者各自含 package.xml，是独立的 ROS 包
#   （revo2_description / RH5DG2_L / RH5DG2_R），由 colcon 单独构建；在此安装会造成重复分发。
# - 未安装 script/ 与 关键参数/：属于开发期工具及其输入数据，不是运行时资源。script/ 下的
#   5 个脚本均以 Path(__file__).parent.parent 定位包根，安装到 share/ 后会在 install 树内
#   改写文件；且 check_*_update.py 依赖 关键参数/*.csv，该目录不参与安装。
#   脚本应在源码树内、按 script/README.md 在包目录下执行。
# - 未安装 mujoco/：其中的 meshes_convex/ 与 meshes_simplify/ 由 script/ 下的脚本生成，
#   两份 MJCF 以相对路径引用它们，仅在源码树内可用。如需随包分发，先生成网格，再把
#   mujoco 或其中的子目录加入下方列表。
install(
  DIRECTORY launch urdf meshes config
  DESTINATION share/${PROJECT_NAME}
)


ament_package()
