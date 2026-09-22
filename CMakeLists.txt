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


# 只安装本包自己的资源目录（且必须都是实际存在的目录）。
# 注意 1：不要在此列出 BrainCo_revo2/ 与 Inspire_RH5DG2/ —— 它们各自带 package.xml，
#         是独立的 ROS 包（revo2_description / RH5DG2_L / RH5DG2_R），由 colcon 单独构建，
#         若在此 install 会造成重复分发。
# 注意 2：不要安装 script/ 与 关键参数/ —— 它们属于开发期工具及其输入数据，不是运行时资源：
#         4 个脚本都用 Path(__file__).parent.parent 定位包根，装进 share/ 后会去改写 install 树
#         （check_*_update.py --update 覆盖 urdf/tiangong3.urdf，prime_to_convex.py 写
#         meshes_convex/，make_mjcf_torq.py 写 tiangong3_torq.xml）；且 check_*_update.py
#         依赖 关键参数/*.csv，该目录不会被安装，装在 share/ 里必然报错。
#         这些脚本请留在源码树内，按 script/README.md 在包目录下执行。
# 注意 3：meshes_convex/ 由 script/prime_to_convex.py 生成，不是常驻资源；
#         如需随包分发凸包，先生成再把它加回下面的列表。
install(
  DIRECTORY launch urdf meshes config
  DESTINATION share/${PROJECT_NAME}
)


ament_package()
