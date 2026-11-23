cmake_minimum_required(VERSION 3.8)
project(sentry_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/verify_s1a.py
  scripts/verify_s1b.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()