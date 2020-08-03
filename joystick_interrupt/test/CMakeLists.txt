add_rostest_gtest(test_joystick_interrupt
  test/joystick_interrupt_rostest.test
  src/test_joystick_interrupt.cpp
)
target_link_libraries(test_joystick_interrupt ${catkin_LIBRARIES})
add_dependencies(test_joystick_interrupt joystick_interrupt joystick_mux)
