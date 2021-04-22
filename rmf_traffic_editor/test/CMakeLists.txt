add_executable(
  test_gui
  ../resources/resource.qrc
  test_gui.cpp)

target_link_libraries(
  test_gui
  gui_lib
  Qt5::Test
)

ament_add_test(
  test_gui
  COMMAND "$<TARGET_FILE:test_gui>" -o ${AMENT_TEST_RESULTS_DIR}/rmf_traffic_editor/test_gui.xml,xml -o -,txt
  OUTPUT_FILE ${AMENT_TEST_RESULTS_DIR}/rmf_traffic_editor/test_gui/output.log
)
