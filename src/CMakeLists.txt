aux_source_directory(. SRC_LIST)

SET( SRC_LIST_PRM prm_DT_based.cpp prm_general_operation.cpp prm_sampling.cpp )

add_executable( unit_test_timeusage unit_test_timeusage.cpp ${SRC_LIST_PRM} )
target_link_libraries( unit_test_timeusage ${OpenCV_LIBS})