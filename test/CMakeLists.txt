set(TESTS testAccuracyBA testRotationAveraging testRotationRobustOptimization testTranslationAveraging testLoRANSAC)

foreach(TEST ${TESTS})
  add_executable(${TEST} ${TEST}.cpp)
  target_link_libraries(${TEST} gtest GDR_LIB)
endforeach(TEST)

foreach(TEST ${TESTS})
  add_test(${TEST} ${TEST})
endforeach(TEST)
