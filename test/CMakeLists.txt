set(TESTS testBundleAdjustment testAbsolutePoseStructure testAbsoluteTranslationsComputation testAbsoluteRotationsComputation testPointCloudCorrespondences testPoint3dRepresentation testRotationRepresentation testRelativePoseExtraction testUmeyamaRansac testPoseFileCreation testRelativePosesComputation testICP testRotationAveraging testTranslationAveraging testVectors3dRepresentation testRobustRotationOptimization)

foreach(TEST ${TESTS})
  add_executable(${TEST} ${TEST}.cpp)
  target_link_libraries(${TEST} gtest GDR_LIB)
endforeach(TEST)

foreach(TEST ${TESTS})
  add_test(${TEST} ${TEST})
endforeach(TEST)
