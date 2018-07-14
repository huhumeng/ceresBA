add_executable(testICPOptim testICPOptim.cc)
target_link_libraries(testICPOptim ${ceresBA_LIBS} ${Third_LIBS})

add_executable(testPnPOptim testPnPOptim.cc)
target_link_libraries(testPnPOptim ${ceresBA_LIBS} ${Third_LIBS})

add_executable(testPnPSE3Optim testPnPSE3Optim.cc)
target_link_libraries(testPnPSE3Optim ${ceresBA_LIBS} ${Third_LIBS})

add_executable(testQuickly testQuickly.cc)
target_link_libraries(testQuickly ${ceresBA_LIBS} ${Third_LIBS})

add_executable(testDirectOpti testDirectOpti.cc)
target_link_libraries(testDirectOpti ${ceresBA_LIBS} ${Third_LIBS})
