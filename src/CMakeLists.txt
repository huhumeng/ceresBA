add_library(
    ceresBA_LIBS SHARED 
    CeresCost/CostFunctions.cc
    Util/GenData.cc
)

target_link_libraries(
    ceresBA_LIBS
    ${Third_LIBS}
)