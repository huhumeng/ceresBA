add_library(
    ceresBA_LIBS
    ceresBA.cc
)

target_link_libraries(
    ceresBA_LIBS
    ${Third_LIBS}
)