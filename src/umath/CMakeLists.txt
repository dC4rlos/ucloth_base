add_library(umath STATIC
    umath.hpp
    umath.cpp
)

target_link_libraries(umath
    PUBLIC glm
)

target_include_directories(umath PUBLIC .)