add_library( ucloth_c_api SHARED
    uclothexport.h
    uclothhandles.h
    uclothinterface.h
    uclothinterface.cpp
    uclothstructures.h
)

target_link_libraries(ucloth_c_api
    PRIVATE ucloth
)

target_include_directories(ucloth_c_api PUBLIC .)