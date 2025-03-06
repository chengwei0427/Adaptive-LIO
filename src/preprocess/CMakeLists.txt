
add_library(${PROJECT_NAME}.pre
        cloud_convert/cloud_convert.cc
        cloud_convert/cloud_convert2.cc
        )

target_link_libraries(${PROJECT_NAME}.pre
        ${third_party_libs}
        )

