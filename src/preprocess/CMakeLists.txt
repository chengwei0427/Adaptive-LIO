add_library(${PROJECT_NAME}.pre
        cloud_convert/cloud_convert.cc
        cloud_convert/cloud_convert2.cc
        )
        add_dependencies(${PROJECT_NAME}.pre livox_ros_driver_gencpp)

target_link_libraries(${PROJECT_NAME}.pre
        ${third_party_libs}
        )

