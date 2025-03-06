add_executable(${PROJECT_NAME} main_ada.cpp  
                               common_utility.cpp)
target_link_libraries(${PROJECT_NAME} 
                         ${PROJECT_NAME}.common
                         ${PROJECT_NAME}.pre
                         ${PROJECT_NAME}.lio
                         ${PROJECT_NAME}.algo
                         ${third_party_libs}
                        )
