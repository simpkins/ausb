set(
    ausb_test_srcs
    descriptor_test.cpp
)

if(ESP_PLATFORM)
    idf_component_register(
        SRCS ${ausb_test_srcs}
        REQUIRES ausb asel
        WHOLE_ARCHIVE
    )
else()
    add_library(
        test_cases
        OBJECT
        "${ausb_test_srcs}"
    )
    target_link_libraries(test_cases PUBLIC ausb)
endif()
