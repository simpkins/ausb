set(
    ausb_test_srcs
    descriptor_test.cpp
)
if(CONFIG_AUSB_HW_MOCK)
    list(
        APPEND
        ausb_test_srcs
        "StdControlHandlerTest.cpp"
    )
endif()

if(ESP_PLATFORM)
    idf_component_register(
        SRCS ${ausb_test_srcs}
        REQUIRES ausb asel
        WHOLE_ARCHIVE
    )
else()
    add_library(
        ausb_test_cases
        OBJECT
        "${ausb_test_srcs}"
    )
    target_link_libraries(ausb_test_cases PUBLIC ausb)
endif()
