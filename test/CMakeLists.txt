set(
    ausb_test_srcs
    descriptor_test.cpp
    KbdBootReport.cpp
)
set(
    ausb_test_lib_srcs
    "lib/empty.cpp"
)
if(CONFIG_AUSB_HW_MOCK)
    list(
        APPEND
        ausb_test_srcs
        "StdControlHandlerTest.cpp"
        "HidKeyboardTest.cpp"
    )
    list(
        APPEND
        ausb_test_lib_srcs
        "lib/mock_utils.cpp"
    )
endif()

if(ESP_PLATFORM)
    idf_component_register(
        SRCS ${ausb_test_srcs} ${ausb_test_lib_srcs}
        REQUIRES ausb asel
        WHOLE_ARCHIVE
    )
else()
    add_library(
        ausb_test_lib
        STATIC
        "${ausb_test_lib_srcs}"
    )
    target_link_libraries(ausb_test_lib PUBLIC ausb)
    add_library(
        ausb_test_cases
        OBJECT
        "${ausb_test_srcs}"
    )
    target_link_libraries(ausb_test_cases PUBLIC ausb ausb_test_lib)
endif()
