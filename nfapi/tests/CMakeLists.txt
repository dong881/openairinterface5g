# SPDX-License-Identifier: LicenseRef-CSSL-1.0

if (ENABLE_TESTS)
    add_executable(nfapi_pnf_pacing_test
        nfapi_pnf_pacing_test.c
        ../oai_integration/nfapi_pnf_pacing.c)
    target_include_directories(nfapi_pnf_pacing_test PRIVATE ${CMAKE_SOURCE_DIR})
    add_dependencies(tests nfapi_pnf_pacing_test)
    add_test(nfapi_pnf_pacing_test nfapi_pnf_pacing_test)
    set_tests_properties(nfapi_pnf_pacing_test PROPERTIES LABELS "nfapi;rfsim")

    add_subdirectory(p5)
    add_subdirectory(p7)
endif ()
