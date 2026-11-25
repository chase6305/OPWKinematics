function(expect_rejection model diagnostic)
    execute_process(COMMAND "${OPW_TOOL}" "${model}" ${ARGN}
        RESULT_VARIABLE result OUTPUT_VARIABLE output ERROR_VARIABLE errors)
    if(NOT result STREQUAL "1" OR NOT output STREQUAL "" OR NOT errors MATCHES "${diagnostic}")
        message(FATAL_ERROR "Unexpected URDF diagnostic. Exit ${result}\n${output}${errors}")
    endif()
endfunction()

expect_rejection("${OPW_FIXTURE}" "six independent revolute or continuous joints"
    --tip gripper_finger)

file(READ "${OPW_FIXTURE}" urdf)
string(REPLACE "xyz=\"0 0 0.13\"" "xyz=\"0.01 0 0.13\"" offset_wrist "${urdf}")
set(offset_fixture "${OPW_TEST_DIR}/opw_offset_wrist.urdf")
file(WRITE "${offset_fixture}" "${offset_wrist}")
expect_rejection("${offset_fixture}" "axis 5 gap.*axis 6 gap.*tolerance")

set(second_tip [=[
  <link name="camera_mount"/>
  <joint name="camera_joint" type="fixed">
    <parent link="link6"/><child link="camera_mount"/>
  </joint>
</robot>
]=])
string(REPLACE "</robot>" "${second_tip}" ambiguous "${urdf}")
set(ambiguous_fixture "${OPW_TEST_DIR}/opw_ambiguous_tip.urdf")
file(WRITE "${ambiguous_fixture}" "${ambiguous}")
expect_rejection("${ambiguous_fixture}" "--tip.*camera_mount.*tool0")

execute_process(COMMAND "${OPW_TOOL}" "${ambiguous_fixture}" --tip tool0
    RESULT_VARIABLE result OUTPUT_VARIABLE output ERROR_VARIABLE errors)
if(NOT result STREQUAL "0" OR NOT output MATCHES "\"tip_link\": \"tool0\"")
    message(FATAL_ERROR "Explicit URDF tip selection failed. Exit ${result}\n${errors}")
endif()
