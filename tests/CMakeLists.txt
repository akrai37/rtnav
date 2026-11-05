function(rtnav_test name)
  add_executable(${name} ${name}.c)
  target_link_libraries(${name} PRIVATE ${ARGN} rtnav_warnings)
  add_test(NAME ${name} COMMAND ${name})
endfunction()

rtnav_test(nav_test rtnav_nav)
