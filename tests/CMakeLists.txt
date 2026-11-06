function(rtnav_test name)
  add_executable(${name} ${name}.c)
  target_link_libraries(${name} PRIVATE ${ARGN} rtnav_warnings)
  add_test(NAME ${name} COMMAND ${name})
endfunction()

rtnav_test(nav_test rtnav_nav)
rtnav_test(sim_test rtnav_sim)

# Firmware in the loop: the real firmware drives each track for a lap; fails
# on any lockstep mismatch, ADC read-back error or missed tick.
if(RTNAV_FIL)
  foreach(track oval lobes gaps obstacle)
    add_test(NAME fil_${track}
             COMMAND rtnav-fil --elf ${RTNAV_FIRMWARE_ELF} --track ${track})
  endforeach()
endif()
