set(MODEL_NAME transdec)

set (files
  model.sdf
  model.config
  transdec.dae
)
install(FILES ${files} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/gazebo-${GAZEBO_VERSION}/models/${MODEL_NAME})
