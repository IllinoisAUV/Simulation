set(dirs
  transdec
)

foreach(dir ${dirs})
  add_subdirectory(${dir})
endforeach()
