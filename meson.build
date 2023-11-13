project('gridbenchmark2sipp', 'cpp',
  version : '0.1',
  default_options : ['warning_level=3', 'default_library=static', 'cpp_std=c++20'])
boost_dep = dependency('boost', static:true, modules : ['program_options', 'random', 'iostreams'])
zdep = dependency('zlib', static:true)
executable('atsipp',
           ['main.cpp', 'graph.cpp', 'sipp.cpp', 'augmentedsipp.cpp'],
           install : true,
           dependencies : [boost_dep, zdep],
           )
