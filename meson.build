project('gridbenchmark2sipp', 'cpp',
  version : '0.1',
  default_options : ['warning_level=3', 'default_library=static', 'cpp_std=c++20'])
boost_dep = dependency('boost', static:true, modules : ['program_options', 'random', 'iostreams', 'timer'])
zdep = dependency('zlib', static:true)
cgal = dependency('CGAL', static:true)
gmp = dependency('gmp')
mpfr = dependency('mpfr')
executable('atsipp',
           ['main.cpp', 'graph.cpp', 'segment.cpp','sipp.cpp', 'augmentedsipp.cpp', 'repeat.cpp', 'rtasipp.cpp'],
           install : true,
           dependencies : [boost_dep, zdep,  gmp, mpfr],
           )
executable('lhtest',
            ['lhtest.cpp', 'segment.cpp'],
            install: true,
            dependencies : [boost_dep,],
            )
executable('atftest',
            ['atftest.cpp', 'segment.cpp'],
            install: true,
            dependencies : [boost_dep,],
            )
