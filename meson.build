project('gridbenchmark2sipp', 'cpp',
  version : '0.1',
  default_options : ['warning_level=3', 'default_library=static', 'cpp_std=c++20'])
boost_dep = dependency('boost', static:true, modules : ['program_options', 'random', 'iostreams'])
zdep = dependency('zlib', static:true)
executable('atsipp',
<<<<<<< HEAD
           ['main.cpp', 'graph.cpp', 'segment.cpp','sipp.cpp', 'augmentedsipp.cpp', 'repeat.cpp'],
=======
           ['main.cpp', 'graph.cpp', 'sipp.cpp', 'augmentedsipp.cpp', 'repeat.cpp', 'segment.cpp'],
>>>>>>> 95086b936f179f604d14bffbacc68ca2fc880c31
           install : true,
           dependencies : [boost_dep, zdep],
           )
executable('lhtest',
            ['lhtest.cpp', 'segment.cpp'],
            install: true,
            dependencies : [boost_dep,],
<<<<<<< HEAD
            )
=======
)
>>>>>>> 95086b936f179f604d14bffbacc68ca2fc880c31
executable('atftest',
            ['atftest.cpp', 'segment.cpp'],
            install: true,
            dependencies : [boost_dep,],
<<<<<<< HEAD
            )
=======
            )
>>>>>>> 95086b936f179f604d14bffbacc68ca2fc880c31
