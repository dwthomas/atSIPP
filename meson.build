project('gridbenchmark2sipp', 'cpp',
  version : '0.1',
  default_options : ['warning_level=3', 'default_library=static', 'cpp_std=c++20', 'b_ndebug=if-release'])
boost_dep = dependency('boost', static:true, modules : ['program_options', 'random', 'iostreams', 'timer'])
zdep = dependency('zlib', static:true)
cgal = dependency('CGAL', static:true)
gmp = dependency('gmp')
mpfr = dependency('mpfr')
executable('atsipp',
           ['main.cpp', 'randomsippgraph.cpp', 'segment.cpp','sipp.cpp', 'augmentedsipp.cpp', 'repeat.cpp','rtasipp.cpp', 'plrtosipphonly.cpp', 'hybrid.cpp', 'plrtosipp.cpp'],
           install : true,
           dependencies : [boost_dep, zdep,  gmp, mpfr],
           cpp_args: '-DCGAL_DISABLE_ROUNDING_MATH_CHECK=ON'
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

executable('testrandom',
            ['testrandomsipp.cpp', 'randomsippgraph.cpp'],
            install: true,
            dependencies : [boost_dep,],
            )
