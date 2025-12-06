project('gridbenchmark2sipp', 'cpp',
  version : '0.1',
  default_options : ['warning_level=3', 'default_library=static', 'cpp_std=c++20', 'b_ndebug=if-release'])
boost_dep = dependency('boost', static:true, modules : ['program_options', 'random', 'iostreams', 'timer'])
gmp_dep = dependency('gmp', static:true)
executable('grid2d',
           ['domains/grid2d/main.cpp', 'domains/grid2d/randomsippgraph.cpp', 'data_structures/segment.cpp',
            'search_algorithms/SIPP/sipp.cpp', 'search_algorithms/augmentedSIPP/augmentedsipp.cpp', 'search_algorithms/aSIPPnew/augmentedsipp.cpp', 'search_algorithms/abSIPP/absipp.cpp',
            'search_algorithms/RePEAT/repeat.cpp','search_algorithms/RTAS/rtasipp.cpp',
            'search_algorithms/PLRTS/plrtosipphonly.cpp', 'search_algorithms/MinATFS/hybrid.cpp',
            'search_algorithms/MaxATFS/plrtosipp.cpp', 'search_algorithms/WTH/wth.cpp',
            'search_algorithms/OTH/oth.cpp'],
           install : true,
           dependencies : [boost_dep, gmp_dep],
           cpp_args: '-DCGAL_DISABLE_ROUNDING_MATH_CHECK=ON'
           )
executable('lhtest',
            ['lhtest.cpp', 'data_structures/segment.cpp'],
            install: true,
            dependencies : [boost_dep,],
            )
executable('atftest',
            ['atftest.cpp', 'data_structures/segment.cpp'],
            install: true,
            dependencies : [boost_dep, gmp_dep],
            )

executable('testrandom',
            ['domains/grid2d/testrandomsipp.cpp', 'domains/grid2d/randomsippgraph.cpp'],
            install: true,
            dependencies : [boost_dep, gmp_dep],
            )
