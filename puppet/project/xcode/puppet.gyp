{

    'includes': {
        '../sources.gypi'
    },


    'conditions': [

        # --------------------------------------------------------------------------------
        # mac/ios targets
        # --------------------------------------------------------------------------------

        ['OS=="mac"', {
    
            'targets': [

            {
                'target_name': 'puppet',
                'type': 'static_library',
                'standalone_static_library': 1,
                'product_name': 'puppet',
                'hard_dependency': 1,
                'suppress_wildcard' : 1,
                
                'configurations': { 
                    'Debug': { 
                        'xcode_config_file': 'config/puppet.xcconfig', 
                    }, 
                    'Release': { 
                        'xcode_config_file': 'config/puppet.xcconfig', 
                    }, 
                },
                
                'include_dirs':[
                ],
                
                'libraries': [
                ],

                'dependencies': [
                ],
                
                'sources': [
                
                '<@(source_files)',
                
                ],
                
                'sources!': [
                ],
                
                'sources/': [
                # Metal shaders are complied into the umbrella bundle but included for reference purposes
                ['exclude', 'shaders/.*'],
                ['exclude', '.*/uwp/.*'],
                ],
            },
            
            {
                'target_name': 'puppet-ut',
                'type': 'static_library',
                'standalone_static_library': 1,
                'product_name': 'puppet-ut',
                'hard_dependency': 1,
                'suppress_wildcard' : 1,
                
                'configurations': {
                    'Debug': {
                        'xcode_config_file': 'config/puppet-ut.xcconfig',
                    },
                    'Release': {
                        'xcode_config_file': 'config/puppet-ut.xcconfig',
                    },
                },
                
                'dependencies': [
                ],

                'include_dirs': [
                ],
                
                'sources': [
                
                '<@(test_files)'
                
                ],

                'sources/': [
                ],
            },
            
            ]
    
        }],
                
    ],
}
