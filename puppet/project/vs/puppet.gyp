{

    'includes': {
        '../../../../sketchlib/shared/sharedconfig/common.gypi',
        '../sources.gypi'
    },

    'variables': {
        
        'import_props_files': [
                
        '$([MSBuild]::GetPathOfFileAbove(Versions.props))',
        'puppet.props',
                
        ],

    },

    'conditions': [

        # --------------------------------------------------------------------------------
        # visual studio targets
        # --------------------------------------------------------------------------------

        ['OS=="win"', {
    
            'targets': [

            {
                'target_name': 'puppet',
                'type': 'static_library',
                'standalone_static_library': 1,
                'product_name': 'puppet',
                'msvs_guid': '9481FE9C-5541-4ED2-8A4E-BC55B5000FDD',
                'hard_dependency': 1,
                'suppress_wildcard' : 1,
                
                'msvs_settings': {
                    'VCCLCompilerTool': {
                        'CompileAsWinRT': 'true',
                    },
                },                

                'include_dirs': [
                ],

                'libraries': [
                ],

                'dependencies': [
                ],
                
                'sources': [
                
                '<@(source_files)',
                
                ],

                'conditions': [

                ['"<(OS_Platform)"=="winui"', {
                    'props_files': [
                        '<@(import_props_files)',
                    ],
                    'nuget_packages': [
                        '<@(import_packages)',
                    ],
                    'nugets_extension_targets': [
                        '<@(import_packages_extension_targets)',
                    ],
                    'nugets_checks': [
                        '<@(import_packages_checks)',
                    ],
                }],

                ],

                
                'msvs_winrt_sources': [
                ],

                'sources!': [
                ],
                
                'sources/': [
                ['exclude', '.*/ios/.*'],
                ],
            },
            
            ]
    
        }],
                
    ],
}
