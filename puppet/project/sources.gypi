{

    'includes': {
        '../../../sketchlib/shared/sharedconfig/common.gypi'
    },

    # --------------------------------------------------------------------------------
    # variables
    # --------------------------------------------------------------------------------
    'variables': {
        
        'source_files' : [
        
        '<!@(<(LIST_SOURCES)  ../../api)',
        '<!@(<(LIST_SOURCES)  ../../shaders)',
        '<!@(<(LIST_SOURCES)  ../../src)',
        '<!@(<(LIST_SOURCES)  ../../PuppetWarpV4)',

        ],

        'test_files' : [
        
        '<!@(<(LIST_SOURCES)  ../../tests)',

        ],
            
    },
    
}
