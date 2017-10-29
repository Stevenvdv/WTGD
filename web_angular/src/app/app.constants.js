(function () {
    'use strict';

    angular.module('app')
        // Production
        // .constant('ApiUrl', 'https://bis-inspect.com/api/');
        // Development
        .constant('ApiUrl', 'http://localhost:53419/api/');
})();
