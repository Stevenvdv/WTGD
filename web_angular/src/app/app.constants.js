(function () {
    'use strict';

    angular.module('app')
        // Production
        // .constant('ApiUrl', 'http://' + window.location.hostname + '/api/');
        // Development
        .constant('ApiUrl', 'http://localhost:54215/api/');
})();
