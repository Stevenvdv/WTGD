(function () {
    'use strict';

    angular.module('app')
    // Production
    .constant('ApiUrl', 'http://' + window.location.hostname + '/api/')
    .constant('SignalRUrl', 'http://' + window.location.hostname + '/signalr/');
    // Development
    // .constant('ApiUrl', 'http://localhost:56232/api/')
    // .constant('SignalRUrl', 'http://localhost:56232/signalr/');
})();
