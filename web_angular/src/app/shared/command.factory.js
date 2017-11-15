(function () {
    'use strict';

    angular.module('app').factory('Command', function ($resource, ApiUrl) {
        return $resource(ApiUrl + 'commands/:commandId', {commandId: '@CommandId'});
    });


})();
