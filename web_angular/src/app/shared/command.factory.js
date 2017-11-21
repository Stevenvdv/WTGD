(function () {
    'use strict';

    angular.module('app').factory('Command', function ($resource, ApiUrl) {
        return $resource(ApiUrl + 'controlpanel/commands/:commandId', {commandId: '@CommandId'});
    });


})();
