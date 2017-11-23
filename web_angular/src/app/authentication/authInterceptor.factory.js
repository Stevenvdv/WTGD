(function () {
    'use strict';

    angular.module('app').factory('authInterceptorService', function ($q, $injector, $location, localStorageService) {
        var authInterceptorServiceFactory = {};

        var _request = function (config) {
            config.headers = config.headers || {};

            var authData = localStorageService.get('authorizationData');
            if (authData) {
                config.headers.Authorization = 'Bearer ' + authData.token;
            }

            return config;
        };

        var _responseError = function (rejection) {
            if (rejection.status === 401) {
                var authService = $injector.get('authService');
                authService.logOut();
            }

            return $q.reject(rejection);
        };

        authInterceptorServiceFactory.request = _request;
        authInterceptorServiceFactory.responseError = _responseError;

        return authInterceptorServiceFactory;
    })
})();