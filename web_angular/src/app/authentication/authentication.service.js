(function () {
    'use strict';

    angular.module('app').factory('authenticationService', function ($rootScope, $http, $q, $state, localStorageService, ApiUrl) {
        var authenticationServiceFactory = {};
        var _authentication = {
            isAuth: false,
            username: ''
        };

        var _login = function (loginData) {
            var deferred = $q.defer();
            $http.post(ApiUrl + 'authentication/token/create', loginData).then(function (response) {
                var tokenContent = parseJwt(response.data.token);
                localStorageService.set('authorizationData', {
                    token: response.data.token,
                    tokenContent: tokenContent,
                    username: loginData.username
                });

                _authentication.isAuth = true;
                _authentication.username = loginData.username;

                $rootScope.currentUser = loginData.username;
                $rootScope.$broadcast('authorized');

                deferred.resolve(response);
            }).catch(function (err) {
                _logOut();
                deferred.reject(err);
            });

            return deferred.promise;
        };

        var _logOut = function () {
            localStorageService.remove('authorizationData');

            _authentication.isAuth = false;
            _authentication.username = '';
            _authentication.tokenContent = '';

            console.log('Logged out');
            // If you're logged out there's nothing else to do but log in again ¯\_(ツ)_/¯
            $state.go('login');
            $rootScope.$broadcast('deauthorized');
        };

        var _fillAuthData = function () {
            var authData = localStorageService.get('authorizationData');
            if (authData) {
                _authentication.isAuth = true;
                _authentication.username = authData.username;
                _authentication.tokenContent = authData.tokenContent;
                _authentication.token = authData.token;

                $rootScope.$broadcast('authorized');
            }
        };

        function parseJwt(token) {
            var base64Url = token.split('.')[1];
            var base64 = base64Url.replace('-', '+').replace('_', '/');
            return JSON.parse(window.atob(base64));
        }

        authenticationServiceFactory.login = _login;
        authenticationServiceFactory.logOut = _logOut;
        authenticationServiceFactory.fillAuthData = _fillAuthData;
        authenticationServiceFactory.authentication = _authentication;

        _fillAuthData();

        return authenticationServiceFactory;
    })
})();
