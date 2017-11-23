(function () {
    'use strict';

    angular.module('app').directive('isAuth', function ($rootScope, authenticationService) {
        return {
            restrict: 'A',
            scope: {
                isAuth: '='
            },
            link: function (scope, element) {
                var toggleDisplay = function (isAuthorized) {
                    if (isAuthorized) {
                        if (scope.isAuth) {
                            element.css('display', 'block');
                        } else {
                            element.css('display', 'none');
                        }
                    } else {
                        if (scope.isAuth) {
                            element.css('display', 'none');
                        } else {
                            element.css('display', 'block');
                        }
                    }
                };

                toggleDisplay(authenticationService.authentication.isAuth);
                $rootScope.$on('authorized', function () {
                    toggleDisplay(authenticationService.authentication.isAuth);
                });
                $rootScope.$on('deauthorized', function () {
                    toggleDisplay(authenticationService.authentication.isAuth);
                });
            }
        }
    });
})();
