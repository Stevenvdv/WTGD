(function () {
    'use strict';

    angular.module('app').config(configBlock);

    /** @ngInject */
    function configBlock($locationProvider, $logProvider, $httpProvider, NotificationProvider) {
        $locationProvider.html5Mode(false);
        $logProvider.debugEnabled(true);

        $httpProvider.interceptors.push('authInterceptorService');

        NotificationProvider.setOptions({
            delay: 5000,
            startTop: 10,
            startRight: 10,
            verticalSpacing: 10,
            horizontalSpacing: 20,
            positionX: 'right',
            positionY: 'bottom',
            templateUrl: 'app/shared/shared.notification.view.html'
        });
    }
})();
