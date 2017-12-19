(function () {
    'use strict';

    angular.module('app').run(runBlock);

    /** @ngInject */
    function runBlock($log, $rootScope, $transitions, $state, authenticationService) {
        // Set the page title on route change
        $transitions.onSuccess({}, function ($transition) {
            $rootScope.title = $transition.to().title;

            if ($transition.to().name !== 'login'){
                if (!authenticationService.authentication.isAuth) {
                    $state.go('login');
                }
            }
        });

        $log.debug('App run block end');
    }

})();
